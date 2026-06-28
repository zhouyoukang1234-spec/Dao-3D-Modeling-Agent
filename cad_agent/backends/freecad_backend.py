"""Host-side driver for the persistent live FreeCAD kernel.

Launches ``freecadcmd freecad_kernel.py`` once, keeps it alive, and exchanges
JSON-RPC frames with it over stdin/stdout. The live document persists across
calls, so the agent drives FreeCAD like an interactive session. Ops discovered
from the kernel are auto-registered into a :class:`ToolRegistry` under their
group prefix (``solid.*`` / ``param.*`` / ``asm.*``).
"""
from __future__ import annotations

import glob
import json
import os
import queue
import subprocess
import sys
import threading
from typing import Any, Dict, List, Optional

from ..tools import ToolRegistry, ToolResult

SENTINEL = "@@DAO@@"
_HERE = os.path.dirname(os.path.abspath(__file__))
KERNEL = os.path.join(_HERE, "freecad_kernel.py")

# Ops whose name maps to the ``solid.*`` group (direct BREP). param.*/asm.* ops
# already carry a dotted prefix from their modules.
_SOLID_OPS = {
    "box", "cylinder", "sphere", "cone", "torus", "extrude", "revolve", "loft",
    "shell", "translate", "rotate", "mirror", "union", "cut", "common", "fillet",
    "chamfer", "pattern_linear", "pattern_polar", "measure", "inspect",
    "inertia", "curvature", "obb", "symmetry", "fingerprint", "match", "chirality", "holes",
    "library_match", "library_index", "interference", "draft", "thickness", "undercut", "overhang", "section",
    "dfm_report", "compound", "decompose", "joints", "mechanism", "drive",
    "recognize", "reverse", "coaxial", "fourbar", "geartrain", "gearmesh",
    "rackpinion", "cam", "planetary", "geneva", "cam_profile", "spatial_mobility",
    "list", "delete", "export", "import_step",
}


def find_freecadcmd() -> str:
    env = os.environ.get("FREECADCMD")
    if env and os.path.exists(env):
        return env
    pats = [
        r"C:\Program Files\FreeCAD*\bin\freecadcmd.exe",
        r"C:\Program Files\FreeCAD*\bin\FreeCADCmd.exe",
        "/usr/bin/freecadcmd",
        "/usr/local/bin/freecadcmd",
    ]
    for pat in pats:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    raise FileNotFoundError("freecadcmd not found; set FREECADCMD env var")


class FreeCADKernel:
    """Owns the long-lived freecadcmd subprocess and a request/response pipe."""

    def __init__(self, freecadcmd: Optional[str] = None, log_stderr: bool = True):
        self.exe = freecadcmd or find_freecadcmd()
        self._id = 0
        self._lock = threading.Lock()
        self._frames: "queue.Queue[dict]" = queue.Queue()
        self._pending: Dict[int, dict] = {}
        self._log_stderr = log_stderr
        self.ops: List[str] = []
        self.freecad_version = ""
        self._start()

    def _start(self) -> None:
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        env["PYTHONUTF8"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        self.proc = subprocess.Popen(
            [self.exe, KERNEL],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=env, bufsize=1, universal_newlines=True, encoding="utf-8", errors="replace",
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._errth = threading.Thread(target=self._drain_stderr, daemon=True)
        self._errth.start()
        ready = self._await(0, timeout=120)  # kernel emits id=0 ready frame
        self.ops = ready["data"]["ops"]
        self.freecad_version = ready["data"].get("freecad", "")

    def _read_loop(self) -> None:
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            idx = line.find(SENTINEL)
            if idx < 0:
                continue
            try:
                frame = json.loads(line[idx + len(SENTINEL):])
            except Exception:
                continue
            self._frames.put(frame)

    def _drain_stderr(self) -> None:
        for line in self.proc.stderr:
            if self._log_stderr and line.strip():
                sys.stderr.write("[kernel] " + line)

    def _await(self, rid: Optional[int], timeout: float) -> dict:
        if rid in self._pending:
            return self._pending.pop(rid)
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                frame = self._frames.get(timeout=max(0.05, deadline - time.time()))
            except queue.Empty:
                break
            if frame.get("id") == rid:
                return frame
            self._pending[frame.get("id")] = frame
        raise TimeoutError("no response for id=%s within %ss" % (rid, timeout))

    def call(self, op: str, args: Optional[dict] = None, timeout: float = 60) -> dict:
        with self._lock:
            self._id += 1
            rid = self._id
            self.proc.stdin.write(json.dumps({"id": rid, "op": op, "args": args or {}}) + "\n")
            self.proc.stdin.flush()
            return self._await(rid, timeout)

    def reset(self) -> None:
        self.call("__reset__")

    def shutdown(self) -> None:
        try:
            self.call("__shutdown__", timeout=10)
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass


# Heavy ops keep the live kernel busy well past the snappy default: TechDraw
# projection/section rendering, CalculiX FEM solves and CAM path posting can each
# run for minutes (and stack up when suites share one box). They get a generous
# budget so a legitimately slow render is not mistaken for a hang, while light
# BREP ops keep the short default for fast failure detection.
_HEAVY_PREFIXES = ("draw.", "fem.", "path.", "cam.")
_HEAVY_TIMEOUT = 300.0


def _op_timeout(op: str) -> float:
    return _HEAVY_TIMEOUT if op.startswith(_HEAVY_PREFIXES) else 60.0


def register_kernel_tools(registry: ToolRegistry, kernel: FreeCADKernel) -> None:
    """Register every kernel op as a tool, prefixing bare BREP ops with ``solid.``."""
    for op in kernel.ops:
        tool_name = ("solid." + op) if op in _SOLID_OPS else op

        def make(op_name: str):
            def handler(args: Dict[str, Any]) -> ToolResult:
                frame = kernel.call(op_name, args, timeout=_op_timeout(op_name))
                if frame.get("ok"):
                    return ToolResult.success(**frame.get("data", {}))
                return ToolResult.failure(frame.get("error", "kernel error"),
                                          trace=frame.get("trace"))
            return handler

        if not registry.has(tool_name):
            registry.register(tool_name, make(op), summary="FreeCAD live op: " + op)


def build_freecad_registry(kernel: Optional[FreeCADKernel] = None) -> ToolRegistry:
    kernel = kernel or FreeCADKernel()
    reg = ToolRegistry()
    register_kernel_tools(reg, kernel)
    reg.kernel = kernel  # type: ignore[attr-defined]
    return reg
