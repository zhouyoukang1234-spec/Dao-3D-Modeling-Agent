"""Host-side driver for the persistent live FreeCAD **GUI** kernel.

Launches the full ``freecad`` binary once under the Qt ``offscreen`` platform
(no X server / Xvfb required), keeps it alive, and exchanges the same ``@@DAO@@``
JSON-RPC frames as :class:`~cad_agent.backends.freecad_backend.FreeCADKernel`.
The live GUI application persists across calls, so the agent drives workbenches,
the menu/toolbar command registry and the selection exactly like a human at the
FreeCAD window — only headless.
"""
from __future__ import annotations

import glob
import json
import os
import queue
import subprocess
import sys
import threading
from typing import Dict, List, Optional

SENTINEL = "@@DAO@@"
_HERE = os.path.dirname(os.path.abspath(__file__))
GUI_KERNEL = os.path.join(_HERE, "freecad_gui_kernel.py")


def find_freecad() -> str:
    env = os.environ.get("FREECAD")
    if env and os.path.exists(env):
        return env
    cmd = os.environ.get("FREECADCMD")
    if cmd:
        # freecadcmd and freecad live in the same bin dir; derive the GUI binary.
        base = os.path.dirname(cmd)
        for name in ("freecad", "FreeCAD", "freecad.exe", "FreeCAD.exe"):
            cand = os.path.join(base, name)
            if os.path.exists(cand):
                return cand
    pats = [
        r"C:\Program Files\FreeCAD*\bin\freecad.exe",
        r"C:\Program Files\FreeCAD*\bin\FreeCAD.exe",
        "/usr/bin/freecad",
        "/usr/local/bin/freecad",
    ]
    for pat in pats:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    raise FileNotFoundError("freecad (GUI binary) not found; set FREECAD env var")


class FreeCADGuiKernel:
    """Owns the long-lived full-``freecad`` subprocess and a request/response pipe."""

    def __init__(self, freecad: Optional[str] = None, log_stderr: bool = False,
                 ready_timeout: float = 180.0):
        self.exe = freecad or find_freecad()
        self._id = 0
        self._lock = threading.Lock()
        self._frames: "queue.Queue[dict]" = queue.Queue()
        self._pending: Dict[int, dict] = {}
        self._log_stderr = log_stderr
        self.ops: List[str] = []
        self.freecad_version = ""
        self._start(ready_timeout)

    def _start(self, ready_timeout: float) -> None:
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        env["PYTHONUTF8"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        # Boot the real GUI with no display: the Qt offscreen platform needs no
        # X server, so this is self-contained (no Xvfb dependency).
        env["QT_QPA_PLATFORM"] = "offscreen"
        env.setdefault("XDG_RUNTIME_DIR", "/tmp/dao-runtime")
        try:
            os.makedirs(env["XDG_RUNTIME_DIR"], mode=0o700, exist_ok=True)
        except Exception:
            pass
        self.proc = subprocess.Popen(
            [self.exe, GUI_KERNEL],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=env, bufsize=1, universal_newlines=True, encoding="utf-8",
            errors="replace",
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._errth = threading.Thread(target=self._drain_stderr, daemon=True)
        self._errth.start()
        ready = self._await(0, timeout=ready_timeout)
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
                sys.stderr.write("[gui-kernel] " + line)

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
        raise TimeoutError("no gui response for id=%s within %ss" % (rid, timeout))

    def call(self, op: str, args: Optional[dict] = None, timeout: float = 60) -> dict:
        with self._lock:
            self._id += 1
            rid = self._id
            self.proc.stdin.write(
                json.dumps({"id": rid, "op": op, "args": args or {}}) + "\n")
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
