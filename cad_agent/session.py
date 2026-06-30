"""Agent session — the perceive / act / verify loop.

An :class:`AgentSession` wraps a :class:`ToolRegistry` and records a history of
actions, mirroring how an interactive CAD operator works: take an action, look
at the result (perceive), check it against intent (verify), iterate. It is
engine-agnostic: it discovers whichever ``measure`` / ``perceive`` tools the
registry exposes (``solid.*`` for the FreeCAD backend) rather than hard-coding
one backend.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .tools import ToolRegistry, ToolResult


@dataclass
class HistoryEntry:
    tool: str
    args: Dict[str, Any]
    ok: bool
    error: Optional[str]
    data: Dict[str, Any] = field(default_factory=dict)


class AgentSession:
    def __init__(self, name: str, registry: ToolRegistry):
        self.name = name
        self.registry = registry
        self.history: List[HistoryEntry] = []
        self._measure_tool = registry.first_matching("solid.measure", "mesh.measure")
        self._list_tool = registry.first_matching("solid.list", "mesh.list")

    # -- core action ------------------------------------------------------- #
    def act(self, tool: str, args: Optional[Dict[str, Any]] = None) -> ToolResult:
        result = self.registry.call(tool, args or {})
        self.history.append(HistoryEntry(
            tool=tool, args=args or {}, ok=result.ok, error=result.error, data=result.data))
        return result

    # -- orchestration (意念 -> 选模块 -> 执行) ---------------------------- #
    def build(self, script: str, planner: Optional[Any] = None) -> ToolResult:
        """Plan a natural-language script into tool calls and run them on the
        live kernel — closing the loop the standalone :class:`Planner` only
        prepared but never executed here.

        ``script`` may hold several intents separated by newlines or ``;``
        (e.g. ``"box 20x10x5; cylinder r=4 h=20; cut cyl1 from box1; fillet it
        radius 2"``). A single stateful planner threads object names and the
        ``it``/``this`` back-reference across lines, so a multi-step *pipeline*
        executes as one fused build. Steps whose tool the registry lacks are
        recorded as skipped (planner control markers like ``__reset__`` too),
        not raised — orchestration degrades gracefully.
        """
        if not isinstance(script, str) or not script.strip():
            return ToolResult.failure("build 'script' must be a non-empty string")
        if planner is None:
            from .planner import Planner
            planner = Planner()
        known = set(self.registry.names())
        lines = [ln.strip() for ln in script.replace(";", "\n").splitlines()
                 if ln.strip()]
        transcript, executed, failed = [], 0, 0
        for line in lines:
            plan = planner.plan(line)
            entry: Dict[str, Any] = {"line": line, "note": plan.note, "steps": []}
            if plan.error:
                entry["error"] = plan.error
                failed += 1
                transcript.append(entry)
                continue
            for step in plan.steps:
                tool = step["tool"]
                if tool == "recipe":
                    ra = step.get("args", {})
                    rr = self.make(ra.get("name", ""), **(ra.get("params") or {}))
                    entry["steps"].append({"tool": "recipe", "args": ra, "ok": rr.ok,
                                           "error": rr.error,
                                           "executed": rr.data.get("executed"),
                                           "planned": rr.data.get("planned")})
                    if rr.ok:
                        executed += 1
                    else:
                        failed += 1
                    continue
                if tool.startswith("__") or tool not in known:
                    entry["steps"].append({"tool": tool, "skipped": True})
                    continue
                r = self.act(tool, step.get("args", {}))
                entry["steps"].append({"tool": tool, "args": step.get("args", {}),
                                       "ok": r.ok, "error": r.error})
                if r.ok:
                    executed += 1
                else:
                    failed += 1
            transcript.append(entry)
        return ToolResult.success(transcript=transcript, lines=len(lines),
                                  executed=executed, failed=failed)

    # -- recipes (parametric, reusable orchestration) --------------------- #
    def make(self, recipe: str, verify: bool = False, tol: float = 1e-2,
             **params: Any) -> ToolResult:
        """Expand a parametric recipe (:mod:`cad_agent.recipes`) into tool steps
        and run them on the live kernel -- the reusable counterpart to
        :meth:`build`, where the plan comes from a named, parameterised generator
        instead of free text. Steps whose tool the registry lacks are recorded
        as skipped (orchestration degrades gracefully), and the recipe's
        closed-form ``meta`` is returned alongside the transcript so a caller can
        verify the result without re-deriving the expectations.

        With ``verify=True`` the act loop is closed automatically: once every
        step runs, the built result is *perceived* on the kernel and checked
        against the recipe's own closed-form ``meta`` (volume, bounding box,
        component count). The recipe thus becomes self-validating -- the same
        check for any recipe at any parameters -- and ``failed`` is bumped on a
        mismatch so a green ``make`` means "built AND physically correct".
        """
        from . import recipes
        try:
            rec = recipes.generate(recipe, **params)
        except (ValueError, TypeError) as exc:
            return ToolResult.failure("recipe %r: %s" % (recipe, exc))
        known = set(self.registry.names())
        steps_out, executed, failed = [], 0, 0
        for step in rec.steps:
            tool = step["tool"]
            if tool not in known:
                steps_out.append({"tool": tool, "skipped": True})
                continue
            r = self.act(tool, step.get("args", {}))
            steps_out.append({"tool": tool, "args": step.get("args", {}),
                              "ok": r.ok, "error": r.error})
            if r.ok:
                executed += 1
            else:
                failed += 1
        out: Dict[str, Any] = {"recipe": rec.name, "meta": rec.meta, "steps": steps_out,
                               "planned": len(rec.steps), "executed": executed,
                               "failed": failed}
        if verify and failed == 0:
            ver = self._verify_recipe(rec.meta, tol)
            out["verified"] = ver["verified"]
            out["mismatches"] = ver["mismatches"]
            if not ver["verified"]:
                out["failed"] = failed + len(ver["mismatches"])
        return ToolResult.success(**out)

    def _verify_recipe(self, meta: Dict[str, Any], tol: float) -> Dict[str, Any]:
        """Perceive the just-built result and compare it to a recipe's closed-form
        ``meta``. Assemblies are read through ``asm.*``; single parts through
        ``solid.*`` / ``analyze.*`` -- the ``meta`` shape (an ``assembly`` vs a
        ``part`` key) selects which, so one routine covers every recipe."""
        mism: Dict[str, Any] = {}

        def _close(have: Any, want: Any) -> bool:
            if isinstance(want, list):
                return (isinstance(have, list) and len(have) == len(want)
                        and all(_close(h, w) for h, w in zip(have, want)))
            if isinstance(want, (int, float)):
                return have is not None and abs(float(have) - float(want)) <= \
                    tol * max(1.0, abs(float(want)))
            return have == want

        def _check(label: str, have: Any, want: Any) -> None:
            if not _close(have, want):
                mism[label] = {"want": want, "got": have}

        if "assembly" in meta:
            m = self.act("asm.measure", {})
            if m.ok:
                _check("volume", m.data.get("volume"), meta.get("total_volume"))
                _check("bbox_size", m.data.get("bbox_size"), meta.get("bbox_size"))
                _check("components", m.data.get("components"), meta.get("component_count"))
            else:
                mism["measure"] = m.error
        elif "part" in meta:
            m = self.act("solid.measure", {"name": meta["part"]})
            if m.ok:
                _check("volume", m.data.get("volume"), meta.get("volume"))
            else:
                mism["measure"] = m.error
            bb = self.act("analyze.bbox", {"name": meta["part"]})
            if bb.ok:
                _check("bbox_size", bb.data.get("size"), meta.get("bbox_size"))
            else:
                mism["bbox"] = bb.error
        return {"verified": not mism, "mismatches": mism}

    # -- perception -------------------------------------------------------- #
    def perceive(self, name: str) -> ToolResult:
        """Inspect a model's current geometric state (metrics)."""
        if not self._measure_tool:
            return ToolResult.failure("no measure tool registered")
        return self.act(self._measure_tool, {"name": name})

    # -- verification ------------------------------------------------------ #
    def verify(self, name: str, expect: Dict[str, Any], tol: float = 1e-3) -> ToolResult:
        """Check measured metrics against expected values within tolerance.

        ``expect`` may contain scalar keys (volume, area, faces, ...) and
        ``valid``/``closed`` booleans. Returns ok=True only if all match.
        """
        m = self.perceive(name)
        if not m.ok:
            return m
        got = m.data
        mismatches = {}
        for key, want in expect.items():
            have = got.get(key)
            if isinstance(want, bool):
                if bool(have) != want:
                    mismatches[key] = {"want": want, "got": have}
            elif isinstance(want, (int, float)):
                if have is None or abs(float(have) - float(want)) > tol * max(1.0, abs(float(want))):
                    mismatches[key] = {"want": want, "got": have}
            else:
                if have != want:
                    mismatches[key] = {"want": want, "got": have}
        if mismatches:
            return ToolResult.failure("verification failed", mismatches=mismatches, measured=got)
        return ToolResult.success(verified=True, measured=got)

    # -- convenience ------------------------------------------------------- #
    def tools(self) -> List[str]:
        return self.registry.names()

    def last(self) -> Optional[HistoryEntry]:
        return self.history[-1] if self.history else None

    def summary(self) -> Dict[str, Any]:
        ok = sum(1 for h in self.history if h.ok)
        return {"session": self.name, "actions": len(self.history),
                "ok": ok, "failed": len(self.history) - ok}
