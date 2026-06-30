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
    def make(self, recipe: str, **params: Any) -> ToolResult:
        """Expand a parametric recipe (:mod:`cad_agent.recipes`) into tool steps
        and run them on the live kernel -- the reusable counterpart to
        :meth:`build`, where the plan comes from a named, parameterised generator
        instead of free text. Steps whose tool the registry lacks are recorded
        as skipped (orchestration degrades gracefully), and the recipe's
        closed-form ``meta`` is returned alongside the transcript so a caller can
        verify the result without re-deriving the expectations.
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
        return ToolResult.success(recipe=rec.name, meta=rec.meta, steps=steps_out,
                                  planned=len(rec.steps), executed=executed, failed=failed)

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
