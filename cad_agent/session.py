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
