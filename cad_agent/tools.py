"""Tool protocol layer — an MCP-like contract for CAD operations.

Every capability the agent has (build a box, pad a sketch, solve an assembly,
inspect mass properties, render a view) is expressed as a :class:`Tool` with a
name, a JSON-serialisable argument schema, and a handler. A :class:`ToolRegistry`
collects them so the session loop and the MCP server can discover and invoke
them uniformly. This mirrors how Cursor exposes editor actions to the model —
here the "editor" is FreeCAD.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional


@dataclass
class ToolResult:
    """Uniform result envelope returned by every tool handler."""

    ok: bool
    data: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None
    tool: str = ""
    args: Dict[str, Any] = field(default_factory=dict)
    elapsed_ms: float = 0.0

    @classmethod
    def success(cls, **data: Any) -> "ToolResult":
        return cls(ok=True, data=data)

    @classmethod
    def failure(cls, error: str, **data: Any) -> "ToolResult":
        return cls(ok=False, error=error, data=data)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "tool": self.tool,
            "args": self.args,
            "data": self.data,
            "error": self.error,
            "elapsed_ms": round(self.elapsed_ms, 2),
        }


Handler = Callable[[Dict[str, Any]], ToolResult]


@dataclass
class Tool:
    """A single named capability."""

    name: str
    handler: Handler
    summary: str = ""
    schema: Dict[str, Any] = field(default_factory=dict)
    group: str = ""

    def __post_init__(self) -> None:
        if not self.group and "." in self.name:
            self.group = self.name.split(".", 1)[0]


class ToolRegistry:
    """Ordered collection of tools, keyed by name."""

    def __init__(self) -> None:
        self._tools: Dict[str, Tool] = {}

    def register(
        self,
        name: str,
        handler: Handler,
        summary: str = "",
        schema: Optional[Dict[str, Any]] = None,
    ) -> Tool:
        if name in self._tools:
            raise ValueError(f"tool already registered: {name}")
        tool = Tool(name=name, handler=handler, summary=summary, schema=schema or {})
        self._tools[name] = tool
        return tool

    def get(self, name: str) -> Optional[Tool]:
        return self._tools.get(name)

    def has(self, name: str) -> bool:
        return name in self._tools

    def names(self) -> List[str]:
        return list(self._tools.keys())

    def groups(self) -> Dict[str, List[str]]:
        out: Dict[str, List[str]] = {}
        for t in self._tools.values():
            out.setdefault(t.group, []).append(t.name)
        return out

    def first_matching(self, *candidates: str) -> Optional[str]:
        """Return the first registered name among the candidates (engine-agnostic)."""
        for c in candidates:
            if c in self._tools:
                return c
        return None

    def call(self, name: str, args: Optional[Dict[str, Any]] = None) -> ToolResult:
        args = args or {}
        tool = self._tools.get(name)
        if tool is None:
            return ToolResult(ok=False, error=f"unknown tool: {name}", tool=name, args=args)
        start = time.perf_counter()
        try:
            result = tool.handler(args)
        except Exception as exc:  # handlers must not crash the loop
            result = ToolResult(ok=False, error=f"{type(exc).__name__}: {exc}")
        result.tool = name
        result.args = args
        result.elapsed_ms = (time.perf_counter() - start) * 1000.0
        return result

    def manifest(self) -> List[Dict[str, Any]]:
        return [
            {"name": t.name, "group": t.group, "summary": t.summary, "schema": t.schema}
            for t in self._tools.values()
        ]
