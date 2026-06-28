"""dao-freecad-agent — a deeply-integrated "Cursor for FreeCAD".

Drives a persistent, live FreeCAD kernel through a uniform tool protocol, with a
perceive / act / verify loop, parametric PartDesign features, assembly, mass
properties, interference, sketch-health diagnostics and an MCP surface.
"""
from .tools import Tool, ToolRegistry, ToolResult
from .session import AgentSession

__all__ = [
    "Tool", "ToolRegistry", "ToolResult", "AgentSession",
    "new_session", "build_freecad_registry", "FreeCADKernel",
]


def build_freecad_registry(kernel=None):
    from .backends.freecad_backend import build_freecad_registry as _b
    return _b(kernel)


def FreeCADKernel(*a, **k):
    from .backends.freecad_backend import FreeCADKernel as _K
    return _K(*a, **k)


def new_session(name: str, registry=None):
    """Create an :class:`AgentSession`; spins up a FreeCAD kernel if none given."""
    if registry is None:
        registry = build_freecad_registry()
    return AgentSession(name, registry)
