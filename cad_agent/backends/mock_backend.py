"""Analytic mock backend — no FreeCAD required.

Provides a tiny set of ``solid.*`` tools backed by closed-form volume/bbox math
so the framework (registry, session, perceive/verify loop) can be exercised in
CI where FreeCAD is not installed. It is intentionally not a geometry engine —
just enough truth to validate the agent plumbing.
"""
from __future__ import annotations

import math
from typing import Any, Dict

from ..tools import ToolRegistry, ToolResult


def build_mock_registry() -> ToolRegistry:
    reg = ToolRegistry()
    shapes: Dict[str, Dict[str, Any]] = {}

    def _bbox(size):
        return {"bbox_size": [round(float(x), 4) for x in size]}

    def box(a):
        v = a["length"] * a["width"] * a["height"]
        shapes[a["name"]] = {"volume": float(v), "valid": True, "faces": 6,
                             **_bbox((a["length"], a["width"], a["height"]))}
        return ToolResult.success(**shapes[a["name"]])

    def cylinder(a):
        v = math.pi * a["radius"] ** 2 * a["height"]
        shapes[a["name"]] = {"volume": float(v), "valid": True, "faces": 3,
                             **_bbox((2 * a["radius"], 2 * a["radius"], a["height"]))}
        return ToolResult.success(**shapes[a["name"]])

    def cut(a):
        base = shapes[a["a"]]["volume"]
        tool = shapes[a["b"]]["volume"]
        out = a.get("out", a["a"])
        shapes[out] = {"volume": max(0.0, base - tool), "valid": True,
                       "faces": shapes[a["a"]]["faces"] + 1,
                       "bbox_size": shapes[a["a"]]["bbox_size"]}
        return ToolResult.success(**shapes[out])

    def measure(a):
        s = shapes.get(a["name"])
        if s is None:
            return ToolResult.failure("no such solid: %s" % a["name"])
        return ToolResult.success(**s)

    def list_(a):
        return ToolResult.success(solids=list(shapes.keys()))

    reg.register("solid.box", box, "mock box")
    reg.register("solid.cylinder", cylinder, "mock cylinder")
    reg.register("solid.cut", cut, "mock boolean cut")
    reg.register("solid.measure", measure, "mock measure")
    reg.register("solid.list", list_, "mock list")
    return reg
