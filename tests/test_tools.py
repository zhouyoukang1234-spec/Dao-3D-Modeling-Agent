"""Framework unit tests — no FreeCAD required."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent.tools import ToolRegistry, ToolResult  # noqa: E402


def test_toolresult_envelope():
    r = ToolResult.success(volume=10)
    assert r.ok and r.data["volume"] == 10 and r.error is None
    d = r.to_dict()
    assert d["ok"] and d["data"]["volume"] == 10
    f = ToolResult.failure("boom", code=3)
    assert not f.ok and f.error == "boom" and f.data["code"] == 3


def test_register_and_call():
    reg = ToolRegistry()
    reg.register("solid.box", lambda a: ToolResult.success(v=a["l"] ** 3), "cube")
    r = reg.call("solid.box", {"l": 2})
    assert r.ok and r.data["v"] == 8
    assert r.tool == "solid.box" and r.args == {"l": 2}
    assert r.elapsed_ms >= 0


def test_duplicate_registration_rejected():
    reg = ToolRegistry()
    reg.register("a.b", lambda a: ToolResult.success())
    with pytest.raises(ValueError):
        reg.register("a.b", lambda a: ToolResult.success())


def test_unknown_tool():
    reg = ToolRegistry()
    r = reg.call("nope", {})
    assert not r.ok and "unknown tool" in r.error


def test_handler_exception_is_caught():
    reg = ToolRegistry()

    def boom(a):
        raise RuntimeError("kaboom")

    reg.register("x.y", boom)
    r = reg.call("x.y", {})
    assert not r.ok and "kaboom" in r.error


def test_groups_and_first_matching():
    reg = ToolRegistry()
    reg.register("solid.box", lambda a: ToolResult.success())
    reg.register("param.pad", lambda a: ToolResult.success())
    g = reg.groups()
    assert set(g) == {"solid", "param"}
    assert reg.first_matching("mesh.measure", "solid.box") == "solid.box"
    assert reg.first_matching("nope") is None


def test_manifest():
    reg = ToolRegistry()
    reg.register("solid.box", lambda a: ToolResult.success(), "make a box")
    m = reg.manifest()
    assert m[0]["name"] == "solid.box" and m[0]["group"] == "solid"
    assert m[0]["summary"] == "make a box"
