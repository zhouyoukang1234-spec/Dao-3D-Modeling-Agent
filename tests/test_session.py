"""Session perceive/act/verify tests on the analytic mock backend (no FreeCAD)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent.backends.mock_backend import build_mock_registry  # noqa: E402
from cad_agent.session import AgentSession  # noqa: E402


def _session():
    return AgentSession("mock", build_mock_registry())


def test_act_records_history():
    s = _session()
    s.act("solid.box", {"name": "b", "length": 2, "width": 3, "height": 4})
    assert len(s.history) == 1
    assert s.last().tool == "solid.box" and s.last().ok


def test_perceive_and_verify_pass():
    s = _session()
    s.act("solid.box", {"name": "b", "length": 10, "width": 10, "height": 10})
    p = s.perceive("b")
    assert p.ok and p.data["volume"] == 1000
    v = s.verify("b", {"volume": 1000, "valid": True})
    assert v.ok and v.data["verified"]


def test_verify_detects_mismatch():
    s = _session()
    s.act("solid.box", {"name": "b", "length": 1, "width": 1, "height": 1})
    v = s.verify("b", {"volume": 999})
    assert not v.ok and "volume" in v.data["mismatches"]


def test_boolean_cut_closed_loop():
    s = _session()
    s.act("solid.box", {"name": "plate", "length": 20, "width": 20, "height": 10})
    s.act("solid.cylinder", {"name": "pin", "radius": 3, "height": 10})
    s.act("solid.cut", {"a": "plate", "b": "pin", "out": "flange"})
    expected = 20 * 20 * 10 - math.pi * 9 * 10
    v = s.verify("flange", {"volume": expected}, tol=1e-6)
    assert v.ok, v.data


def test_summary():
    s = _session()
    s.act("solid.box", {"name": "b", "length": 1, "width": 1, "height": 1})
    s.act("solid.measure", {"name": "missing"})  # fails
    sm = s.summary()
    assert sm["actions"] == 2 and sm["ok"] == 1 and sm["failed"] == 1
