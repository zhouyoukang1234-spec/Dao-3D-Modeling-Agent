"""Unit tests for the closed-loop agent's *pure* logic (no FreeCAD needed).

The geometry execution path needs FreeCAD and is exercised separately inside the
GUI; here we lock down the deterministic parts: each goal emits a well-formed
plan, and each goal's remediation provably converges its design variables toward
the acceptance band.
"""
import os
import sys

import pytest

_DAO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "freecad", "DAO")
if _DAO not in sys.path:
    sys.path.insert(0, _DAO)

dao_agent = pytest.importorskip("dao_agent")


def _well_formed(plan):
    assert isinstance(plan, list) and plan
    for step in plan:
        assert "tool" in step and "args" in step
        assert step["tool"].startswith(("solid.", "asm.", "param."))


def test_all_goals_emit_well_formed_plans():
    for cls in dao_agent.GOALS.values():
        _well_formed(cls().plan())


def test_press_fit_remediation_converges_into_band():
    g = dao_agent.PressFit()                       # starts interfering (pin>hole)
    for _ in range(20):
        clr = g.params["hole_r"] - g.params["pin_r"]
        if g.params["lo"] <= clr <= g.params["hi"]:
            break
        assert g.remediate([]) is True
    clr = g.params["hole_r"] - g.params["pin_r"]
    assert g.params["lo"] <= clr <= g.params["hi"]


def test_safe_fillet_backs_off_radius_then_stops():
    g = dao_agent.SafeFillet()
    start = g.params["radius"]
    assert g.remediate([]) is True
    assert g.params["radius"] < start
    for _ in range(50):
        if not g.remediate([]):
            break
    assert g.params["radius"] >= g.params["min_radius"]
    assert g.remediate([]) is False               # cannot shrink past the floor


def test_bolt_circle_pulls_holes_interior():
    g = dao_agent.BoltCircle()                     # starts breaching the rim
    assert g.params["bcr"] + g.params["hole_r"] > g.params["flange_r"] - 2.0
    for _ in range(50):
        if g.params["bcr"] + g.params["hole_r"] <= g.params["flange_r"] - 2.0:
            break
        assert g.remediate([]) is True
    assert g.params["bcr"] + g.params["hole_r"] <= g.params["flange_r"] - 2.0


def test_bolt_circle_plan_scales_with_hole_count():
    plan4 = dao_agent.BoltCircle(n=4).plan()
    plan8 = dao_agent.BoltCircle(n=8).plan()
    assert len(plan8) > len(plan4)


def test_bearing_block_running_fit_converges_by_bisection():
    g = dao_agent.BearingBlock()                   # starts interfering (shaft>bore)
    steps = 0
    for _ in range(20):
        clr = g.params["bore_r"] - g.params["shaft_r"]
        if g.params["lo"] <= clr <= g.params["hi"]:
            break
        assert g.remediate([]) is True
        steps += 1
    clr = g.params["bore_r"] - g.params["shaft_r"]
    assert g.params["lo"] <= clr <= g.params["hi"]
    assert steps <= 6                              # bisection: log-fast


def test_bearing_block_plan_has_shaft_and_four_mounts():
    plan = dao_agent.BearingBlock().plan()
    names = [s["args"].get("name") for s in plan if "name" in s["args"]]
    assert "shaft" in names
    assert sum(1 for n in names if n and n.startswith("m")) >= 4


def test_intent_resolves_english_and_chinese():
    assert dao_agent.resolve_goal_intent("please make a press fit")[0] == "press_fit"
    assert dao_agent.resolve_goal_intent("做一个铰链")[0] == "hinge"
    assert dao_agent.resolve_goal_intent("轴承座 with holes")[0] == "bearing_block"
    assert dao_agent.resolve_goal_intent("just a box 10x10x10") is None


def test_intent_parses_gear_module_and_teeth():
    name, ov = dao_agent.resolve_goal_intent("make a meshing M2 20/30 gear pair")
    assert name == "gear_pair"
    assert ov["m"] == 2.0 and ov["z1"] == 20 and ov["z2"] == 30
    # centre distance must start off the pitch sum so the loop has work to do
    assert ov["center"] > (ov["m"] * (ov["z1"] + ov["z2"]) / 2.0)


def test_intent_overrides_drive_goal_params():
    name, ov = dao_agent.resolve_goal_intent("齿轮副 模数3 齿数 18 24")
    g = dao_agent.GOALS[name](**ov)
    assert g.params["m"] == 3.0 and g.params["z1"] == 18 and g.params["z2"] == 24


def test_intent_parses_clearance_band_and_hole():
    name, ov = dao_agent.resolve_goal_intent("press fit hole 8 clearance 0.2-0.6")
    assert name == "press_fit"
    assert ov["hole_r"] == 8.0 and ov["lo"] == 0.2 and ov["hi"] == 0.6


def test_intent_band_orders_lo_hi():
    _, ov = dao_agent.resolve_goal_intent("bearing block 间隙 0.8 到 0.3")
    assert ov["lo"] == 0.3 and ov["hi"] == 0.8


def test_intent_parses_bolt_count():
    name, ov = dao_agent.resolve_goal_intent("bolt circle with 8 holes")
    assert name == "bolt_circle"
    assert ov["n"] == 8


def test_intent_ignores_unknown_keys():
    # hinge has no hole_r/n; numbers that don't map are dropped, not crammed in
    name, ov = dao_agent.resolve_goal_intent("hinge clearance 0.15-0.4")
    g = dao_agent.GOALS[name](**ov)            # must not raise on unknown kwargs
    assert g.params["lo"] == 0.15 and g.params["hi"] == 0.4


def test_gear_pair_converges_to_pitch_sum():
    g = dao_agent.GearPair()                        # starts with too-large centre
    r1 = g.params["m"] * g.params["z1"] / 2.0
    r2 = g.params["m"] * g.params["z2"] / 2.0
    for _ in range(30):
        gap = g.params["center"] - (r1 + r2)
        if 0.0 <= gap <= g.params["tol"]:
            break
        assert g.remediate([]) is True
    gap = g.params["center"] - (r1 + r2)
    assert 0.0 <= gap <= g.params["tol"]            # meshes, no interference


def test_hinge_plan_has_three_components_and_mate():
    plan = dao_agent.Hinge().plan()
    tools = [s["tool"] for s in plan]
    assert tools.count("asm.add") == 3
    assert "asm.coaxial" in tools


def test_hinge_clearance_converges_by_bisection():
    g = dao_agent.Hinge()                           # starts clashing (pin>bore)
    for _ in range(20):
        clr = g.params["bore_r"] - g.params["pin_r"]
        if g.params["lo"] <= clr <= g.params["hi"]:
            break
        assert g.remediate([]) is True
    clr = g.params["bore_r"] - g.params["pin_r"]
    assert g.params["lo"] <= clr <= g.params["hi"]


def test_pin_joint_plan_uses_assembly_mate():
    plan = dao_agent.PinJoint().plan()
    tools = [s["tool"] for s in plan]
    assert "asm.create" in tools
    assert "asm.coaxial" in tools
    assert tools.count("asm.add") == 2


def test_pin_joint_clearance_converges_by_bisection():
    g = dao_agent.PinJoint()                        # starts clashing (pin>bore)
    for _ in range(20):
        clr = g.params["bore_r"] - g.params["pin_r"]
        if g.params["lo"] <= clr <= g.params["hi"]:
            break
        assert g.remediate([]) is True
    clr = g.params["bore_r"] - g.params["pin_r"]
    assert g.params["lo"] <= clr <= g.params["hi"]


def test_l_bracket_fuses_then_backs_off_fillet():
    plan = dao_agent.LBracket().plan()
    assert any(s["tool"] == "solid.union" for s in plan)
    g = dao_agent.LBracket()
    start = g.params["radius"]
    assert g.remediate([]) is True
    assert g.params["radius"] < start
    for _ in range(50):
        if not g.remediate([]):
            break
    assert g.params["radius"] >= g.params["min_radius"]
