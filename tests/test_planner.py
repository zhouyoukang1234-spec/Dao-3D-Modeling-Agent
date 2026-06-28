"""Unit tests for the NL planner (FreeCAD-free — runs everywhere, incl. CI)."""
from cad_agent.planner import Planner


def first(plan):
    assert plan.error is None, plan.error
    assert plan.steps, "no steps produced"
    return plan.steps[0]


def test_box_shorthand():
    s = first(Planner().plan("box 60x40x20"))
    assert s["tool"] == "solid.box"
    assert (s["args"]["length"], s["args"]["width"], s["args"]["height"]) == (60, 40, 20)


def test_box_named_and_keyword_dims():
    s = first(Planner().plan("box length 30 width 10 height 5 name plate"))
    assert s["tool"] == "solid.box" and s["args"]["name"] == "plate"
    assert s["args"]["length"] == 30 and s["args"]["height"] == 5


def test_cylinder_diameter_to_radius():
    s = first(Planner().plan("cylinder diameter 16 h 40"))
    assert s["tool"] == "solid.cylinder" and s["args"]["radius"] == 8 and s["args"]["height"] == 40


def test_cut_from():
    s = first(Planner().plan("cut hole from plate"))
    assert s["tool"] == "solid.cut"
    assert s["args"] == {"a": "plate", "b": "hole", "out": "plate"}


def test_union_and_intersect():
    assert first(Planner().plan("union a and b"))["tool"] == "solid.union"
    assert first(Planner().plan("intersect a with b"))["tool"] == "solid.common"


def test_fillet_resolves_it_to_last_object():
    p = Planner()
    p.plan("box 10x10x10 name blk")
    s = first(p.plan("fillet it radius 2"))
    assert s["tool"] == "solid.fillet"
    assert s["args"]["name"] == "blk" and s["args"]["radius"] == 2


def test_transform_move_and_rotate():
    p = Planner()
    p.plan("box 10x10x10 name b")
    mv = first(p.plan("move b by 5 0 0"))
    assert mv["tool"] == "solid.translate" and mv["args"]["vector"] == [5, 0, 0]
    rot = first(p.plan("rotate b 90 about z"))
    assert rot["tool"] == "solid.rotate" and rot["args"]["angle"] == 90


def test_polar_pattern():
    s = first(Planner().plan("polar pattern lug count 6"))
    assert s["tool"] == "solid.pattern_polar"
    assert s["args"]["name"] == "lug" and s["args"]["count"] == 6


def test_reset_and_list_and_render():
    assert first(Planner().plan("reset"))["tool"] == "__reset__"
    assert first(Planner().plan("list objects"))["tool"] == "solid.list"
    assert first(Planner().plan("render"))["tool"] == "view.scene"


def test_direct_tool_call():
    s = first(Planner().plan('solid.box {"name":"x","length":2,"width":2,"height":2}'))
    assert s["tool"] == "solid.box" and s["args"]["name"] == "x"


def test_raw_json_plan_list():
    plan = Planner().plan('[{"tool":"solid.box","args":{"name":"a","length":1,"width":1,"height":1}},'
                          ' {"tool":"solid.measure","args":{"name":"a"}}]')
    assert plan.error is None and len(plan.steps) == 2
    assert plan.steps[1]["tool"] == "solid.measure"


def test_unparseable_returns_error_not_crash():
    plan = Planner().plan("please make something nice and shiny")
    assert plan.error and not plan.steps


def test_delete_and_measure():
    assert first(Planner().plan("delete plate"))["tool"] == "solid.delete"
    assert first(Planner().plan("measure plate"))["tool"] == "solid.measure"
