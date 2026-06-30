"""Recipe library capstone -- the distilled wisdom must generalise.

``smoke_project`` / ``smoke_assembly_project`` built ONE part / ONE assembly at
fixed numbers. Here the same shapes are produced through the parametric recipe
library (:mod:`cad_agent.recipes`) at DIFFERENT sizes and a different spacer
count, and the result is verified against each recipe's own closed-form ``meta``
-- proving the experience was promoted into a reusable, broadly-adaptive system
component, not a one-off. Pure-text recipe-expansion (no kernel) is checked too,
including the guided-error contract on bad parameters.
"""
from cad_agent import new_session, recipes
from cad_agent.planner import Planner


def _approx(a, b, tol=1.0):
    return abs(a - b) < tol


def main():
    # ---- pure expansion: parametric & guarded (no kernel needed) --------- #
    r5 = recipes.generate("bolted_stack", n_spacers=5)
    assert r5.meta["component_count"] == 8, r5.meta          # 5 + base/bolt/nut
    assert r5.meta["spacer_count"] == 5, r5.meta
    # one extra spacer == one extra add + one extra stack vs the 3-spacer form.
    r3 = recipes.generate("bolted_stack", n_spacers=3)
    assert len(r5.steps) - len(r3.steps) == 4, (len(r5.steps), len(r3.steps))
    for bad, msg in (
            ({"n_spacers": 0}, "n_spacers"),
            ({"bolt_r": 6, "bore_r": 6}, "clearance"),
            ({"spacer_r": 40, "plate_size": 60}, "exceeds")):
        try:
            recipes.generate("bolted_stack", **bad)
            raise AssertionError("expected ValueError for %r" % bad)
        except ValueError as exc:
            assert msg in str(exc), (bad, str(exc))
    try:
        recipes.generate("nope")
        raise AssertionError("expected unknown-recipe error")
    except ValueError as exc:
        assert "unknown recipe" in str(exc), exc

    # ---- NL routes to a recipe (the wisdom reachable from plain text) ---- #
    pl = Planner()
    pn = pl.plan("build a bolted stack with 5 spacers")
    assert pn.steps and pn.steps[0]["tool"] == "recipe", pn
    assert pn.steps[0]["args"] == {"name": "bolted_stack",
                                   "params": {"n_spacers": 5}}, pn.steps
    pb = pl.plan("螺栓垫片堆叠 3 个垫片")
    assert pb.steps[0]["args"]["name"] == "bolted_stack", pb.steps
    assert pb.steps[0]["args"]["params"].get("n_spacers") == 3, pb.steps
    pbr = pl.plan("make a mounting bracket")
    assert pbr.steps[0]["args"]["name"] == "flanged_bracket", pbr.steps
    # the recipe router must not poach ordinary modelling/measuring intents.
    assert pl.plan("box 20x10x5").steps[0]["tool"] == "solid.box"
    assert pl.plan("measure plate").steps[0]["tool"] == "solid.measure"
    assert pl.plan("cylinder r=5 h=20").steps[0]["tool"] != "recipe"

    # ---- run a 5-spacer stack at non-default sizes on the live kernel ----- #
    s = new_session("recipes")
    print("FreeCAD", s.registry.kernel.freecad_version)
    res = s.make("bolted_stack", n_spacers=5, plate_size=70, plate_h=12,
                 spacer_r=22, spacer_h=6, bore_r=7, bolt_r=6, nut_r=10, nut_h=9)
    assert res.ok, res.error
    assert res.data["failed"] == 0, res.data
    assert res.data["executed"] == res.data["planned"], res.data
    meta = res.data["meta"]

    bom = s.act("asm.bom", {"density": 0.00785})
    assert bom.ok, bom.error
    assert bom.data["component_count"] == meta["component_count"], (bom.data, meta)
    assert bom.data["line_items"]["spacer"]["count"] == 5, bom.data
    assert _approx(bom.data["line_items"]["spacer"]["unit_volume"],
                   meta["unit_volume"]["spacer"]), (bom.data, meta)

    clash = s.act("asm.interference", {})
    assert clash.ok and clash.data["clash_count"] == 0, clash.data   # clearance fit

    m = s.act("asm.measure", {"density": 0.00785,
                              "inertia_axis": {"point": meta["axis"] + [0], "dir": [0, 0, 1]}})
    assert m.ok, m.error
    assert m.data["components"] == meta["component_count"], m.data
    assert _approx(m.data["volume"], meta["total_volume"]), (m.data["volume"], meta)
    assert m.data["bbox_size"] == meta["bbox_size"], (m.data["bbox_size"], meta)
    assert m.data["inertia_axis"] > 0, m.data

    # ---- a flanged bracket at fresh dimensions through the same path ------ #
    s2 = new_session("recipes2")
    rb = s2.make("flanged_bracket", length=100, width=60, height=12,
                 boss_r=10, boss_h=16, bore_r=5, hole_r=4, hole_inset=12)
    assert rb.ok and rb.data["failed"] == 0, rb.data
    part = rb.data["meta"]["part"]
    meas = s2.act("solid.measure", {"name": part})
    assert meas.ok and meas.data["valid"], meas
    assert _approx(meas.data["volume"], rb.data["meta"]["volume"], tol=2.0), \
        (meas.data["volume"], rb.data["meta"]["volume"])
    bb = s2.act("analyze.bbox", {"name": part})
    assert bb.ok and bb.data["size"] == rb.data["meta"]["bbox_size"], bb.data

    # ---- end-to-end: plain text -> build() -> recipe pseudo-step -> kernel  #
    s3 = new_session("recipes3")
    bt = s3.build("bolted stack with 2 spacers")
    assert bt.ok, bt.error
    rstep = bt.data["transcript"][0]["steps"][0]
    assert rstep["tool"] == "recipe" and rstep["ok"], rstep
    assert rstep["executed"] == rstep["planned"] and rstep["executed"] > 0, rstep
    bom3 = s3.act("asm.bom", {})
    assert bom3.data["component_count"] == 5, bom3.data        # 2 + base/bolt/nut
    s3.registry.kernel.shutdown()

    print("recipe library: bolted_stack(n=5,custom) vol %.2f bbox %s clash-free; "
          "flanged_bracket(100x60) vol %.2f -- recipes generalise" % (
              m.data["volume"], m.data["bbox_size"], meas.data["volume"]))
    print("RECIPES SMOKE OK", s.summary(), s2.summary())
    s.registry.kernel.shutdown()
    s2.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
