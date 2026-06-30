"""配方组合 capstone -- compose several recipes into one larger project.

The recipe layer (``cad_agent.recipes``) distilled each capstone into a single
parametric generator. The next rung of orchestration wisdom is *composition*:
a real brief rarely asks for one part -- it asks for several that must coexist
in one document. This suite proves the distilled recipes compose, through the
ordinary multi-line :meth:`AgentSession.build` path, with no new machinery:

    "parametric plate\\n bolted stack with 2 spacers"

routes to TWO recipe pseudo-steps that run back to back on the same kernel. The
point is that the two recipes occupy independent name-spaces (a PartDesign body
``Plate`` vs the ``asm.*`` stack components), so the combined document holds
both intact -- each still verifiable against its own closed-form ``meta``. Any
collision or cross-talk between composed recipes would fail the suite.
"""
from cad_agent import new_session
from cad_agent import recipes


def _approx(a, b, tol=1.0):
    return abs(a - b) < tol


def main():
    s = new_session("compose")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- one brief composes two distinct recipes back to back ------------- #
    r = s.build("parametric plate\nbolted stack with 2 spacers")
    assert r.ok, r.error
    steps = [st for e in r.data["transcript"] for st in e["steps"]]
    rec_steps = [st for st in steps if st["tool"] == "recipe"]
    assert len(rec_steps) == 2, steps
    for st in rec_steps:
        assert st["ok"] and st["executed"] == st["planned"] and st["executed"] > 0, st
    names = {st["args"]["name"] for st in rec_steps}
    assert names == {"parametric_plate", "bolted_stack"}, names

    # ---- the PartDesign plate survived intact (its own closed-form meta) --- #
    plate_meta = recipes.generate("parametric_plate").meta
    mp = s.act("solid.measure", {"name": plate_meta["part"]})
    assert mp.ok and mp.data["valid"], mp
    assert _approx(mp.data["volume"], plate_meta["volume"], tol=2.0), \
        (mp.data["volume"], plate_meta["volume"])

    # ---- the bolted stack survived intact, in the SAME document ----------- #
    bom = s.act("asm.bom", {})
    assert bom.ok, bom.error
    assert bom.data["component_count"] == 5, bom.data       # 2 spacers + base/bolt/nut
    assert bom.data["line_items"]["spacer"]["count"] == 2, bom.data
    # the plate body is a PartDesign part, NOT an assembly component -- the two
    # recipes did not bleed into one another.
    assert plate_meta["part"] not in bom.data["line_items"], bom.data

    clash = s.act("asm.interference", {})
    assert clash.ok and clash.data["clash_count"] == 0, clash.data

    print("compose capstone: parametric_plate (vol %.2f) + bolted_stack(n=2, "
          "%d components) coexist in one document, namespaces independent" % (
              mp.data["volume"], bom.data["component_count"]))
    print("COMPOSE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
