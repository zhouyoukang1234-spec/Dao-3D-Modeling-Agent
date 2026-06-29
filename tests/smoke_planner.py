"""自动编排 (planner -> 真实内核) smoke.

The standalone :class:`Planner` turns text into ``{tool,args}`` steps but, in
this package, nothing ran those steps against the live kernel. ``Session.build``
closes that loop: a multi-intent script (``;``/newline separated) is planned by
one stateful planner and executed as a single fused pipeline, threading object
names across lines. Here we prove a real PartDesign-free pipeline
(box -> cylinder -> move -> cut -> pattern) actually changes geometry on the
FreeCAD kernel, and that bad input degrades gracefully (no raw leaks).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

_RAW = ("TypeError", "AttributeError", "could not convert", "has no attribute",
        "KeyError", "OCCError", "Standard_", "NullShape", "NoneType")


def main():
    s = new_session("planner")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # a multi-step natural-language pipeline, executed on the live kernel.
    script = ("box length 40 width 30 height 10 name plate; "
              "cylinder r=4 h=10 name hole; "
              "move hole by 10 8 0; "
              "cut hole from plate")
    r = s.build(script)
    assert r.ok, r.error
    assert r.data["lines"] == 4, r.data
    assert r.data["executed"] == 4 and r.data["failed"] == 0, r.data
    # every recorded step error (if any) must be guided, never a raw leak.
    for e in r.data["transcript"]:
        for st in e["steps"]:
            err = st.get("error") or ""
            assert not any(x in err for x in _RAW), "raw leak in plan step: %r" % err

    # the pipeline fused: 40*30*10 minus a r=4 h=10 hole (~502.7).
    m = s.act("solid.measure", {"name": "plate"})
    assert m.ok, m.error
    vol = m.data["volume"]
    assert abs(vol - (12000 - 502.65)) < 5.0, vol
    print("planner pipeline fused on kernel -> plate volume %.1f (hole cut)" % vol)

    # 'it' back-reference threads across lines (last created object).
    s2 = new_session("planner2")
    r2 = s2.build("cylinder r=5 h=20 name post; fillet it radius 1")
    assert r2.ok, r2.error
    steps = [st for e in r2.data["transcript"] for st in e["steps"]]
    assert any(st["tool"] == "solid.fillet" and st.get("args", {}).get("name") == "post"
               for st in steps), steps
    print("'it' resolved to last object: %s"
          % [st.get("args", {}).get("name") for st in steps if st["tool"] == "solid.fillet"])

    # graceful degradation: empty refused (guided), garbage recorded not raised.
    assert not s.build("").ok
    assert not s.build("   ").ok
    g = s.build("xyzzy frobnicate the widget")
    assert g.ok, g.error  # the call succeeds; the unparsable line is reported
    assert g.data["transcript"][0].get("error"), g.data
    # an unknown/control tool in a raw plan is skipped, not raised.
    sk = s.build('[{"tool": "__reset__", "args": {}}, '
                 '{"tool": "no.such_tool", "args": {}}]')
    assert sk.ok, sk.error
    assert all(st.get("skipped") for e in sk.data["transcript"] for st in e["steps"]), sk.data
    print("planner build: empty refused, garbage + unknown tools degrade gracefully")

    print("PLANNER SMOKE OK", s.summary())
    s.registry.kernel.shutdown()
    s2.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_planner"):
    main()
