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

    # ---- orchestration policy: when to FETCH vs when to MODEL --------------- #
    # The planner decides search-before-model on its own (pure text, no kernel,
    # no network here -- we only assert the routing & distilled query/platform).
    from cad_agent.planner import Planner  # noqa: E402

    def _route(text):
        p = Planner().plan(text)
        assert p.steps, p.error
        return p.steps[0]["tool"], p.steps[0]["args"]

    tool, args = _route("find a gear bracket online")
    assert tool == "resource.search" and args["query"] == "gear bracket", (tool, args)

    tool, args = _route("search printables for a raspberry pi case")
    assert tool == "resource.search", (tool, args)
    assert args["query"] == "raspberry pi case", args
    assert args.get("platforms") == ["printables"], args

    tool, args = _route("我需要一个现成的齿轮模型")
    assert tool == "resource.search" and args["query"] == "齿轮", (tool, args)

    tool, args = _route("download printables id 12345")
    assert tool == "resource.download", (tool, args)
    assert args == {"platform": "printables", "id": "12345"}, args

    # ambiguous verbs must NOT hijack a measure/model request into a web search.
    tool, _ = _route("box 20x10x5")
    assert tool == "solid.box", tool
    assert Planner().plan("find the volume of it").error, "‘find volume’ mis-routed"
    print("orchestration policy: fetch-vs-model routing holds (EN+中文, no false fetch)")

    print("PLANNER SMOKE OK", s.summary())
    s.registry.kernel.shutdown()
    s2.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_planner"):
    main()
