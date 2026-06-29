"""FEM false-pass guard + a full-stack bracket pipeline.

Two things on one realistic part (an aluminium mounting plate, 80x40x8, with two
bolt holes cut near the fixed end):

  1. Full-stack pipeline — model (box + boolean holes) -> ``solid.measure`` ->
     ``solid.section`` (the second moment of area away from the holes matches
     b*h^3/12 exactly) -> ``solid.dfm_report`` (the flat plate prints support-
     free) -> a real CalculiX static solve giving a finite stress and a sensible
     safety factor. This is geometry, DFM and physics driven end to end.

  2. The guard — a 2nd-order mesh on this holed plate trips CalculiX's
     nonpositive-Jacobian failure: the solver writes a mesh but *no* nodal field.
     The old code read that back as max_vm = 0 / safety = inf — a silent, and
     dangerous, false "pass". ``fem.solve`` must now refuse it: the solve either
     raises, or returns a genuinely non-empty field. It must never report a
     pass off an empty result.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bracket(s):
    s.act("solid.box", {"name": "plate", "length": 80, "width": 40, "height": 8})
    for i, (x, y) in enumerate([(10, 10), (10, 30)]):
        s.act("solid.cylinder", {"name": "h%d" % i, "radius": 4, "height": 20})
        s.act("solid.translate", {"name": "h%d" % i, "vector": [x, y, -5]})
        assert s.act("solid.cut", {"a": "plate", "b": "h%d" % i, "out": "plate"}).ok


def main():
    print("FEM guard + full-stack bracket pipeline")

    # ---- 1. full-stack pipeline on a clean-solving mesh --------------------- #
    s = new_session("brkt_ok")
    _bracket(s)
    m = s.act("solid.measure", {"name": "plate"}).data
    assert m["volume"] > 0, m

    sec = s.act("solid.section", {"name": "plate", "normal": [1, 0, 0], "d": 50}).data
    print("  section@x=50 area=%.1f Ix=%.3f (exp %.3f)"
          % (sec["area"], sec["Ix"], 40 * 8 ** 3 / 12))
    assert abs(sec["area"] - 40 * 8) < 1e-6, sec
    assert abs(sec["Ix"] - 40 * 8 ** 3 / 12) <= 1e-3 * (40 * 8 ** 3 / 12), sec

    dfm = s.act("solid.dfm_report", {"name": "plate", "process": "print"}).data
    assert dfm["manufacturable"] and dfm["checks"]["overhang"]["pass"], dfm

    assert s.act("fem.setup", {"target": "plate", "material": "aluminum",
                               "order": 1, "mesh_size": 4}).ok
    assert s.act("fem.fix", {"select": {"axis": "x", "side": "min"}}).ok
    assert s.act("fem.load", {"select": {"axis": "x", "side": "max"},
                              "kind": "force", "value": 200, "direction": [0, 0, -1]}).ok
    sol = s.act("fem.solve", {}).data
    print("  FEM maxVM=%.2f MPa safety=%.2f disp=%.4f mm nodes=%d"
          % (sol["max_von_mises_mpa"], sol["safety_factor"],
             sol["max_disp_mm"], sol["result_nodes"]))
    assert sol["max_von_mises_mpa"] > 1.0 and sol["result_nodes"] > 0, sol
    assert sol["safety_factor"] < float("inf") and sol["passed"], sol

    # malformed face selectors / load values used to leak a bare
    # 'str has no attribute lower', V(*seq) 'Expected sequence of size 3',
    # a KeyError on a bad axis letter, or 'could not convert' from float().
    def _bad(r, *needles):
        err = r.error or ""
        assert not r.ok, ("expected failure", r.data)
        for raw in ("could not convert", "TypeError", "AttributeError",
                    "KeyError", "invalid literal", "Expected sequence",
                    "not enough values", "string indices", "has no attribute"):
            assert raw not in err, (raw, err)
        for nd in needles:
            assert nd in err, (nd, err)

    _bad(s.act("fem.fix", {"select": "x"}), "select")
    _bad(s.act("fem.fix", {"select": {"axis": "w"}}), "axis")
    _bad(s.act("fem.fix", {"select": {"index": [0]}}), "out of range")
    _bad(s.act("fem.fix", {"select": {"index": 1.5}}), "whole face")
    _bad(s.act("fem.fix", {"select": {"normal": [0, 0, "x"]}}), "normal")
    _bad(s.act("fem.support", {"select": {"axis": "x", "side": "min"},
                               "fix": "q"}), "x'/'y'/'z")
    _bad(s.act("fem.load", {"select": {"axis": "x", "side": "max"},
                            "value": "x"}), "value")
    _bad(s.act("fem.load", {"select": {"axis": "x", "side": "max"},
                            "value": 10, "direction": "x"}), "direction")
    _bad(s.act("fem.temperature", {"select": {"axis": "x", "side": "min"},
                                   "value": "x"}), "value")
    _bad(s.act("fem.spin", {"hz": 10, "axis": "x"}), "axis")
    _bad(s.act("fem.buckle", {"modes": "x"}), "modes")
    print("  malformed fem.* selectors/values refused cleanly")

    # the document now holds FEM analysis / mesh / result objects whose 'Shape'
    # attribute is not a TopoShape; view.scene used to leak a raw
    # "'Part.Feature' object has no attribute 'isNull'" walking doc.Objects.
    vs = s.act("view.scene", {})
    assert vs.ok, ("view.scene leaked over FEM objects", vs.error)
    assert "isNull" not in (vs.error or ""), vs.error
    print("  view.scene tolerates non-TopoShape FEM objects")
    s.registry.kernel.shutdown()

    # ---- 2. the false-pass guard ------------------------------------------- #
    g = new_session("brkt_guard")
    _bracket(g)
    g.act("fem.setup", {"target": "plate", "material": "aluminum", "order": 2})
    g.act("fem.fix", {"select": {"axis": "x", "side": "min"}})
    g.act("fem.load", {"select": {"axis": "x", "side": "max"},
                       "kind": "force", "value": 200, "direction": [0, 0, -1]})
    bad = g.act("fem.solve", {})
    print("  guard: solve ok=%s err=%s" % (bad.ok, (bad.error or "")[:48]))
    # never a silent pass off an empty field: either it failed loudly, or it
    # really did produce a field. The old bug was ok=True with result_nodes==0.
    assert (not bad.ok) or bad.data["result_nodes"] > 0, bad.data
    g.registry.kernel.shutdown()

    # ---- 3. out-of-order use guides, never leaks a raw RuntimeError -------- #
    #     fem.fix/load/modal before fem.setup used to surface as
    #     "RuntimeError: call fem.setup first"; the type name must not leak.
    q = new_session("fem_seq")
    for op in ("fem.fix", "fem.load", "fem.modal"):
        r = q.act(op, {"select": {"axis": "x", "side": "min"}})
        err = r.error or ""
        assert not r.ok, (op, r.data)
        assert "RuntimeError" not in err, (op, err)
        assert "fem.setup first" in err, (op, err)
    q.registry.kernel.shutdown()
    print("  out-of-order fem.* guided cleanly (no raw RuntimeError)")

    # ---- 4. solve with no boundary conditions guides, never leaks ---------- #
    #     fem.solve / fem.thermal after setup but with no fix/load used to
    #     surface CalculiX's "RuntimeError: FEM prerequisites not met: ...".
    p = new_session("fem_noc")
    p.act("solid.box", {"name": "bar", "length": 60, "width": 10, "height": 10})
    assert p.act("fem.setup", {"target": "bar", "material": "steel",
                               "order": 1, "mesh_size": 5}).ok
    nc = p.act("fem.solve", {})
    err = nc.error or ""
    assert not nc.ok, nc.data
    assert "RuntimeError" not in err, err
    assert "fem.fix" in err or "boundary" in err, err
    print("  fem.solve w/o constraints guided cleanly (no raw RuntimeError)")
    p.registry.kernel.shutdown()

    print("FEM GUARD SMOKE OK")


if __name__ in ("__main__", "smoke_fem_guard"):
    main()
