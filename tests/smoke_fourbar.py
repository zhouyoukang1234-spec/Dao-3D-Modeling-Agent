"""Four-bar linkage smoke -- a more complex planar mechanism than slider-crank.

The four-bar is the canonical 1-DOF planar linkage. We drive a *crank-rocker*
(Grashof) through a full input revolution and assert closed-form loop closure at
every step: the coupler pin B must stay exactly the coupler length from the crank
pin A and exactly the rocker length from the fixed pivot O4 (circle-circle
intersection), and the ground span O2->O4 must equal the ground length. We also
check the Grashof classification, and that a *non-Grashof* link set is reported
as a double-rocker whose input crank cannot complete a full turn (the linkage
honestly refuses to assemble at the unreachable angles instead of faking a pose).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("fourbar")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- crank-rocker: shortest link is the crank, Grashof satisfied -------- #
    spec = {"ground": 4.0, "crank": 1.0, "coupler": 3.0, "rocker": 3.5,
            "ground_point": [0, 0], "ground_dir": [1, 0]}
    ok = 0
    for ang in range(0, 360, 10):
        r = s.act("solid.fourbar", dict(spec, angle=ang))
        assert r.ok, (ang, r.error)
        d = r.data
        # independent loop-closure checks
        ax, ay = d["A"]
        bx, by = d["B"]
        o2 = d["O2"]
        o4 = d["O4"]
        # recomputed from rounded (4-dp) output coords -> ~1e-4 slack; the op's
        # own coupler_ok/rocker_ok below assert exactness on the raw values.
        assert abs(math.hypot(ax - o2[0], ay - o2[1]) - spec["crank"]) < 1e-3, ("crank", ang, d)
        assert abs(math.hypot(bx - ax, by - ay) - spec["coupler"]) < 1e-3, ("coupler", ang, d)
        assert abs(math.hypot(bx - o4[0], by - o4[1]) - spec["rocker"]) < 1e-3, ("rocker", ang, d)
        assert abs(math.hypot(o4[0] - o2[0], o4[1] - o2[1]) - spec["ground"]) < 1e-3, ("ground", ang, d)
        assert d["coupler_ok"] and d["rocker_ok"], d
        assert d["grashof"] and d["grashof_type"] == "crank-rocker", d
        ok += 1
    assert ok == 36, ok
    print("crank-rocker closed loop at all %d crank angles (|AB|, |BO4|, ground all exact)" % ok)

    # crossed circuit is a distinct valid assembly (different B, same lengths)
    op = s.act("solid.fourbar", dict(spec, angle=70)).data
    cr = s.act("solid.fourbar", dict(spec, angle=70, branch="crossed")).data
    assert op["B"] != cr["B"] and cr["coupler_ok"] and cr["rocker_ok"], (op, cr)
    print("open vs crossed circuits both close: B=%s vs %s" % (op["B"], cr["B"]))

    # --- non-Grashof: no link fully rotates -> double-rocker ---------------- #
    ng = {"ground": 2.0, "crank": 3.0, "coupler": 2.0, "rocker": 2.5,
          "ground_point": [0, 0], "ground_dir": [1, 0]}
    fails = 0
    seen_type = None
    for ang in range(0, 360, 10):
        r = s.act("solid.fourbar", dict(ng, angle=ang))
        if not r.ok:
            fails += 1
            assert "cannot assemble" in (r.error or ""), r.error
        else:
            seen_type = r.data["grashof_type"]
            assert r.data["grashof"] is False, r.data
    assert seen_type == "double-rocker", seen_type
    assert fails > 0, "non-Grashof crank should fail to assemble at some angles"
    print("non-Grashof set classified double-rocker; input crank blocked at %d/36 angles (honest refusal)" % fails)

    print("FOURBAR SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_fourbar"):
    main()
