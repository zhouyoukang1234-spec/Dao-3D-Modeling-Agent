"""STEP round-trip reverse-engineering smoke -- the real shape a download arrives in.

Downloaded models almost never come as live FreeCAD bodies; they come as a STEP
assembly. So the honest reverse test is: build the slider-crank, *write it to
STEP*, then in a clean session import that file and recover the mechanism from
nothing but the imported geometry.

This surfaced a real defect: a STEP assembly imports as an ``App::Part``
container *plus* its leaf parts, and the container carries a compound Shape of
every child. Registering it as a solid invented a phantom 5th "part" overlapping
all the others, which fabricated spurious joints (10 instead of 4). The fix
skips objects that group other objects. This test locks that down: the import
must register exactly the four leaf parts, and the recovered joints + Kutzbach
mobility must match the in-memory mechanism (3 revolutes + 1 prismatic, M=1).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402
from tests.smoke_mechanism import R, XB, build_slidercrank  # noqa: E402

STEP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "slidercrank_rt.step")


def main():
    # ---- author the assembly, then write it out as a STEP "download" -------- #
    s = new_session("step_author")
    print("FreeCAD", s.registry.kernel.freecad_version)
    parts = build_slidercrank(s)
    os.makedirs(os.path.dirname(STEP), exist_ok=True)
    e = s.act("solid.export", {"names": parts, "path": STEP, "format": "step"})
    assert e.ok and e.data["bytes"] > 0, e.error
    s.registry.kernel.shutdown()

    # ---- clean session: import the STEP exactly like a downloaded model ----- #
    s2 = new_session("step_reverse")
    imp = s2.act("solid.import_step", {"path": STEP})
    assert imp.ok, imp.error
    # the assembly container (App::Part, a compound of all children) must NOT
    # leak in as a phantom part -- exactly the four leaf solids, nothing else.
    got = sorted(imp.data["imported"])
    assert got == sorted(parts), ("phantom/missing part after STEP import", got)
    assert sorted(s2.act("solid.list", {}).data["solids"]) == sorted(parts)

    # ---- recover the mechanism from the imported geometry ------------------- #
    j = s2.act("solid.joints", {"parts": parts})
    assert j.ok, j.error
    jl = j.data["joint_list"]
    rev = [x for x in jl if x["type"] == "revolute"]
    pris = [x for x in jl if x["type"] == "prismatic"]
    assert len(rev) == 3 and len(pris) == 1, ("STEP round-trip changed the joints", jl)
    pts = sorted([tuple(round(c, 3) for c in r["axis_point"][:2]) for r in rev])
    want = sorted([(0.0, 0.0), (0.0, R), (round(XB, 3), 0.0)])
    assert pts == want, (pts, want)
    fax = pris[0]["axis_dir"]
    assert abs(abs(fax[0]) - 1.0) < 1e-6 and abs(fax[1]) < 1e-6, fax
    print("imported %d leaf parts (no container phantom); joints survive STEP round trip" % len(parts))

    m = s2.act("solid.mechanism", {"parts": parts})
    assert m.ok, m.error
    assert m.data["joint_types"] == {"revolute": 3, "prismatic": 1}, m.data
    assert m.data["mobility_planar"] == 1, m.data
    assert all(len(v) == 2 for v in m.data["graph"].values()), m.data["graph"]
    print("mechanism recovered from STEP: closed 4-link loop, Kutzbach mobility = 1")

    # ---- and it still drives the exact piston law -------------------------- #
    L = math.hypot(XB, R)
    d = s2.act("solid.drive", {"ground_point": [0, 0], "guide_point": [0, 0],
                               "guide_dir": [1, 0], "crank_len": R, "rod_len": L, "angle": 90})
    assert d.ok and d.data["rod_len_ok"], d.error or d.data
    assert abs(d.data["B"][0] - XB) < 1e-3, d.data
    print("STEP-REVERSE SMOKE OK", s2.summary())
    s2.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_step_reverse"):
    main()
