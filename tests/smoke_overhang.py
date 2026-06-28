"""Overhang smoke — additive-manufacturing support DFM vs a build axis.

The additive counterpart of the mould trio (draft/thickness/undercut): a
down-facing surface prints support-free only if it is steeper than the printer's
overhang limit (typ. 45 deg from horizontal). ``solid.overhang`` flags the faces
that need support. Validated against closed-form geometry:

  * a box grown +Z -> 4 vertical walls (fine), the base rests on the plate and
    the top faces up -> 0 overhangs, printable;
  * a tall inverted cone (apex down, H=30 R=10) -> slant beta = atan(H/R) =
    71.6 deg >= 45 -> printable;
  * a squat inverted cone (H=10 R=30) -> slant beta = atan(H/R) = 18.4 deg
    < 45 -> 1 overhang face, not printable (turning the part over or adding a
    chamfer is the fix);
  * a stepped bracket (boolean union -> a compound) with a flat down-facing ledge
    above the plate -> beta = 0 -> an overhang, exercising a compound;
  * a sphere -> its whole lower hemisphere overhangs; this is the case a single
    centre-normal sample misses (the sphere is one face), so it pins the
    per-face grid sampling that makes curved overhangs detectable.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("overhang")
    print("FreeCAD", s.registry.kernel.freecad_version)

    s.act("solid.box", {"name": "b", "length": 20, "width": 20, "height": 10})
    b = s.act("solid.overhang", {"name": "b", "build": [0, 0, 1]}).data
    print("box  -> overhangs=%d printable=%s walls=%d plate=%d"
          % (b["overhangs"], b["printable"], b["vertical_walls"], b["plate_faces"]))
    assert b["overhangs"] == 0 and b["printable"], b
    assert b["vertical_walls"] == 4 and b["plate_faces"] == 1, b

    # tall inverted cone: apex down (r1=0 at bottom), base up -> slant is steep.
    s.act("solid.cone", {"name": "tall", "radius1": 0, "radius2": 10, "height": 30})
    t = s.act("solid.overhang", {"name": "tall", "build": [0, 0, 1]}).data
    print("tall -> overhangs=%d printable=%s" % (t["overhangs"], t["printable"]))
    assert t["printable"] and t["overhangs"] == 0, t

    # squat inverted cone: shallow slant -> needs support; angle = atan(10/30).
    s.act("solid.cone", {"name": "squat", "radius1": 0, "radius2": 30, "height": 10})
    q = s.act("solid.overhang", {"name": "squat", "build": [0, 0, 1]}).data
    print("squat-> overhangs=%d angle=%s" % (q["overhangs"], q["overhang_faces"]))
    assert q["overhangs"] == 1 and not q["printable"], q
    exp = math.degrees(math.atan(10.0 / 30.0))
    assert abs(q["overhang_faces"][0]["angle_deg"] - exp) <= 0.05, (q, exp)

    # stepped bracket: a tall pillar + an offset slab on top -> the slab underside
    # overhangs the gap with a flat (beta=0) down face above the plate.
    s.act("solid.box", {"name": "post", "length": 10, "width": 20, "height": 20})
    s.act("solid.box", {"name": "roof", "length": 40, "width": 20, "height": 6})
    s.act("solid.translate", {"name": "roof", "vector": [0, 0, 20]})
    assert s.act("solid.union", {"a": "post", "b": "roof", "out": "brkt"}).ok
    g = s.act("solid.overhang", {"name": "brkt", "build": [0, 0, 1]}).data
    print("brkt -> overhangs=%d printable=%s faces=%s"
          % (g["overhangs"], g["printable"], [o["angle_deg"] for o in g["overhang_faces"]]))
    assert g["overhangs"] >= 1 and not g["printable"], g
    assert any(o["angle_deg"] < 1.0 for o in g["overhang_faces"]), g

    # sphere: the entire lower hemisphere overhangs -> the one spherical face must
    # be flagged. A single centre normal would miss it; the grid sampling catches
    # the shallow strip near the bottom (worst beta -> 0).
    s.act("solid.sphere", {"name": "ball", "radius": 15})
    sp = s.act("solid.overhang", {"name": "ball", "build": [0, 0, 1], "samples": 7}).data
    print("ball -> overhangs=%d printable=%s worst=%s"
          % (sp["overhangs"], sp["printable"],
             [o["angle_deg"] for o in sp["overhang_faces"]]))
    assert sp["overhangs"] >= 1 and not sp["printable"], sp
    assert all(o["angle_deg"] < 45.0 for o in sp["overhang_faces"]), sp

    print("OVERHANG SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_overhang"):
    main()
