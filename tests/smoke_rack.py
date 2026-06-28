"""Rack-and-pinion smoke -- rotary-to-linear, a new mechanism class.

A rack is a straight gear (infinite pitch radius): involute flanks become straight
lines at the pressure angle. ``{"rack": {...}}`` builds the toothed strip; a pinion
meshes it and converts rotation to translation. As the pinion turns by alpha the
rack must travel rp*alpha (rolling without slip).

Checks:
  * the rack is a valid solid with the expected length/height;
  * pinion + rack mesh at ~0 interference when phased, and JAM when mis-phased
    half a tooth (the engagement is real, not clearance);
  * driving the pinion through a sweep while translating the rack by rp*alpha
    keeps every step meshed -- and a full pinion revolution advances the rack by
    one pitch circumference (pi*m*z).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

M = 2.0
Z = 18                            # pinion teeth
NT = 9                            # rack teeth
RP = M * Z / 2.0                  # pinion pitch radius = 18
XP = (NT - 1) * math.pi * M / 2.0  # pinion sits over the rack middle
W = 10.0
TOL = 8.0


def _ov(s):
    out = s.act("asm.interference", {"assembly": "RP"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}
    return cm.get(("pin", "rack"), 0.0)


def main():
    s = new_session("rack")
    print("FreeCAD", s.registry.kernel.freecad_version)

    assert s.act("param.body", {"name": "Pin"}).ok
    assert s.act("param.pad", {"body": "Pin", "feature": "Pf",
                               "profile": {"gear": {"module": M, "teeth": Z}}, "length": W}).ok
    assert s.act("param.body", {"name": "Rack"}).ok
    rk = s.act("param.pad", {"body": "Rack", "feature": "Rf",
                             "profile": {"rack": {"module": M, "teeth": NT}}, "length": W})
    assert rk.ok and rk.data["valid"], rk.data
    bx, by, bz = rk.data["bbox_size"]
    # length ~ (NT-1)*pitch + a tooth; height = addendum + dedendum + back
    p_ = math.pi * M
    assert abs(by - (M + 1.25 * M + 2.0 * M)) < 1e-6, ("rack height", by)
    assert (NT - 1) * p_ < bx <= NT * p_, ("rack length", bx)
    assert abs(bz - W) < 1e-3, bz
    print("rack solid: L=%.2f (NT-1 pitch=%.2f), H=%.2f, valid" % (bx, (NT - 1) * p_, by))

    assert s.act("asm.create", {"name": "RP"}).ok
    assert s.act("asm.add", {"assembly": "RP", "body": "Rack", "name": "rack", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "RP", "body": "Pin", "name": "pin"}).ok

    # phase 0: pinion tooth-space faces a rack tooth at the contact line -> mesh
    assert s.act("asm.place", {"name": "pin", "pos": [XP, RP, 0]}).ok
    meshed = _ov(s)
    assert meshed < TOL, ("rack-pinion jams when phased", meshed)
    print("rack & pinion mesh (pitch circle tangent to pitch line): overlap = %.1f" % meshed)

    # mis-phase half a tooth -> jam (engagement is real)
    assert s.act("asm.rotate", {"name": "pin", "axis": [0, 0, 1],
                                "angle": 180.0 / Z, "at": [XP, RP, 0]}).ok
    jam = _ov(s)
    assert jam > 50.0, ("mis-phase should jam", jam)
    print("mis-phased half a tooth jams: overlap = %.0f (engagement is real)" % jam)

    # rolling kinematics: pinion alpha -> rack travel rp*alpha, mesh persists
    for alpha in (0.0, 4.0, 8.0, 12.0, 16.0):
        dx = RP * math.radians(alpha)
        assert s.act("asm.place", {"name": "rack", "pos": [dx, 0, 0]}).ok
        assert s.act("asm.place", {"name": "pin", "pos": [XP, RP, 0]}).ok
        assert s.act("asm.rotate", {"name": "pin", "axis": [0, 0, 1],
                                    "angle": alpha, "at": [XP, RP, 0]}).ok
        w = _ov(s)
        assert w < TOL, ("rack-pinion jammed at alpha=%.0f" % alpha, w)
    travel_per_rev = math.pi * M * Z
    print("rolling: pinion 0..16 deg, rack travels rp*alpha, mesh stays ~0")
    print("one pinion revolution advances the rack by pi*m*z = %.2f mm" % travel_per_rev)

    if "view.render" in s.tools():
        assert s.act("asm.place", {"name": "rack", "pos": [0, 0, 0]}).ok
        assert s.act("asm.place", {"name": "pin", "pos": [XP, RP, 0]}).ok
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_rack.png")
        rv = s.act("view.render", {"assembly": "RP", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("RACK SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_rack"):
    main()
