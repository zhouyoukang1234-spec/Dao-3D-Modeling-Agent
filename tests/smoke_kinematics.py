"""Planetary KINEMATICS smoke -- does the mesh persist as the train turns?

The static suites prove gears can be *assembled* meshed (phased to interleave, ~0
interference). This one proves the mechanism actually *rotates* as a coherent gear
train: drive it through a range of angles and assert every mesh stays ~0 (rolling
without jamming) at every step, with the gear motions tied by the standard ratios.

Two configurations:

1. Fixed-carrier (star): planets on fixed centres. Spin the sun by alpha; each
   planet must counter-spin by -(Zs/Zp)*alpha. Sun-planet mesh must stay ~0.

2. Full planetary (ring fixed, carrier output, Willis): theta_sun =
   (1+Zr/Zs)*theta_carrier; planets orbit with the carrier and spin by
   (1-Zr/Zp)*theta_carrier. BOTH the sun-planet and ring-planet meshes must stay
   ~0 across the sweep. Reduction ratio carrier:sun = 1:(1+Zr/Zs) = 1:3.5.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
Z_S, Z_P = 24, 18
Z_R = Z_S + 2 * Z_P                 # 60
A = M * (Z_S + Z_P) / 2.0          # 42
GW = 10.0
TOL = 8.0


def _worst(s, asm, pairs):
    out = s.act("asm.interference", {"assembly": asm})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}
    return max((cm.get(tuple(sorted(p)), 0.0) for p in pairs), default=0.0)


def main():
    s = new_session("kinematics")
    print("FreeCAD", s.registry.kernel.freecad_version)

    s.act("param.body", {"name": "Ring"})
    assert s.act("param.pad", {"body": "Ring", "feature": "Rf",
                               "profile": {"gear": {"module": M, "teeth": Z_R, "internal": True}},
                               "length": GW}).ok
    for nm, z in [("Sun", Z_S)] + [("Planet%d" % i, Z_P) for i in range(3)]:
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"gear": {"module": M, "teeth": z}},
                                   "length": GW}).ok

    phase0 = [meshing_phase_deg(120.0 * i, Z_S, Z_P) for i in range(3)]

    # ---- 1) fixed-carrier (star) rolling: no ring, planets on fixed centres ----
    assert s.act("asm.create", {"name": "Star"}).ok
    assert s.act("asm.add", {"assembly": "Star", "body": "Sun", "name": "sun"}).ok
    star_centres = []
    for i in range(3):
        assert s.act("asm.add", {"assembly": "Star", "body": "Planet%d" % i,
                                 "name": "planet%d" % i}).ok
        th = math.radians(120.0 * i)
        star_centres.append((A * math.cos(th), A * math.sin(th)))
    sp_pairs = [("sun", "planet%d" % i) for i in range(3)]

    roll = Z_S / float(Z_P)            # planet spin per unit sun spin
    for alpha in (0.0, 7.0, 15.0, 23.0, 30.0):
        assert s.act("asm.place", {"name": "sun", "pos": [0, 0, 0]}).ok
        if alpha:
            assert s.act("asm.rotate", {"name": "sun", "axis": [0, 0, 1],
                                        "angle": alpha, "at": [0, 0, 0]}).ok
        for i in range(3):
            x, y = star_centres[i]
            assert s.act("asm.place", {"name": "planet%d" % i, "pos": [x, y, 0]}).ok
            assert s.act("asm.rotate", {"name": "planet%d" % i, "axis": [0, 0, 1],
                                        "angle": phase0[i] - roll * alpha, "at": [x, y, 0]}).ok
        w = _worst(s, "Star", sp_pairs)
        assert w < TOL, ("star train jammed at sun=%.0f" % alpha, w)
    print("fixed-carrier: sun 0..30 deg, planets roll -(Zs/Zp)*a -> mesh stays ~0")

    # ---- 2) full planetary motion: ring fixed, carrier output -----------------
    assert s.act("asm.create", {"name": "Plan"}).ok
    assert s.act("asm.add", {"assembly": "Plan", "body": "Ring",
                             "name": "ring2", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "Plan", "body": "Sun", "name": "sun2"}).ok
    for i in range(3):
        assert s.act("asm.add", {"assembly": "Plan", "body": "Planet%d" % i,
                                 "name": "p%d" % i}).ok
    sun_ratio = 1.0 + Z_R / float(Z_S)        # 3.5
    spin_ratio = 1.0 - Z_R / float(Z_P)       # -7/3
    sr_pairs = ([("sun2", "p%d" % i) for i in range(3)]
                + [("ring2", "p%d" % i) for i in range(3)])
    for thc in (0.0, 10.0, 25.0, 40.0):
        assert s.act("asm.place", {"name": "sun2", "pos": [0, 0, 0]}).ok
        ts = sun_ratio * thc
        if ts:
            assert s.act("asm.rotate", {"name": "sun2", "axis": [0, 0, 1],
                                        "angle": ts, "at": [0, 0, 0]}).ok
        for i in range(3):
            beta = 120.0 * i + thc                # planet centre orbits with carrier
            th = math.radians(beta)
            x, y = A * math.cos(th), A * math.sin(th)
            assert s.act("asm.place", {"name": "p%d" % i, "pos": [x, y, 0]}).ok
            assert s.act("asm.rotate", {"name": "p%d" % i, "axis": [0, 0, 1],
                                        "angle": phase0[i] + spin_ratio * thc,
                                        "at": [x, y, 0]}).ok
        w = _worst(s, "Plan", sr_pairs)
        assert w < TOL, ("planetary jammed at carrier=%.0f" % thc, w)
    print("full planetary: carrier 0..40 deg, sun=%.2f*carrier, both meshes stay ~0"
          % sun_ratio)
    print("reduction ratio carrier:sun = 1 : %.2f" % sun_ratio)

    # evidence: render the carrier at 0 and 40 deg so the rotation is visible
    if "view.render" in s.tools():
        for thc, tag in ((0.0, "a"), (40.0, "b")):
            for i in range(3):
                beta = 120.0 * i + thc
                th = math.radians(beta)
                x, y = A * math.cos(th), A * math.sin(th)
                s.act("asm.place", {"name": "p%d" % i, "pos": [x, y, 0]})
                s.act("asm.rotate", {"name": "p%d" % i, "axis": [0, 0, 1],
                                     "angle": phase0[i] + spin_ratio * thc, "at": [x, y, 0]})
            s.act("asm.place", {"name": "sun2", "pos": [0, 0, 0]})
            if thc:
                s.act("asm.rotate", {"name": "sun2", "axis": [0, 0, 1],
                                     "angle": sun_ratio * thc, "at": [0, 0, 0]})
            o = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "_out", "smoke_kinematics_%s.png" % tag)
            rv = s.act("view.render", {"assembly": "Plan", "view": "top", "path": o})
            assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("rendered carrier@0 and carrier@40 frames")

    print("KINEMATICS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_kinematics"):
    main()
