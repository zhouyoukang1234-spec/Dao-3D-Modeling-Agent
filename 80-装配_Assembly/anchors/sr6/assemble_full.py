# -*- coding: utf-8 -*-
"""SR6 full 6-leg home assembly, solved by the GENERAL kernel `uam.assembly`.

This is the keystone instance: it does NOT hand-roll any leg math.  It declares
the mechanism as a mate graph -- parts carrying connectors, plus constraints
between them -- and hands the whole thing to `uam.assembly.solve`, the same
mechanism-agnostic Gauss-Newton pose solver that knows nothing about the SR6.
If the general engine drives every constraint residual to zero, then the general
L2/L3 layer (the layer whose ABSENCE caused every prior failure -- see
docs/ROOT_CAUSE) actually assembles the real machine, not just a toy.

Authority chain, every number external to the printed parts (no hand-tuning):
  * Receiver home pose  : L1/2 datum -- Kabsch fit of perceived pivots onto the
        firmware-authoritative world pivots (closure_kabsch.py, RMS 0.014mm).
  * Servo shaft places  : firmware IK home geometry (shaft 162.48 below + horiz
        offset from each receiver pivot, z=46).  Servos are COTS -> never sensed
        from printed-part holes (principle P2).
  * Arm / rod lengths   : perceived bearing-centre spans (Arm 50, MainLink 175,
        Pitcher arm 75, PitcherLink 185).

Mate graph (per leg):  ground.shaft --(Distance = arm)--> rod.s ;
                        rod.r --(Coincident)--> receiver.pivot ;
                        rod is a rigid body whose two connectors are `link` apart.
The only physical freedom left is each leg's swing about its shaft<->pivot line
(spherical rod-end bearings) -- a REAL DOF, not a fudge factor.  Closure RMS = 0
means a real rod of the perceived length physically spans shaft-arm to pivot.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from uam.assembly import Coincident, Distance, Part, PointAt, qnorm, solve  # noqa: E402

# ---- authority-backed world geometry (see module docstring) ----------------
HOME_H, SHAFT_Z = 208.48, 46.0
MAIN_ARM, MAIN_LINK = 50.0, 175.0
PITCH_ARM, PITCH_LINK = 75.0, 185.0

MAIN_PIV = {"L": np.array([-59.5, 0.0, HOME_H]), "R": np.array([59.5, 0.0, HOME_H])}
PITCH_PIV = {"L": np.array([-61.0, 53.13, 222.71]), "R": np.array([61.0, 53.13, 222.71])}

# leg = (name, shaft_world, pivot_world, arm_len, link_len)
LEGS = []
for side, sx in (("L", -59.5), ("R", 59.5)):
    for ysign, nm in ((+1, "lower"), (-1, "upper")):
        LEGS.append((f"main-{side}-{nm}", np.array([sx, ysign * 15.0, SHAFT_Z]),
                     MAIN_PIV[side], MAIN_ARM, MAIN_LINK))
for side, sx in (("L", -59.5), ("R", 59.5)):
    LEGS.append((f"pitch-{side}", np.array([sx, 61.25, SHAFT_Z]),
                 PITCH_PIV[side], PITCH_ARM, PITCH_LINK))


def quat_x_to(direction):
    """Unit quaternion (x,y,z,w) rotating local +x onto `direction`."""
    d = np.asarray(direction, float)
    d = d / (np.linalg.norm(d) + 1e-12)
    x = np.array([1.0, 0.0, 0.0])
    c = float(np.dot(x, d))
    if c > 1 - 1e-9:
        return np.array([0, 0, 0, 1.0])
    if c < -1 + 1e-9:
        return np.array([0, 0, 1.0, 0])  # 180 deg about z
    axis = np.cross(x, d)
    axis /= np.linalg.norm(axis)
    half = np.arccos(c) / 2
    return qnorm(np.array([*(axis * np.sin(half)), np.cos(half)]))


def build():
    """Declare the mate graph and return (parts, constraints, rods-by-name)."""
    ground = Part("ground", fixed=True)
    for name, shaft, _piv, _arm, _link in LEGS:
        ground.add(f"shaft::{name}", shaft, [1, 0, 0])
    parts, constraints, rods = [ground], [], {}
    for name, shaft, piv, arm, link in LEGS:
        rod = Part(f"rod::{name}")
        rod.add("s", [0.0, 0.0, 0.0], [1, 0, 0])     # arm-end (near servo)
        rod.add("r", [link, 0.0, 0.0], [1, 0, 0])    # receiver-end, `link` away
        # home seed: arm horizontal outward in Y-Z plane, then rod spans to pivot
        tip0 = shaft + arm * np.array([0.0, np.sign(shaft[1] - piv[1]) or 1.0, 0.0])
        rod.t = tip0
        rod.q = quat_x_to(piv - tip0)
        parts.append(rod)
        rods[name] = rod
        # the mate graph: arm length, and rod receiver-end on the pivot
        constraints.append(Distance((rod, "s"), (ground, f"shaft::{name}"), arm))
        constraints.append(PointAt((rod, "r"), piv))
    return parts, constraints, rods


def run():
    parts, constraints, rods = build()
    print("=== SR6 full 6-leg HOME assembly via the GENERAL uam.assembly kernel ===")
    print(f"    receiver level @ z={HOME_H}; shafts z={SHAFT_Z}; "
          f"links main={MAIN_LINK} pitch={PITCH_LINK}\n")
    res, rms = solve(parts, constraints, verbose=False)
    per = []
    for name, shaft, piv, arm, link in LEGS:
        rod = rods[name]
        s_w = rod.world_point("s")
        r_w = rod.world_point("r")
        arm_err = abs(np.linalg.norm(s_w - shaft) - arm)
        piv_err = float(np.linalg.norm(r_w - piv))
        rod_len = float(np.linalg.norm(r_w - s_w))
        tilt = np.degrees(np.arctan2(s_w[2] - shaft[2], abs(s_w[1] - shaft[1])))
        leg_res = max(arm_err, piv_err)
        per.append(leg_res)
        flag = "CLOSES" if leg_res < 1e-4 else "OPEN"
        print(f"  {name:13s} arm_tip=({s_w[0]:6.1f},{s_w[1]:6.1f},{s_w[2]:5.1f}) "
              f"rod_len={rod_len:6.2f} arm_err={arm_err:.2e} piv_err={piv_err:.2e} "
              f"tilt={tilt:+5.1f}  [{flag}]")
    leg_rms = float(np.sqrt(np.mean(np.square(per))))
    print(f"\n  general-solver constraint RMS = {rms:.3e} mm")
    print(f"  per-leg worst closure residual RMS = {leg_rms:.3e} mm")
    print("  => the mechanism-agnostic mate solver assembles the real 6-leg SR6")
    print("     at home from declared relationships alone (no hand-coded poses).")
    assert leg_rms < 1e-4, "general kernel failed to close the 6-leg assembly"
    return leg_rms


if __name__ == "__main__":
    run()
