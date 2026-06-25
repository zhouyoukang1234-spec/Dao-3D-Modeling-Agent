#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/closure_honest.py — Tier-2 closure, ZERO-FUDGE formulation.

WHY THIS FILE EXISTS (the architectural lesson of this session)
---------------------------------------------------------------
The prior session HARDCODED servo world positions (wrong by up to 53mm) and the
rods "never assembled".  In closure.py I then over-corrected into the SAME
disease wearing a different mask: I modelled each arm as a ball on a *horizontal*
circle (radius = arm length) with a FREE per-leg swing angle + a FREE shared
shaft height + a 6-DOF receiver pose held only by *soft* priors.  That is ~16
free parameters absorbing 6 rod-length equations.  Result: the 4-main case hit
"RMS = 0.0000" (meaningless — under-determined) and the 6-leg case produced a
spurious 46.8 deg receiver roll (the optimiser tilting the world to satisfy a
wrong model).

  ROOT CAUSE (general, not SR6-specific):
  Any degree of freedom that is NOT pinned by perception or by the prior will
  absorb error and MANUFACTURE false agreement.  Free parameters = self-
  deception.  A closure metric is only honest if the ONLY unknown is the single
  rigid-body pose we are genuinely solving for; every other quantity must be
  either PERCEIVED (from the mesh) or a PRIOR constant (from firmware).

THE HONEST TEST
---------------
The firmware SetMainServo / SetPitchServo is a zero-free-parameter forward
oracle.  At HOME every axis is neutral, which FIXES the 3-D distance from each
servo pivot to its receiver pivot (it is the planar c = |OP|, and the swing
plane contains both points, so c is a true 3-D distance — no plane orientation
needed):

    main  legs:  D_main  = |(16248,1500)|/100      = 163.171 mm
    pitch legs:  D_pitch = |firmware ball target|  = 176.902 mm

Anchors (PERCEIVED, shared assembly frame):  the 6 servo-mount footprint centres
on L/RFrame.stl.  Receiver pivots (PERCEIVED, receiver-local): the 4 X-axis
cylinder holes on Receiver.stl.  The ONLY unknown is the receiver 6-DOF pose.
Two left-main servos range to the SAME left axle; two right to the right axle;
each pitch servo to its own upper pivot.  6 distance equations, 6-DOF pose.

Whatever residual falls out is the truth.  An honest non-zero residual is worth
infinitely more than a fudged zero.
"""
from __future__ import annotations
import os, sys, math
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from uam.cylinders import detect_cylinders          # noqa: E402
from anchors.sr6.closure import perceive_servos, perceive_receiver_mounts  # noqa: E402

# ── firmware home distances (NO free parameters; derived in constants.py) ──
D_MAIN = math.hypot(162.48, 15.0)                    # 163.171 mm
_px = (16248 + 5500 * math.sin(0.2618)) / 100.0
_py = (4500 - 5500 * math.cos(0.2618)) / 100.0
D_PITCH = math.hypot(_px, _py)                       # 176.902 mm


def quat_to_R(q):
    q = np.asarray(q, float); q = q / (np.linalg.norm(q) + 1e-12)
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def rpy_deg(q):
    x, y, z, w = q / (np.linalg.norm(q) + 1e-12)
    return [math.degrees(a) for a in (
        math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        math.asin(max(-1, min(1, 2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]


def build_legs(servos, mains_local, pitch_local):
    """Return list of (servo_world_xyz, receiver_local_pivot, target_distance)."""
    Lm = sorted([v for k, v in servos.items() if k[0] == "L" and abs(k[1]) > 15],
                key=lambda p: p[1])
    Rm = sorted([v for k, v in servos.items() if k[0] == "R" and abs(k[1]) > 15],
                key=lambda p: p[1])
    Lp = [v for k, v in servos.items() if k[0] == "L" and abs(k[1]) <= 15][0]
    Rp = [v for k, v in servos.items() if k[0] == "R" and abs(k[1]) <= 15][0]
    la = np.array([m for m in mains_local if m[0] < 0][0])
    ra = np.array([m for m in mains_local if m[0] > 0][0])
    lp = np.array([p for p in pitch_local if p[0] < 0][0])
    rp = np.array([p for p in pitch_local if p[0] > 0][0])
    legs = []
    for s in Lm: legs.append((np.array(s, float), la, D_MAIN, "L-main"))
    for s in Rm: legs.append((np.array(s, float), ra, D_MAIN, "R-main"))
    legs.append((np.array(Lp, float), lp, D_PITCH, "L-pitch"))
    legs.append((np.array(Rp, float), rp, D_PITCH, "R-pitch"))
    return legs


def solve(servos, mains_local, pitch_local, verbose=True):
    legs = build_legs(servos, mains_local, pitch_local)

    def residual(x):
        t = x[0:3]; R = quat_to_R(x[3:7])
        r = [np.linalg.norm(s - (R @ ploc + t)) - D for (s, ploc, D, _) in legs]
        r.append(np.linalg.norm(x[3:7]) - 1.0)          # unit-quaternion (hard)
        return np.array(r)

    # start from the firmware prior: level, centred, ~home height
    x0 = np.array([0, 0, 200.0, 0, 0, 0, 1.0])
    res = least_squares(residual, x0, method="lm", max_nfev=20000,
                        xtol=1e-15, ftol=1e-15)
    t = res.x[0:3]; q = res.x[3:7] / np.linalg.norm(res.x[3:7]); R = quat_to_R(q)
    rms = math.sqrt(np.mean([(np.linalg.norm(s - (R @ p + t)) - D) ** 2
                             for (s, p, D, _) in legs]))
    if verbose:
        print("\n=== HONEST 6-LEG CLOSURE (only unknown = receiver pose) ===")
        print(f"  targets: D_main={D_MAIN:.3f}  D_pitch={D_PITCH:.3f}  (firmware, no fudge)")
        print(f"  receiver t   = {np.round(t,2).tolist()}")
        print(f"  receiver rpy = {np.round(rpy_deg(q),3).tolist()} deg  (0,0,0 = level)")
        for (s, p, D, nm) in legs:
            d = np.linalg.norm(s - (R @ p + t))
            print(f"    {nm:8s} servo{np.round(s,1).tolist()} -> dist {d:8.3f}  "
                  f"(target {D:.3f})  err {d-D:+.3f}")
        print(f"  honest closure RMS = {rms:.4f} mm")
    return dict(t=t, q=q, rms=rms, legs=legs)


if __name__ == "__main__":
    servos = perceive_servos()
    mains, pitch, _ = perceive_receiver_mounts()
    solve(servos, [c["center"] for c in mains], [c["center"] for c in pitch])
