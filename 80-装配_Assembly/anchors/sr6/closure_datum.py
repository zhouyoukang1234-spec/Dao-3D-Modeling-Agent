#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/closure_datum.py — datum-locked closure (the honest test).

WHY THE PRIOR CLOSURES CHEATED
------------------------------
closure_honest : servo footprints used directly as kinematic pivots -> drove the
                 receiver to a non-physical 78.5 deg roll (under-determined: 6
                 point-to-rigid-body distances have no orientation anchor).
closure_grounded: added a *free* shared shaft offset (a,b,c) -> RMS=0 but with an
                 absurd |offset|=171mm. A free 3-vector absorbs all residual; it
                 manufactures agreement instead of testing it.

THE TWO MISSING, NON-FABRICATED INPUTS (now supplied)
-----------------------------------------------------
1. L-1/2 DATUM (from the SR6 build guide PDF, the user's "原图", step 9 p.24):
   at HOME every servo arm is horizontal and the platform is level & centred.
   => receiver orientation is FIXED (R = I), and tx = 0. Not solved, not free.
2. HOST-KINEMATICS metric (from the firmware IK, decoded exactly):
   the home servo-shaft -> receiver-pivot distance is fixed by the control law:
       main : c = hypot(162.48, 15.0)          = 163.171 mm
       pitch: c = hypot(176.715, -8.126)       = 176.902 mm   (after 55mm@15deg)
   These c are the SHAFT-to-pivot distances (the 50/75mm arm is folded in), NOT
   the 175mm printed link.

WHAT IS PERCEIVED (never fabricated)
------------------------------------
* servo screw-hole footprints on L/R Frame  (centroid + plane), z ~ 18-26mm
* receiver rod-end pivots on Receiver.stl    (2 main axes +-59.5, 2 pitch axes)
The footprint is NOT the shaft: a standard servo body lifts the output shaft
~25-30mm above its mounting-screw plane. That single COTS body-lift is the ONLY
free scalar we allow, and we DEMAND it land in the physical 15-35mm range.

UNKNOWNS (4, all physically meaningful)            EQUATIONS (>=4)
  ty  : home fore/aft offset of receiver            4 main shaft->pivot distances
  tz  : home height of receiver (~208)              2 pitch shaft->pivot distances
  lift: shared footprint->shaft body lift (+z)
  (R = I, tx = 0 are DATUM, held fixed)

If the optimum gives tz ~ 208, a lift in the physical servo-body range, and a
small honest RMS, the assembly closes for an HONEST reason -- the missing inputs
were the datum + the COTS shaft geometry, exactly as predicted. Non-zero residual
is reported as-is; we do not add free parameters to drive it to zero.
"""
from __future__ import annotations
import os, sys, math
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from uam.cylinders import detect_cylinders            # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")

# firmware-decoded shaft->pivot home distances (see constants.py / module docstring)
C_MAIN  = math.hypot(162.48, 15.0)                      # 163.171
C_PITCH = math.hypot(162.48 + 55*math.sin(0.2618),
                     45.0   - 55*math.cos(0.2618))      # 176.902
HOME_H_GUESS = 208.48


def servo_footprints():
    """Perceive the 3 servo screw-hole footprints on each wall (centroid, z)."""
    walls = {}
    for nm, side, sx in [("LFrame", "L", -1), ("RFrame", "R", +1)]:
        cyl = detect_cylinders(os.path.join(STL, f"{nm}.stl"), rmin=1.4, rmax=3.2)
        H = np.array([c["center"] for c in cyl
                      if abs(c["axis"][2]) > 0.9 and c["kind"] == "hole"])
        order = np.argsort(H[:, 1]); bands, cur = [], [order[0]]
        for k in order[1:]:
            if H[k, 1] - H[cur[-1], 1] < 6.0: cur.append(k)
            else: bands.append(cur); cur = [k]
        bands.append(cur)
        bc = [float(np.mean(H[b, 1])) for b in bands]
        rows, i = [], 0
        while i < len(bands):
            if i + 1 < len(bands) and abs(bc[i+1] - bc[i]) < 16.0:
                rows.append(bands[i] + bands[i+1]); i += 2
            else:
                rows.append(bands[i]); i += 1
        pts = sorted([H[r].mean(0) for r in rows], key=lambda p: p[1])
        walls[side] = pts                                # 3 footprints, sorted by y
    return walls


def receiver_pivots():
    cyl = detect_cylinders(os.path.join(STL, "Receiver.stl"), rmin=1.4, rmax=3.0)
    xax = [c for c in cyl if abs(c["axis"][0]) > 0.9 and c["kind"] == "hole"]
    main = sorted([np.array(c["center"]) for c in xax if abs(c["center"][2]) < 20],
                  key=lambda p: p[0])                    # [left(-), right(+)]
    pit  = sorted([np.array(c["center"]) for c in xax if c["center"][2] > 35],
                  key=lambda p: p[0])
    return main, pit


def build_legs(walls, main_piv, pit_piv):
    """Assign footprints->pivots by SR6 layout: per wall the 2 main servos drive
    that side's single (coaxial) main pivot; the 1 pitch servo drives that side's
    pitch pivot. The pitch servo is the higher-mounted footprint (larger z)."""
    legs = []
    for side, sgn in (("L", 0), ("R", 1)):
        pts = walls[side]
        pitch_fp = min(pts, key=lambda p: abs(p[1]))     # MIDDLE (y~0) -> pitch
        mains_fp = [p for p in pts if not np.array_equal(p, pitch_fp)]  # fore/aft -> main
        mp = main_piv[0] if side == "L" else main_piv[1]
        pp = pit_piv[0]  if side == "L" else pit_piv[1]
        for fp in mains_fp:
            legs.append((np.asarray(fp, float), "main", side))
        legs.append((np.asarray(pitch_fp, float), "pitch", side))
    return legs, main_piv, pit_piv


def solve(legs, main_piv, pit_piv, verbose=True):
    Lm, Rm = main_piv
    Lp, Rp = pit_piv

    def piv_world(kind, side, ty, tz):
        # receiver LEVEL + x-CENTRED (datum): world = local + (0,ty,tz), tx=0
        loc = (Lm if side == "L" else Rm) if kind == "main" else (Lp if side == "L" else Rp)
        return np.array([loc[0], loc[1] + ty, loc[2] + tz])

    def residual(x):
        ty, tz, lift = x
        r = []
        for fp, kind, side in legs:
            shaft = fp + np.array([0.0, 0.0, lift])      # COTS body lift along +z
            tgt = C_MAIN if kind == "main" else C_PITCH
            r.append(np.linalg.norm(shaft - piv_world(kind, side, ty, tz)) - tgt)
        return np.array(r)

    x0 = np.array([0.0, HOME_H_GUESS, 28.0])
    res = least_squares(residual, x0, method="trf",
                        bounds=([-40, 150, 10], [40, 260, 40]),
                        xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=20000)
    ty, tz, lift = res.x
    rr = residual(res.x)
    rms = math.sqrt(float(np.mean(rr**2)))
    if verbose:
        print("\n=== DATUM-LOCKED CLOSURE (R=I, tx=0 from PDF; c from firmware) ===")
        print(f"  receiver home : tx=0(datum)  ty={ty:+.2f}  tz={tz:.2f}  (firmware home_h~{HOME_H_GUESS})")
        print(f"  shared shaft body-lift (footprint->shaft, +z) = {lift:.2f} mm  "
              f"[physical standard-servo body 15-35mm]")
        for (fp, kind, side), e in zip(legs, rr):
            tgt = C_MAIN if kind == "main" else C_PITCH
            print(f"     {side}-{kind:5s} fp z={fp[2]:5.1f}  dist={tgt+e:8.3f}  "
                  f"(c={tgt:.3f})  err={e:+.3f}")
        print(f"  closure RMS = {rms:.4f} mm   nfev={res.nfev}")
        physical = 15 <= lift <= 35 and 195 <= tz <= 222
        print(f"  VERDICT: {'PHYSICAL closure (datum+COTS were the missing inputs)' if physical else 'still off -- residual reveals remaining model gap'}")
    return dict(ty=ty, tz=tz, lift=lift, rms=rms, res=rr)


if __name__ == "__main__":
    walls = servo_footprints()
    main_piv, pit_piv = receiver_pivots()
    print("=== perceived servo footprints (per wall, sorted by y) ===")
    for s in ("L", "R"):
        print(f"  {s}: " + "  ".join(np.round(p, 1).tolist().__repr__() for p in walls[s]))
    print(f"  receiver main pivots = {[np.round(p,1).tolist() for p in main_piv]}")
    print(f"  receiver pitch pivots= {[np.round(p,1).tolist() for p in pit_piv]}")
    print(f"  firmware c_main={C_MAIN:.3f}  c_pitch={C_PITCH:.3f}")
    legs, mp, pp = build_legs(walls, main_piv, pit_piv)
    solve(legs, mp, pp)
