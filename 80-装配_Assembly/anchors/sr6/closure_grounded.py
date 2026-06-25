#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/closure_grounded.py — the grounded closure test.

THE ROOT-CAUSE, stated plainly (see docs/DATUM.md):
  We perceive the mating interfaces on the *printed* parts (the 4 servo screw
  holes on the Frame, the rod-end pivots on the Receiver). But the true
  *kinematic joint* of a leg is the SERVO OUTPUT SHAFT, which is displaced from
  the screw-hole footprint by the servo's own body — a commercial-off-the-shelf
  (COTS) fact that exists in the servo datasheet, NOT in the printed-part STL,
  NOT in the firmware (which is purely per-servo-local), NOT in the build PDF
  text. The prior session FABRICATED that displacement (its HALLUCINATION_MAP
  admits errors up to 53mm). closure_honest.py, using the raw footprints as
  pivots, drove the receiver to a non-physical 78.5° roll.

THE TEST:
  Perceive each servo's LOCAL FRAME from its 4 screw holes:
      f   = footprint centroid           (world, perceived)
      n   = screw-hole-plane normal       (world, perceived  ~ +z)
      u   = footprint long axis           (world, perceived  ~ the 49.5mm span)
      w   = n x u                          (short axis)
  Model the true shaft pivot as
      p_i = f_i + a*u_i + b*w_i + c*n_i
  with (a,b,c) a SINGLE shared COTS offset (the servo datasheet number, mirrored
  by each servo's own perceived frame — NOT six free vectors). Solve

      unknowns = receiver pose (t, q)  +  shaft offset (a,b,c)        [9]
      equations= 6 rod-length closures + datum (level, centred)       [>=9]

  If the optimum yields (a,b,c) matching a real standard servo (~5-12mm) AND a
  LEVEL receiver at HOME_H with small residual, the principle is proven: the
  missing input was COTS joint geometry, not modelling skill, and once supplied
  the assembly closes physically — no fabrication, no spurious roll.
"""
from __future__ import annotations
import os, sys, math
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from uam.cylinders import detect_cylinders   # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")

# firmware metric prior (rod lengths; NO world coords)
MAIN_ROD, PITCH_ROD = 175.0, 185.0
HOME_H = 208.48


def servo_frames():
    """Perceive each servo's local frame (centroid f, normal n, long axis u, short w)
    from its 4 vertical-axis screw holes on the L/R Frame. Zero hardcoded positions."""
    frames = {}
    for nm, side in [("LFrame", "L"), ("RFrame", "R")]:
        cyl = detect_cylinders(os.path.join(STL, f"{nm}.stl"), rmin=1.4, rmax=3.2)
        H = np.array([c["center"] for c in cyl
                      if abs(c["axis"][2]) > 0.9 and c["kind"] == "hole"])
        # group screw holes into servos by Y band (3 servos per side)
        ys = np.sort(H[:, 1]); order = np.argsort(H[:, 1])
        bands, cur = [], [order[0]]
        for k in order[1:]:
            if H[k, 1] - H[cur[-1], 1] < 6.0:
                cur.append(k)
            else:
                bands.append(cur); cur = [k]
        bands.append(cur)
        bc = [float(np.mean(H[b, 1])) for b in bands]
        servo_rows, i = [], 0
        while i < len(bands):
            if i + 1 < len(bands) and abs(bc[i + 1] - bc[i]) < 16.0:
                servo_rows.append(bands[i] + bands[i + 1]); i += 2
            else:
                servo_rows.append(bands[i]); i += 1
        for r in servo_rows:
            P = H[r]
            f = P.mean(0)
            # plane normal via SVD (smallest singular direction)
            U, S, Vt = np.linalg.svd(P - f)
            n = Vt[2]; n = n * np.sign(n[2] + 1e-9)        # point +z
            u = Vt[0]; u = u / np.linalg.norm(u)           # long (max spread)
            w = np.cross(n, u); w = w / np.linalg.norm(w)
            frames[(side, round(f[1]))] = dict(f=f, n=n, u=u, w=w, k=len(r))
    return frames


def receiver_pivots():
    cyl = detect_cylinders(os.path.join(STL, "Receiver.stl"), rmin=1.4, rmax=3.0)
    xax = [c for c in cyl if abs(c["axis"][0]) > 0.9 and c["kind"] == "hole"]
    mains = [np.array(c["center"]) for c in xax if abs(c["center"][2]) < 20]
    pitch = [np.array(c["center"]) for c in xax if c["center"][2] > 35]
    return mains, pitch


def quat_to_R(q):
    q = q / (np.linalg.norm(q) + 1e-12); x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def rpy_deg(q):
    x, y, z, w = q / np.linalg.norm(q)
    return [math.degrees(a) for a in (
        math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        math.asin(max(-1, min(1, 2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]


def build_legs(frames, mains, pitch):
    """Pair each servo frame with the receiver pivot it drives (perceived geometry)."""
    Lm = sorted([v for k, v in frames.items() if k[0] == "L" and abs(k[1]) > 15],
                key=lambda d: d["f"][1])
    Rm = sorted([v for k, v in frames.items() if k[0] == "R" and abs(k[1]) > 15],
                key=lambda d: d["f"][1])
    Lp = [v for k, v in frames.items() if k[0] == "L" and abs(k[1]) <= 15][0]
    Rp = [v for k, v in frames.items() if k[0] == "R" and abs(k[1]) <= 15][0]
    la = [m for m in mains if m[0] < 0][0]; ra = [m for m in mains if m[0] > 0][0]
    lp = [p for p in pitch if p[0] < 0][0]; rp = [p for p in pitch if p[0] > 0][0]
    legs = []
    for fr in Lm: legs.append((fr, la, MAIN_ROD, "L-main"))
    for fr in Rm: legs.append((fr, ra, MAIN_ROD, "R-main"))
    legs.append((Lp, lp, PITCH_ROD, "L-pitch"))
    legs.append((Rp, rp, PITCH_ROD, "R-pitch"))
    return legs


def solve(legs, verbose=True):
    """unknowns x = [tx,ty,tz, qx,qy,qz,qw, a,b,c]; close 6 rods + datum."""
    def shaft(fr, a, b, c):
        return fr["f"] + a*fr["u"] + b*fr["w"] + c*fr["n"]

    def residual(x):
        t = x[0:3]; q = x[3:7]; a, b, c = x[7], x[8], x[9]
        R = quat_to_R(q); qn = q/np.linalg.norm(q)
        r = []
        for fr, mloc, rod, _ in legs:
            p = shaft(fr, a, b, c)
            mount = R @ mloc + t
            r.append(np.linalg.norm(mount - p) - rod)
        # datum (grounded in 3 sources: firmware symmetry, ref image, VESA-level):
        r += [8.0*qn[0], 8.0*qn[1], 8.0*qn[2]]   # receiver LEVEL (no roll/pitch/yaw)
        r += [3.0*t[0]]                          # centred in x
        r += [1.0*(np.linalg.norm(q) - 1.0)]
        return np.array(r)

    x0 = np.array([0, 0, HOME_H, 0, 0, 0, 1.0, 0.0, 0.0, 8.0])
    res = least_squares(residual, x0, method="trf", max_nfev=40000,
                        xtol=1e-15, ftol=1e-15, gtol=1e-15)
    t = res.x[0:3]; qn = res.x[3:7]/np.linalg.norm(res.x[3:7])
    a, b, c = res.x[7], res.x[8], res.x[9]
    R = quat_to_R(qn)
    rods = []
    for fr, mloc, rod, nm in legs:
        p = fr["f"] + a*fr["u"] + b*fr["w"] + c*fr["n"]
        rods.append((nm, float(np.linalg.norm(R@mloc + t - p)), rod))
    rms = math.sqrt(np.mean([(rl-tg)**2 for _, rl, tg in rods]))
    off = math.sqrt(a*a + b*b + c*c)
    if verbose:
        print("\n=== GROUNDED 6-LEG CLOSURE (perceived frames + shared COTS shaft offset) ===")
        print(f"  receiver t          = {np.round(t,2).tolist()}  (home z~{HOME_H})")
        print(f"  receiver roll/pit/yaw = {np.round(rpy_deg(qn),3).tolist()} deg  (0,0,0 = level)")
        print(f"  shared shaft offset = u:{a:+.2f}  w:{b:+.2f}  n:{c:+.2f} mm  |offset|={off:.2f}")
        print(f"     (a real standard servo shaft sits ~5-12mm from its screw footprint)")
        for nm, rl, tg in rods:
            print(f"     {nm:8s} rod = {rl:8.3f}  (target {tg})  err={rl-tg:+.4f}")
        print(f"  6-leg closure RMS   = {rms:.4f} mm   nfev={res.nfev}")
    return dict(t=t, q=qn, offset=(a, b, c), rods=rods, rms=rms)


if __name__ == "__main__":
    frames = servo_frames()
    print("=== perceived servo local frames (from 4 screw holes each) ===")
    for k, fr in sorted(frames.items()):
        print(f"  {str(k):12s} f={np.round(fr['f'],1).tolist()} "
              f"n={np.round(fr['n'],2).tolist()} u={np.round(fr['u'],2).tolist()} holes={fr['k']}")
    mains, pitch = receiver_pivots()
    print(f"\n  receiver main pivots  = {[np.round(m,1).tolist() for m in mains]}")
    print(f"  receiver pitch pivots = {[np.round(p,1).tolist() for p in pitch]}")
    legs = build_legs(frames, mains, pitch)
    solve(legs)
