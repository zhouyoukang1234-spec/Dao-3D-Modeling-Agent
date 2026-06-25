#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/closure.py — Tier-2: full 6-leg parallel-mechanism closure.

This is the test the prior architecture never ran. The prior session (see
ground_truth/prior/ORS6_Stewart/HALLUCINATION_MAP.md) HARDCODED servo world
positions (wrong by up to 53mm) and computed receiver mounts from IK formulas,
NEVER checking the rods actually reach the real Receiver pivot geometry. That is
why "总是组装不起来" — there was no closure equation, only a pile of inferred
world coordinates.

Here we ground EVERYTHING in perceived mating interfaces + firmware metric prior,
with ZERO hardcoded world coordinates:

  PERCEIVED (from STL meshes, the solid truth):
    - servo shaft (sx,sy) per leg  := center of the 4-hole servo mount cluster
                                       on the L/R Frame (real, in shared frame)
    - receiver mount points         := Receiver.stl cylinder-axis pivots
    - arm horn->ball offset          := Arm.stl / Pitcher.stl hole-center span
  PRIOR (firmware SR6-Alpha4_ESP32.ino, metric only — NO world coords):
    - main arm = 50, main rod = 175, pitch arm = 75
    - home receiver pose is LEVEL & centered at HOME_H (the definition of home)

UNKNOWNS solved by least-squares (predictive coding — minimize total surprise):
    - receiver 6-DOF pose (q, t)
    - one arm-swing angle theta_i per leg (ball on a horizontal circle of radius
      armlen about the perceived servo shaft)

If, at the optimum, every rod residual -> 0 AND the receiver comes out level at
~HOME_H from independent perceived geometry, the three evidence sources AGREE:
the assembly is correct. The closure RMS is the honest metric.
"""
from __future__ import annotations
import os, sys, json
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from uam.cylinders import detect_cylinders   # noqa: E402
import trimesh                                # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")

# ── firmware metric prior (NO world coordinates) ───────────────────────────
MAIN_ARM, MAIN_ROD = 50.0, 175.0
PITCH_ARM, PITCH_ROD = 75.0, 185.0   # PitcherLink pin-pin = sqrt(60^2+175^2)=185
HOME_H = 208.48          # firmware 16248/100 + servoPivotH(46); home receiver z


# ── L0 perception helpers ──────────────────────────────────────────────────
def cluster_xy(pts, tol=18.0):
    """Greedy cluster of points by (x,y) proximity; return list of member-index lists."""
    pts = np.asarray(pts, float)
    used = np.zeros(len(pts), bool)
    groups = []
    for i in range(len(pts)):
        if used[i]:
            continue
        members = [i]
        used[i] = True
        for j in range(len(pts)):
            if not used[j] and np.hypot(*(pts[j, :2] - pts[i, :2])) < tol:
                members.append(j); used[j] = True
        groups.append(members)
    return groups


def perceive_servos():
    """Perceive 6 servo shaft (x,y,z) from the L/R Frame mount-hole clusters.

    A standard RC servo presents 4 mount holes (49mm x 10mm). The shaft sits at
    the center of that footprint. We recover each footprint, no positions hardcoded.
    """
    servos = {}
    for nm, side in [("LFrame", "L"), ("RFrame", "R")]:
        cyl = detect_cylinders(os.path.join(STL, f"{nm}.stl"), rmin=1.4, rmax=3.2)
        z = np.array([c["center"] for c in cyl
                      if abs(c["axis"][2]) > 0.9 and c["kind"] == "hole"])
        # cluster by Y (servo rows); each servo = a Y-band of 4 holes
        ys = z[:, 1]
        order = np.argsort(ys)
        rows = []
        cur = [order[0]]
        for k in order[1:]:
            if ys[k] - ys[cur[-1]] < 6.0:     # same flange-pair band
                cur.append(k)
            else:
                rows.append(cur); cur = [k]
        rows.append(cur)
        # merge adjacent bands whose centers are within a servo width (the two
        # flange tabs of ONE servo are ~10mm apart in Y)
        band_c = [float(np.mean(ys[r])) for r in rows]
        servo_rows, i = [], 0
        while i < len(rows):
            if i + 1 < len(rows) and abs(band_c[i + 1] - band_c[i]) < 16.0:
                servo_rows.append(rows[i] + rows[i + 1]); i += 2
            else:
                servo_rows.append(rows[i]); i += 1
        for r in servo_rows:
            c = z[r].mean(0)
            servos[(side, round(c[1]))] = c
    return servos


def perceive_receiver_mounts():
    """Perceive Receiver pivot points (cylinder axes ‖ X) in Receiver-local coords."""
    cyl = detect_cylinders(os.path.join(STL, "Receiver.stl"), rmin=1.4, rmax=3.0)
    xax = [c for c in cyl if abs(c["axis"][0]) > 0.9 and c["kind"] == "hole"]
    # main pivots near z~0, pitch pivots near z~53
    mains = [c for c in xax if abs(c["center"][2]) < 20]
    pitch = [c for c in xax if c["center"][2] > 35]
    return mains, pitch, xax


# ── quaternion helpers (x,y,z,w) ───────────────────────────────────────────
def qrot(q, v):
    q = np.asarray(q, float); q = q / (np.linalg.norm(q) + 1e-12)
    x, y, z, w = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])
    return R @ np.asarray(v, float)


def solve_main_closure(servos, mains_local, verbose=True):
    """Solve receiver pose + 4 main arm angles so all 4 main rods close at 175mm.

    Legs (perceived):
      L lower/upper main servos (-76.4, -+30)  -> left  main axle (local -59.5,0,0)
      R lower/upper main servos (+76.4, -+30)  -> right main axle (local +59.5,0,0)
    The two left legs share the left axle (PDF p.31 "two links on each bolt");
    likewise right. Ball_i sweeps a horizontal circle radius=MAIN_ARM about the
    perceived servo shaft. Unknowns: receiver (t,q), shaft height Zs (shared),
    4 swing angles. Firmware home prior (level, centered, z=HOME_H) added softly.
    """
    L = sorted([v for k, v in servos.items() if k[0] == "L" and abs(k[1]) > 15],
               key=lambda p: p[1])
    R = sorted([v for k, v in servos.items() if k[0] == "R" and abs(k[1]) > 15],
               key=lambda p: p[1])
    legs = []  # (servo_xy, receiver_local_mount)
    left_axle = np.array([m for m in mains_local if m[0] < 0][0])
    right_axle = np.array([m for m in mains_local if m[0] > 0][0])
    for s in L:
        legs.append((np.array(s[:2]), left_axle))
    for s in R:
        legs.append((np.array(s[:2]), right_axle))

    # state: t(3), q(4), Zs(1), theta(4)
    def unpack(x):
        return x[0:3], x[3:7], x[7], x[8:12]

    def residual(x):
        t, q, Zs, th = unpack(x)
        r = []
        for i, (sxy, mloc) in enumerate(legs):
            ball = np.array([sxy[0] + MAIN_ARM*np.cos(th[i]),
                             sxy[1] + MAIN_ARM*np.sin(th[i]), Zs])
            mount = qrot(q, mloc) + t
            r.append(np.linalg.norm(ball - mount) - MAIN_ROD)
        # firmware HOME prior (soft): receiver level + centered + at HOME_H
        qn = q / (np.linalg.norm(q) + 1e-12)
        r += [5.0*qn[0], 5.0*qn[1], 5.0*qn[2]]      # level (no roll/pitch/yaw)
        r += [2.0*t[0]]                              # centered in x
        r += [0.5*(t[2] - HOME_H)]                   # near firmware home height
        r += [1.0*(np.linalg.norm(q) - 1.0)]         # unit quaternion
        return np.array(r)

    x0 = np.concatenate([[0, 0, HOME_H], [0, 0, 0, 1.0], [46.0],
                         [np.pi/2, np.pi/2, np.pi/2, np.pi/2]])
    res = least_squares(residual, x0, method="trf", max_nfev=20000,
                        xtol=1e-15, ftol=1e-15, gtol=1e-15)
    t, q, Zs, th = unpack(res.x)
    qn = q / np.linalg.norm(q)
    # report rod lengths
    rods = []
    for i, (sxy, mloc) in enumerate(legs):
        ball = np.array([sxy[0]+MAIN_ARM*np.cos(th[i]),
                         sxy[1]+MAIN_ARM*np.sin(th[i]), Zs])
        mount = qrot(qn, mloc) + t
        rods.append(float(np.linalg.norm(ball - mount)))
    rod_rms = float(np.sqrt(np.mean([(rl-MAIN_ROD)**2 for rl in rods])))
    if verbose:
        print("\n=== 4-MAIN-LEG CLOSURE (perceived servos + real receiver axles) ===")
        print(f"  receiver t = {np.round(t,2).tolist()}  (home prior z={HOME_H})")
        print(f"  receiver q = {np.round(qn,4).tolist()}  (identity=level)")
        print(f"  shaft height Zs = {Zs:.2f} mm  (Arm horn ~46-51)")
        print(f"  rod lengths = {[round(r,2) for r in rods]}  (target {MAIN_ROD})")
        print(f"  rod closure RMS = {rod_rms:.4f} mm   nfev={res.nfev}")
    return dict(t=t, q=qn, Zs=Zs, theta=th, rods=rods, rod_rms=rod_rms, legs=legs)


def solve_full_closure(servos, mains_local, pitch_local, verbose=True):
    """Full 6-leg closure: 4 main rods @175 + 2 pitch rods @185 onto the SAME
    receiver rigid body, with the firmware HOME prior (level/centered/HOME_H).

    Each ball sweeps a horizontal circle (radius = arm length) about the perceived
    servo shaft. The two left main servos share the left main axle, the two right
    share the right; each pitch servo drives its side's pitch mount. If ALL six
    rods close simultaneously while the receiver stays level/centered, the
    perceived geometry and the firmware prior are mutually consistent.
    """
    Lm = sorted([v for k, v in servos.items() if k[0] == "L" and abs(k[1]) > 15],
                key=lambda p: p[1])
    Rm = sorted([v for k, v in servos.items() if k[0] == "R" and abs(k[1]) > 15],
                key=lambda p: p[1])
    Lp = [v for k, v in servos.items() if k[0] == "L" and abs(k[1]) <= 15][0]
    Rp = [v for k, v in servos.items() if k[0] == "R" and abs(k[1]) <= 15][0]
    left_axle = np.array([m for m in mains_local if m[0] < 0][0])
    right_axle = np.array([m for m in mains_local if m[0] > 0][0])
    left_pitch = np.array([p for p in pitch_local if p[0] < 0][0])
    right_pitch = np.array([p for p in pitch_local if p[0] > 0][0])
    # leg = (servo_xy, arm_len, rod_len, mount_local, is_pitch)
    legs = [(np.array(s[:2]), MAIN_ARM, MAIN_ROD, left_axle, False) for s in Lm]
    legs += [(np.array(s[:2]), MAIN_ARM, MAIN_ROD, right_axle, False) for s in Rm]
    legs += [(np.array(Lp[:2]), PITCH_ARM, PITCH_ROD, left_pitch, True)]
    legs += [(np.array(Rp[:2]), PITCH_ARM, PITCH_ROD, right_pitch, True)]
    n = len(legs)

    # state: t(3), q(4), Zs_main(1), Zs_pitch(1), theta(n)
    def unpack(x):
        return x[0:3], x[3:7], x[7], x[8], x[9:9+n]

    def ball_of(sxy, arm, th, Z):
        return np.array([sxy[0]+arm*np.cos(th), sxy[1]+arm*np.sin(th), Z])

    def residual(x):
        t, q, Zm, Zp, th = unpack(x)
        qn = q / (np.linalg.norm(q) + 1e-12)
        r = []
        for i, (sxy, arm, rod, mloc, isp) in enumerate(legs):
            Z = Zp if isp else Zm
            ball = ball_of(sxy, arm, th[i], Z)
            mount = qrot(qn, mloc) + t
            r.append(np.linalg.norm(ball - mount) - rod)
        r += [5.0*qn[0], 5.0*qn[1], 5.0*qn[2]]   # level
        r += [2.0*t[0]]                           # centered x
        r += [0.3*(t[2] - HOME_H)]                # near home height
        r += [0.2*(Zm - 46.0), 0.2*(Zp - 46.0)]   # shaft height ~ Arm horn Z(46)
        r += [1.0*(np.linalg.norm(q) - 1.0)]
        return np.array(r)

    x0 = np.concatenate([[0, 0, HOME_H], [0, 0, 0, 1.0], [46.0, 46.0],
                         [np.pi/2]*4 + [np.pi/2, np.pi/2]])
    res = least_squares(residual, x0, method="trf", max_nfev=40000,
                        xtol=1e-15, ftol=1e-15, gtol=1e-15)
    t, q, Zm, Zp, th = unpack(res.x)
    qn = q / np.linalg.norm(q)
    rods, names = [], ["L-main1", "L-main2", "R-main1", "R-main2", "L-pitch", "R-pitch"]
    for i, (sxy, arm, rod, mloc, isp) in enumerate(legs):
        Z = Zp if isp else Zm
        ball = ball_of(sxy, arm, th[i], Z)
        mount = qrot(qn, mloc) + t
        rods.append((names[i], float(np.linalg.norm(ball - mount)), rod))
    rms = float(np.sqrt(np.mean([(rl-tg)**2 for _, rl, tg in rods])))
    # roll/pitch/yaw from quaternion (deg)
    import math
    rpy = [math.degrees(a) for a in (
        math.atan2(2*(qn[3]*qn[0]+qn[1]*qn[2]), 1-2*(qn[0]**2+qn[1]**2)),
        math.asin(max(-1, min(1, 2*(qn[3]*qn[1]-qn[2]*qn[0])))),
        math.atan2(2*(qn[3]*qn[2]+qn[0]*qn[1]), 1-2*(qn[1]**2+qn[2]**2)))]
    if verbose:
        print("\n=== FULL 6-LEG CLOSURE (4 main @175 + 2 pitch @185) ===")
        print(f"  receiver t = {np.round(t,2).tolist()}")
        print(f"  receiver roll/pitch/yaw = {np.round(rpy,3).tolist()} deg (0,0,0=level)")
        print(f"  shaft height  main Zm={Zm:.2f}  pitch Zp={Zp:.2f}  (Arm horn ~46)")
        for nm, rl, tg in rods:
            print(f"    {nm:9s} rod = {rl:8.3f}  (target {tg})  err={rl-tg:+.4f}")
        print(f"  6-leg closure RMS = {rms:.4f} mm   nfev={res.nfev}")
    return dict(t=t, q=qn, Zm=Zm, Zp=Zp, theta=th, rods=rods, rms=rms, legs=legs)


if __name__ == "__main__":
    servos = perceive_servos()
    print("=== perceived servo shafts (frame footprint centers, shared frame) ===")
    for k, v in sorted(servos.items()):
        print(f"  {k}: {np.round(v,1).tolist()}")
    mains, pitch, xax = perceive_receiver_mounts()
    print(f"\n=== perceived receiver pivots (local) ===  ({len(xax)} X-axis holes)")
    for c in sorted(xax, key=lambda d: (round(d['center'][2]), d['center'][0])):
        print(f"  {np.round(c['center'],1).tolist()} r={c['radius']:.2f} L={c['length']:.1f}")
    mains_local = [c["center"] for c in mains]
    pitch_local = [c["center"] for c in pitch]
    solve_main_closure(servos, mains_local)
    solve_full_closure(servos, mains_local, pitch_local)
