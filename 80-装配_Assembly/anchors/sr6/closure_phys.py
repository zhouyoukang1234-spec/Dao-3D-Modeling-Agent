"""Physically-correct 6-leg closure for the SR6 receiver.

Why this file exists
--------------------
`closure.py` swept every ball on a *horizontal* circle about a vertical axis:
    ball = (sx + arm*cos th, sy + arm*sin th, Z)
That gives the ball a translational degree of freedom in **x** that the real
linkage does NOT have.  Every servo horn on this machine turns about a
**horizontal (X) shaft** (Arm.stl / L-RPitcher.stl holes are all bored along X),
so the ball sweeps a circle in the **Y-Z plane at the fixed wall x = +-76.4**.
The horizontal model let the solver slide each ball up to ~47 mm in x to force
RMS=0 -- a fabricated closure.  This module removes that fake DOF: x is pinned.

Geometry (all perceived / firmware, no world coordinates invented)
    main servos : wall x=+-76.4, y in {-30,+30}, horn radius 50, sweep in Y-Z
    pitch servos: wall x=+-76.4, y = 0,           horn radius 75, sweep in Y-Z,
                  with a +3.8 mm along-shaft (inward) offset = the "kink"
    receiver    : 2 main bolts (+-59.5,0,0)  [2 links each, PDF p.31]
                  2 pitch bolts (+-61,-14.2,53.1)
    every rod (main AND pitch) = 175 mm bearing-centre to bearing-centre
                  (PDF p.26: "all 6 links ... 175 mm"; the 185 mm figure was the
                   kinked Alpha *grommet* variant -- a variant trap, now rejected)
"""
import os, sys, math
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
from closure import perceive_servos, perceive_receiver_mounts, qrot  # noqa: E402

MAIN_ARM, MAIN_ROD = 50.0, 175.0
PITCH_ARM, PITCH_ROD = 75.0, 175.0     # PDF p.26: all 6 links 175 mm
PITCH_ALONG = 3.8                      # LPitcher ball is 3.8 mm along the shaft
HOME_H = 208.48


def solve(verbose=True):
    servos = perceive_servos()
    mains, pitch, _ = perceive_receiver_mounts()
    main_local = {("L" if c["center"][0] < 0 else "R"): np.array(c["center"]) for c in mains}
    pitch_local = {("L" if c["center"][0] < 0 else "R"): np.array(c["center"]) for c in pitch}

    # leg = (servo_xy, side, radius, along, mount_local, name)
    legs = []
    for k, v in sorted(servos.items()):
        side, y = k
        sxy = np.array(v[:2])
        if abs(y) > 15:                                   # main
            legs.append((sxy, side, MAIN_ARM, 0.0, main_local[side], f"{side}-main{int(y):+d}"))
        else:                                             # pitch
            legs.append((sxy, side, PITCH_ARM, PITCH_ALONG, pitch_local[side], f"{side}-pitch"))

    def ball_of(sxy, side, R, along, Z, th):
        # shaft axis = +-X.  main horns point OUTWARD, pitch horns point INWARD.
        # the along-offset sits on the inward shaft direction (kink toward centre).
        inward = +1.0 if side == "L" else -1.0          # toward x=0
        bx = sxy[0] + inward * along
        by = sxy[1] + R * math.cos(th)
        bz = Z + R * math.sin(th)
        return np.array([bx, by, bz])

    n = len(legs)
    def unpack(x):
        return x[0:3], x[3:7], x[7], x[8], x[9:9 + n]

    def residual(x):
        t, q, Zm, Zp, th = unpack(x)
        qn = q / (np.linalg.norm(q) + 1e-12)
        r = []
        for i, (sxy, side, R, along, mloc, nm) in enumerate(legs):
            Z = Zp if "pitch" in nm else Zm
            rod = PITCH_ROD if "pitch" in nm else MAIN_ROD
            ball = ball_of(sxy, side, R, along, Z, th[i])
            mount = qrot(qn, mloc) + t
            r.append(np.linalg.norm(ball - mount) - rod)
        r += [5.0 * qn[0], 5.0 * qn[1], 5.0 * qn[2]]      # level
        r += [2.0 * t[0]]                                 # centred in x
        r += [0.3 * (t[2] - HOME_H)]                      # near home height
        r += [0.2 * (Zm - 46.0), 0.2 * (Zp - 46.0)]       # shaft height ~ horn Z
        r += [1.0 * (np.linalg.norm(q) - 1.0)]
        return np.array(r)

    best = None
    for z0 in (46.0, 60.0, 80.0):                         # multi-start over shaft height
        for thp in (math.pi / 2, -math.pi / 2):           # pitch horn up / down
            x0 = np.concatenate([[0, 0, HOME_H], [0, 0, 0, 1.0], [46.0, z0],
                                 [math.pi / 2] * 4 + [thp, thp]])
            res = least_squares(residual, x0, method="trf", max_nfev=40000,
                                xtol=1e-15, ftol=1e-15, gtol=1e-15)
            if best is None or res.cost < best.cost:
                best = res
    res = best
    t, q, Zm, Zp, th = unpack(res.x)
    qn = q / np.linalg.norm(q)
    rods = []
    for i, (sxy, side, R, along, mloc, nm) in enumerate(legs):
        Z = Zp if "pitch" in nm else Zm
        tg = PITCH_ROD if "pitch" in nm else MAIN_ROD
        ball = ball_of(sxy, side, R, along, Z, th[i])
        mount = qrot(qn, mloc) + t
        rods.append((nm, float(np.linalg.norm(ball - mount)), tg, ball))
    rms = float(np.sqrt(np.mean([(rl - tg) ** 2 for _, rl, tg, _ in rods])))
    rpy = [math.degrees(a) for a in (
        math.atan2(2 * (qn[3] * qn[0] + qn[1] * qn[2]), 1 - 2 * (qn[0] ** 2 + qn[1] ** 2)),
        math.asin(max(-1, min(1, 2 * (qn[3] * qn[1] - qn[2] * qn[0])))),
        math.atan2(2 * (qn[3] * qn[2] + qn[0] * qn[1]), 1 - 2 * (qn[1] ** 2 + qn[2] ** 2)))]
    if verbose:
        print("=== PHYSICAL 6-LEG CLOSURE (Y-Z sweep about X shaft; rod=175 all) ===")
        print(f"  receiver t   = {np.round(t,2).tolist()}")
        print(f"  roll/pitch/yaw = {np.round(rpy,2).tolist()} deg   (0,0,0 = level)")
        print(f"  shaft Z  main={Zm:.2f}  pitch={Zp:.2f}")
        for nm, rl, tg, ball in rods:
            print(f"    {nm:9s} rod={rl:8.3f} (target {tg})  err={rl-tg:+7.3f}   "
                  f"ball=({ball[0]:6.1f},{ball[1]:6.1f},{ball[2]:6.1f})")
        print(f"  closure RMS = {rms:.4f} mm")
    return dict(t=t, q=qn, Zm=Zm, Zp=Zp, theta=th, rods=rods, rms=rms, rpy=rpy)


if __name__ == "__main__":
    solve()
