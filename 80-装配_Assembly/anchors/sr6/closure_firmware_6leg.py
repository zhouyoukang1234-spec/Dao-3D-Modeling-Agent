"""Full 6-leg SR6 home closure from PERCEIVED pivots + FIRMWARE-datum servo placement.

This is the honest "does the whole thing assemble at home?" test.  It does NOT
fabricate any world coordinate and uses NO free fitting DOF beyond the one real
physical DOF per leg (the servo arm angle).

Datum sources (authority order, all external to the printed parts):
  * Receiver pivot layout  -> PERCEIVED from Receiver.stl (real geometry):
        main pivots  (+-59.48,   0.0,  0.0)   axis X   (len 13.3 axle)
        pitch pivots (+-61.00, -14.23, 53.13)  axis X   (len 12.0 axle)
    main->pitch offset = 55.0 mm @ 15.0 deg  (matches firmware 5500*sin/cos(0.2618))
  * Receiver home pose     -> firmware HOME_H: main pivots level at world z=208.48.
  * Servo placement        -> firmware IK (SetMainServo/SetPitchServo):
        main : arm 50, shaft 162.48 below + 15 horiz from its main pivot, z=46
        pitch: arm 75, shaft 162.48 below + 45 horiz from the main pivot, z=46
  * Link lengths           -> PERCEIVED bearing-centre spans:
        MainLink    = 175.0   PitcherLink = 185.0 (L-shaped: 60 lateral, 175 long)

Each leg's ONLY free parameter is the servo arm angle theta (a real DOF).  We
solve theta so the link closes to its physical length; a leg "assembles" iff a
real theta exists (target within [|L-a|, L+a]).  Residual is reported honestly.
"""
import math
import numpy as np
from scipy.optimize import brentq

HOME_H = 208.48                       # firmware receiver home height (main pivot z)
SHAFT_Z = 46.0                        # = HOME_H - 162.48
MAIN_ARM, MAIN_LINK = 50.0, 175.0
PITCH_ARM, PITCH_LINK = 75.0, 185.0   # PitcherLink physical span (perceived)
PITCH_LINK_FW = math.sqrt(36250.0)    # firmware equivalent rod = 190.39

# --- Receiver rigid body (perceived in its own STL frame) -------------------
# Home pose is the PROPER rotation Rx(-90 deg) then lift, proven by a Kabsch
# fit of the perceived pivots onto the firmware-authoritative world pivots
# (closure_kabsch.py, RMS 0.014mm).  Rx(-90): world = (rx, rz, -ry)+(0,0,HOME_H).
# (The earlier (rx,-rz,-ry) was a det=-1 reflection that mirrored the receiver
#  and put the pitch pivots on the wrong (-Y) side.)
def recv_to_world(p):
    rx, ry, rz = p
    return np.array([rx, rz, -ry]) + np.array([0.0, 0.0, HOME_H])

MAIN_PIV_RECV  = {"L": np.array([-59.48, 0.0, 0.0]),  "R": np.array([59.48, 0.0, 0.0])}
PITCH_PIV_RECV = {"L": np.array([-61.0, -14.23, 53.13]), "R": np.array([61.0, -14.23, 53.13])}


def solve_arm(shaft, pivot, arm, link, theta_hint=0.0):
    """Arm sweeps the world Y-Z plane about +X. Tip = shaft + arm*(0,cos,sin).
    Solve theta so |pivot - tip| = link. Returns (theta, tip, residual, reachable)."""
    d = pivot - shaft
    reach = float(np.linalg.norm(d))
    reachable = abs(link - arm) <= reach <= (link + arm)

    def tip(th):
        return shaft + arm * np.array([0.0, math.cos(th), math.sin(th)])

    def f(th):
        return float(np.linalg.norm(pivot - tip(th)) - link)

    if reachable:
        # bracket a root near the hint by scanning (pad past +-pi so boundary
        # roots like theta=pi, the horizontal-outward branch, are captured)
        ths = np.linspace(-math.pi - 0.4, math.pi + 0.4, 801)
        vals = [f(t) for t in ths]
        roots = []
        for i in range(len(ths) - 1):
            if vals[i] == 0 or vals[i] * vals[i + 1] < 0:
                roots.append(brentq(f, ths[i], ths[i + 1]))
        if roots:
            theta = min(roots, key=lambda t: abs(t - theta_hint))
            t = tip(theta)
            return theta, t, abs(f(theta)), True
    # unreachable: closest approach
    ths = np.linspace(-math.pi, math.pi, 1441)
    theta = min(ths, key=lambda t: abs(f(t)))
    return theta, tip(theta), abs(f(theta)), False


def arm_tilt(shaft, tip):
    d = tip - shaft
    return math.degrees(math.atan2(d[2], abs(d[1])))


def legs():
    out = []
    # 4 main legs: 2 per side at +-15 horiz (world Y) about the main pivot
    for side in ("L", "R"):
        piv = recv_to_world(MAIN_PIV_RECV[side])
        for ysign, nm in ((+1, "lower"), (-1, "upper")):
            shaft = np.array([piv[0], piv[1] + ysign * 15.0, SHAFT_Z])
            # neutral arm points OUTWARD (away from the y=0 pivot): +Y lower, -Y upper.
            hint = 0.0 if ysign > 0 else math.pi
            th, tip, res, ok = solve_arm(shaft, piv, MAIN_ARM, MAIN_LINK, hint)
            out.append((f"main-{side}-{nm}", shaft, tip, piv, MAIN_LINK, th, res, ok))
    # 2 pitch legs: pitch pivot is now on +Y; firmware places the pitch shaft
    # 8.12mm beyond it (servo-frame horizontal -8.124 from a 61.25mm shaft).
    for side in ("L", "R"):
        ppiv = recv_to_world(PITCH_PIV_RECV[side])
        shaft = np.array([MAIN_PIV_RECV[side][0], 61.25, SHAFT_Z])
        th, tip, res, ok = solve_arm(shaft, ppiv, PITCH_ARM, PITCH_LINK, 0.6)
        out.append((f"pitch-{side}", shaft, tip, ppiv, PITCH_LINK, th, res, ok))
    return out


if __name__ == "__main__":
    print("=== FULL 6-LEG HOME CLOSURE (perceived pivots + firmware-datum servos) ===")
    print(f"    receiver level @ z={HOME_H}, all shafts z={SHAFT_Z}")
    print(f"    links: main={MAIN_LINK}  pitch={PITCH_LINK} (fw-equiv {PITCH_LINK_FW:.2f})\n")
    res_all = []
    for nm, shaft, tip, piv, link, th, res, ok in legs():
        reach = float(np.linalg.norm(piv - shaft))
        tilt = arm_tilt(shaft, tip)
        res_all.append(res)
        flag = "CLOSES" if ok and res < 1e-3 else ("near" if ok else "UNREACHABLE")
        print(f"  {nm:13s} shaft=({shaft[0]:6.1f},{shaft[1]:6.1f},{shaft[2]:4.1f}) "
              f"pivot=({piv[0]:6.1f},{piv[1]:6.1f},{piv[2]:6.1f}) reach={reach:6.2f} "
              f"arm_tilt={tilt:+6.1f}deg link_resid={res:7.4f}  [{flag}]")
    rms = float(np.sqrt(np.mean(np.square(res_all))))
    print(f"\n  6-leg closure RMS link residual = {rms:.5f} mm")
    print("  (residual 0 => every leg has a real arm angle that closes its physical link at home)")
