"""Honest receiver home pose: roll about X solved by the pitch legs (a real DOF).

Why this file supersedes the hand-placed receiver tilt:
  * The 4 MAIN legs with arms horizontal (PDF p24 datum) force the two main
    pivots onto the world line {y=0, z=208.48} (rod 175 from balls at +-65).
    But both main pivots lie ON that X-axis line, so rotating the receiver
    about it does NOT move them -> the 4 main legs CANNOT fix the receiver's
    roll. That roll is a genuine mechanism DOF.
  * The roll is fixed by the 2 PITCH legs: as the receiver rolls by phi about
    the world X-axis at z=208.48, each perceived pitch pivot orbits; we solve
    phi so the physical 185mm PitcherLink closes from the pitch servo arm.

No fabricated world coordinate, no free fitting DOF beyond the one physical
roll phi (plus each servo's own arm angle).  Residual is reported honestly.
"""
import math
import numpy as np
from scipy.optimize import brentq

HOME_H = 208.48
SHAFT_Z = 46.0
MAIN_ARM, MAIN_LINK = 50.0, 175.0
PITCH_ARM, PITCH_LINK = 75.0, 185.0

# receiver pivots in its own perceived STL frame
MAIN_PIV  = {"L": np.array([-59.48, 0.0, 0.0]),     "R": np.array([59.48, 0.0, 0.0])}
PITCH_PIV = {"L": np.array([-61.0, -14.23, 53.13]), "R": np.array([61.0, -14.23, 53.13])}

AXIS_Z = HOME_H                      # world X-axis line the receiver rolls about


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def receiver_world(phi):
    """Roll the receiver by phi about the world X-axis line at z=HOME_H.
    Main pivots are on that line (recv y=z=0) so they stay at (+-59.48,0,HOME_H)
    for every phi -- which is exactly why the 4 main legs cannot fix phi."""
    R = Rx(phi)
    lift = np.array([0.0, 0.0, HOME_H])
    out = {}
    for s in ("L", "R"):
        out[("main", s)] = R @ MAIN_PIV[s] + lift
        out[("pitch", s)] = R @ PITCH_PIV[s] + lift
    return out


def pitch_ball(side):
    """Pitch servo arm HORIZONTAL (PDF calib). Shaft 45 in -Y from main pivot,
    z=46. Horizontal 75 arm pointing further -Y (outboard, toward pitch links)."""
    x = -59.5 if side == "L" else 59.5
    shaft = np.array([x, -45.0, SHAFT_Z])
    ball = shaft + np.array([0.0, -75.0, 0.0])      # horizontal, -Y
    return shaft, ball


def pitch_gap(phi, side):
    piv = receiver_world(phi)[("pitch", side)]
    _, ball = pitch_ball(side)
    return float(np.linalg.norm(piv - ball) - PITCH_LINK)


def solve_phi():
    phis = np.linspace(math.radians(-120), math.radians(120), 2401)
    g = [pitch_gap(p, "L") for p in phis]
    roots = []
    for i in range(len(phis) - 1):
        if g[i] == 0 or g[i] * g[i + 1] < 0:
            roots.append(brentq(lambda p: pitch_gap(p, "L"), phis[i], phis[i + 1]))
    return roots


if __name__ == "__main__":
    print("=== receiver roll about X solved by pitch-leg closure ===")
    print("    (4 main legs fix position+level but leave roll free; pitch fixes roll)\n")
    main_pivW = receiver_world(0.0)
    print(f"  main pivots @ phi=0: L={main_pivW[('main','L')]}  R={main_pivW[('main','R')]}")
    roots = solve_phi()
    print(f"  pitch-closure roots for phi (deg): {[round(math.degrees(r),2) for r in roots]}")
    for r in roots:
        piv = receiver_world(r)
        gl = pitch_gap(r, "L"); gr = pitch_gap(r, "R")
        pL = piv[("pitch", "L")]
        print(f"\n  phi = {math.degrees(r):+7.2f} deg")
        print(f"    pitch pivot L (world) = ({pL[0]:6.2f},{pL[1]:6.2f},{pL[2]:6.2f})")
        print(f"    pitch link residual: L={gl:+.4f}  R={gr:+.4f} mm")
        # height of pitch pivot vs main pivot
        print(f"    pitch pivot z - main pivot z = {pL[2]-HOME_H:+.2f} mm")
