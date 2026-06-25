# -*- coding: utf-8 -*-
"""SR6 receiver home pose as an INSTANCE of the general L1/2 datum layer.

All the machinery (proper-rotation Kabsch, residual, under-determination guard)
lives in uam.datum; this file only supplies the SR6-specific data:

  perceived : receiver pivots in the Receiver.stl frame  (from L0 cylinders.py)
  authority : the same pivots in world coords, READ OUT OF THE CONTROL LAW at
              the home input (roll=pitch=fwd=thrust=side=0) -- Authority.CONTROL_LAW

  Main  SetMainServo(16248,1500): out=0 (neutral, arm horizontal); local vertical
        x=162.48, horizontal y=15.0 from the main shaft -> world (+-59.5,0,208.48).
  Pitch SetPitchServo(16248,4500,0,0): x+=5500*sin15=+14.23, y-=5500*cos15=-53.13
        -> world (+-61, 53.13, 222.71).

Four non-collinear world pivots => the rigid pose is unique; the Kabsch residual
is the honest perception<->firmware prediction error (no guessed angle, no DOF).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from uam.datum import Authority, solve_placement  # noqa: E402

# perceived receiver pivots in the receiver's own STL frame (from cylinders.py)
RECV = {
    "main_L":  np.array([-59.48,   0.00,  0.00]),
    "main_R":  np.array([ 59.48,   0.00,  0.00]),
    "pitch_L": np.array([-61.00, -14.23, 53.13]),
    "pitch_R": np.array([ 61.00, -14.23, 53.13]),
}

# firmware-authoritative world pivots at the home input (the control law IS the datum)
WORLD = {
    "main_L":  np.array([-59.50,  0.00, 208.48]),
    "main_R":  np.array([ 59.50,  0.00, 208.48]),
    "pitch_L": np.array([-61.00, 53.13, 222.71]),
    "pitch_R": np.array([ 61.00, 53.13, 222.71]),
}


def solve():
    return solve_placement(RECV, WORLD, Authority.CONTROL_LAW)


if __name__ == "__main__":
    fit = solve()
    print("=== receiver home pose: Kabsch fit perceived -> firmware world ===\n")
    print(f"  authority = {fit.authority.name}\n")
    print("  rotation R (recv -> world):")
    for row in fit.R:
        print("    [{:+.4f} {:+.4f} {:+.4f}]".format(*row))
    ax, ang = fit.axis_angle()
    print(f"  => rotation {ang:+.2f} deg about axis "
          f"({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f}) ;  "
          f"t = ({fit.t[0]:+.2f},{fit.t[1]:+.2f},{fit.t[2]:+.2f})\n")
    for k, r in zip(fit.keys, fit.resid):
        p = fit.apply(RECV[k]); q = WORLD[k]
        print(f"  {k:8s} pred=({p[0]:7.2f},{p[1]:7.2f},{p[2]:7.2f})  "
              f"fw=({q[0]:7.2f},{q[1]:7.2f},{q[2]:7.2f})  resid={r:.4f} mm")
    print(f"\n  Kabsch RMS (perception vs firmware) = {fit.rms:.4f} mm")
    print("  (a small honest residual = perceived part geometry agrees with the")
    print("   control law; the receiver home orientation is authority-backed.)")
