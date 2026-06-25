"""Honest datum test.

The PDF gives TWO independent home facts:
  (A) every servo arm is HORIZONTAL at home (p.24 step 9 calibration line),
  (B) the receiver is LEVEL at HOME_H = 208.48 (firmware).
If the perceived ABSOLUTE placement (servo wall x, mount x, shaft Z) were exact,
then with rod = 175 both (A) and (B) would hold at once.  We pin the arms
horizontal (hard) + receiver level (hard), solve only the receiver height and the
ball Y-sense, and read the rod residual.  Whatever it is, it is the honest
Bayesian surprise in the ABSOLUTE placement.
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
from closure import perceive_servos, perceive_receiver_mounts  # noqa: E402

MAIN_ARM, MAIN_ROD = 50.0, 175.0
PITCH_ARM, PITCH_ROD = 75.0, 175.0
HOME_H = 208.48

servos = perceive_servos()
mains, pitch, _ = perceive_receiver_mounts()
main_local = {("L" if c["center"][0] < 0 else "R"): np.array(c["center"]) for c in mains}
pitch_local = {("L" if c["center"][0] < 0 else "R"): np.array(c["center"]) for c in pitch}

# arm horizontal at home => ball is in the +-Y direction at the shaft height Z.
# main shaft Z ~ 46 (Arm horn).  Try the Y-sense that best matches each mount.
def rod_for_main(side, sy, ZH, recv_z):
    sx = -76.4 if side == "L" else 76.4
    mount = main_local[side] + np.array([0, 0, recv_z])     # level, centred
    best = 1e9
    for sgn in (+1, -1):
        ball = np.array([sx, sy + sgn * MAIN_ARM, ZH])      # horizontal arm
        best = min(best, np.linalg.norm(ball - mount))
    return best

print("If we DEMAND main arms horizontal + receiver level at HOME_H, the main rod")
print("would have to be (target 175):")
for side in ("L", "R"):
    for sy in (-30, 30):
        rl = rod_for_main(side, sy, 46.0, HOME_H)
        print(f"   {side} servo y={sy:+d}:  rod={rl:7.2f}  err={rl-175:+6.2f} ({(rl-175)/175*100:+5.1f}%)")

# What receiver height would make a horizontal main arm give exactly 175?
print("\nReceiver height that makes a horizontal main arm hit rod=175 exactly:")
for side in ("L",):
    for sy in (-30, 30):
        sx = -76.4
        # |(sx,sy+/-50,46) - (-59.5, 0, recv_z)| = 175  ->  solve recv_z
        for sgn in (+1, -1):
            dx = sx - main_local[side][0]
            dy = (sy + sgn * 50) - main_local[side][1]
            rem = 175.0**2 - dx*dx - dy*dy
            if rem > 0:
                dz = math.sqrt(rem)
                print(f"   {side} y={sy:+d} sense={sgn:+d}: recv_z = 46 + {dz:.1f} = {46+dz:6.1f}"
                      f"   (firmware HOME_H={HOME_H})")
