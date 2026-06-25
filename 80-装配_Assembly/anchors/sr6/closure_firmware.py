"""Firmware-datum closure (the control law IS the absolute-placement datum).

Unlike closure.py (fabricated horizontal sweep) and closure_phys.py (free arm
angles that compensated for a mis-perceived shaft position), this module does NOT
perceive servo positions at all.  It LIFTS the exact home geometry from the
firmware IK (SR6-Alpha4_ESP32.ino, SetMainServo/SetPitchServo) and the receiver
home height from HOME_H, then checks the rod closes to 175 in world coordinates.

Firmware decode (law-of-cosines on beta):
  main : arm 50, rod 175, home pivot offset (vertical 162.48, horizontal 15.0)
  pitch: arm 75, rod 175, home pivot offset (vertical 162.48, horizontal 45.0, lateral 0)
  => servo shaft z = receiver_z - 162.48 = 208.48 - 162.48 = 46.0

Main embedding (derived, not fabricated):
  4 main servos bolt flat to the L/R side walls => shaft axis = world X,
  arm sweeps the Y-Z plane.  Shaft sits 15 mm (world Y) from the main pivot and
  162.48 mm (world Z) below it.  A HORIZONTAL arm (PDF p.24 calibration) pointing
  along +-Y lands the ball 65 mm out in Y; hypot(65, 162.48) = 175.00 exactly.
"""
import math
import numpy as np

HOME_H   = 208.48
VERT     = 162.48          # firmware 16248/100, shaft->pivot vertical at home
MAIN_H   = 15.0            # firmware 1500/100, main horizontal offset
PITCH_H  = 45.0            # firmware 4500/100, pitch horizontal offset
MAIN_ARM, MAIN_ROD   = 50.0, 175.0
PITCH_ARM, PITCH_ROD = 75.0, 175.0
SHAFT_Z  = HOME_H - VERT   # = 46.0

# Receiver main pivots (perceived axle centres, one per side, 2 links each).
MAIN_PIVOT = {"L": np.array([-59.5, 0.0, HOME_H]),
              "R": np.array([+59.5, 0.0, HOME_H])}


def main_legs():
    """Return the 4 main legs at home with firmware-exact geometry."""
    out = []
    for side in ("L", "R"):
        piv = MAIN_PIVOT[side]
        # the two links sit symmetrically +-MAIN_H in Y about the pivot's y
        for ysign, name in ((+1, "lower"), (-1, "upper")):
            shaft = np.array([piv[0], piv[1] + ysign * MAIN_H, SHAFT_Z])
            # horizontal arm points further out in Y (away from pivot) by ARM
            ball = shaft + np.array([0.0, ysign * MAIN_ARM, 0.0])
            rod = float(np.linalg.norm(piv - ball))
            arm_tilt = math.degrees(math.atan2(ball[2] - shaft[2],
                                               abs(ball[1] - shaft[1])))
            out.append((f"{side}-{name}", shaft, ball, piv, rod, arm_tilt))
    return out


if __name__ == "__main__":
    print("=== FIRMWARE-DATUM main closure (shaft z=%.1f, level receiver @%.2f) ===" % (SHAFT_Z, HOME_H))
    errs = []
    for name, shaft, ball, piv, rod, tilt in main_legs():
        errs.append(rod - MAIN_ROD)
        print(f"  {name:8s} shaft=({shaft[0]:6.1f},{shaft[1]:6.1f},{shaft[2]:5.1f})"
              f"  ball=({ball[0]:6.1f},{ball[1]:6.1f},{ball[2]:5.1f})"
              f"  rod={rod:7.3f} (tgt 175)  err={rod-MAIN_ROD:+.4f}  arm_tilt={tilt:+5.1f}deg")
    rms = float(np.sqrt(np.mean(np.square(errs))))
    print(f"  main closure RMS = {rms:.4f} mm  (arms horizontal, balls land at |y|=65 inside frame)")
