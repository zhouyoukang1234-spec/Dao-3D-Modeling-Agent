# -*- coding: utf-8 -*-
"""SR6 anchor: drive the REAL firmware main IK across the full workspace and
measure, with zero fitted DOF, how far the independently-commanded receiver
pivots drift from one rigid body.  Thin instance of uam.kinematics.

The firmware (SR6-Alpha4_ESP32.ino, lines 765-768) commands the four main servos
INDEPENDENTLY:
    out1 = SetMainServo(16248 - fwd, 1500 + thrust + roll)   # lower  left
    out2 = SetMainServo(16248 - fwd, 1500 - thrust - roll)   # upper  left
    out5 = SetMainServo(16248 - fwd, 1500 - thrust + roll)   # upper  right
    out6 = SetMainServo(16248 - fwd, 1500 + thrust - roll)   # lower  right
SetMainServo(X, Y) solves the exact planar two-link (arm 50, rod 175) IK to a
target (X, Y) given in 1/100 mm in the servo's own vertical plane:  X is the
vertical reach above the shaft, Y the horizontal offset.  Because the IK is
exact, every rod is ALWAYS 175 mm -- that is true by construction and tells us
nothing.  The real question is whether the FOUR commanded targets stay consistent
with ONE rigid receiver.  Each servo plane is the physical world Y-Z plane at its
shaft's x (=+-59.5), an authority-backed fact from the home datum -- so the world
position of every commanded pivot is reconstructed WITHOUT fabricating anything.

Mapping of the firmware inputs onto rigid receiver DOF (derived, not tuned):
    thrust ->  +Y stroke    (both main points shift together in Y)
    fwd    ->  -Z heave     (both main points shift together in Z)
    roll   ->  yaw about the vertical axis through the receiver centre
               (the two points move oppositely in Y -- a rotation the firmware
                renders as a pure linear Y offset, i.e. its first-order form)
    side   ->  no effect on the main servos (sanity check: must stay 0)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from uam.kinematics import consistency, rot_z  # noqa: E402

SHAFT_Z = 46.0
HALF_X = 59.5
SHAFT_Y = 15.0
CENTER = np.array([0.0, 0.0, 208.48])      # receiver centre at home (firmware HOME_H)

# four main shafts (world), matching firmware leg labels
SHAFT = {
    "LL": np.array([-HALF_X, +SHAFT_Y, SHAFT_Z]),
    "UL": np.array([-HALF_X, -SHAFT_Y, SHAFT_Z]),
    "UR": np.array([+HALF_X, -SHAFT_Y, SHAFT_Z]),
    "LR": np.array([+HALF_X, +SHAFT_Y, SHAFT_Z]),
}

# the two rigid receiver attachment points at home (the datum, Kabsch-confirmed)
HOME = {
    "main_L": np.array([-HALF_X, 0.0, 208.48]),
    "main_R": np.array([+HALF_X, 0.0, 208.48]),
}


def _target(fwd, thrust, roll):
    return {
        "LL": (16248 - fwd, 1500 + thrust + roll),
        "UL": (16248 - fwd, 1500 - thrust - roll),
        "UR": (16248 - fwd, 1500 - thrust + roll),
        "LR": (16248 - fwd, 1500 + thrust - roll),
    }


def _pivot_world(leg, X, Y):
    s = SHAFT[leg]
    vert = X / 100.0
    horiz = Y / 100.0
    py = s[1] - np.sign(s[1]) * horiz
    return np.array([s[0], py, SHAFT_Z + vert])


def control_law(u):
    """Firmware-commanded world positions of the two main receiver points.

    Each side has two servos that must agree on their shared point; we return
    their midpoint as the commanded point and expose the disagreement via gap()."""
    tg = _target(u.get("fwd", 0), u.get("thrust", 0), u.get("roll", 0))
    P = {leg: _pivot_world(leg, *tg[leg]) for leg in SHAFT}
    return {
        "main_L": 0.5 * (P["LL"] + P["UL"]),
        "main_R": 0.5 * (P["UR"] + P["LR"]),
    }, P


def gap(P):
    """Per-side servo disagreement (mm): a pure rigidity residual on its own."""
    return max(np.linalg.norm(P["LL"] - P["UL"]),
               np.linalg.norm(P["UR"] - P["LR"]))


def intended(u):
    """The rigid (R, t) the input is meant to produce, about the receiver centre.

    yaw angle is fixed by the firmware's own commanded tangential offset at the
    pivot radius (roll/100 mm at radius HALF_X) -- derived, not hand-tuned."""
    roll = u.get("roll", 0)
    theta = np.arcsin(np.clip((roll / 100.0) / HALF_X, -1.0, 1.0))
    R = rot_z(theta)
    extra = np.array([0.0, -u.get("thrust", 0) / 100.0, -u.get("fwd", 0) / 100.0])
    t = CENTER - R @ CENTER + extra
    return R, t


def run(u):
    cmd, P = control_law(u)
    R, t = intended(u)
    res = consistency(HOME, cmd, R, t)
    return res, gap(P)


def banner(s):
    print("\n" + "=" * 74 + "\n" + s + "\n" + "=" * 74)


if __name__ == "__main__":
    banner("SR6 MAIN SUBSYSTEM -- firmware command vs ONE rigid body (no fitted DOF)")
    print(" resid = ||firmware pivot - rigid-body pivot||;  drift = |LR distance - 119|;"
          "\n gap = the two paired servos' disagreement on their shared point.\n")

    print(" home check:")
    res, g = run({})
    print("   resid_rms=%.4f  max_resid=%.4f  dist_drift=%.4f  gap=%.4f mm"
          % (res.rms, res.max_resid, res.max_drift, g))

    banner("pure translation  (thrust=Y stroke, fwd=Z heave)  -- expect EXACT")
    for u in [{"thrust": 6000}, {"thrust": -6000}, {"fwd": 3000}, {"fwd": -3000},
              {"thrust": 6000, "fwd": 3000}]:
        res, g = run(u)
        print("   %-26s resid_max=%.2e  drift=%.2e  gap=%.2e"
              % (str(u), res.max_resid, res.max_drift, g))

    banner("pure roll (yaw)  -- firmware linearises a rotation; residual is honest")
    print("   roll[0.01] | yaw deg | resid_max (mm) | dist_drift (mm) | gap")
    for roll in (250, 500, 1000, 2000, 3000):
        res, g = run({"roll": roll})
        theta = np.degrees(np.arcsin((roll / 100.0) / HALF_X))
        print("     %5d    | %6.2f  |   %8.4f    |   %+8.4f     | %.1e"
              % (roll, theta, res.max_resid, res.max_drift, g))

    banner("COUPLED  roll + translation  -- does the error stay additive?")
    print("   input                              resid_max   drift      gap")
    for u in [{"roll": 1000, "thrust": 6000},
              {"roll": 1000, "fwd": 3000},
              {"roll": 3000, "thrust": 6000},
              {"roll": 3000, "thrust": 6000, "fwd": 3000},
              {"roll": 2000, "thrust": -4000, "fwd": -2000}]:
        res, g = run(u)
        print("   %-34s %8.4f  %+8.4f  %.1e"
              % (str(u), res.max_resid, res.max_drift, g))

    banner("isolated roll error vs pure-yaw prediction  (is drift exactly geometric?)")
    print("   the receiver radius is HALF_X=59.5; a true yaw keeps |LR|=119 so ALL")
    print("   distance drift is the firmware's linearisation.  predicted drift for a")
    print("   firmware-style linear offset d=roll/100 is sqrt(119^2+(2d)^2)-119:")
    for roll in (1000, 3000):
        d = roll / 100.0
        pred = np.hypot(119.0, 2 * d) - 119.0
        res, g = run({"roll": roll})
        print("     roll=%-5d  measured drift=%+.4f   geometric prediction=%+.4f"
              % (roll, res.max_drift, pred))
