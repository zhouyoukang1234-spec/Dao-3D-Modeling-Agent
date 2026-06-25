# -*- coding: utf-8 -*-
"""Visual proof of the L5 finding: the firmware main IK is an EXACT rigid solver
for translation and a first-order LINEARISATION for rotation.

Left  panel: top-down (world X-Y) view of the two main receiver pivots as yaw is
             commanded.  The TRUE rigid body sweeps them along an arc of radius
             59.5 (|L-R| stays 119); the FIRMWARE holds x=+-59.5 fixed and only
             offsets y -- the chord-vs-arc gap is the model error, visibly zero
             near home and growing with angle.
Right panel: inter-pivot distance drift vs commanded yaw, measured (dots) on top
             of the closed-form chord-minus-arc curve (line) -- they coincide.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
import rigid_consistency as rc  # noqa: E402

fig, (axL, axR) = plt.subplots(1, 2, figsize=(12, 5.2))

# ---- left: chord (firmware) vs arc (rigid) in the X-Y plane ----
axL.set_title("main pivots under commanded yaw  (top-down X-Y)")
rolls = [0, 1000, 2000, 3000]
colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(rolls)))
for roll, col in zip(rolls, colors):
    cmd, _ = rc.control_law({"roll": roll})
    R, t = rc.intended({"roll": roll})
    fw = np.array([cmd["main_L"], cmd["main_R"]])
    rg = np.array([R @ rc.HOME["main_L"] + t, R @ rc.HOME["main_R"] + t])
    deg = np.degrees(np.arcsin((roll / 100.0) / rc.HALF_X))
    axL.plot(rg[:, 0], rg[:, 1], "o-", color=col, lw=1.4, ms=6,
             label="rigid yaw %.1f deg" % deg)
    axL.plot(fw[:, 0], fw[:, 1], "x--", color=col, lw=1.0, ms=8,
             label="firmware  %.1f deg" % deg)
# true arc the rigid pivots ride on
th = np.linspace(-0.6, 0.6, 100)
axL.plot(rc.HALF_X * np.cos(th), rc.HALF_X * np.sin(th), ":", color="0.6", lw=1)
axL.plot(-rc.HALF_X * np.cos(th), -rc.HALF_X * np.sin(th), ":", color="0.6", lw=1)
axL.axvline(rc.HALF_X, color="0.85", lw=0.8)
axL.axvline(-rc.HALF_X, color="0.85", lw=0.8)
axL.set_xlabel("X (mm)")
axL.set_ylabel("Y (mm)  [stroke axis]")
axL.set_aspect("equal")
axL.legend(fontsize=7, ncol=2, loc="upper center")
axL.grid(alpha=0.3)

# ---- right: distance drift vs yaw, measured vs closed form ----
axR.set_title("rigidity error: |L-R| distance drift vs commanded yaw")
rr = np.arange(0, 3201, 100)
meas, degs = [], []
for roll in rr:
    res, _ = rc.run({"roll": int(roll)})
    meas.append(res.max_drift)
    degs.append(np.degrees(np.arcsin((roll / 100.0) / rc.HALF_X)))
degs = np.array(degs)
d = rr / 100.0
pred = np.hypot(119.0, 2 * d) - 119.0
axR.plot(degs, pred, "-", color="crimson", lw=2,
         label=r"closed form  $\sqrt{119^2+(2d)^2}-119$")
axR.plot(degs, meas, "o", color="navy", ms=4, label="measured (firmware vs rigid)")
axR.axhline(0, color="0.7", lw=0.8)
axR.set_xlabel("commanded yaw (deg)")
axR.set_ylabel("distance drift (mm)")
axR.legend(fontsize=9)
axR.grid(alpha=0.3)
axR.text(2, 11, "translation DOF: drift = 0 exactly\n(rigid solver, all amplitudes)",
         fontsize=8, color="green",
         bbox=dict(boxstyle="round", fc="honeydew", ec="green", alpha=0.9))

out = os.path.join(os.path.dirname(__file__), "rigid_consistency.png")
fig.tight_layout()
fig.savefig(out, dpi=130)
print("wrote", out)
