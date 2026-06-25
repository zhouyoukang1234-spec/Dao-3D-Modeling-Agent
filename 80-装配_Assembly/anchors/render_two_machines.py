# -*- coding: utf-8 -*-
"""One engine, two machines -- a single figure.

Both panels are drawn from poses produced by the SAME `uam.assembly.solve`:
left, the real SR6 6-leg home; right, a Stewart platform's solved home.  Nothing
here is hand-placed -- every point comes back from the solver.  This is the
visual companion to docs/CLOSURE_FINDINGS.md sec.9.
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
sys.path.insert(0, os.path.join(HERE, "sr6"))
sys.path.insert(0, os.path.join(HERE, "stewart"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from uam.assembly import solve  # noqa: E402

import assemble_full as sr6  # noqa: E402
from assemble import base_anchors, solve_pose  # noqa: E402
from synthetic import stewart_home  # noqa: E402


def _equal_aspect(ax, pts):
    pts = np.asarray(pts)
    c = pts.mean(0)
    r = np.max(np.abs(pts - c)) * 1.05 + 1e-9
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def draw_sr6(ax):
    parts, constraints, rods = sr6.build()
    solve(parts, constraints, verbose=False)
    allpts = []
    main_piv, pitch_piv = [], []
    for name, shaft, piv, arm, link in sr6.LEGS:
        rod = rods[name]
        s = rod.world_point("s")
        r = rod.world_point("r")
        is_pitch = name.startswith("pitch")
        # servo arm: shaft -> arm tip
        ax.plot(*zip(shaft, s), color="#2e8b57", lw=2.5)
        # rod: arm tip -> receiver pivot
        ax.plot(*zip(s, r), color="#c0392b", lw=2.0)
        ax.scatter(*shaft, color="#555", s=18)
        ax.scatter(*r, color="#1f3a93", s=22)
        allpts += [shaft, s, r]
        (pitch_piv if is_pitch else main_piv).append(r)
    # receiver carriage: ring through the 4 main pivots + bar to the 2 pitch pivots
    mp = np.array(main_piv)
    order = np.argsort(np.arctan2(mp[:, 2] - mp[:, 2].mean(), mp[:, 0] - mp[:, 0].mean()))
    loop = np.vstack([mp[order], mp[order][0]])
    ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="#e67e22", lw=2.5)
    for p in pitch_piv:
        ax.plot(*zip(mp.mean(0), p), color="#e67e22", lw=1.5, ls="--")
    ax.set_title("SR6 — real 6-leg, solved home\n(green=servo arm, red=link, "
                 "orange=receiver)", fontsize=9)
    _equal_aspect(ax, allpts)


def draw_stewart(ax):
    home = stewart_home()
    base = base_anchors()
    _, _, got = solve_pose(home, base, home)   # solved platform joints
    allpts = []
    bnames = list(base.keys())
    pnames = list(home.keys())
    B = np.array([base[k] for k in bnames])
    P = np.array([got[k] for k in pnames])
    # base ring, platform ring
    for ring, col in ((B, "#555"), (P, "#e67e22")):
        c = ring.mean(0)
        order = np.argsort(np.arctan2(ring[:, 1] - c[1], ring[:, 0] - c[0]))
        loop = np.vstack([ring[order], ring[order][0]])
        ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color=col, lw=2.5)
    # six legs: base_i -> platform_i (paired by index)
    for b, p in zip(B, P):
        ax.plot(*zip(b, p), color="#c0392b", lw=2.0)
        ax.scatter(*b, color="#555", s=18)
        ax.scatter(*p, color="#1f3a93", s=22)
        allpts += [b, p]
    ax.set_title("Stewart — synthetic 6-6, solved home\n(same solver, zero SR6 "
                 "geometry)", fontsize=9)
    _equal_aspect(ax, allpts)


def main():
    fig = plt.figure(figsize=(12, 5.6))
    axl = fig.add_subplot(1, 2, 1, projection="3d")
    axr = fig.add_subplot(1, 2, 2, projection="3d")
    draw_sr6(axl)
    draw_stewart(axr)
    for ax in (axl, axr):
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.view_init(elev=18, azim=-60)
    fig.suptitle("One general mate solver (uam.assembly.solve) — two unrelated "
                 "machines, both assembled from declared relationships",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(HERE, "..", "results", "two_machines.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130)
    print("wrote", os.path.abspath(out))


if __name__ == "__main__":
    main()
