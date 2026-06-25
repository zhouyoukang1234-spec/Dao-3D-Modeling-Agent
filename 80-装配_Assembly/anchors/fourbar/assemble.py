# -*- coding: utf-8 -*-
"""Onboard a NEW mechanism with zero code: a hand-written planar four-bar.

Unlike sr6.spec.json / stewart.spec.json (which were *serialized* from existing
build() functions), fourbar.spec.json was typed by hand as the FIRST contact a
human/agent would have with the data layer.  Nothing about a four-bar exists
anywhere in this repo's code.  If this closes, "onboard a machine = author a
spec" is proven end-to-end — the loader+solver never knew four-bars existed.

Closure here means more than small constraint residuals: a linkage is correct
only if every rigid link keeps its length and the loop stays planar.  We check
all three, derived from the SOLVED poses, with no fitted DOF.
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))

from uam.spec import load_file  # noqa: E402

LINKS = {  # rigid bodies and their authored pivot-to-pivot length
    "ground link (A-B)": (("ground", "A"), ("ground", "B"), 120.0),
    "crank":             (("crank", "o"),  ("crank", "p"),    40.0),
    "coupler":           (("coupler", "o"),("coupler", "p"),  70.0),
    "rocker":            (("rocker", "o"), ("rocker", "p"),   70.0),
}


def main():
    spec = load_file(os.path.join(HERE, "fourbar.spec.json"))
    res, rms = spec.solve(verbose=False)

    print("=== hand-written four-bar, assembled by the generic loader+solver ===\n")
    worst_con = max(float(np.linalg.norm(np.atleast_1d(c.residual())))
                    for c in spec.constraints)
    print(f"  solver RMS residual          = {rms:.3e}")
    print(f"  worst constraint residual    = {worst_con:.3e}\n")

    print("  rigid-link length preservation (authored vs solved):")
    worst_len = 0.0
    for label, (a, b, L) in LINKS.items():
        pa = spec.world_point(*a)
        pb = spec.world_point(*b)
        got = float(np.linalg.norm(pa - pb))
        worst_len = max(worst_len, abs(got - L))
        print(f"    {label:18s} authored={L:6.2f}  solved={got:8.4f}  err={got-L:+.2e}")

    # planarity: every solved pivot must stay at z = 0
    zs = [abs(spec.world_point(p, c)[2])
          for p in ("crank", "coupler", "rocker") for c in ("o", "p")]
    worst_z = max(zs)
    print(f"\n  planarity (max |z| over all pivots) = {worst_z:.2e}")

    # report the solved input angle so the closed configuration is legible
    cp = spec.world_point("crank", "p")
    theta = np.degrees(np.arctan2(cp[1], cp[0]))
    print(f"  solved crank angle           = {theta:.2f} deg (1-DOF mechanism; "
          "trf settled on the closed branch near the open seed)")

    ok = worst_con < 1e-6 and worst_len < 1e-6 and worst_z < 1e-6
    print(f"\n  => {'CLOSED' if ok else 'OPEN'}: loop closes, all links rigid, motion planar"
          "\n     — a machine the code had never seen, onboarded as data alone.")
    assert ok, "four-bar failed to close"


if __name__ == "__main__":
    main()
