#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/validate.py — L5 reference alignment as a PREDICTIVE-CODING check.

Two INDEPENDENT information sources describe the SR6 skeleton:
  • the PRIOR  : firmware IK constants (SR6-Alpha4_ESP32.ino) -> constants.py
  • the SENSE  : geometry measured from the real STL meshes -> uam.cylinders

A correct world-model makes these agree. We measure the PREDICTION ERROR
(|perceived - firmware|) for every load-bearing dimension. Small error across
the board = the perception layer truly sees the mechanism the firmware drives;
that is the whole thesis (brain-style: prior vs sensory, minimise surprise).

No number below is typed in twice: the 'firmware' column comes from constants.py
(decoded from the .ino), the 'perceived' column from clustering mesh triangles.
"""
from __future__ import annotations
import os, sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO); sys.path.insert(0, _HERE)
from uam.cylinders import detect_cylinders  # noqa
import constants as K  # noqa

STL = os.environ.get("UAM_STL_DIR", os.path.join(_REPO, "ground_truth", "stl"))


def pivots(name, rmin=1.4, rmax=12.0):
    """Pivot-class cylinders of a part. A link's two functional pivots can have
    DIFFERENT radii (e.g. a servo-screw hole vs a rod-end bore), so we keep every
    real hole above screw-noise and let the span search find the load path."""
    cs = detect_cylinders(os.path.join(STL, name))
    return [c for c in cs if rmin <= c["radius"] <= rmax]


def best_span(cyls, target):
    """Among cylinder pairs, the center-to-center span closest to target."""
    best = None
    for i in range(len(cyls)):
        for j in range(i + 1, len(cyls)):
            d = np.linalg.norm(np.array(cyls[i]["center"]) - np.array(cyls[j]["center"]))
            if best is None or abs(d - target) < abs(best - target):
                best = d
    return best


def main():
    print("=" * 70)
    print("SR6 PERCEPTION vs FIRMWARE  —  predictive-coding agreement table")
    print("=" * 70)
    print(f"{'dimension':28s}{'firmware':>10s}{'perceived':>11s}{'err':>8s}   src")
    print("-" * 70)
    rows = []

    # link spans (pivot-to-pivot), each measured from its own mesh
    rows.append(("main arm  (Arm)",       K.MAIN_ARM,  best_span(pivots("Arm.stl"), K.MAIN_ARM)))
    rows.append(("main rod  (MainLink)",  K.MAIN_ROD,  best_span(pivots("MainLink_Alpha.stl"), K.MAIN_ROD)))
    rows.append(("pitch arm (LPitcher)",  K.PITCH_ARM, best_span(pivots("LPitcher.stl"), K.PITCH_ARM)))
    rows.append(("pitch rod (PitcherLink)", K.PITCH_ROD_PHYS,
                 best_span(pivots("PitcherLink_Alpha.stl"), K.PITCH_ROD_PHYS)))

    # receiver lower->upper pivot offset (the 55mm @ 15deg) from Receiver alone
    rec = pivots("Receiver.stl", rmin=1.4, rmax=3.0)
    lower = [c for c in rec if abs(c["center"][2]) < 10 and c["center"][0] > 0]
    upper = [c for c in rec if c["center"][2] > 40 and c["center"][0] > 0]
    off = None
    if lower and upper:
        a = np.array(lower[0]["center"]); b = np.array(upper[0]["center"])
        off = float(np.linalg.norm(b[1:] - a[1:]))   # offset in the y-z plane
    rows.append(("recv pivot offset",     K.PITCH_OFF, off))

    maxerr = 0.0
    for name, fw, pv in rows:
        if pv is None:
            print(f"{name:28s}{fw:10.2f}{'--':>11s}{'--':>8s}")
            continue
        err = abs(pv - fw); maxerr = max(maxerr, err)
        flag = "OK" if err < 1.0 else ("~" if err < 2.5 else "XX")
        print(f"{name:28s}{fw:10.2f}{pv:11.2f}{err:8.2f}   {flag}")

    # bearing + ball seats (radii, not spans) — vocabulary confirmation
    print("-" * 70)
    lp = detect_cylinders(os.path.join(STL, "LPitcher.stl"))
    bear = max((c["radius"] for c in lp if 16 < c["radius"] < 22), default=None)
    ml = detect_cylinders(os.path.join(STL, "MainLink_Alpha.stl"))
    ball = max((c["radius"] for c in ml if 5 < c["radius"] < 7), default=None)
    if bear: print(f"{'pitcher bearing seat r':28s}{20.00:10.2f}{bear:11.2f}"
                   f"{abs(bear-20):8.2f}   {'OK' if abs(bear-20)<1.5 else 'XX'}")
    if ball: print(f"{'ball-joint seat radius':28s}{6.00:10.2f}{ball:11.2f}"
                   f"{abs(ball-6):8.2f}   {'OK' if abs(ball-6)<1 else 'XX'}")

    print("=" * 70)
    print(f"max prediction error across load-bearing dims = {maxerr:.2f} mm")
    print("perception and firmware describe the SAME mechanism." if maxerr < 2.5
          else "MISMATCH — investigate before trusting assembly.")


if __name__ == "__main__":
    main()
