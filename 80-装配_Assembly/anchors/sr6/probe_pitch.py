"""Perceive the pitch bellcrank geometry from L/RPitcher + PitcherLink + Receiver.

Goal: find, in each part's own frame, the through-holes that define
  (a) the bellcrank PIVOT axis (where the L-lever turns on the receiver),
  (b) the bellcrank INPUT hole (where the pitch rod attaches),
  (c) the bellcrank OUTPUT hole (how it couples to the platform),
  (d) the PitcherLink rod end-to-end length (should be ~175 bearing-centres).
We only REPORT geometry; no world placement is invented here.
"""
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
from uam.cylinders import detect_cylinders            # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")


def dump(name, rmin=1.0, rmax=9.0):
    cyl = detect_cylinders(os.path.join(STL, name), rmin=rmin, rmax=rmax)
    holes = [c for c in cyl if c["kind"] == "hole"]
    print(f"\n=== {name}: {len(holes)} holes ===")
    for c in sorted(holes, key=lambda c: (round(c['radius'], 1), c['center'][2])):
        ax = c["axis"]; ce = c["center"]
        print(f"  r={c['radius']:5.2f}  c=({ce[0]:7.2f},{ce[1]:7.2f},{ce[2]:7.2f})"
              f"  axis=({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f})  len={c.get('length',0):5.1f}")
    return holes


if __name__ == "__main__":
    for n in ("LPitcher.stl", "RPitcher.stl", "PitcherLink_Alpha.stl", "BearingPitcherLink.stl"):
        dump(n)
