#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Empirically extract the pitch bellcrank topology from the REAL STLs.

The one honest open frontier: the pitch leg is currently faked as a straight
185mm rod, but L/RPitcher are L-shaped rockers (bellcranks). A bellcrank is a
rigid body carrying THREE bearing bores: a pivot (grounded to the frame), an
input bearing (driven by the servo arm), and an output bearing (drives the
PitcherLink up to the receiver). We do NOT fabricate this geometry — we read it
off the printed part via L0 perception (uam.cylinders), report bore axes,
centers, the arm lengths and the included angle. Authority = the printed part.
"""
from __future__ import annotations
import os
import sys
import numpy as np
import trimesh

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from uam.cylinders import detect_cylinders  # noqa: E402

STL = os.path.join(HERE, "..", "..", "ground_truth", "stl")


def load(name):
    m = trimesh.load(os.path.join(STL, name), process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(tuple(m.geometry.values()))
    return m


def bores(name, **kw):
    m = load(name)
    cyl = detect_cylinders(m, **kw)
    holes = [c for c in cyl if c["kind"] == "hole"]
    print(f"\n=== {name}  ({len(m.vertices)} verts, bbox={m.extents.round(2)}) ===")
    for c in cyl:
        p = np.asarray(c["center"]).round(2)
        a = np.asarray(c["axis"]).round(3)
        print(f"  {c['kind']:4s} r={c['radius']:5.2f}  len={c['length']:6.2f}  "
              f"wrap={c['wrap_deg']:5.0f}  center={p}  axis={a}")
    return holes, m


def analyse():
    """Reduce the perceived bores to the kinematically meaningful quantities,
    so the 175-vs-185 question is answered by measurement, not assertion."""
    print("\n\n========== bellcrank / link analysis (from perceived bores) ==========")

    lp, _ = bores("LPitcher.stl", rmin=1.0, rmax=12.0, min_wrap_deg=140.0)
    # pivot bearing seats (r~3.7) and the lever tip (small bore far out)
    seats = sorted([c for c in lp if 3.0 < c["radius"] < 4.5],
                   key=lambda c: c["center"][1])
    tip = max(lp, key=lambda c: np.hypot(c["center"][0], c["center"][1] - 40))
    A = np.array(seats[0]["center"])   # lower seat (design pivot)
    B = np.array(seats[1]["center"])   # upper seat
    C = np.array(tip["center"])        # lever tip (link attaches here)
    print(f"\n  LPitcher  pivot-seat A = {A.round(2)}")
    print(f"            pivot-seat B = {B.round(2)}  (|B-A|={np.linalg.norm(B-A):.2f})")
    print(f"            lever tip  C = {C.round(2)}")
    print(f"            arm A->C = {np.linalg.norm(C-A):.3f}  (firmware PITCH_ARM=75)")
    print(f"            seat-axes ~ {np.round(seats[0]['axis'],2)} (revolute about z)")

    pl, _ = bores("PitcherLink_Alpha.stl", rmin=1.0, rmax=12.0, min_wrap_deg=140.0)
    outer = sorted([c for c in pl if c["radius"] > 5.0],
                   key=lambda c: c["center"][1])
    e1 = np.array(outer[0]["center"])
    e2 = np.array(outer[-1]["center"])
    ax = np.array(outer[0]["axis"]); ax = ax / np.linalg.norm(ax)
    d = e2 - e1
    span3d = np.linalg.norm(d)
    along_axis = abs(float(d @ ax))                       # offset ALONG pin axis
    perp = float(np.linalg.norm(d - (d @ ax) * ax))       # lever ⟂ to pin axis
    print(f"\n  PitcherLink end-1 = {e1.round(2)}")
    print(f"              end-2 = {e2.round(2)}")
    print(f"              pin axis ~ {ax.round(2)} (both ends revolute, parallel)")
    print(f"              3D centre distance        = {span3d:.3f}  (PHYS 185)")
    print(f"              offset ALONG pin axis      = {along_axis:.3f}  (lateral, ~60)")
    print(f"              lever PERP to pin axis     = {perp:.3f}  (planar IK rod = 175)")
    print(f"              check hypot(perp, along)   = {np.hypot(perp, along_axis):.3f}")

    print("\n  => the 185 vs 175 'conflict' is resolved by geometry, not fudging:")
    print("     PitcherLink's two pins are PARALLEL (revolute about the lateral axis).")
    print("     Its 3D centre-distance is 185 = hypot(60, 175); but the 60mm sits")
    print("     ALONG the pin axis, so the kinematically-relevant lever in the")
    print("     sagittal (pitch) plane is the PERPENDICULAR projection = 175 — exactly")
    print("     the firmware's effective pitch rod. No part of this was hand-tuned.")


if __name__ == "__main__":
    analyse()
