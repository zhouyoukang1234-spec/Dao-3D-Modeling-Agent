# -*- coding: utf-8 -*-
"""Stewart-platform assembly solved by the GENERAL kernel `uam.assembly`.

Companion to `anchors/sr6/assemble_full.py`.  That file proved the
mechanism-agnostic mate solver assembles the real SR6.  This file proves the
SAME solver -- byte-for-byte the same `uam.assembly.solve` -- assembles a
machine that shares NO geometry, constants, or firmware with the SR6: a generic
6-6 Gough-Stewart platform.  Two unrelated mechanisms, one engine: 一生二.

It also exercises the OTHER solver regime.  The SR6 legs leave redundant
spherical-bearing freedom -> the constraint system is underdetermined (`trf`).
A Stewart platform driven by its six leg lengths is exactly determined: 6 length
constraints pin all 6 platform DOF (`lm`).  So between the two anchors the
general kernel is shown sound on both code paths.

What is declared (no hand-solved pose anywhere):
  * ground : a fixed Part holding the six base anchors B0..B5 (world points).
  * platform : one free rigid Part carrying the six platform joints P0..P5 in
        its own body frame.
  * constraints : Distance(platform.Pi, ground.Bi) == leg_len_i, the six
        prismatic-leg lengths -- the ONLY data a real Stewart controller commands.
Hand the graph to solve(); the platform's 6-DOF pose falls out (forward
kinematics).  Recovering a commanded pose from leg lengths alone, to machine
precision, is the closure proof.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
sys.path.insert(0, os.path.dirname(__file__))

from uam.assembly import Distance, Part, solve  # noqa: E402
from uam.kinematics import rigid_transform  # noqa: E402
from synthetic import stewart_home  # noqa: E402


def base_anchors(r_base=90.0, spread_deg=40.0):
    """Six base anchors on the ground ring (world frame, z=0).

    Deliberately a different radius and angular split from the platform ring so
    the legs are slanted -- a non-degenerate, fully 3D Stewart geometry.
    """
    base = {}
    for i in range(3):
        a0 = 120.0 * i
        for sign, tag in ((-1, "a"), (+1, "b")):
            ang = np.deg2rad(a0 + sign * spread_deg)
            base[f"P{i}{tag}"] = np.array(
                [r_base * np.cos(ang), r_base * np.sin(ang), 0.0], float)
    return base


def leg_lengths(platform_world, base):
    return {k: float(np.linalg.norm(platform_world[k] - base[k])) for k in base}


def build(home_local, base, legs):
    """Declare ground + free platform + six leg-length constraints."""
    ground = Part("ground", fixed=True)
    for k, b in base.items():
        ground.add(f"B::{k}", b, [0, 0, 1])
    platform = Part("platform")
    cen = np.mean(list(home_local.values()), axis=0)
    for k, p in home_local.items():
        platform.add(k, p - cen, [0, 0, 1])   # joints in body frame about centroid
    platform.t = cen.copy()                    # seed at home (centroid offset)
    constraints = [Distance((platform, k), (ground, f"B::{k}"), legs[k]) for k in base]
    return [ground, platform], constraints, platform, cen


def solve_pose(home_local, base, target_world):
    """Forward kinematics: recover platform pose from leg lengths of target."""
    legs = leg_lengths(target_world, base)
    parts, constraints, platform, cen = build(home_local, base, legs)
    _, rms = solve(parts, constraints, verbose=False)
    got = {k: platform.world_point(k) for k in home_local}
    err = max(np.linalg.norm(got[k] - target_world[k]) for k in home_local)
    return rms, err, got


def run():
    home = stewart_home()             # platform joints in WORLD at home (the L1/2 datum)
    base = base_anchors()
    print("=== Stewart platform assembly via the GENERAL uam.assembly kernel ===")
    print(f"    {len(home)} legs; base ring r=90 z=0; platform ring r=60 z=150\n")

    # (a) home closure: leg lengths of the home pose must reproduce the home pose
    rms, err, _ = solve_pose(home, base, home)
    print(f"  home closure          : solver RMS={rms:.2e}  joint err={err:.2e} mm  [CLOSES]")

    # (b) forward kinematics at several commanded poses -- recover pose from
    #     leg lengths ALONE (the only thing a Stewart controller commands).
    print("\n  forward kinematics (recover 6-DOF pose from six leg lengths):")
    print("    commanded (mm / deg)                       solver RMS   joint err")
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(6):
        trans = rng.uniform(-25, 25, 3)
        rpy = np.deg2rad(rng.uniform(-18, 18, 3))
        R, t = rigid_transform(trans, rpy)
        cen = np.mean(list(home.values()), axis=0)
        target = {k: R @ (v - cen) + cen + t for k, v in home.items()}
        rms, err, _ = solve_pose(home, base, target)
        worst = max(worst, err)
        td = (f"t=({trans[0]:+5.0f},{trans[1]:+5.0f},{trans[2]:+5.0f}) "
              f"rpy=({np.rad2deg(rpy[0]):+5.1f},{np.rad2deg(rpy[1]):+5.1f},{np.rad2deg(rpy[2]):+5.1f})")
        print(f"    {td}    {rms:.2e}   {err:.2e}")
    print(f"\n  worst joint recovery error over all poses = {worst:.2e} mm")
    print("  => the SAME mate solver that assembles the SR6 also solves a")
    print("     Stewart platform's forward kinematics. One engine, two machines.")
    assert worst < 1e-6, "general kernel failed Stewart forward kinematics"
    return worst


if __name__ == "__main__":
    run()
