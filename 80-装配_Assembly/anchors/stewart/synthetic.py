# -*- coding: utf-8 -*-
"""A SECOND anchor -- a synthetic, generic Gough-Stewart platform.

Why this file exists (道生一, 一生二).  Everything else in this repo validates
the architecture on ONE machine, the SR6.  A claim of *generality* that has only
ever been exercised on its origin example is not yet proven -- it is a sample of
size one.  This anchor shares ZERO geometry, constants, or firmware with the
SR6.  It is a plain 6-6 Stewart platform with arbitrary radii.  If the very same
`uam.kinematics` validator -- unchanged -- says something true and sharp here,
then the validator is mechanism-agnostic in fact, not merely in intention.

Two control laws drive the SAME platform, and the validator judges both against
the one yardstick a rigid body must obey (it preserves every pairwise distance):

  * EXACT law:   commanded_i = R @ home_i + t           (full rotation)
  * LINEAR law:  commanded_i = (I + [w]x) @ home_i + t  (first-order in angle)

Results that emerge (run this file):

  1. The EXACT law drifts by ~0 (machine epsilon) at ALL amplitudes, including
     large arbitrary roll/pitch/yaw.  => the validator introduces no spurious
     drift; SR6's non-zero rotation drift is therefore a real property of SR6's
     firmware linearisation, not an artefact of how we measure.

  2. The LINEAR law drifts quadratically in angle, matching the EXACT closed
     form
        drift(a,b) = sqrt(|d|^2 + |w x d|^2) - |d|        d = home_a - home_b
     to machine precision -- the SAME chord-vs-arc signature found on the SR6
     (whose form was sqrt(119^2 + (2d)^2) - 119), now on an unrelated mechanism.
     Its leading term is 1/2 * theta^2 * |k x d|^2 / |d| (the quadratic growth).

So the lesson "inter-point distance drift measures the linearisation ORDER of a
control law, independent of the mechanism" is established on two machines, not
one.  That is the general architecture showing through the instance.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from uam.kinematics import consistency, rigid_transform  # noqa: E402


def skew(w):
    wx, wy, wz = w
    return np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]], float)


def stewart_home(r_platform=60.0, height=150.0, spread_deg=15.0):
    """Six platform attachment points in the WORLD frame at home pose.

    The classic 6-6 layout: three pairs of anchors spaced around a circle, each
    pair split by +/- `spread_deg`.  Exact numbers are irrelevant to the proof;
    all that matters is that the six points are non-degenerate.  Returns a dict
    name -> world point, i.e. an L1/2 datum for a machine that is NOT the SR6.
    """
    home = {}
    for i in range(3):
        base = 120.0 * i
        for sign, tag in ((-1, "a"), (+1, "b")):
            ang = np.deg2rad(base + sign * spread_deg)
            home[f"P{i}{tag}"] = np.array(
                [r_platform * np.cos(ang), r_platform * np.sin(ang), height], float
            )
    return home


def exact_law(home, trans, rpy):
    R, t = rigid_transform(trans, rpy)
    return {k: R @ v + t for k, v in home.items()}


def linear_law(home, trans, rpy):
    """First-order: R ~ I + [w]x.  A deliberately linearised actuator model."""
    Rlin = np.eye(3) + skew(rpy)
    t = np.asarray(trans, float)
    return {k: Rlin @ v + t for k, v in home.items()}


def closed_form_drift(home, rpy):
    """EXACT linear-law drift per pair: sqrt(|d|^2 + |w x d|^2) - |d|.

    This is the full distance |(I+[w]x) d| - |d| (the I+[w]x cross term d.(wxd)
    vanishes since w x d _|_ d), NOT a truncation.  Its leading term is the
    familiar quadratic 1/2 theta^2 |k x d|^2 / |d|, but we compare against the
    exact value so the agreement is to machine precision -- 反者道之動: the
    earlier mismatch exposed that the *formula* was approximate, not the data.
    """
    w = np.asarray(rpy, float)
    keys = list(home)
    out = {}
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            d = home[keys[i]] - home[keys[j]]
            out[(keys[i], keys[j])] = np.sqrt(np.dot(d, d) + np.dot(np.cross(w, d), np.cross(w, d))) - np.linalg.norm(d)
    return out


def run():
    home = stewart_home()
    print("synthetic Gough-Stewart platform -- a SECOND anchor (no SR6 geometry)")
    print(f"  {len(home)} platform anchors, world frame, at home pose\n")

    # 1. EXACT law: rigid by construction -> validator must report ~0 everywhere,
    #    including large arbitrary rotation.  This is the control on the control.
    print("EXACT control law (full rotation), across the workspace:")
    print("   input (mm / deg)                     max_resid     max_drift")
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(8):
        trans = rng.uniform(-40, 40, 3)
        rpy = np.deg2rad(rng.uniform(-35, 35, 3))
        cmd = exact_law(home, trans, rpy)
        R, t = rigid_transform(trans, rpy)
        res = consistency(home, cmd, R, t)
        worst = max(worst, res.max_drift, res.max_resid)
        td = f"t=({trans[0]:+5.0f},{trans[1]:+5.0f},{trans[2]:+5.0f}) rpy=({np.rad2deg(rpy[0]):+5.1f},{np.rad2deg(rpy[1]):+5.1f},{np.rad2deg(rpy[2]):+5.1f})"
        print(f"   {td}   {res.max_resid:.2e}    {res.max_drift:.2e}")
    print(f"   -> worst over all = {worst:.2e} mm  (machine epsilon: exact rigid)\n")
    assert worst < 1e-9, "exact law must be rigid to machine precision"

    # 2. LINEAR law: first-order -> drift grows quadratically and matches the
    #    closed form to machine precision.  Same signature as the SR6 firmware.
    print("LINEAR (first-order) control law -- pure roll about x, growing angle:")
    print("   angle(deg)   measured max_drift   closed-form max_drift   |diff|")
    for deg in (2, 5, 10, 20, 30):
        rpy = np.array([np.deg2rad(deg), 0, 0])
        trans = np.zeros(3)
        cmd = linear_law(home, trans, rpy)
        R, t = rigid_transform(trans, rpy)
        res = consistency(home, cmd, R, t)
        cf = closed_form_drift(home, rpy)
        meas_max = max(abs(v) for v in res.dist_drift.values())
        cf_max = max(abs(v) for v in cf.values())
        print(f"     {deg:5.0f}        {meas_max:10.5f}          {cf_max:10.5f}         {abs(meas_max - cf_max):.2e}")
    # verify pair-by-pair agreement at one angle
    rpy = np.array([np.deg2rad(20), 0, 0])
    cmd = linear_law(home, np.zeros(3), rpy)
    R, t = rigid_transform(np.zeros(3), rpy)
    res = consistency(home, cmd, R, t)
    cf = closed_form_drift(home, rpy)
    err = max(abs(res.dist_drift[k] - cf[k]) for k in cf)
    print(f"\n   per-pair max |measured - closed form| at 20 deg = {err:.2e} mm")
    assert err < 1e-6, "linear-law drift must match closed form"

    print("\nBOTH conclusions hold on a mechanism with no SR6 DNA:")
    print("  * a truly rigid control law -> zero drift (validator is honest)")
    print("  * a linearised one -> quadratic, closed-form drift (the universal signature)")


if __name__ == "__main__":
    run()
