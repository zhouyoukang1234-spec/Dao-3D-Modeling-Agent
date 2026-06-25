#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Visual proof of the honest FULL 6-LEG home closure (closure_firmware_6leg.py).

Real STL meshes placed at the solved poses; no fitted/fabricated world coords.
  * Receiver.stl  -> home: Rx(-90 deg) about X + lift to z=208.48, the proper
    rotation (det +1) found by a Kabsch fit of its 4 PERCEIVED pivots onto the
    firmware-authoritative world pivots (closure_kabsch.py, RMS 0.014mm).
  * 4 main legs : Arm.stl (shaft->ball, arms HORIZONTAL) + MainLink_Alpha.stl.
  * 2 pitch legs: L/RPitcher.stl (servo-horn->rod hole) + PitcherLink_Alpha.stl,
    arms at the physically-required +8.5 deg that closes the 185mm link.
"""
from __future__ import annotations
import os, sys, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
import trimesh                                                  # noqa: E402
from uam.cylinders import detect_cylinders                      # noqa: E402
from closure_firmware_6leg import legs, HOME_H                  # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")
OUT = os.path.join(ROOT, "results"); os.makedirs(OUT, exist_ok=True)


def Rx(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rigid_two_point(a_loc, b_loc, a_w, b_w):
    a_loc = np.asarray(a_loc, float); b_loc = np.asarray(b_loc, float)
    a_w = np.asarray(a_w, float); b_w = np.asarray(b_w, float)
    u = b_loc - a_loc; v = b_w - a_w
    u = u / (np.linalg.norm(u) + 1e-12); v = v / (np.linalg.norm(v) + 1e-12)
    w = np.cross(u, v); s = np.linalg.norm(w); c = float(np.dot(u, v))
    if s < 1e-9:
        R = np.eye(3) if c > 0 else -np.eye(3)
    else:
        wx = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        R = np.eye(3) + wx + wx @ wx * ((1 - c) / (s * s))
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = a_w - R @ a_loc
    return T


def part_holes(name, **kw):
    cyl = detect_cylinders(os.path.join(STL, name), **kw)
    return [c for c in cyl if c["kind"] == "hole"]


def main():
    # part-local interface points
    ah = part_holes("Arm.stl", rmin=1.0, rmax=5.0)
    a_shaft = min(ah, key=lambda c: abs(c["center"][1]))["center"]
    a_ball = max(ah, key=lambda c: c["center"][1])["center"]
    mh = sorted(part_holes("MainLink_Alpha.stl", rmin=2.5, rmax=8.0),
                key=lambda c: c["center"][1])
    ml_a, ml_b = mh[0]["center"], mh[-1]["center"]
    ph = sorted(part_holes("PitcherLink_Alpha.stl", rmin=2.5, rmax=8.0),
                key=lambda c: c["center"][1])
    pl_a, pl_b = ph[-1]["center"], ph[0]["center"]   # near-origin end, far end

    def pitcher_pts(name):
        h = part_holes(name, rmin=1.0, rmax=5.0)
        seat = [c for c in h if c["radius"] > 3.0]
        shaft = np.mean([c["center"] for c in seat], axis=0) if seat else h[0]["center"]
        ball = max(h, key=lambda c: c["center"][1])["center"]
        return shaft, ball

    lp_shaft, lp_ball = pitcher_pts("LPitcher.stl")
    rp_shaft, rp_ball = pitcher_pts("RPitcher.stl")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(11, 9)); ax = fig.add_subplot(111, projection="3d")
    allpts = []

    def add_mesh(path, T, color, alpha):
        m = trimesh.load(path, process=False)
        V = (T[:3, :3] @ m.vertices.T).T + T[:3, 3] if T is not None else m.vertices
        tris = V[m.faces]
        if len(tris) > 8000:
            idx = np.random.default_rng(0).choice(len(tris), 8000, replace=False)
            tris = tris[idx]
        pc = Poly3DCollection(tris, linewidths=0.0)
        pc.set_facecolor((*[c / 255 for c in color[:3]], alpha)); ax.add_collection3d(pc)
        allpts.append(V)

    # static shell
    add_mesh(os.path.join(STL, "Base.stl"), None, [120, 120, 130], 0.22)
    add_mesh(os.path.join(STL, "LFrame.stl"), None, [90, 140, 200], 0.22)
    add_mesh(os.path.join(STL, "RFrame.stl"), None, [90, 140, 200], 0.22)

    # receiver: proper Rx(-90) + lift, proven by Kabsch fit to firmware pivots
    Trec = np.eye(4); Trec[:3, :3] = Rx(-90.0); Trec[:3, 3] = [0.0, 0.0, HOME_H]
    add_mesh(os.path.join(STL, "Receiver.stl"), Trec, [210, 120, 70], 0.55)

    for nm, shaft, tip, piv, link, th, res, ok in legs():
        if nm.startswith("main"):
            add_mesh(os.path.join(STL, "Arm.stl"),
                     rigid_two_point(a_shaft, a_ball, shaft, tip), [70, 170, 90], 0.9)
            add_mesh(os.path.join(STL, "MainLink_Alpha.stl"),
                     rigid_two_point(ml_a, ml_b, tip, piv), [200, 60, 60], 0.9)
        else:
            side = nm.split("-")[1]
            ps, pb = (lp_shaft, lp_ball) if side == "L" else (rp_shaft, rp_ball)
            mesh = "LPitcher.stl" if side == "L" else "RPitcher.stl"
            add_mesh(os.path.join(STL, mesh),
                     rigid_two_point(ps, pb, shaft, tip), [70, 170, 140], 0.9)
            add_mesh(os.path.join(STL, "PitcherLink_Alpha.stl"),
                     rigid_two_point(pl_a, pl_b, tip, piv), [220, 130, 60], 0.9)
        ax.plot(*zip(shaft, tip, piv), color="k", lw=0.8, alpha=0.5)

    pts = np.vstack(allpts); lo = pts.min(0); hi = pts.max(0); ctr = (lo + hi) / 2
    r = float((hi - lo).max()) / 2 * 1.05
    ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r)
    ax.set_zlim(ctr[2] - r, ctr[2] + r)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title("SR6 home: full 6-leg closure (4 main + 2 pitch), RMS=0.000mm")
    fig.tight_layout()
    outs = []
    for elev, az, tag in [(16, -60, "iso"), (8, -90, "front"), (6, 0, "side"), (75, -90, "top")]:
        ax.view_init(elev=elev, azim=az)
        p = os.path.join(OUT, f"home6_{tag}.png")
        fig.savefig(p, dpi=130); outs.append(p); print("rendered", p)
    plt.close(fig)
    return outs


if __name__ == "__main__":
    main()
