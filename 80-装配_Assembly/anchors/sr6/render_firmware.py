#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Visual proof of the FIRMWARE-DATUM 4-main closure (corrected, honest).

Every world coordinate comes from the firmware control law (closure_firmware.py),
NOT from a fitted solver or a mis-perceived hole grid:
  * Receiver placed at level home (t=(0,0,208.48), identity rotation).
  * 4 Arm.stl meshes at the firmware shafts (+-59.5, +-15, 46), HORIZONTAL,
    mapping shaft->ball (ball at +-59.5, +-65, 46).
  * 4 MainLink_Alpha.stl meshes mapping ball->receiver main pivot (rod = 175.0).
"""
from __future__ import annotations
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
import trimesh                                              # noqa: E402
from uam.cylinders import detect_cylinders                  # noqa: E402
from closure_firmware import main_legs, HOME_H              # noqa: E402

STL = os.path.join(ROOT, "ground_truth", "stl")
OUT = os.path.join(ROOT, "results"); os.makedirs(OUT, exist_ok=True)


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
    ah = part_holes("Arm.stl", rmin=1.0, rmax=5.0)
    a_shaft = min(ah, key=lambda c: abs(c["center"][1]))["center"]
    a_ball = max(ah, key=lambda c: c["center"][1])["center"]
    mh = sorted(part_holes("MainLink_Alpha.stl", rmin=2.5, rmax=8.0),
                key=lambda c: c["center"][1])
    ml_a, ml_b = mh[0]["center"], mh[-1]["center"]

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
        if len(tris) > 9000:
            idx = np.random.default_rng(0).choice(len(tris), 9000, replace=False)
            tris = tris[idx]
        pc = Poly3DCollection(tris, linewidths=0.0)
        pc.set_facecolor((*[c/255 for c in color[:3]], alpha)); ax.add_collection3d(pc)
        allpts.append(V)

    Trec = np.eye(4); Trec[:3, 3] = [0.0, 0.0, HOME_H]      # receiver level home
    add_mesh(os.path.join(STL, "Base.stl"), None, [120, 120, 130], 0.28)
    add_mesh(os.path.join(STL, "LFrame.stl"), None, [90, 140, 200], 0.28)
    add_mesh(os.path.join(STL, "RFrame.stl"), None, [90, 140, 200], 0.28)
    add_mesh(os.path.join(STL, "Receiver.stl"), Trec, [210, 120, 70], 0.55)

    for name, shaft, ball, piv, rod, tilt in main_legs():
        add_mesh(os.path.join(STL, "Arm.stl"),
                 rigid_two_point(a_shaft, a_ball, shaft, ball), [70, 170, 90], 0.9)
        add_mesh(os.path.join(STL, "MainLink_Alpha.stl"),
                 rigid_two_point(ml_a, ml_b, ball, piv), [200, 60, 60], 0.9)
        ax.plot(*zip(shaft, ball, piv), color="k", lw=0.8, alpha=0.5)

    pts = np.vstack(allpts); lo = pts.min(0); hi = pts.max(0); ctr = (lo+hi)/2
    r = float((hi-lo).max())/2*1.05
    ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r); ax.set_zlim(ctr[2]-r, ctr[2]+r)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title("SR6 home (FIRMWARE DATUM): 4 main legs, arms horizontal, rod=175.00")
    fig.tight_layout()
    for az, tag in [(-60, "iso"), (-90, "front"), (0, "side")]:
        ax.view_init(elev=16, azim=az)
        p = os.path.join(OUT, f"home_fw_{tag}.png")
        fig.savefig(p, dpi=130); print("rendered", p)
    plt.close(fig)


if __name__ == "__main__":
    main()
