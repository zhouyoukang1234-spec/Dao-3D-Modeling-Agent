#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/render_home.py — visual proof of the VALIDATED 4-main closure.

Renders, in the single shared assembly frame, the real STL meshes placed at the
poses the closure solver discovered (NOT hand-placed):
  * shell  : Base, L/RFrame in their authored frame; Receiver moved by the
             solved (t, q) home pose.
  * 4 main legs: real Arm.stl + MainLink_Alpha.stl rigidly aligned so each
             Arm maps shaft->ball and each MainLink maps arm-ball->receiver-pivot.

Every world coordinate here is solved/perceived; none is hand-fabricated. The
pitch legs are intentionally omitted -- they are a still-open frontier (the
naive radial model cannot close them at level home; see closure.py notes).
"""
from __future__ import annotations
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
import trimesh                                            # noqa: E402
from uam.cylinders import detect_cylinders                # noqa: E402
from closure import (perceive_servos, perceive_receiver_mounts,        # noqa: E402
                     solve_main_closure, qrot, MAIN_ARM, MAIN_ROD)

STL = os.path.join(ROOT, "ground_truth", "stl")
OUT = os.path.join(ROOT, "results"); os.makedirs(OUT, exist_ok=True)


def rigid_two_point(a_loc, b_loc, a_w, b_w):
    """Rigid transform mapping local segment a_loc->b_loc onto world a_w->b_w
    (translation + rotation aligning the two direction vectors; roll about the
    axis is left at the minimal-rotation value). Returns 4x4 matrix."""
    a_loc = np.asarray(a_loc, float); b_loc = np.asarray(b_loc, float)
    a_w = np.asarray(a_w, float); b_w = np.asarray(b_w, float)
    u = b_loc - a_loc; v = b_w - a_w
    nu = np.linalg.norm(u); nv = np.linalg.norm(v)
    u = u / (nu + 1e-12); v = v / (nv + 1e-12)
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
    servos = perceive_servos()
    mains, pitch, _ = perceive_receiver_mounts()
    mains_local = [c["center"] for c in mains]
    sol = solve_main_closure(servos, mains_local, verbose=True)
    t, q, Zs, th, legs = sol["t"], sol["q"], sol["Zs"], sol["theta"], sol["legs"]

    # Arm.stl local pivots: shaft hole (y=0) and ball hole (y=50)
    ah = part_holes("Arm.stl", rmin=1.0, rmax=5.0)
    a_shaft = min(ah, key=lambda c: abs(c["center"][1]))["center"]
    a_ball = max(ah, key=lambda c: c["center"][1])["center"]
    # MainLink local pivots: the two large ball holes 175 apart
    mh = part_holes("MainLink_Alpha.stl", rmin=2.5, rmax=8.0)
    mh = sorted(mh, key=lambda c: c["center"][1])
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

    # shell (authored frame); Receiver moved by solved home pose
    Rq = np.array([qrot(q, e) for e in np.eye(3)]).T
    Trec = np.eye(4); Trec[:3, :3] = Rq; Trec[:3, 3] = t
    add_mesh(os.path.join(STL, "Base.stl"), None, [120, 120, 130], 0.30)
    add_mesh(os.path.join(STL, "LFrame.stl"), None, [90, 140, 200], 0.30)
    add_mesh(os.path.join(STL, "RFrame.stl"), None, [90, 140, 200], 0.30)
    add_mesh(os.path.join(STL, "Receiver.stl"), Trec, [210, 120, 70], 0.55)

    # 4 main legs: place real Arm + MainLink at solved geometry
    for i, (sxy, mloc) in enumerate(legs):
        ball = np.array([sxy[0] + MAIN_ARM*np.cos(th[i]),
                         sxy[1] + MAIN_ARM*np.sin(th[i]), Zs])
        shaft = np.array([sxy[0], sxy[1], Zs])
        mount = qrot(q, mloc) + t
        add_mesh(os.path.join(STL, "Arm.stl"),
                 rigid_two_point(a_shaft, a_ball, shaft, ball), [70, 170, 90], 0.9)
        add_mesh(os.path.join(STL, "MainLink_Alpha.stl"),
                 rigid_two_point(ml_a, ml_b, ball, mount), [200, 60, 60], 0.9)
        ax.plot(*zip(shaft, ball, mount), color="k", lw=0.8, alpha=0.5)

    pts = np.vstack(allpts); lo = pts.min(0); hi = pts.max(0); ctr = (lo+hi)/2
    r = float((hi-lo).max())/2*1.05
    ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r); ax.set_zlim(ctr[2]-r, ctr[2]+r)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title(f"SR6 home: 4 main legs CLOSED (rod RMS={sol['rod_rms']:.4f}mm, "
                 f"receiver level @ z={t[2]:.1f})")
    ax.view_init(elev=18, azim=-60)
    fig.tight_layout()
    for az, tag in [(-60, "iso"), (-90, "front"), (0, "side")]:
        ax.view_init(elev=16, azim=az)
        p = os.path.join(OUT, f"home_4main_{tag}.png")
        fig.savefig(p, dpi=130); print("rendered", p)
    plt.close(fig)


if __name__ == "__main__":
    main()
