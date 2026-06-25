#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
anchors/sr6/render_leg.py — visual proof that the SR6 main leg is ASSEMBLED by the
solver, not glued. It loads the *real* Arm.stl and MainLink_Alpha.stl meshes and
places each one at the 6-DOF pose the mate solver computed (uam.assembly), then
renders the result. No vertex of either mesh is touched by hand; the only inputs
are perceive.json (hole centers) + the firmware home pivots, everything else is
the least-squares solution.

Output: results/leg_solved.png  (matplotlib, headless Agg)
        results/leg_solved.glb  (interactive, real meshes at solved poses)
"""
from __future__ import annotations
import os, sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO); sys.path.insert(0, _HERE)

import trimesh
from uam.assembly import qmat
from leg import solve_leg

STL_DIR = os.environ.get("UAM_STL_DIR", os.path.join(_REPO, "ground_truth", "stl"))
OUT = os.path.join(_REPO, "results")


def part_transform(part):
    """4x4 homogeneous transform mapping the part's local mesh frame to world,
    straight from the solved pose (R(q), t)."""
    T = np.eye(4)
    T[:3, :3] = qmat(part.q)
    T[:3, 3] = part.t
    return T


def load_placed(part):
    m = trimesh.load(os.path.join(STL_DIR, part.mesh_name), process=False)
    m.apply_transform(part_transform(part))
    return m


def render_png(meshes, markers, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    colors = {"Arm.stl": (0.20, 0.55, 0.90, 0.85),
              "MainLink_Alpha.stl": (0.90, 0.45, 0.20, 0.85)}
    allpts = []
    for name, m in meshes:
        tris = m.vertices[m.faces]
        allpts.append(m.vertices)
        pc = Poly3DCollection(tris, alpha=0.85, linewidths=0.0)
        pc.set_facecolor(colors.get(name, (0.6, 0.6, 0.6, 0.85)))
        ax.add_collection3d(pc)
    for label, p, col in markers:
        ax.scatter([p[0]], [p[1]], [p[2]], color=col, s=60)
        ax.text(p[0], p[1], p[2], f"  {label}", fontsize=9)
    pts = np.vstack(allpts)
    lo = pts.min(0); hi = pts.max(0); ctr = (lo + hi) / 2
    r = float((hi - lo).max()) / 2 * 1.1
    ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r); ax.set_zlim(ctr[2]-r, ctr[2]+r)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
    ax.set_title("SR6 main leg — real meshes at SOLVED poses (rms=0)")
    ax.view_init(elev=22, azim=-60)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    r = solve_leg("MainLink_Alpha.stl")
    arm = r["parts"]["arm"]; rod = r["parts"]["rod"]
    arm_m = load_placed(arm); rod_m = load_placed(rod)

    S = np.array(r["S"]); P = np.array(r["P"]); A = np.array(r["A"])
    markers = [("S servo", S, "k"), ("P recv", P, "g"), ("A pivot", A, "r")]

    png = os.path.join(OUT, "leg_solved.png")
    render_png([("Arm.stl", arm_m), ("MainLink_Alpha.stl", rod_m)], markers, png)

    scene = trimesh.Scene()
    arm_m.visual.face_colors = [52, 140, 230, 255]
    rod_m.visual.face_colors = [230, 115, 52, 255]
    scene.add_geometry(arm_m, node_name="Arm")
    scene.add_geometry(rod_m, node_name="MainLink_Alpha")
    glb = os.path.join(OUT, "leg_solved.glb")
    scene.export(glb)

    print(f"rms={r['rms']:.4f}  coax_gap={r['coax_gap']:.4f}mm  "
          f"tip->P={r['tip_to_P']:.3f}mm  closed={r['closed']}")
    print(f"wrote {png}")
    print(f"wrote {glb}")


if __name__ == "__main__":
    main()
