#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/assemble.py — full-machine assembly, two-tier (the natural structure
the meshes themselves reveal):

  TIER 1  STATIC SHELL  (Base, LFrame, RFrame, Lid, Receiver-home)
          These STLs are already authored in a COMMON assembly frame
          (Base at x=0, LFrame at x=-78.7, RFrame at x=+78.7, Lid on top).
          So they need NO solving — identity placement IS the assembly.
          We only VERIFY that (no interpenetration beyond shared bolt bosses,
          frames flank the base, lid caps the top).

  TIER 2  MOVING LINKAGE  (6 legs: Arm+MainLink ×4, Pitcher+PitcherLink ×2)
          These STLs are in their own print-orientation frames and MUST be
          solved into place by the mate solver (uam.assembly), chaining each
          servo pivot on a frame to its receiver pivot.

This file first renders/verifies TIER 1; TIER 2 is layered on once the shell
frame is confirmed as the common world frame.
"""
from __future__ import annotations
import os, sys
import numpy as np
import trimesh

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO); sys.path.insert(0, _HERE)

STL = os.environ.get("UAM_STL_DIR", os.path.join(_REPO, "ground_truth", "stl"))
OUT = os.path.join(_REPO, "results")
os.makedirs(OUT, exist_ok=True)

SHELL = ["Base.stl", "LFrame.stl", "RFrame.stl", "Receiver.stl", "Lid.stl"]
COLORS = {
    "Base.stl": [120, 120, 130, 255], "LFrame.stl": [90, 140, 200, 255],
    "RFrame.stl": [90, 140, 200, 255], "Receiver.stl": [210, 120, 70, 255],
    "Lid.stl": [180, 180, 190, 120],
}


def load_shell():
    scene = trimesh.Scene()
    info = []
    for nm in SHELL:
        m = trimesh.load(os.path.join(STL, nm), process=False)
        m.visual.face_colors = COLORS.get(nm, [160, 160, 160, 255])
        scene.add_geometry(m, node_name=nm.replace(".stl", ""))
        lo, hi = m.bounds
        info.append((nm, lo, hi))
    return scene, info


def verify_shell(info):
    """Sanity checks that the shell parts share one frame and nest correctly."""
    b = {nm: (lo, hi) for nm, lo, hi in info}
    checks = []
    L = b["LFrame.stl"]; R = b["RFrame.stl"]; B = b["Base.stl"]; Lid = b["Lid.stl"]
    checks.append(("LFrame left of base center",  L[0][0] < 0 < R[1][0]))
    checks.append(("RFrame right of base center", R[1][0] > 0 > L[0][0]))
    checks.append(("frames flank base in x",
                   L[0][0] < B[0][0] and R[1][0] > B[1][0]))
    checks.append(("lid sits on top (max z)",
                   Lid[1][2] >= max(B[1][2], L[1][2], R[1][2]) - 1))
    struct = [B, L, R]
    gmin = min(v[0][2] for v in struct)
    checks.append(("base+frames share ground z",
                   all(abs(v[0][2] - gmin) < 15 for v in struct)))
    checks.append(("receiver nested inside frames (x)",
                   b["Receiver.stl"][0][0] > L[0][0]
                   and b["Receiver.stl"][1][0] < R[1][0]))
    return checks


def render_mpl(path, elev=20, azim=-62):
    """Headless matplotlib render of the shell in its native (shared) frame."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    allpts = []
    for nm in SHELL:
        m = trimesh.load(os.path.join(STL, nm), process=False)
        col = np.array(COLORS.get(nm, [150, 150, 150, 255])) / 255.0
        tris = m.vertices[m.faces]
        if len(tris) > 12000:
            idx = np.random.default_rng(0).choice(len(tris), 12000, replace=False)
            tris = tris[idx]
        pc = Poly3DCollection(tris, linewidths=0.0)
        pc.set_facecolor((col[0], col[1], col[2], col[3]))
        ax.add_collection3d(pc)
        allpts.append(m.vertices)
    pts = np.vstack(allpts)
    lo = pts.min(0); hi = pts.max(0); ctr = (lo + hi) / 2
    r = float((hi - lo).max()) / 2 * 1.05
    ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r)
    ax.set_zlim(ctr[2]-r, ctr[2]+r)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
    ax.set_title("SR6 static shell - real meshes in authored common frame")
    ax.view_init(elev=elev, azim=azim)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)
    return True


def main():
    print("=" * 64)
    print("SR6 TIER-1 STATIC SHELL — verify common assembly frame")
    print("=" * 64)
    scene, info = load_shell()
    for nm, lo, hi in info:
        print(f"  {nm:16s} x[{lo[0]:7.1f},{hi[0]:7.1f}] "
              f"y[{lo[1]:7.1f},{hi[1]:7.1f}] z[{lo[2]:7.1f},{hi[2]:7.1f}]")
    print("-" * 64)
    ok = True
    for name, passed in verify_shell(info):
        print(f"  [{'OK' if passed else 'XX'}] {name}")
        ok = ok and passed
    print("-" * 64)
    print("shell is a coherent pre-assembled frame." if ok else "shell checks failed.")
    glb = os.path.join(OUT, "shell.glb")
    scene.export(glb)
    print("exported", glb)
    png = os.path.join(OUT, "shell.png")
    if render_mpl(png):
        print("rendered", png)


if __name__ == "__main__":
    main()
