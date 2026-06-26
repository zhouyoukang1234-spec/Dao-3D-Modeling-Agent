"""v2/identity_render.py -- load every STL at identity, render multiview.
Empirical question: do these STLs share a common assembly coordinate frame
(i.e. are fixed parts already positioned), or is each centered in its own frame?
"""
import os, glob, numpy as np, trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")
OUT = os.path.join(os.path.dirname(__file__), "..", "results")

COLORS = {
    "Base": "#888888", "Lid": "#aaaaaa",
    "LFrame": "#1f77b4", "RFrame": "#1f77b4",
    "Arm": "#2ca02c", "LPitcher": "#9467bd", "RPitcher": "#9467bd",
    "MainLink_Alpha": "#d62728", "PitcherLink_Alpha": "#ff7f0e",
    "BearingMainLink": "#8c564b", "BearingPitcherLink": "#e377c2",
    "Receiver": "#ff9896",
}


def main():
    meshes = {}
    for p in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
        name = os.path.splitext(os.path.basename(p))[0]
        meshes[name] = trimesh.load(p, process=True)
        b = meshes[name].bounds
        print(f"{name:20s} bounds_min={np.array2string(b[0],precision=0):>20} "
              f"max={np.array2string(b[1],precision=0):>20}")

    views = [("iso", 22, 35), ("front", 0, -90), ("side", 0, 0), ("top", 89, -90)]
    fig = plt.figure(figsize=(20, 5))
    for k, (tag, el, az) in enumerate(views):
        ax = fig.add_subplot(1, 4, k + 1, projection="3d")
        for name, m in meshes.items():
            c = COLORS.get(name, "#cccccc")
            tris = m.vertices[m.faces]
            pc = Poly3DCollection(tris, alpha=0.55, facecolor=c, edgecolor="none")
            ax.add_collection3d(pc)
        allv = np.vstack([m.vertices for m in meshes.values()])
        lo, hi = allv.min(0), allv.max(0)
        ctr = (lo + hi) / 2
        rng = (hi - lo).max() / 2
        ax.set_xlim(ctr[0] - rng, ctr[0] + rng)
        ax.set_ylim(ctr[1] - rng, ctr[1] + rng)
        ax.set_zlim(ctr[2] - rng, ctr[2] + rng)
        ax.view_init(elev=el, azim=az)
        ax.set_title(tag)
        ax.set_xlabel("x"); ax.set_ylabel("y")
    fig.tight_layout()
    out = os.path.join(OUT, "v2_identity.png")
    fig.savefig(out, dpi=90)
    print("saved", out)


if __name__ == "__main__":
    main()
