"""v2/render.py -- solid multi-view renderer (painter's algorithm + flat shading).

Pure matplotlib, headless-safe. Each part is a (mesh, transform, color). Faces are
depth-sorted per view and shaded by normal.light so solids read as solids, not wire.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection


def _view_dirs(elev, azim):
    e, a = np.radians(elev), np.radians(azim)
    # camera direction (pointing from scene to camera)
    cam = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    up = np.array([0, 0, 1.0])
    right = np.cross(up, cam); right /= np.linalg.norm(right)
    trueup = np.cross(cam, right)
    return cam, right, trueup


def render_views(parts, out_path, title="", views=None, figsize=None,
                 light=np.array([0.4, 0.3, 1.0])):
    """parts: list of (vertices Nx3 already in world frame, faces Mx3, color)."""
    if views is None:
        views = [("iso", 20, -60), ("front", 5, -90), ("side", 5, 0), ("top", 89, -90)]
    light = light / np.linalg.norm(light)
    n = len(views)
    if figsize is None:
        figsize = (5 * n, 5)
    fig, axes = plt.subplots(1, n, figsize=figsize)
    if n == 1:
        axes = [axes]

    allv = np.vstack([v for v, f, c in parts])
    ctr = (allv.min(0) + allv.max(0)) / 2
    rng = (allv.max(0) - allv.min(0)).max() / 2 * 1.05

    for ax, (tag, elev, azim) in zip(axes, views):
        cam, right, trueup = _view_dirs(elev, azim)
        polys, facecolors, depths = [], [], []
        for v, f, color in parts:
            base = np.array(matplotlib.colors.to_rgb(color))
            tris = v[f]                                  # M,3,3
            nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
            ln = np.linalg.norm(nrm, axis=1, keepdims=True)
            nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-9)
            shade = np.abs(nrm @ light)
            shade = 0.35 + 0.65 * shade
            cen = tris.mean(1)
            d = cen @ cam
            u = tris @ right
            w = tris @ trueup
            screen = np.stack([u, w], axis=-1)           # M,3,2
            for i in range(len(tris)):
                polys.append(screen[i])
                facecolors.append(np.clip(base * shade[i], 0, 1))
                depths.append(d[i])
        order = np.argsort(depths)
        polys = [polys[i] for i in order]
        facecolors = [facecolors[i] for i in order]
        pc = PolyCollection(polys, facecolors=facecolors, edgecolors="none",
                            linewidths=0, antialiased=False)
        ax.add_collection(pc)
        cu, cw = ctr @ right, ctr @ trueup
        ax.set_xlim(cu - rng, cu + rng)
        ax.set_ylim(cw - rng, cw + rng)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(tag, fontsize=11)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, facecolor="white")
    plt.close(fig)
    return out_path
