#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uam/cylinders.py — L0 perception, evolved: axis-AGNOSTIC cylindrical-feature
detection (holes AND bosses/studs/trunnions, any orientation).

Why this exists: the plane-slice hole finder (perceive.py) only catches internal
bores whose axis is near the slice normal. Real assemblies mate on *posts*,
*trunnions* and *ball studs* too — external cylinders, often on horizontal axes.
Missing them is exactly why the SR6 Receiver's 6 rod pivots were invisible.

Method (general, no part-specific code):
  1. Segment the mesh into smooth patches: connected faces whose shared-edge
     dihedral angle is small (a developable surface — plane or cylinder).
  2. Fit a cylinder to each patch:
       axis a   = direction MOST perpendicular to all face normals
                  (smallest right-singular vector of the normal matrix).
       radius r = best-fit circle of patch centers projected onto plane ⟂ a.
  3. Keep patches whose fit residual is small and whose angular wrap is enough
     to be a real cylinder (not a flat fillet). Report (point_on_axis, axis,
     radius, length, wrap, convex=boss / concave=hole).
"""
from __future__ import annotations
import numpy as np
import trimesh


def _circle_fit(P2):
    """Kasa algebraic circle fit. P2: (n,2). Returns (cx,cy,r,residual)."""
    x = P2[:, 0]; y = P2[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(P2))]
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0], sol[1]
    r = float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))
    d = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    return cx, cy, r, float(np.std(d))


def _basis(a):
    a = a / (np.linalg.norm(a) + 1e-12)
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, t); u /= np.linalg.norm(u) + 1e-12
    v = np.cross(a, u)
    return u, v, a


def detect_cylinders(mesh, smooth_deg=18.0, min_faces=6, max_resid=0.25,
                     min_wrap_deg=120.0, rmin=0.5, rmax=40.0):
    """Return list of cylinder features detected on a trimesh mesh."""
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = trimesh.load(mesh, process=True)
    adj = mesh.face_adjacency                    # (m,2) face index pairs
    ang = mesh.face_adjacency_angles             # (m,) dihedral angle per pair
    keep = ang < np.radians(smooth_deg)
    # union-find over faces connected by smooth edges
    n = len(mesh.faces)
    parent = np.arange(n)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj: parent[ri] = rj

    for (i, j), k in zip(adj, keep):
        if k: union(int(i), int(j))

    roots = np.array([find(i) for i in range(n)])
    fn = mesh.face_normals
    fc = mesh.triangles_center
    fa = mesh.area_faces
    out = []
    for root in np.unique(roots):
        idx = np.where(roots == root)[0]
        if len(idx) < min_faces:
            continue
        N = fn[idx]
        # axis = smallest right singular vector of normals (most ⟂ to all normals)
        U, S, Vt = np.linalg.svd(N - 0 * N.mean(0), full_matrices=False)
        a = Vt[-1]
        u, v, a = _basis(a)
        C = fc[idx]
        P2 = np.c_[C @ u, C @ v]
        cx, cy, r, resid = _circle_fit(P2)
        if not (rmin <= r <= rmax):
            continue
        if resid > max_resid:
            continue
        # angular wrap: how much of the circle the patch covers
        ang2 = np.arctan2(P2[:, 1] - cy, P2[:, 0] - cx)
        wrap = np.degrees(_angular_span(ang2))
        if wrap < min_wrap_deg:
            continue
        axis_pt = cx * u + cy * v + (C @ a).mean() * a
        ext = C @ a
        length = float(ext.max() - ext.min())
        # convex (boss) vs concave (hole): do normals point AWAY from axis?
        radial = (C - (axis_pt + np.outer(C @ a - axis_pt @ a, a)))
        radial /= (np.linalg.norm(radial, axis=1, keepdims=True) + 1e-12)
        sign = np.sign((radial * N).sum(1)).mean()
        out.append({
            "center": [round(float(z), 3) for z in axis_pt],
            "axis": [round(float(z), 4) for z in a],
            "radius": round(r, 3), "length": round(length, 2),
            "wrap_deg": round(wrap, 1), "fit_resid": round(resid, 3),
            "kind": "boss" if sign > 0 else "hole",
            "area": round(float(fa[idx].sum()), 1), "nfaces": int(len(idx)),
        })
    out.sort(key=lambda d: -d["area"])
    return out


def _angular_span(angles):
    """Largest covered arc (radians) given sample angles on a circle."""
    s = np.sort(angles % (2 * np.pi))
    if len(s) < 2:
        return 0.0
    gaps = np.diff(np.r_[s, s[0] + 2 * np.pi])
    return 2 * np.pi - gaps.max()


if __name__ == "__main__":
    import os, sys, json
    _HERE = os.path.dirname(os.path.abspath(__file__))
    _REPO = os.path.dirname(_HERE)
    stl_dir = os.environ.get("UAM_STL_DIR", os.path.join(_REPO, "ground_truth", "stl"))
    names = sys.argv[1:] or ["Arm.stl", "MainLink_Alpha.stl", "Receiver.stl"]
    for nm in names:
        cyl = detect_cylinders(os.path.join(stl_dir, nm))
        print(f"== {nm}: {len(cyl)} cylinders")
        for c in cyl[:12]:
            print(f"   {c['kind']:4s} r={c['radius']:6.2f} len={c['length']:6.1f} "
                  f"wrap={c['wrap_deg']:5.0f} resid={c['fit_resid']:.3f} "
                  f"axis={c['axis']} c={c['center']}")
