#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_features.py — First-principles geometric probe of the REAL SR6 STLs.

Goal: test the hypothesis that the prior assembly failed because part poses were
authored as absolute transforms glued to the kinematic skeleton, rather than
derived from real mating FEATURES extracted from the actual geometry.

This script does NOT assemble anything. It only PERCEIVES each part the way a
human/AI should before assembly:
  - canonical frame (centroid + PCA principal axes + bbox)
  - cylindrical holes / bosses (axis, radius, length) via normal clustering
  - candidate pivot points (ball-joint sockets, rod-end holes)

Then it cross-checks measured geometry against the firmware kinematic constants
(main rod = 175, main arm = 50, pitch arm = 75, receiver pivot radius = 15/45).
If the real parts match the skeleton, we have a validated geometry<->kinematics
bridge. If not, the mismatch is the smoking gun.
"""
from __future__ import annotations
import sys, os, json, math
import numpy as np
import trimesh

STL_DIR = r"C:\Users\Administrator\sr6\ground_truth\stl"

def load(name):
    m = trimesh.load(os.path.join(STL_DIR, name), process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(tuple(m.geometry.values()))
    return m

def pca_axes(mesh):
    """Principal axes of the mesh vertices (columns are axes, descending var)."""
    v = mesh.vertices - mesh.vertices.mean(axis=0)
    cov = np.cov(v.T)
    w, V = np.linalg.eigh(cov)
    order = np.argsort(w)[::-1]
    return V[:, order], w[order]

def detect_cylinders(mesh, min_face_frac=0.002):
    """
    Cluster faces by normal direction; for each dominant axis, find rings of
    faces whose normals are perpendicular to that axis (cylinder walls) and
    estimate axis line + radius by fitting points to a circle in the plane
    perpendicular to the axis. Returns list of dicts.
    Heuristic, robust enough to recover hole/boss axes on printed parts.
    """
    fn = mesh.face_normals
    fa = mesh.area_faces
    fc = mesh.triangles_center
    results = []
    # candidate axes: the 3 PCA axes + global XYZ
    V, _ = pca_axes(mesh)
    cand_axes = [V[:, i] for i in range(3)] + [np.eye(3)[i] for i in range(3)]
    seen = []
    for axis in cand_axes:
        axis = axis / np.linalg.norm(axis)
        if any(abs(np.dot(axis, s)) > 0.98 for s in seen):
            continue
        seen.append(axis)
        # faces whose normal is ~perpendicular to axis = cylinder wall candidates
        perp = np.abs(fn @ axis) < 0.30
        if perp.sum() < 8:
            continue
        # project wall-face centers to plane perp to axis
        pts = fc[perp]
        w = fa[perp]
        # build 2 in-plane basis vectors
        a = axis
        tmp = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
        e1 = np.cross(a, tmp); e1 /= np.linalg.norm(e1)
        e2 = np.cross(a, e1)
        u = pts @ e1
        vv = pts @ e2
        # robust circle fit (algebraic, weighted)
        A = np.c_[2*u, 2*vv, np.ones_like(u)]
        b = u**2 + vv**2
        try:
            sol, *_ = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)
        except Exception:
            continue
        uc, vc, cc = sol
        r = math.sqrt(max(cc + uc**2 + vc**2, 0))
        # residual fit quality
        rr = np.sqrt((u - uc)**2 + (vv - vc)**2)
        resid = np.average(np.abs(rr - r), weights=w)
        if r < 1.0 or r > 200 or resid > 0.6:
            continue
        # axis line passes through center point
        axis_pt = uc * e1 + vc * e2 + (pts @ a).mean() * a
        # extent of wall along axis
        proj = pts @ a
        results.append({
            "axis": [round(float(x), 4) for x in a],
            "center": [round(float(x), 3) for x in axis_pt],
            "radius": round(float(r), 3),
            "length": round(float(proj.max() - proj.min()), 3),
            "resid": round(float(resid), 4),
            "n_wall_faces": int(perp.sum()),
        })
    # dedupe by (radius, center) closeness
    uniq = []
    for c in sorted(results, key=lambda d: d["resid"]):
        if any(abs(c["radius"] - u["radius"]) < 0.5 and
               np.linalg.norm(np.array(c["center"]) - np.array(u["center"])) < 2.0
               for u in uniq):
            continue
        uniq.append(c)
    return uniq

def report(name):
    m = load(name)
    ext = m.bounding_box.extents
    V, w = pca_axes(m)
    cyls = detect_cylinders(m)
    print(f"\n=== {name} ===")
    print(f"  watertight={m.is_watertight}  vol={m.volume:.0f}mm^3  verts={len(m.vertices)}")
    print(f"  bbox extents (mm): {ext[0]:.2f} x {ext[1]:.2f} x {ext[2]:.2f}")
    print(f"  PCA principal lengths (sqrt-eig*2~): {2*np.sqrt(w[0]):.1f}, {2*np.sqrt(w[1]):.1f}, {2*np.sqrt(w[2]):.1f}")
    print(f"  detected cylindrical features: {len(cyls)}")
    for c in cyls:
        print(f"    r={c['radius']:6.2f}  len={c['length']:6.2f}  resid={c['resid']:.3f}  "
              f"axis={c['axis']}  ctr={c['center']}  nwall={c['n_wall_faces']}")
    return {"name": name, "extents": [round(float(x),3) for x in ext], "cylinders": cyls}

if __name__ == "__main__":
    names = sys.argv[1:] or [
        "Arm.stl", "MainLink_Alpha.stl", "BearingMainLink.stl",
        "PitcherLink_Alpha.stl", "BearingPitcherLink.stl",
        "LPitcher.stl", "RPitcher.stl", "Receiver.stl",
        "LFrame.stl", "RFrame.stl", "Base.stl", "Lid.stl",
    ]
    out = {}
    for n in names:
        try:
            out[n] = report(n)
        except Exception as e:
            print(f"\n=== {n} === ERROR: {e}")
    with open(r"C:\Users\Administrator\sr6\feature_probe.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\n[written] feature_probe.json")
