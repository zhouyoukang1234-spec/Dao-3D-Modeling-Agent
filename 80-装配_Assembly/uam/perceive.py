#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perceive.py — Robust geometric PERCEPTION layer for the universal assembly framework.

The lesson from probe_features.py: naive circle-fitting over all wall faces fails on
low-poly printed parts. Real parts have holes that are coarse N-gon tunnels. Humans
perceive these instantly via gestalt grouping + priors. Algorithmically we need a
robust grouping + model-fitting layer. This module is pillar #1 of the framework:

  PERCEPTION:  raw mesh -> structured features (holes/shafts, planar faces, symmetry)

Method for holes/shafts (cylindrical features), robust to low tessellation:
  1. Build face graph; connect adjacent faces whose dihedral angle is consistent and
     moderate (a smoothly-curving wall), splitting at sharp creases.
  2. For each connected curved patch, fit a cylinder axis (the direction perpendicular
     to all face normals = smallest singular vector of the normal matrix) and a radius
     (mean distance of face centers to the axis line). Accept if normals are truly
     radial (good fit) and the patch wraps enough angle.
  3. Classify hole (normals point inward, toward axis) vs boss/shaft (outward).

This recovers M3/M4 screw holes, rod-end pivots and bearing bores even at 8-gon res.
"""
from __future__ import annotations
import sys, os, json, math
import numpy as np
import trimesh
from trimesh.graph import connected_component_labels

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
STL_DIR = os.environ.get("UAM_STL_DIR", os.path.join(_REPO, "ground_truth", "stl"))
RESULTS = os.path.join(_REPO, "results")

def load(name):
    m = trimesh.load(os.path.join(STL_DIR, name), process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(tuple(m.geometry.values()))
    return m

def fit_cylinder(centers, normals, areas):
    """Given a patch of faces, fit a cylinder.
    Axis = direction minimizing variance of normal projections (normals are radial,
    so they are all perpendicular to the axis). That is the smallest-singular-vector
    of the stacked unit normals. Returns (axis, point_on_axis, radius, fit_error)."""
    N = normals / np.linalg.norm(normals, axis=1, keepdims=True)
    # axis ~ null space of normals: SVD, smallest singular vector
    u, s, vt = np.linalg.svd(N - N.mean(axis=0, keepdims=True))
    axis = vt[-1]
    axis = axis / np.linalg.norm(axis)
    # project centers to plane perp to axis
    c0 = centers.mean(axis=0)
    rel = centers - c0
    rel_perp = rel - np.outer(rel @ axis, axis)
    # circle center in-plane via algebraic fit weighted by area
    tmp = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(axis, tmp); e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    u_ = rel_perp @ e1
    v_ = rel_perp @ e2
    A = np.c_[2*u_, 2*v_, np.ones_like(u_)]
    b = u_**2 + v_**2
    w = areas
    sol, *_ = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)
    uc, vc, cc = sol
    r = math.sqrt(max(cc + uc**2 + vc**2, 1e-9))
    center = c0 + uc*e1 + vc*e2
    rr = np.sqrt((u_-uc)**2 + (v_-vc)**2)
    err = float(np.average(np.abs(rr - r), weights=w))
    # angular wrap: spread of points around the circle
    ang = np.arctan2(v_-vc, u_-uc)
    wrap = float(np.ptp(np.sort(ang)))  # crude; refine below
    order = np.sort(ang)
    gaps = np.diff(np.r_[order, order[0]+2*math.pi])
    wrap = float(2*math.pi - gaps.max())  # covered angle = 2pi - largest gap
    return axis, center, r, err, wrap

def detect_holes(mesh, crease_deg=55.0, min_faces=4, max_radius=40.0, max_err=0.35):
    """Detect cylindrical holes/shafts as smooth curved patches."""
    adj = mesh.face_adjacency
    ang = mesh.face_adjacency_angles  # radians, unsigned
    # keep adjacency that is smooth-ish (curved wall) -> below crease threshold
    keep = ang < math.radians(crease_deg)
    edges = adj[keep]
    # connected components over faces using only smooth edges
    labels = connected_component_labels(edges, node_count=len(mesh.faces))
    fc = mesh.triangles_center
    fn = mesh.face_normals
    fa = mesh.area_faces
    feats = []
    for lab in np.unique(labels):
        idx = np.where(labels == lab)[0]
        if len(idx) < min_faces:
            continue
        c, n, a = fc[idx], fn[idx], fa[idx]
        # only consider patches that actually curve (normals span a range)
        if np.linalg.norm(n.max(axis=0) - n.min(axis=0)) < 0.3:
            continue  # flat patch
        try:
            axis, center, r, err, wrap = fit_cylinder(c, n, a)
        except Exception:
            continue
        if r < 0.8 or r > max_radius or err > max_err:
            continue
        if wrap < math.radians(160):   # need at least ~half a tunnel
            continue
        # hole vs boss: do face normals point toward axis (inward=hole) or away?
        rel = c - center
        rel_perp = rel - np.outer(rel @ axis, axis)
        radial = rel_perp / (np.linalg.norm(rel_perp, axis=1, keepdims=True) + 1e-9)
        inward = float((np.sum(radial * n, axis=1) < 0).mean())  # frac pointing inward
        kind = "hole" if inward > 0.5 else "boss"
        proj = c @ axis
        feats.append({
            "kind": kind, "radius": round(r, 3),
            "length": round(float(proj.max()-proj.min()), 3),
            "axis": [round(float(x),4) for x in axis],
            "center": [round(float(x),3) for x in center],
            "err": round(err,4), "wrap_deg": round(math.degrees(wrap),1),
            "n_faces": int(len(idx)),
        })
    # merge near-duplicate coaxial features
    out = []
    for f in sorted(feats, key=lambda d: d["err"]):
        dup = False
        for g in out:
            if (abs(f["radius"]-g["radius"]) < 0.6 and
                np.linalg.norm(np.array(f["center"])-np.array(g["center"])) < 1.5 and
                abs(abs(np.dot(f["axis"], g["axis"]))-1) < 0.05):
                dup = True; break
        if not dup:
            out.append(f)
    return out

def report(name):
    m = load(name)
    holes = detect_holes(m)
    ext = m.bounding_box.extents
    print(f"\n=== {name} ===  bbox {ext[0]:.1f}x{ext[1]:.1f}x{ext[2]:.1f}  | {len(holes)} cyl features")
    # sort pivot-like holes (small radius) and show
    for h in sorted(holes, key=lambda d: d["radius"]):
        print(f"   {h['kind']:4} r={h['radius']:6.2f} len={h['length']:6.2f} "
              f"err={h['err']:.3f} wrap={h['wrap_deg']:5.1f} ctr={h['center']} axis={h['axis']}")
    # pairwise hole-center distances (candidate mate spans)
    centers = [np.array(h["center"]) for h in holes if h["kind"]=="hole"]
    dists = []
    for i in range(len(centers)):
        for j in range(i+1, len(centers)):
            dists.append(round(float(np.linalg.norm(centers[i]-centers[j])),2))
    if dists:
        print(f"   hole-center distances (mm): {sorted(dists)}")
    return {"name": name, "extents":[round(float(x),3) for x in ext], "holes": holes}

if __name__ == "__main__":
    names = sys.argv[1:] or [
        "Arm.stl", "MainLink_Alpha.stl", "PitcherLink_Alpha.stl",
        "BearingMainLink.stl", "BearingPitcherLink.stl",
        "LPitcher.stl", "RPitcher.stl", "Receiver.stl",
        "LFrame.stl", "RFrame.stl", "Base.stl",
    ]
    out = {}
    for n in names:
        try: out[n] = report(n)
        except Exception as e:
            import traceback; print(f"\n=== {n} === ERROR {e}"); traceback.print_exc()
    os.makedirs(RESULTS, exist_ok=True)
    outpath = os.path.join(RESULTS, "perceive.json")
    with open(outpath, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[written] {outpath}")
