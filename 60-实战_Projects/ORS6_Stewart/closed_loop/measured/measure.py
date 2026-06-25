#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Measure cylindrical features (holes / bores / pins) from SR6 STL parts.

For each principal axis a, collect faces whose normal is ~perpendicular to a
(the side wall of a hole/pin whose axis is parallel to a), cluster the wall
faces in 3D, fit a circle in the plane perpendicular to a, and report
center / radius / axis-span for each clean cylinder. Pure measured truth."""
import os, sys, math, json
import numpy as np
import trimesh
import warnings
warnings.filterwarnings("ignore")
from scipy.cluster.hierarchy import fcluster, linkage

ROOT = os.environ["SR6_STL_ROOT"]
AXES = {"X": np.array([1., 0, 0]), "Y": np.array([0, 1., 0]), "Z": np.array([0, 0, 1.])}
IDX = {"X": 0, "Y": 1, "Z": 2}


def find(*frags):
    import glob
    hits = [p for p in glob.glob(os.path.join(ROOT, "**", "*.stl"), recursive=True)
            if all(f in os.path.basename(p) for f in frags)]
    if len(hits) != 1:
        raise RuntimeError(f"{frags} -> {len(hits)}: {[os.path.basename(h) for h in hits]}")
    return hits[0]


def load(*frags):
    m = trimesh.load(find(*frags), process=False)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(tuple(m.geometry.values()))
    return m


def fit_circle(P2):
    x, y = P2[:, 0], P2[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = math.sqrt(max(0, c + cx * cx + cy * cy))
    resid = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return cx, cy, r, resid


def detect(m, rmin=1.0, rmax=22.0, perp_tol=0.2, min_faces=18,
           min_axis_span=2.0, max_resid=0.6, clust_t=3.5):
    fn = m.face_normals
    fc = m.triangles_center
    found = []
    for an, a in AXES.items():
        i, j = [(1, 2), (0, 2), (0, 1)][IDX[an]]
        k = IDX[an]
        perp = np.abs(fn @ a) < perp_tol
        C = fc[perp]
        if len(C) < min_faces:
            continue
        P3 = C[:, [i, j, k]].copy()
        # weight axis coordinate equally so holes at different axis pos separate
        if len(P3) > 5000:
            idx = np.random.RandomState(0).choice(len(P3), 5000, replace=False)
            P3 = P3[idx]
        Z = linkage(P3, method="single", metric="euclidean")
        lab = fcluster(Z, t=clust_t, criterion="distance")
        for L in np.unique(lab):
            sel = lab == L
            if sel.sum() < min_faces:
                continue
            pts3 = P3[sel]
            P2 = pts3[:, :2]
            spread2 = P2.max(0) - P2.min(0)
            if spread2.max() > 2.6 * rmax or spread2.min() < 1.0:
                continue
            aspan = (pts3[:, 2].min(), pts3[:, 2].max())
            if aspan[1] - aspan[0] < min_axis_span:
                continue
            cx, cy, r, res = fit_circle(P2)
            if not (rmin <= r <= rmax) or res > max_resid:
                continue
            # angular coverage: must wrap most of the circle to be a real bore
            ang = np.arctan2(P2[:, 1] - cy, P2[:, 0] - cx)
            cov = len(np.unique((ang / (2 * math.pi / 24)).astype(int)))
            if cov < 12:
                continue
            ctr = np.zeros(3)
            ctr[i] = cx; ctr[j] = cy; ctr[k] = np.mean(aspan)
            found.append(dict(axis=an, r=round(r, 2), resid=round(res, 3),
                              center=[round(v, 2) for v in ctr],
                              span=[round(aspan[0], 1), round(aspan[1], 1)],
                              n=int(sel.sum()), cov=cov))
    found.sort(key=lambda d: (d["axis"], -d["r"]))
    return found


def report(name, *frags, **kw):
    m = load(*frags)
    print(f"\n=== {name} ({os.path.basename(find(*frags))})")
    print(f"    faces={len(m.faces)}  bounds={np.round(m.bounds, 2).tolist()}  "
          f"size={np.round(m.extents, 2).tolist()}")
    for d in detect(m, **kw):
        print(f"    {d['axis']} r={d['r']:5.2f} resid={d['resid']:.3f} "
              f"c={d['center']} span={d['span']} n={d['n']} cov={d['cov']}")
    return m


if __name__ == "__main__":
    report("MainArm", "SR6 臂")
    report("L_Pitcher", "L-投手")
    report("R_Pitcher", "R-投手")
    report("Receiver", "Receiver")
    report("MainLink", "Main Link")
    report("PitchLink", "Pitcher Link Alpha")
