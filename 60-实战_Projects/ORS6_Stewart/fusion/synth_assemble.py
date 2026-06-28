# -*- coding: utf-8 -*-
"""Synthesis: place body shell + receiver ring at the Tripo-registered poses
(true photo layout: box on one side, ring offset to the other), and connect them
with 6 rods of exactly 175mm joining arm-tip <-> ring-mount. Nothing floats by
construction. Combines the fusion registration (correct global layout vs the
photo-derived ground truth) with the closed-loop linkage (no floating).

Inputs (all in Tripo mm frame, produced by fusion/kreg.py + kfit.py):
  data/kfit_body.npz : R,t   body shell -> Tripo
  data/kfit_pose.npz : tips(6,3), mounts(6,3), ring_ctr, ring_nrm, ring_r
"""
import os, sys, numpy as np, trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.dirname(ROOT)
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
sys.path.insert(0, PROJ)
from ORS6_Stewart import render_mate as RM
from ORS6_Stewart.parts import stl_path


def _R_align(z, n):
    n = n / np.linalg.norm(n)
    v = np.cross(z, n); s = np.linalg.norm(v); c = float(z @ n)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def build(out_prefix="output/sr6_synth", res=900):
    bf = np.load(os.path.join(WORK, "kfit_body.npz"))
    bR, bt = bf["R"], bf["t"]
    d = np.load(os.path.join(WORK, "kfit_pose.npz"), allow_pickle=True)
    tips, mounts = d["tips"], d["mounts"]
    ctr = d["ring_ctr"]; nrm = d["ring_nrm"]; r = float(d["ring_r"])
    rods = np.linalg.norm(tips - mounts, axis=1)

    def Tb():
        T = np.eye(4); T[:3, :3] = bR; T[:3, 3] = bt; return T

    pls = []
    for nm, col in [("Base", (200, 40, 40)), ("L_Frame", (210, 60, 60)),
                    ("R_Frame", (210, 60, 60)), ("Lid", (200, 40, 40))]:
        try:
            m = trimesh.load(stl_path(nm), force="mesh")
            pls.append(RM.Placement(nm, m, Tb(), col, alpha=1.0))
        except Exception as e:
            print("skip", nm, e)

    ring = trimesh.creation.torus(r, 6.0)
    Tr = np.eye(4); Tr[:3, :3] = _R_align(np.array([0, 0, 1.0]), nrm); Tr[:3, 3] = ctr
    pls.append(RM.Placement("ring", ring, Tr, (30, 60, 160), alpha=1.0))

    for i in range(6):
        a, b = tips[i], mounts[i]
        cyl = trimesh.creation.cylinder(radius=3.0, segment=np.array([a, b]))
        pls.append(RM.Placement(f"rod{i}", cyl, np.eye(4), (235, 233, 228), alpha=1.0))
        for p in (a, b):
            sph = trimesh.creation.icosphere(subdivisions=2, radius=5.0)
            T = np.eye(4); T[:3, 3] = p
            pls.append(RM.Placement("ball", sph, T, (140, 140, 150), alpha=1.0))

    RM.render_views(pls, out_prefix, res=res)
    print("rod lengths:", np.round(rods, 2).tolist())
    print("ring ctr", ctr.round(1), "r", round(r, 1))
    return pls


if __name__ == "__main__":
    build()
