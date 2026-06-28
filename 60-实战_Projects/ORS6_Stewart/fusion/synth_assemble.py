# -*- coding: utf-8 -*-
"""Synthesis assembler.

Place body shell + receiver ring at the Tripo-registered poses (the true photo
layout: enclosure box on one side, receiver ring offset to the other), then
connect them with 6 rods of exactly 175mm joining arm-tip <-> ring-mount, and
hang the 6 servo arms between each servo pivot and its rod tip. Nothing floats
by construction: this fuses the fusion registration (correct global layout vs
the photo-derived ground truth) with the closed-loop linkage.

Inputs (Tripo mm frame, from fusion/kreg.py + kfit.py):
  data/kfit_body.npz : R,t   body shell -> Tripo
  data/kfit_pose.npz : tips(6,3), mounts(6,3), ring_ctr, ring_nrm, ring_r
"""
import os, sys, numpy as np, trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.dirname(ROOT)
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
sys.path.insert(0, PROJ)
from ORS6_Stewart import render_mate as RM
from ORS6_Stewart.parts import stl_path, SERVO_SLOTS, SR6


def _R_align(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); s = np.linalg.norm(v); c = float(a @ b)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def _ends(mesh):
    """Two functional ends of a lever mesh via its longest principal axis."""
    V = mesh.vertices; Vc = V - V.mean(0)
    _, _, vt = np.linalg.svd(Vc, full_matrices=False)
    t = Vc @ vt[0]
    return (V[t < np.quantile(t, 0.06)].mean(0),
            V[t > np.quantile(t, 0.94)].mean(0))


def _sim_map(p0, p1, q0, q1):
    """Similarity transform mapping segment p0->q0, p1->q1 (scale+rot+trans)."""
    vp = p1 - p0; vq = q1 - q0
    sc = np.linalg.norm(vq) / max(np.linalg.norm(vp), 1e-9)
    R = _R_align(vp, vq)
    T = np.eye(4); T[:3, :3] = sc * R; T[:3, 3] = q0 - sc * R @ p0
    return T, abs(sc - 1.0)


def build(out_prefix="renders/sr6_synth", res=900):
    bf = np.load(os.path.join(WORK, "kfit_body.npz"))
    bR, bt = bf["R"], bf["t"]
    d = np.load(os.path.join(WORK, "kfit_pose.npz"), allow_pickle=True)
    tips, mounts = d["tips"], d["mounts"]
    ctr, nrm, r = d["ring_ctr"], d["ring_nrm"], float(d["ring_r"])
    rods = np.linalg.norm(tips - mounts, axis=1)

    Tb = np.eye(4); Tb[:3, :3] = bR; Tb[:3, 3] = bt
    pls = []
    for nm, col in [("Base", (200, 40, 40)), ("L_Frame", (210, 60, 60)),
                    ("R_Frame", (210, 60, 60)), ("Lid", (200, 40, 40))]:
        try:
            pls.append(RM.Placement(nm, trimesh.load(stl_path(nm), force="mesh"),
                                    Tb.copy(), col, alpha=1.0))
        except Exception as e:
            print("skip", nm, e)

    ring = trimesh.creation.torus(r, 6.0)
    Tr = np.eye(4); Tr[:3, :3] = _R_align(np.array([0, 0, 1.0]), nrm); Tr[:3, 3] = ctr
    pls.append(RM.Placement("ring", ring, Tr, (30, 60, 160), alpha=1.0))

    arm = trimesh.load(stl_path("Arm"), force="mesh"); ae = _ends(arm)
    lp = trimesh.load(stl_path("L_Pitcher"), force="mesh"); lpe = _ends(lp)
    rp = trimesh.load(stl_path("R_Pitcher"), force="mesh"); rpe = _ends(rp)

    for i, (s_nm, styp, sx, sy, sg) in enumerate(SERVO_SLOTS):
        pivot = bR @ np.array([sx, sy, SR6["servoPivotH"]]) + bt
        ball = tips[i]
        if styp == "pitch":
            m, (e0, e1) = (lp, lpe) if sx < 0 else (rp, rpe)
        else:
            m, (e0, e1) = arm, ae
        best = None
        for a0, a1 in [(e0, e1), (e1, e0)]:
            T, dev = _sim_map(a0, a1, pivot, ball)
            if best is None or dev < best[1]:
                best = (T, dev)
        pls.append(RM.Placement(f"arm{i}", m, best[0], (235, 233, 228), alpha=1.0))
        cyl = trimesh.creation.cylinder(radius=3.0, segment=np.array([ball, mounts[i]]))
        pls.append(RM.Placement(f"rod{i}", cyl, np.eye(4), (200, 200, 205), alpha=1.0))

    RM.render_views(pls, out_prefix, res=res)
    print("rod lengths:", np.round(rods, 2).tolist())
    print("ring ctr", ctr.round(1), "r", round(r, 1))
    return pls


if __name__ == "__main__":
    build()
