# -*- coding: utf-8 -*-
"""Improved receiver-pose fit (v2): optimize the FULL 6-DOF receiver pose so the
real rod centerlines + receiver ring best match the Tripo truth surface (chamfer),
with a twist multi-start.  Legs are solved at the fixed 175mm length about the Y
servo shaft (consistent with SR6 kinematics).  Saves kfit_pose2.npz consumed by
kassemble2.py."""
import os, sys, math, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
PROJ = os.path.dirname(ROOT)
for p in (PROJ, ROOT, WORK):
    sys.path.insert(0, p)
import trimesh
from scipy.spatial import cKDTree
from scipy.optimize import minimize
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME
from ORS6_Stewart.parts import SERVO_SLOTS, SR6, stl_path

ROD = 175.0


def expm_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def axis_angle(axis, ang):
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def main():
    tp = trimesh.load(os.path.join(WORK, "tripo_mm.glb"), process=False)
    if isinstance(tp, trimesh.Scene):
        tp = tp.to_geometry()
    Vt = np.asarray(tp.vertices, float); Ft = np.asarray(tp.faces, int)
    Ptri = trimesh.Trimesh(Vt, Ft, process=False).sample(120000)
    tree = cKDTree(Ptri)

    bf = np.load(os.path.join(WORK, "kfit_body.npz")); bR, bt = bf["R"], bf["t"]
    def Tb(P): return (bR @ np.asarray(P, float).T).T + bt

    ik = StewartIK(); g = ik.compute_full_geometry(*TCODE_HOME)
    sn = [s for s, _, _, _, _ in SERVO_SLOTS]
    piv_cad = []; u_cad = []; Larm = []
    for s, stype, sx, sy, _ in SERVO_SLOTS:
        sign_x = 1 if sx > 0 else -1
        piv_cad.append([sx, sy, SR6["servoPivotH"]])
        u_cad.append([-sign_x, 0, 0])
        Larm.append(SR6["pitchArm"] if stype == "pitch" else SR6["mainArm"])
    Piv = Tb(np.array(piv_cad))
    U = (bR @ np.array(u_cad).T).T
    Wv = (bR @ np.tile([0, 0, 1.0], (6, 1)).T).T
    M0 = Tb(np.array([g["recv_mounts"][s] for s in sn]))
    C0 = M0.mean(0)

    rc = np.load(os.path.join(WORK, "ring_circle.npz"))
    ring_ctr = rc["center"]; ring_nrm = rc["normal"] / np.linalg.norm(rc["normal"])
    ring_r = float(rc["radius"])
    home_axis = bR @ np.array([0, 0, 1.0]); home_axis /= np.linalg.norm(home_axis)
    if home_axis @ ring_nrm < 0:
        ring_nrm = -ring_nrm
    v = np.cross(home_axis, ring_nrm); c = home_axis @ ring_nrm
    if np.linalg.norm(v) < 1e-8:
        Rinit = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rinit = np.eye(3) + vx + vx @ vx / (1 + c)
    tinit0 = ring_ctr - C0

    # receiver ring sample (home, Tripo) for chamfer
    rm = trimesh.load(stl_path("Receiver"), force="mesh")
    ringV = Tb(np.asarray(rm.sample(3000), float))

    def solve_legs(M):
        tips = np.zeros((6, 3)); pen = 0.0
        for i in range(6):
            a = Piv[i] - M[i]
            A = 2 * Larm[i] * (a @ U[i]); B = 2 * Larm[i] * (a @ Wv[i])
            Cc = ROD ** 2 - (a @ a) - Larm[i] ** 2
            Rr = math.hypot(A, B)
            if Rr < 1e-9:
                tips[i] = Piv[i] + Larm[i] * U[i]; pen += abs(Cc); continue
            if Rr < abs(Cc):
                pen += abs(Cc) - Rr; cc = max(-1.0, min(1.0, Cc / Rr))
            else:
                cc = Cc / Rr
            phi = math.atan2(B, A); d = math.acos(max(-1.0, min(1.0, cc)))
            t1 = Piv[i] + Larm[i] * (math.cos(phi + d) * U[i] + math.sin(phi + d) * Wv[i])
            t2 = Piv[i] + Larm[i] * (math.cos(phi - d) * U[i] + math.sin(phi - d) * Wv[i])
            d1, _ = tree.query(t1); d2, _ = tree.query(t2)
            tips[i] = t1 if d1 <= d2 else t2
        return tips, pen

    def pose(x):
        Rg = expm_so3(x[:3]) @ Rinit
        t = tinit0 + x[3:]
        M = (Rg @ (M0 - C0).T).T + C0 + t
        return Rg, t, M

    def sample_assembly(Rg, t, M, tips):
        pts = [np.linspace(tips[i], M[i], 20) for i in range(6)]    # rods
        ringP = (Rg @ (ringV - C0).T).T + C0 + t                    # receiver ring
        pts.append(ringP)
        return np.vstack(pts)

    def objective(x):
        Rg, t, M = pose(x)
        tips, pen = solve_legs(M)
        P = sample_assembly(Rg, t, M, tips)
        d, _ = tree.query(P)
        return d.mean() + 0.02 * pen

    # twist multi-start about ring axis, then full 6-DOF polish
    best = None
    for phi in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        Rg = axis_angle(ring_nrm, phi) @ Rinit
        # rotvec of (Rg @ Rinit^-1)
        Rrel = Rg @ Rinit.T
        ang = math.acos(max(-1, min(1, (np.trace(Rrel) - 1) / 2)))
        if ang < 1e-8:
            w = np.zeros(3)
        else:
            w = np.array([Rrel[2, 1] - Rrel[1, 2], Rrel[0, 2] - Rrel[2, 0],
                          Rrel[1, 0] - Rrel[0, 1]]) / (2 * math.sin(ang)) * ang
        x0 = np.concatenate([w, np.zeros(3)])
        f = objective(x0)
        if best is None or f < best[1]:
            best = (x0, f)
    print("twist-scan best", round(best[1], 3))
    x = best[0]
    for _ in range(3):
        res = minimize(objective, x, method="Powell",
                       options={"maxiter": 8000, "xtol": 1e-3, "ftol": 1e-3})
        x = res.x
    print("optimized objective", round(res.fun, 3))
    Rg, t, M = pose(x)
    tips, pen = solve_legs(M)
    rl = np.linalg.norm(tips - M, axis=1)
    print("rod lengths", rl.round(1), "penalty", round(pen, 2))
    np.savez(os.path.join(WORK, "kfit_pose2.npz"),
             Rg=Rg, tinit=t, C0=C0, M0=M0, tips=tips, mounts=M,
             ring_ctr=ring_ctr, ring_nrm=ring_nrm, ring_r=ring_r)
    print("saved kfit_pose2.npz; mounts centroid", M.mean(0).round(1),
          "ring_ctr", ring_ctr.round(1))


if __name__ == "__main__":
    main()
