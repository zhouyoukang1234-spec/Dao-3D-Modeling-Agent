# -*- coding: utf-8 -*-
"""Register the REAL Receiver cradle rigidly to its region of the TRUE-SOURCE Tripo
   mesh (perception-grounded), so the cradle sits exactly where reality shows it.
   Saves recv_reg.npz: R,t mapping home-world receiver points -> assembly frame."""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K
import kbuild as B
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment
import trimesh

SLOTS = [s for s, *_ in K.SERVO_SLOTS]

WORK = K.WORK
B0 = np.array([0, 0, K.HOME_H])
CH = np.mean([np.array(K.IK.compute_full_geometry(*K.TCODE_HOME)["recv_mounts"][s])
              for s in K.IK.compute_full_geometry(*K.TCODE_HOME)["recv_mounts"]], axis=0)
CR, NR = K.CR, K.NR

# ---- Tripo in assembly frame ----
VT_asm = (K.bR.T @ (K.VT - K.bt).T).T

# ---- receiver-region points: near the detected ring, on the receiver side ----
body_c = K.bR.T @ (K.BODY_T.mean(0) - K.bt)
d_ring = np.linalg.norm(VT_asm - CR, axis=1)
d_body = np.linalg.norm(VT_asm - body_c, axis=1)
# the cradle/ring is the compact cluster around the detected ring, on the side
# away from the body -> tight radius so ICP is not contaminated by body points.
mask = (d_ring < 95) & (d_ring < d_body)
RECV_PTS = VT_asm[mask]
print("receiver-region points:", len(RECV_PTS), "of", len(VT_asm))
DST = RECV_PTS
TREE = cKDTree(DST)

# ---- receiver STL sample in home-world frame ----
rm = []
for n in K.RECV_VIS:
    V, F = K.MESH[n]
    m = trimesh.Trimesh(V + B0, F, process=False)
    try:
        p, _ = trimesh.sample.sample_surface(m, 2500)
    except Exception:
        p = (V + B0)
    rm.append(np.asarray(p))
SRC = np.vstack(rm)


def Raxis(a, b):
    a = a / (np.linalg.norm(a) or 1); b = b / (np.linalg.norm(b) or 1)
    v = np.cross(a, b); c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def twist_R(axis, deg):
    k = axis / (np.linalg.norm(axis) or 1); a = math.radians(deg)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * (kx @ kx)


def procrustes(A, Bd):
    ca, cb = A.mean(0), Bd.mean(0)
    H = (A - ca).T @ (Bd - cb)
    U, _, Vt = np.linalg.svd(H)
    Rn = Vt.T @ U.T
    if np.linalg.det(Rn) < 0:
        Vt[-1] *= -1; Rn = Vt.T @ U.T
    return Rn, cb - Rn @ ca


def icp(R, t, iters=25, trim=0.75):
    for _ in range(iters):
        P = SRC @ R.T + t
        d, idx = TREE.query(P)
        thr = np.quantile(d, trim)
        keep = d <= thr
        R, t = procrustes(SRC[keep], DST[idx[keep]])
    P = SRC @ R.T + t
    d, _ = TREE.query(P)
    return R, t, float(np.sqrt((np.minimum(d, np.quantile(d, trim)) ** 2).mean()))


def leg_resid(R, t):
    """best 1-1 servo->mount assignment; sum |rod-175| (physical reachability)."""
    MW = {s: R @ B.MH[s] + t for s in SLOTS}
    C = np.zeros((6, 6))
    for i, si in enumerate(SLOTS):
        for j, sj in enumerate(SLOTS):
            C[i, j] = abs(B.solve_arm(si, MW[sj])[2] - B.ROD)
    ri, cj = linear_sum_assignment(C)
    return float(C[ri, cj].sum())


def main():
    cands = []
    for sign in (+1, -1):
        R_ax = Raxis(np.array([0, 0, 1.0]), sign * NR)
        for tw in np.linspace(0, 360, 36, endpoint=False):
            R0 = twist_R(sign * NR, tw) @ R_ax
            t0 = CR - R0 @ CH
            R, t, rmse = icp(R0, t0)
            lr = leg_resid(R, t)
            cands.append((rmse + 0.04 * lr, rmse, lr, R, t, sign, tw))
    # physical truth first: rods must reach 175.  Among shape-good fits, pick the
    # one the real 175mm legs can actually connect to.
    cands.sort(key=lambda c: c[0])
    score, rmse, lr, R, t, sign, tw = cands[0]
    print(f"BEST combined={score:.2f}  shape_rmse={rmse:.2f}mm  leg_resid={lr:.1f}mm"
          f"  init sign={sign} twist={tw:.0f}")
    np.savez(os.path.join(WORK, "recv_reg.npz"), R=R, t=t, rmse=rmse, leg_resid=lr)
    print("receiver-region points:", len(RECV_PTS), "saved recv_reg.npz")


if __name__ == "__main__":
    main()
