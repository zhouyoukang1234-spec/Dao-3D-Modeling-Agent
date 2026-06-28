# -*- coding: utf-8 -*-
"""Stage 1+2: robust RIGID (scale=1) registration of the body shell to Tripo,
plus ring re-detection. Canonical frame = Tripo mm. Saves kfit_body.npz."""
import os, sys, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)),"data")
sys.path.insert(0, ROOT); sys.path.insert(0, WORK); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
from scipy.spatial import cKDTree
from PIL import Image
from overlay_check import render_fixed, _look_at, load_concat

rng = np.random.default_rng(0)

def load_tripo():
    tp = trimesh.load(os.path.join(WORK,"tripo_mm.glb") if os.path.exists(os.path.join(WORK,"tripo_mm.glb")) else os.path.join(ROOT,"assets","ORS6_tripo.glb"), process=False)
    if isinstance(tp, trimesh.Scene): tp = tp.to_geometry()
    return np.asarray(tp.vertices, float), np.asarray(tp.faces, int)

def kabsch(A, B):
    """Rigid R,t (no scale) so R@A.T+t ~ B. A,B: (N,3)."""
    ca = A.mean(0); cb = B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    return R, t

def trimmed_icp(src, tree, R0, t0, keep=0.65, iters=60):
    R, t = R0.copy(), t0.copy()
    last = 1e18
    for _ in range(iters):
        P = (R @ src.T).T + t
        d, idx = tree.query(P)
        thr = np.quantile(d, keep)
        m = d <= thr
        tg = tree.data[idx[m]]
        dR, dt = kabsch(P[m], tg)
        R = dR @ R; t = dR @ t + dt
        md = float(d[m].mean())
        if abs(last - md) < 1e-4: 
            last = md; break
        last = md
    return R, t, last

def fib_rots(n=80):
    """n rotation seeds: fibonacci directions for body +Z axis, 4 in-plane each."""
    out = []
    ga = np.pi * (3 - np.sqrt(5))
    for i in range(n):
        z = 1 - 2 * (i + 0.5) / n
        r = np.sqrt(max(0, 1 - z * z)); th = ga * i
        d = np.array([r * np.cos(th), r * np.sin(th), z])
        # rotation mapping +Z -> d
        a = np.array([0, 0, 1.0]); v = np.cross(a, d); c = a @ d
        if np.linalg.norm(v) < 1e-8:
            R0 = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
        else:
            vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
            R0 = np.eye(3) + vx + vx@vx/(1+c)
        for k in range(4):
            ang = k*np.pi/2
            cz, sz = np.cos(ang), np.sin(ang)
            Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1.0]])
            out.append(R0 @ Rz)
    return out

def main():
    Vt, Ft = load_tripo()
    print("tripo", Vt.shape, "bounds", Vt.min(0).round(1), Vt.max(0).round(1))
    tree = cKDTree(Vt)

    # body shell surface sample
    shell = load_concat(["Base","L_Frame","R_Frame","Lid"])
    src = shell.sample(9000)
    src = np.asarray(src, float)
    csrc = src.mean(0)

    # seed translation: align body centroid to high-X (body) half of Tripo
    xq = np.quantile(Vt[:,0], 0.45)
    bodyhalf = Vt[Vt[:,0] > xq]
    cbody = bodyhalf.mean(0)
    print("body-half centroid seed", cbody.round(1))

    best = None
    for R0 in fib_rots(80):
        t0 = cbody - (R0 @ csrc)
        R, t, md = trimmed_icp(src, tree, R0, t0, keep=0.6, iters=40)
        if best is None or md < best[2]:
            best = (R, t, md)
    R, t, md = best
    print("coarse best trimmed-mean-dist", round(md,2))
    # refine with higher keep
    R, t, md = trimmed_icp(src, tree, R, t, keep=0.7, iters=250)
    print("refined trimmed-mean-dist", round(md,2))

    np.savez(os.path.join(WORK,"kfit_body.npz"), R=R, t=t, rmse=md)

    # ---- verification render ----
    Vs = np.asarray(shell.vertices); Fs = np.asarray(shell.faces)
    Vsx = (R @ Vs.T).T + t
    Cs = np.tile([0.85,0.12,0.12],(len(Vsx),1))
    Ct = np.tile([0.20,0.55,0.95],(len(Vt),1))
    allV = np.vstack([Vsx, Vt]); center=(allV.min(0)+allV.max(0))/2
    rows=[]
    for vd in [(1,-1,0.5),(0,-1,0.1),(1,0,0.1),(0,0,1)]:
        span=max(np.ptp(allV@_look_at(vd)[0]),np.ptp(allV@_look_at(vd)[1]))*1.1
        it=render_fixed(Vt,Ft,Ct,vd,center,span,W=440,H=440)
        Vo=np.vstack([Vsx,Vt]); Fo=np.vstack([Fs,Ft+len(Vsx)]); Co=np.vstack([Cs,Ct])
        io=render_fixed(Vo,Fo,Co,vd,center,span,W=440,H=440)
        rows.append(np.hstack([it,io]))
    Image.fromarray(np.vstack(rows)).save(os.path.join(WORK,"kreg_body.png"))
    print("saved kreg_body.png  (left=tripo, right=overlay red=body)")

if __name__=="__main__":
    main()
