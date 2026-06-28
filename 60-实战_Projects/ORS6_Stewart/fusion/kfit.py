# -*- coding: utf-8 -*-
"""Stage 3: kinematic receiver-pose fit. ONE 6-DOF receiver displacement drives
per-leg inverse kinematics so every rod connects arm-tip<->receiver-mount at the
fixed 175mm length, and every arm connects to its servo shaft. Nothing floats.
The single pose is fit to the Tripo truth. Frame = Tripo mm. Saves kfit_pose.npz.
"""
import os, sys, math, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)),"data")
PROJ = os.path.dirname(ROOT)
sys.path.insert(0, PROJ); sys.path.insert(0, ROOT); sys.path.insert(0, WORK); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
from scipy.spatial import cKDTree
from scipy.optimize import minimize
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME
from ORS6_Stewart.parts import SERVO_SLOTS, SR6, stl_path

ROD = 175.0

def load_tripo():
    tp = trimesh.load(os.path.join(WORK,"tripo_mm.glb") if os.path.exists(os.path.join(WORK,"tripo_mm.glb")) else os.path.join(ROOT,"assets","ORS6_tripo.glb"), process=False)
    if isinstance(tp, trimesh.Scene): tp = tp.to_geometry()
    return np.asarray(tp.vertices, float), np.asarray(tp.faces, int)

def expm_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12: return np.eye(3)
    k = w / th
    K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3) + math.sin(th)*K + (1-math.cos(th))*(K@K)

def main():
    Vt, Ft = load_tripo()
    tree = cKDTree(Vt)
    bf = np.load(os.path.join(WORK,"kfit_body.npz"))
    bR, bt = bf["R"], bf["t"]
    def Tb(P): return (bR @ np.asarray(P,float).T).T + bt

    ik = StewartIK()
    g = ik.compute_full_geometry(*TCODE_HOME)
    snames = [s for s,_,_,_,_ in SERVO_SLOTS]
    # per-servo geometry in CAD
    piv_cad=[]; u_cad=[]; L=[]; prefer=[]
    for s,stype,sx,sy,_ in SERVO_SLOTS:
        sign_x = 1 if sx>0 else -1
        piv_cad.append([sx,sy,SR6["servoPivotH"]])
        u_cad.append([-sign_x,0,0])     # cos basis
        L.append(SR6["pitchArm"] if stype=="pitch" else SR6["mainArm"])
        prefer.append(g["arm_angles"][s])
    piv_cad=np.array(piv_cad); u_cad=np.array(u_cad); L=np.array(L); prefer=np.array(prefer)
    w_cad=np.tile([0,0,1.0],(6,1))
    # to Tripo frame
    Piv = Tb(piv_cad)
    U = (bR @ u_cad.T).T          # rotate direction (no translation)
    Wv = (bR @ w_cad.T).T
    # home mounts in Tripo
    M0 = Tb(np.array([g["recv_mounts"][s] for s in snames]))
    C0 = M0.mean(0)
    # receiver ring STL points (home, Tripo)
    rm = trimesh.load(stl_path("Receiver"), force="mesh")
    ringP_cad = np.asarray(rm.sample(1800), float)
    # place ring STL at home: its center -> mounts centroid in CAD
    rc_cad = (rm.bounds[0]+rm.bounds[1])/2
    mc_cad = np.array([g["recv_mounts"][s] for s in snames]).mean(0)
    ringP_home = ringP_cad - rc_cad + mc_cad     # CAD home placement
    RingP0 = Tb(ringP_home)                       # Tripo home

    # init receiver pose from detected ring circle
    rc = np.load(os.path.join(WORK,"ring_circle.npz"))
    ring_ctr = rc["center"]; ring_nrm = rc["normal"]/np.linalg.norm(rc["normal"])
    # home receiver axis in Tripo = Tb_R @ z
    home_axis = bR @ np.array([0,0,1.0]); home_axis/=np.linalg.norm(home_axis)
    if home_axis @ ring_nrm < 0: ring_nrm = -ring_nrm
    # rotation aligning home_axis -> ring_nrm
    v=np.cross(home_axis,ring_nrm); c=home_axis@ring_nrm
    if np.linalg.norm(v)<1e-8: Rinit=np.eye(3) if c>0 else np.diag([1,-1,-1.0])
    else:
        vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
        Rinit=np.eye(3)+vx+vx@vx/(1+c)
    tinit = ring_ctr - (Rinit@(C0-C0)+C0)   # = ring_ctr - C0
    tinit = ring_ctr - C0

    def solve_legs(M):
        """M: (6,3) mount positions. Return tips (6,3), penalty."""
        tips=np.zeros((6,3)); pen=0.0
        for i in range(6):
            a = Piv[i]-M[i]
            A = 2*L[i]*(a@U[i]); B = 2*L[i]*(a@Wv[i])
            C = ROD**2 - (a@a) - L[i]**2
            Rr = math.hypot(A,B)
            if Rr < 1e-9:
                tips[i]=Piv[i]+L[i]*U[i]; pen+=abs(C); continue
            if Rr < abs(C):
                pen += abs(C)-Rr
                cc = max(-1.0,min(1.0, C/Rr))
            else:
                cc = C/Rr
            phi=math.atan2(B,A); d=math.acos(max(-1.0,min(1.0,cc)))
            th1=phi+d; th2=phi-d
            t1=Piv[i]+L[i]*(math.cos(th1)*U[i]+math.sin(th1)*Wv[i])
            t2=Piv[i]+L[i]*(math.cos(th2)*U[i]+math.sin(th2)*Wv[i])
            # data-driven branch: tip closest to Tripo surface (no flips)
            d1,_=tree.query(t1); d2,_=tree.query(t2)
            tips[i]= t1 if d1<=d2 else t2
        return tips, pen

    # receiver = the detected Tripo ring (center+normal+radius are ground truth).
    # the 6-mount constellation is rigidly pinned: centroid -> ring center,
    # axis -> ring normal; only orientation (incl. twist) is free -> fuses 1:1.
    def mounts_of(p):
        Rg = Rinit @ expm_so3(p[:3])
        M = (Rg @ (M0-C0).T).T + C0 + tinit
        return M - M.mean(0) + ring_ctr, Rg     # hard-pin centroid to ring

    def objective(p):
        M,Rg = mounts_of(p)
        tips,pen = solve_legs(M)
        rods=[np.linspace(tips[i],M[i],16) for i in range(6)]
        dd,_=tree.query(np.vstack(rods))
        return dd.mean() + 0.05*pen

    # twist scan about receiver axis (the dominant DOF for rod directions)
    def axis_angle(axis, ang):
        k=axis/np.linalg.norm(axis)
        K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
        return np.eye(3)+math.sin(ang)*K+(1-math.cos(ang))*(K@K)
    def rotvec_from_R(R):
        ang=math.acos(max(-1,min(1,(np.trace(R)-1)/2)))
        if ang<1e-8: return np.zeros(3)
        w=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/(2*math.sin(ang))
        return w*ang
    best=None
    for phi in np.linspace(0,2*math.pi,360,endpoint=False):
        Rg=axis_angle(ring_nrm,phi)@Rinit
        w=rotvec_from_R(Rinit.T@Rg)
        p=np.zeros(6); p[:3]=w
        f=objective(p)
        if best is None or f<best[1]: best=(p,f)
    print("twist-scan best", round(best[1],2))
    p=best[0]
    def obj3(w):
        q=np.zeros(6); q[:3]=w; return objective(q)
    for _ in range(4):
        res=minimize(obj3,p[:3],method="Powell",
                     options={"maxiter":6000,"xtol":1e-4,"ftol":1e-4})
        p=np.zeros(6); p[:3]=res.x
    print("optimized objective", round(res.fun,2))
    M,Rg = mounts_of(p)
    tips,pen = solve_legs(M)
    print("leg penalty", round(pen,2))
    rl=np.linalg.norm(tips-M,axis=1)
    print("rod lengths", rl.round(1))
    np.savez(os.path.join(WORK,"kfit_pose.npz"),
             p=p, Rinit=Rinit, tinit=tinit, C0=C0, M0=M0,
             tips=tips, mounts=M, Rg=Rg,
             ring_ctr=ring_ctr, ring_nrm=ring_nrm, ring_r=float(rc["radius"]))
    print("saved kfit_pose.npz; mounts centroid", M.mean(0).round(1))

if __name__=="__main__":
    main()
