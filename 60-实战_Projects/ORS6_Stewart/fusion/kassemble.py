# -*- coding: utf-8 -*-
"""Build the fully-connected kinematic assembly from kfit_body + kfit_pose,
render colored 4-view + overlay on Tripo. Frame = Tripo mm."""
import os, sys, math, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)),"data")
PROJ = os.path.dirname(ROOT)
sys.path.insert(0, PROJ); sys.path.insert(0, ROOT); sys.path.insert(0, WORK); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
from PIL import Image
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME, ARM_PIVOT_STL
from ORS6_Stewart.parts import (SERVO_SLOTS, SR6, PARTS, RECV_PARTS,
                                DEFAULT_HIDDEN, stl_path)
from overlay_check import render_fixed, _look_at

def L(name):
    p=stl_path(name)
    if not os.path.exists(p): return None,None
    m=trimesh.load(p,force="mesh")
    return np.asarray(m.vertices,float), np.asarray(m.faces,int)

def rot_a2b(a,b):
    a=a/(np.linalg.norm(a) or 1); b=b/(np.linalg.norm(b) or 1)
    v=np.cross(a,b); c=float(a@b)
    if np.linalg.norm(v)<1e-8: return np.eye(3) if c>0 else np.diag([1,-1,-1.0])
    vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+vx+vx@vx/(1+c)

def place_link(name, p0, p1):
    V,F=L(name)
    if V is None: return None,None
    c=(V.min(0)+V.max(0))/2
    ea=np.array([c[0],V[:,1].max(),c[2]]); eb=np.array([c[0],V[:,1].min(),c[2]])
    al=eb-ea; Ll=np.linalg.norm(al) or 1; 
    at=np.array(p1)-np.array(p0); Tl=np.linalg.norm(at) or 1
    R=rot_a2b(al,at)
    return (V-ea)@R.T*(Tl/Ll)+np.array(p0), F

def uvsphere(c,r=5.0,n=10):
    u=np.linspace(0,np.pi,n); v=np.linspace(0,2*np.pi,2*n)
    uu,vv=np.meshgrid(u,v)
    x=c[0]+r*np.sin(uu)*np.cos(vv); y=c[1]+r*np.sin(uu)*np.sin(vv); z=c[2]+r*np.cos(uu)
    V=np.stack([x.ravel(),y.ravel(),z.ravel()],1)
    F=[]; cols=2*n
    for i in range(cols-1):
        for j in range(n-1):
            a=i*n+j; b=a+n
            F+=[[a,b,a+1],[b,b+1,a+1]]
    return V,np.array(F)

PAL={"body":[.72,.10,.10],"frame":[.82,.18,.18],"recv":[.20,.32,.65],
     "rod":[.75,.75,.78],"horn":[.95,.93,.88],"ball":[.55,.56,.60]}

def build():
    bf=np.load(os.path.join(WORK,"kfit_body.npz")); bR,bt=bf["R"],bf["t"]
    def Tb(P): return (bR@np.asarray(P,float).T).T+bt
    pf=np.load(os.path.join(WORK,"kfit_pose.npz"))
    Rg=pf["Rg"]; C0=pf["C0"]; tinit=pf["tinit"]; p=pf["p"]
    tips=pf["tips"]; mounts=pf["mounts"]
    ttot=tinit+p[3:]
    def recv_xf(Vc, rc_cad, mc_cad):
        home=Tb(Vc-rc_cad+mc_cad)
        return (Rg@(home-C0).T).T+C0+ttot

    ik=StewartIK(); g=ik.compute_full_geometry(*TCODE_HOME)
    home=g
    snames=[s for s,_,_,_,_ in SERVO_SLOTS]
    mc_cad=np.array([g["recv_mounts"][s] for s in snames]).mean(0)

    parts=[]  # (V,F,color)
    static=[n for n in PARTS if n not in RECV_PARTS and n not in DEFAULT_HIDDEN
            and n not in ("Arm","L_Pitcher","R_Pitcher")]
    for nm in static:
        V,F=L(nm)
        if V is None: continue
        parts.append((Tb(V),F,PAL["frame"] if nm in("L_Frame","R_Frame") else PAL["body"]))
    # receiver = the real device's simple ring, rebuilt as a torus on the
    # detected Tripo circle (center+normal+radius) -> fuses 1:1 with Tripo.
    rc_ctr=pf["ring_ctr"]; rc_nrm=pf["ring_nrm"]/np.linalg.norm(pf["ring_nrm"])
    rc_R=float(pf["ring_r"]); tube=7.0
    e1=np.cross(rc_nrm,[0,0,1.0])
    if np.linalg.norm(e1)<1e-6: e1=np.cross(rc_nrm,[0,1.0,0])
    e1/=np.linalg.norm(e1); e2=np.cross(rc_nrm,e1)
    nu,nv=120,18; Vr=[]; Fr=[]
    for iu in range(nu):
        a=2*np.pi*iu/nu
        cen=rc_ctr+rc_R*(np.cos(a)*e1+np.sin(a)*e2)
        radial=np.cos(a)*e1+np.sin(a)*e2
        for iv in range(nv):
            b=2*np.pi*iv/nv
            Vr.append(cen+tube*(np.cos(b)*radial+np.sin(b)*rc_nrm))
    for iu in range(nu):
        for iv in range(nv):
            a0=iu*nv+iv; a1=((iu+1)%nu)*nv+iv
            b0=iu*nv+(iv+1)%nv; b1=((iu+1)%nu)*nv+(iv+1)%nv
            Fr+=[[a0,a1,b1],[a0,b1,b0]]
    parts.append((np.array(Vr),np.array(Fr),PAL["recv"]))
    # arms: solved angle. tip known -> rotate Arm STL about Y shaft axis ONLY
    # (physical constraint: servo horn swings in the XZ plane at fixed Y=sy)
    Va,Fa=L("Arm")
    for i,(s,stype,sx,sy,_) in enumerate(SERVO_SLOTS):
        shaft_cad=np.array([sx,sy,SR6["servoPivotH"]])
        htip=np.array(home["arm_tips"][s])
        stip=(bR.T@(tips[i]-bt))            # solved tip back in CAD
        v_home=htip-shaft_cad; v_sol=stip-shaft_cad
        # project onto XZ plane and compute angle delta about Y axis
        ah=math.atan2(v_home[2],v_home[0])
        a_s=math.atan2(v_sol[2],v_sol[0])
        delta=ah-a_s  # sign: Ry convention subtracts angle
        cd,sd=math.cos(delta),math.sin(delta)
        Rr=np.array([[cd,0,sd],[0,1,0],[-sd,0,cd]])  # Ry(delta)
        if stype=="main":
            is_left=sx<0
            V=Va.copy(); F=Fa
            if is_left:
                V=V*np.array([-1,1,1.0]); F=Fa[:,::-1]
                piv=np.array([-ARM_PIVOT_STL[0],ARM_PIVOT_STL[1],ARM_PIVOT_STL[2]])
            else:
                piv=np.array(ARM_PIVOT_STL)
            Vt=(V-piv)@Rr.T+shaft_cad
            parts.append((Tb(Vt),F,PAL["horn"]))
        else:   # pitch arm: rotate the real pitcher STL about Y shaft axis
            pn="L_Pitcher" if sx<0 else "R_Pitcher"
            V,F=L(pn)
            if V is None: continue
            Vt=(V-shaft_cad)@Rr.T+shaft_cad
            parts.append((Tb(Vt),F,PAL["horn"]))
    # rods + balls
    for i,(s,stype,_,_,_) in enumerate(SERVO_SLOTS):
        link="MainLink" if stype=="main" else "PitcherLink"
        V,F=place_link(link,tips[i],mounts[i])
        if V is not None: parts.append((V,F,PAL["rod"]))
        for pt in (tips[i],mounts[i]):
            Vs,Fs=uvsphere(pt,5.0); parts.append((Vs,Fs,PAL["ball"]))
    return parts

def render():
    parts=build()
    tp=trimesh.load(os.path.join(WORK,"tripo_mm.glb") if os.path.exists(os.path.join(WORK,"tripo_mm.glb")) else os.path.join(ROOT,"assets","ORS6_tripo.glb"),process=False)
    if isinstance(tp,trimesh.Scene): tp=tp.to_geometry()
    Vt=np.asarray(tp.vertices); Ft=np.asarray(tp.faces)
    # merge assembly
    allV=[]; allF=[]; allC=[]; off=0
    for V,F,c in parts:
        allV.append(V); allF.append(F+off); allC.append(np.tile(c,(len(V),1))); off+=len(V)
    Va=np.vstack(allV); Fa=np.vstack(allF); Ca=np.vstack(allC)
    Ct=np.tile([0.20,0.55,0.95],(len(Vt),1))
    big=np.vstack([Va,Vt]); center=(big.min(0)+big.max(0))/2
    rows=[]
    for vd in [(1,-1,0.5),(0,-1,0.1),(1,0,0.1),(0,0,1)]:
        span=max(np.ptp(big@_look_at(vd)[0]),np.ptp(big@_look_at(vd)[1]))*1.1
        ia=render_fixed(Va,Fa,Ca,vd,center,span,W=440,H=440)
        Vo=np.vstack([Va,Vt]); Fo=np.vstack([Fa,Ft+len(Va)]); Co=np.vstack([Ca,Ct])
        io=render_fixed(Vo,Fo,Co,vd,center,span,W=440,H=440)
        rows.append(np.hstack([ia,io]))
    Image.fromarray(np.vstack(rows)).save(os.path.join(WORK,"kassembly.png"))
    print("saved kassembly.png (left=assembly, right=overlay on Tripo blue)")
    # symmetric chamfer (assembly surface <-> Tripo)
    from scipy.spatial import cKDTree
    asm=trimesh.Trimesh(np.vstack([V for V,_,_ in parts]),
                        np.vstack([F+sum(len(p[0]) for p in parts[:i]) for i,(V,F,c) in enumerate(parts)]),
                        process=False)
    Pa=asm.sample(60000); Pt=trimesh.Trimesh(Vt,Ft,process=False).sample(60000)
    ta=cKDTree(Pa); tt=cKDTree(Pt)
    da,_=tt.query(Pa); db,_=ta.query(Pt)
    print(f"chamfer A->T mean {da.mean():.2f} med {np.median(da):.2f} p90 {np.percentile(da,90):.2f}")
    print(f"chamfer T->A mean {db.mean():.2f} med {np.median(db):.2f} p90 {np.percentile(db,90):.2f}")
    print(f"symmetric mean {0.5*(da.mean()+db.mean()):.2f}  median {0.5*(np.median(da)+np.median(db)):.2f}")
    export_glb(parts)

def export_glb(parts, path=None):
    allV=[]; allF=[]; allC=[]; off=0
    for V,F,c in parts:
        allV.append(V); allF.append(F+off)
        rgba=np.tile(np.r_[np.array(c)*255,255].astype(np.uint8),(len(V),1))
        allC.append(rgba); off+=len(V)
    m=trimesh.Trimesh(np.vstack(allV),np.vstack(allF),
                      vertex_colors=np.vstack(allC),process=False)
    path=path or os.path.join(WORK,"ORS6_fused.glb")
    m.export(path); print("saved",path)

if __name__=="__main__":
    render()
