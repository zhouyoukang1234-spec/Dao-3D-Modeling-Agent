# -*- coding: utf-8 -*-
"""Render shell, tripo_mm, and their overlay in a COMMON camera frame to judge
registration quality."""
import os, sys, math
import numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)),"data")
sys.path.insert(0,ROOT)
import trimesh, parts as P
from PIL import Image

def load_concat(names):
    ms=[]
    for n in names:
        m=P.load_stl(n)
        if isinstance(m,trimesh.Scene): m=m.to_geometry()
        ms.append(m)
    return trimesh.util.concatenate(ms)

def _look_at(vd,up=(0,0,1)):
    f=np.array(vd,float); f/=np.linalg.norm(f); up=np.array(up,float)
    r=np.cross(f,up)
    if np.linalg.norm(r)<1e-6: up=np.array([0,1.,0]); r=np.cross(f,up)
    r/=np.linalg.norm(r); u=np.cross(r,f); return r,u,f

def render_fixed(V,F,C,vd,center,span,W=560,H=560,bg=(1,1,1),ambient=0.4,light=(0.3,-0.6,0.8)):
    r,u,f=_look_at(vd); P_=V-center
    x=P_@r; y=P_@u; z=P_@f
    nrm=np.array(light,float); nrm/=np.linalg.norm(nrm)
    v0=V[F[:,0]];v1=V[F[:,1]];v2=V[F[:,2]]
    fn=np.cross(v1-v0,v2-v0); ln=np.linalg.norm(fn,axis=1,keepdims=True); ln[ln==0]=1; fn=fn/ln
    px=x/span*W+W/2; py=H/2-y/span*H
    img=np.ones((H,W,3))*np.array(bg); zb=np.full((H,W),1e18); fp=np.stack([px,py],1)
    order=np.argsort(-z[F].mean(1))
    for i in order:
        a,b,c=F[i]; x0,y0=fp[a];x1,y1=fp[b];x2,y2=fp[c]
        minx=int(max(0,math.floor(min(x0,x1,x2))));maxx=int(min(W-1,math.ceil(max(x0,x1,x2))))
        miny=int(max(0,math.floor(min(y0,y1,y2))));maxy=int(min(H-1,math.ceil(max(y0,y1,y2))))
        if minx>maxx or miny>maxy: continue
        den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(den)<1e-9: continue
        ys,xs=np.mgrid[miny:maxy+1,minx:maxx+1]
        l0=((y1-y2)*(xs-x2)+(x2-x1)*(ys-y2))/den
        l1=((y2-y0)*(xs-x2)+(x0-x2)*(ys-y2))/den
        l2=1-l0-l1; ins=(l0>=0)&(l1>=0)&(l2>=0)
        if not ins.any(): continue
        zf=l0*z[a]+l1*z[b]+l2*z[c]
        sh=ambient+(1-ambient)*max(0.,abs(float(fn[i]@nrm)))
        col=(l0[...,None]*C[a]+l1[...,None]*C[b]+l2[...,None]*C[c])*sh
        yy=ys[ins];xx=xs[ins];zz=zf[ins];cc=col[ins]
        cl=zz<zb[yy,xx]; yy,xx,cc=yy[cl],xx[cl],cc[cl]; zb[yy,xx]=zz[cl]; img[yy,xx]=np.clip(cc,0,1)
    return (img*255).astype(np.uint8)

if __name__=="__main__":
 shell=load_concat(["Base","L_Frame","R_Frame","L_Pitcher","R_Pitcher","Arm","Lid"])
 tripo=trimesh.load(os.path.join(WORK,"tripo_mm.glb") if os.path.exists(os.path.join(WORK,"tripo_mm.glb")) else os.path.join(ROOT,"assets","ORS6_tripo.glb"),process=False)
 if isinstance(tripo,trimesh.Scene): tripo=tripo.to_geometry()

 Vs=np.asarray(shell.vertices); Fs=np.asarray(shell.faces)
 Vt=np.asarray(tripo.vertices); Ft=np.asarray(tripo.faces)
 Cs=np.tile([0.85,0.15,0.15],(len(Vs),1))      # shell red
 Ct=np.tile([0.20,0.55,0.95],(len(Vt),1))      # tripo blue
 allV=np.vstack([Vs,Vt]); center=(allV.min(0)+allV.max(0))/2
 Vo=np.vstack([Vs,Vt]); Fo=np.vstack([Fs,Ft+len(Vs)]); Co=np.vstack([Cs,Ct])
 out=os.path.join(WORK,"out_overlay"); os.makedirs(out,exist_ok=True)
 rows=[]
 for vname,vd in [("iso",(1,-1,0.5)),("front",(0,-1,0.12)),("side",(1,0,0.12))]:
    span=max(np.ptp(allV@_look_at(vd)[0]),np.ptp(allV@_look_at(vd)[1]))*1.12
    ish=render_fixed(Vs,Fs,Cs,vd,center,span)
    it=render_fixed(Vt,Ft,Ct,vd,center,span)
    io=render_fixed(Vo,Fo,Co,vd,center,span)
    row=np.hstack([ish,it,io]); rows.append(row)
    Image.fromarray(row).save(os.path.join(out,f"ov_{vname}.png"))
 big=np.vstack(rows); Image.fromarray(big).save(os.path.join(WORK,"overlay_montage.png"))
 print("saved overlay_montage.png")
