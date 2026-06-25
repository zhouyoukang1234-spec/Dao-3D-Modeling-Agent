#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detect cylindrical bores (servo shaft holes / horn recesses) in a part STL.

For each principal axis a, pick faces whose normal is ~perpendicular to a (side
walls of a hole whose axis ‖ a), cluster their centroids in the plane ⟂ a, fit a
circle to each cluster, and report center+radius+axis-span.  This is rotation-
agnostic ground truth for servo positions."""
import os, sys, math
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, warnings
warnings.filterwarnings("ignore")
import build_core as bc

AXES = {"X":np.array([1.,0,0]), "Y":np.array([0,1.,0]), "Z":np.array([0,0,1.])}

def fit_circle(P2):
    """algebraic circle fit to Nx2 pts -> (cx,cy,r,resid)."""
    x=P2[:,0]; y=P2[:,1]
    A=np.column_stack([2*x,2*y,np.ones(len(x))]); b=x*x+y*y
    sol,_,_,_=np.linalg.lstsq(A,b,rcond=None)
    cx,cy,c=sol; r=math.sqrt(max(0,c+cx*cx+cy*cy))
    resid=float(np.sqrt(np.mean((np.hypot(x-cx,y-cy)-r)**2)))
    return cx,cy,r,resid

def detect(frag, rmin=3.0, rmax=20.0):
    m=bc.load(frag)
    fn=m.face_normals; fc=m.triangles_center
    print(f"=== {frag}  faces={len(fc)} bounds={np.round(m.bounds,1).tolist()}")
    found=[]
    for an,a in AXES.items():
        i,j = [(1,2),(0,2),(0,1)][["X","Y","Z"].index(an)]
        perp = np.abs(fn@a) < 0.25
        C = fc[perp]
        if len(C)<20: continue
        P2 = C[:,[i,j]]
        # grid cluster
        from scipy.cluster.hierarchy import fcluster, linkage
        if len(P2)>4000:
            idx=np.random.RandomState(0).choice(len(P2),4000,replace=False); P2s=P2[idx]; Cs=C[idx]
        else: P2s=P2; Cs=C
        Z=linkage(P2s,method="single",metric="euclidean")
        lab=fcluster(Z,t=8.0,criterion="distance")
        for L in np.unique(lab):
            sel=lab==L
            if sel.sum()<15: continue
            pts=P2s[sel]
            spread=pts.max(0)-pts.min(0)
            if spread.max()>2.2*rmax or spread.min()<2: continue
            cx,cy,r,res=fit_circle(pts)
            if not (rmin<=r<=rmax) or res>1.5: continue
            aspan=(Cs[sel][:,["X","Y","Z"].index(an)].min(),
                   Cs[sel][:,["X","Y","Z"].index(an)].max())
            ctr=np.zeros(3); ctr[i]=cx; ctr[j]=cy; ctr[["X","Y","Z"].index(an)]=np.mean(aspan)
            found.append((an,r,res,ctr,aspan,sel.sum()))
    # dedup-ish print, sorted by radius desc then n
    found.sort(key=lambda t:(-t[5]))
    for an,r,res,ctr,aspan,n in found:
        print(f"  axis={an}  r={r:5.1f}  resid={res:4.2f}  center={np.round(ctr,1).tolist()}"
              f"  span[{an}]={np.round(aspan,1).tolist()}  nfaces={n}")
    return found

if __name__=="__main__":
    detect(sys.argv[1] if len(sys.argv)>1 else "R-Frame")
