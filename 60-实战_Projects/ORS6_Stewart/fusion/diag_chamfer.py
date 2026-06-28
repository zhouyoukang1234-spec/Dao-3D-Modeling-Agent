# -*- coding: utf-8 -*-
import os,sys,numpy as np,trimesh
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
WORK=os.path.join(HERE,"data"); sys.path.insert(0,WORK); sys.path.insert(0,HERE)
from scipy.spatial import cKDTree
import kassemble as K
parts=K.build()
tp=trimesh.load(os.path.join(WORK,"tripo_mm.glb") if os.path.exists(os.path.join(WORK,"tripo_mm.glb")) else os.path.join(ROOT,"assets","ORS6_tripo.glb"),process=False)
if isinstance(tp,trimesh.Scene): tp=tp.to_geometry()
Pt=tp.sample(120000); tt=cKDTree(Pt)
def chf(names_pred):
    Vs=[];Fs=[];off=0
    for V,F,c in parts:
        if names_pred(c):
            Vs.append(V);Fs.append(F+off);off+=len(V)
    if not Vs: return None
    m=trimesh.Trimesh(np.vstack(Vs),np.vstack(Fs),process=False)
    P=m.sample(40000); d,_=tt.query(P)
    return d.mean(),np.median(d),np.percentile(d,90)
RED=[.84,.16,.16]; FR=[.84,.16,.16]; RECV=[.78,.12,.12]; HORN=[.94,.92,.86]; BALL=[.76,.78,.82]
body=chf(lambda c: list(c)==K.PAL["body"] or list(c)==K.PAL["frame"])
recv=chf(lambda c: list(c)==K.PAL["recv"])
rod=chf(lambda c: list(c)==K.PAL["rod"])
horn=chf(lambda c: list(c)==K.PAL["horn"])
for nm,r in [("body",body),("recv",recv),("rod",rod),("horn",horn)]:
    if r: print(f"{nm:6s} mean {r[0]:.2f} med {r[1]:.2f} p90 {r[2]:.2f}")
