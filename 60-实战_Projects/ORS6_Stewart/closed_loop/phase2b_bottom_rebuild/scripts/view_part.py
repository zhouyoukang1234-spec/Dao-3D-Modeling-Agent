#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render a single STL part from 6 axis views with labeled axes, to understand
its authored orientation (find servo mounts, bores, etc)."""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, warnings
warnings.filterwarnings("ignore")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import build_core as bc

OUT = r"C:/Users/Administrator/sr6_build/renders"

def view(frag, fname):
    m = bc.load(frag)
    lo,hi = m.bounds; c=(lo+hi)/2; r=(hi-lo).max()/2*1.05
    views=[("+X look (YZ)",(0,0)),("-Y look (XZ)",(0,-90)),("+Z look (XY top)",(89,-90)),
           ("iso1",(25,-60)),("iso2",(25,120)),("-Z look (bottom)",(-89,-90))]
    fig=plt.figure(figsize=(15,9))
    for i,(nm,(el,az)) in enumerate(views):
        ax=fig.add_subplot(2,3,i+1,projection="3d")
        f=m.faces; v=m.vertices; step=max(1,len(f)//12000)
        pc=Poly3DCollection(v[f[::step]],alpha=0.9)
        pc.set_facecolor((0.6,0.6,0.65,1)); pc.set_edgecolor((0,0,0,0.15)); pc.set_linewidth(0.2)
        ax.add_collection3d(pc)
        ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=el,azim=az)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.set_title(nm,fontsize=9)
    fig.suptitle(f"{frag}  size={np.round(hi-lo,1)}  lo={np.round(lo,1)} hi={np.round(hi,1)}")
    fig.tight_layout(); p=os.path.join(OUT,fname); fig.savefig(p,dpi=95); plt.close(fig)
    print("wrote",p)

if __name__=="__main__":
    view(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else "part_view.png")
