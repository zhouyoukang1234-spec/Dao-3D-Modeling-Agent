#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render ONLY the bottom core module (base + 2 frames + 6 servos + 6 arms) at
home, multiple views, to compare directly against PDF p24 (front 'straight line'
photo) and p29 (frame assembly)."""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, warnings
warnings.filterwarnings("ignore")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import build_core as bc, solve_home as sh

OUT = r"C:/Users/Administrator/sr6_build/renders"

def bottom_scene(pose):
    sol = sh.solve_home_at(pose)
    scene=[]
    scene.append(("base", bc.load("底座"), (0.18,0.18,0.20,1.0)))
    scene.append(("R_frame", bc.apply(bc.frame_M("R"), bc.load("R-Frame")), (0.10,0.10,0.12,1.0)))
    scene.append(("L_frame", bc.apply(bc.frame_M("L"), bc.load("L形框架")), (0.13,0.13,0.15,1.0)))
    for leg,(side,yp,kind) in bc.LEG2BUILD.items():
        th,tip,B,rod,err = sol[leg]
        O = sh.SERVOS[leg][0]; td = bc.unit(tip - O)
        scene.append((f"servo_{leg}", bc.servo_box(O), (0.0,0.1,0.55,1.0)))
        if kind=="main":
            arm=bc.load("SR6 臂"); M,bw=bc.place_arm(arm, bc.ARM_HUB, bc.ARM_BALL, O, td)
            scene.append((f"arm_{leg}", bc.apply(M,arm), (0.95,0.95,0.95,1.0)))
        else:
            parm=bc.load("R-投手" if side=="R" else "L-投手")
            hub=bc.RP_HUB if side=="R" else bc.LP_HUB
            ball=bc.RP_BALL if side=="R" else bc.LP_BALL
            M,bw=bc.place_arm(parm, hub, ball, O, td)
            scene.append((f"pitch_{leg}", bc.apply(M,parm), (0.88,0.88,0.88,1.0)))
    return scene

def render(pose, fname):
    scene = bottom_scene(pose)
    allv=np.vstack([m.vertices for _,m,_ in scene])
    c=allv.mean(0); r=(allv.max(0)-allv.min(0)).max()/2*1.05
    views=[("front (vs p24)",(0,-90)),("iso",(20,-60)),("right",(0,0)),("top",(89,-90))]
    fig=plt.figure(figsize=(15,4))
    for i,(nm,(el,az)) in enumerate(views):
        ax=fig.add_subplot(1,4,i+1,projection="3d")
        for _,mesh,col in scene:
            f=mesh.faces; v=mesh.vertices; step=max(1,len(f)//9000)
            pc=Poly3DCollection(v[f[::step]],alpha=0.95)
            pc.set_facecolor(col); pc.set_edgecolor((0,0,0,0.08)); pc.set_linewidth(0.1)
            ax.add_collection3d(pc)
        ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=el,azim=az)
        ax.set_axis_off(); ax.set_title(nm,fontsize=10)
    fig.suptitle("SR6 bottom core @ home (base+2 frames+6 servos+6 arms)")
    fig.tight_layout(); p=os.path.join(OUT,fname); fig.savefig(p,dpi=110); plt.close(fig)
    print("wrote",p); return p

if __name__=="__main__":
    render([0,0,232,0,0,0], "sr6_bottom_home.png")
