#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SR6 bottom-core MOTION proof.

Sweep the receiver through a 6-DOF trajectory (heave/surge/sway/roll/pitch/yaw).
For every pose, per-leg analytic IK solves the 6 servo angles such that all 6
rods stay EXACTLY 175 mm.  We verify rod-length invariance + reachability across
the whole trajectory, then render frames and stitch a GIF -- visual proof the
mechanism truly moves, not a rigid/fixed model.
"""
import os, sys, math, json
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, warnings
warnings.filterwarnings("ignore")
import solve_home as sh
import build_core as bc

OUT = r"C:/Users/Administrator/sr6_build/renders"
H0 = 232.0   # home height

def trajectory(n_per=18):
    """list of (label, pose). Smooth excursions about home, one DOF at a time."""
    segs = [
        ("heave",  lambda t: [0,0,225.0+17.0*math.sin(t),0,0,0]),
        ("surge",  lambda t: [22*math.sin(t),0,H0,0,0,0]),
        ("sway",   lambda t: [0,22*math.sin(t),H0,0,0,0]),
        ("roll",   lambda t: [0,0,H0,math.radians(14)*math.sin(t),0,0]),
        ("pitch",  lambda t: [0,0,H0,0,math.radians(14)*math.sin(t),0]),
        ("yaw",    lambda t: [0,0,H0,0,0,math.radians(14)*math.sin(t)]),
    ]
    poses=[]
    for name,f in segs:
        for k in range(n_per):
            t = 2*math.pi*k/n_per
            poses.append((name, f(t)))
    return poses

def check_pose(pose, tol=1e-6):
    sol = sh.solve_home_at(pose)
    rods = np.array([sol[n][3] for n in sh.SERVOS])
    errs = np.array([abs(sol[n][4]) for n in sh.SERVOS])
    reachable = bool(errs.max() < tol)
    return rods, errs.max(), reachable, sol

def run_verify():
    poses = trajectory()
    print(f"verifying {len(poses)} poses across 6 DOF ...")
    worst_dev = 0.0; n_ok = 0; rows=[]
    for name, pose in poses:
        rods, maxerr, ok, _ = check_pose(pose)
        dev = float(np.abs(rods - sh.ROD).max())
        worst_dev = max(worst_dev, dev if ok else 0.0)
        n_ok += int(ok)
        rows.append({"seg":name,"pose":[round(p,3) for p in pose],
                     "rod_min":float(rods.min()),"rod_max":float(rods.max()),
                     "max_dev_mm":dev,"reachable":ok})
    summary = {"n_poses":len(poses),"n_reachable":n_ok,
               "worst_rod_dev_mm_when_reachable":worst_dev,"rows":rows}
    json.dump(summary, open(os.path.join(OUT,"motion_report.json"),"w"), indent=2)
    print(f"  reachable {n_ok}/{len(poses)}  |  worst rod deviation (reachable) "
          f"= {worst_dev:.3e} mm")
    print(f"  wrote {OUT}/motion_report.json")
    return poses, summary

def render_gif(poses, stride=3):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    try:
        import imageio.v2 as imageio
    except Exception:
        import imageio
    frames=[]
    sel = poses[::stride]
    print(f"rendering {len(sel)} frames ...")
    # fixed view bounds from home build
    scene0,_ = bc.build(pose=[0,0,H0,0,0,0])
    allv=np.vstack([m.vertices for _,m,_ in scene0])
    c=allv.mean(0); r=(allv.max(0)-allv.min(0)).max()/2*1.10
    for i,(name,pose) in enumerate(sel):
        scene,_ = bc.build(pose=pose)
        fig=plt.figure(figsize=(9,5))
        for vi,(vname,(el,az)) in enumerate([("front",(8,-90)),("iso",(22,-58))]):
            ax=fig.add_subplot(1,2,vi+1,projection="3d")
            for nm,mesh,col in scene:
                f=mesh.faces; v=mesh.vertices; step=max(1,len(f)//6000)
                pc=Poly3DCollection(v[f[::step]],alpha=0.92)
                pc.set_facecolor(col); pc.set_edgecolor((0,0,0,0.05)); pc.set_linewidth(0.1)
                ax.add_collection3d(pc)
            ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
            ax.set_box_aspect((1,1,1)); ax.view_init(elev=el,azim=az)
            ax.set_axis_off(); ax.set_title(f"{vname}")
        fig.suptitle(f"SR6 motion — {name}   pose=[{pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},"
                     f"{math.degrees(pose[3]):.0f}°,{math.degrees(pose[4]):.0f}°,{math.degrees(pose[5]):.0f}°]")
        fig.tight_layout()
        p=os.path.join(OUT,f"_motion_{i:03d}.png")
        fig.savefig(p,dpi=70); plt.close(fig); frames.append(imageio.imread(p))
    gif=os.path.join(OUT,"sr6_motion.gif")
    imageio.mimsave(gif, frames, duration=0.08, loop=0)
    print(f"  wrote {gif}")
    return gif

if __name__=="__main__":
    poses, summary = run_verify()
    if "--gif" in sys.argv:
        render_gif(poses)
