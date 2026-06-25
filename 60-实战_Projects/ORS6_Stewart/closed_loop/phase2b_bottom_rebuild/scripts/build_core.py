#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SR6 bottom-core rebuild v3 — CORRECTED architecture (measurement + PDF driven).

反者道之动: the previous model put servo shafts ‖X (arms swing YZ).  p24 + STL prove
the real machine has TWO frames stacked in DEPTH (Y), each spanning the full width;
each frame carries 2 main servos at X=±30 (arms point ±X outward, horizontal "straight
line") + 1 pitch servo at X=0 (arm kinks up).  Servo output shafts are ‖Y.

Frame STLs are authored in print/layout pose:
  R-Frame  : X∈[47,110]  (the +X half)
  L形框架  : X∈[-110,-47] (mirror across X=0)
  print axes: X(62.5)=depth-of-frame, Y(112)=width(servo row), Z(54)=shaft direction
  3 servo shafts per frame: print (X≈105, Y∈{-30,0,30}, Z≈28), bore ‖print-Z.

Standing rotation = cyclic permutation Rstand: world_x=print_y, world_y=print_z,
world_z=print_x.  -> front (R) frame.  Back (L) frame = Rx(180)·Rstand.
Then translate down by T_z to nest both into the base cradle.
"""
import sys, os, glob, json, argparse, math
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np, trimesh, warnings
warnings.filterwarnings("ignore")

ROOT = os.environ.get("SR6_STL_ROOT",
    r"C:/Users/Administrator/sr6_build/parts_raw/SR6 完整资料进阶版本 签收后提供解压密码/STLs")
OUT = r"C:/Users/Administrator/sr6_build/renders"

# ---- measured local geometry (print coords, mm) ----------------------------
ARM_HUB  = np.array([67.5, 0.0, 48.37]);  ARM_BALL  = np.array([67.5, 50.0, 51.2])
LP_HUB   = np.array([-7.5, 30.0, 48.4]);  LP_BALL   = np.array([-39.7, 97.7, 50.3])
RP_HUB   = np.array([ 7.5, 30.0, 48.4]);  RP_BALL   = np.array([ 39.8, 97.7, 50.3])

# ---- frame placement params (tune by render) -------------------------------
T_Z = -44.0          # lower frames into base
SERVO_PRINT_X = 105.0
SERVO_PRINT_Z = 28.0
SERVO_YS = {"R": +1, "L": -1}   # front (R) at +Y depth, back (L) at -Y

def find(frag):
    c = [x for x in glob.glob(os.path.join(ROOT, "**", "*.stl"), recursive=True) if frag in x]
    if not c: raise FileNotFoundError(frag)
    return c[0]
def load(frag): return trimesh.load(find(frag), force="mesh")
def Tr(t):
    M = np.eye(4); M[:3,3]=t; return M
def Rmat(R3):
    M = np.eye(4); M[:3,:3]=R3; return M
def apply(M, m):
    x = m.copy(); x.apply_transform(M); return x
def unit(v):
    n=np.linalg.norm(v); return v/n if n>1e-12 else v

# cyclic standing rotation: world_x=print_y, world_y=print_z, world_z=print_x
RSTAND = np.array([[0,1,0],[0,0,1],[1,0,0]], float)
RX180  = np.array([[1,0,0],[0,-1,0],[0,0,-1]], float)

def frame_M(side):
    """side='R'(front) or 'L'(back)."""
    R3 = RSTAND if side=="R" else (RX180 @ RSTAND)
    return Tr([0,0,T_Z]) @ Rmat(R3)

def servo_world(side, yprint):
    """world position of a servo shaft given its print Y (∈{-30,0,30})."""
    sx = SERVO_PRINT_X if side=="R" else -SERVO_PRINT_X   # L-frame authored at -X
    p = np.array([sx, yprint, SERVO_PRINT_Z])
    R3 = RSTAND if side=="R" else (RX180 @ RSTAND)
    return R3 @ p + np.array([0,0,T_Z])

SERVO_AXIS_WORLD = np.array([0.,1.,0.])   # shafts ‖Y

def place_arm(mesh, hub, ball, O, target_dir, axis_world=SERVO_AXIS_WORLD):
    """Align arm: hub->O, horn-axis(local Z)->axis_world, ball-direction->target_dir."""
    e1 = np.array([0,0,1.]); v = ball-hub
    e2 = unit(v - e1*(e1@v)); e3 = np.cross(e1,e2)
    f1 = unit(axis_world); td = target_dir - f1*(f1@target_dir)
    f2 = unit(td); f3 = np.cross(f1,f2)
    R = np.column_stack([f1,f2,f3]) @ np.column_stack([e1,e2,e3]).T
    t = O - R@hub
    M = np.eye(4); M[:3,:3]=R; M[:3,3]=t
    ball_w = R@ball + t
    return M, ball_w

def servo_box(O, axis=SERVO_AXIS_WORLD, size=(40,20,38)):
    """LW-20MG-ish proxy: body hangs below shaft. shaft along Y; body长40(X? )."""
    b = trimesh.creation.box(extents=size)
    # body centered on shaft, dropped so shaft near top of body
    b.apply_translation([O[0], O[1], O[2]-size[2]/2+6])
    return b

def make_rod(p, q, r=4.0):
    """cylinder from p to q (rod proxy)."""
    p=np.asarray(p,float); q=np.asarray(q,float); v=q-p; L=np.linalg.norm(v)
    cyl=trimesh.creation.cylinder(radius=r, height=L, sections=16)
    z=np.array([0,0,1.]); d=unit(v); ax=np.cross(z,d); s=np.linalg.norm(ax)
    M=np.eye(4)
    if s>1e-9:
        ax/=s; ang=math.acos(max(-1,min(1,z@d)))
        M[:3,:3]=trimesh.transformations.rotation_matrix(ang,ax)[:3,:3]
    M[:3,3]=(p+q)/2; cyl.apply_transform(M); return cyl

# map solve_home leg names -> (frame side, servo print-Y, arm kind)
LEG2BUILD = {
    "R_front":("R", 30,"main"), "R_back":("L", 30,"main"),
    "L_front":("R",-30,"main"), "L_back":("L",-30,"main"),
    "P_front":("R",  0,"pitch"),"P_back":("L",  0,"pitch"),
}

def build(H=None, pose=None):
    import solve_home as sh
    if pose is None:
        pose=[0,0, (232 if H is None else H), 0,0,0]
    sol=sh.solve_home_at(pose)          # name -> (theta,tip,ball,rod,err)

    scene=[]
    base = load("底座"); scene.append(("base", base, (0.80,0.22,0.16,1.0)))
    scene.append(("R_frame", apply(frame_M("R"), load("R-Frame")), (0.30,0.30,0.33,1.0)))
    scene.append(("L_frame", apply(frame_M("L"), load("L形框架")), (0.22,0.22,0.25,1.0)))

    # receiver at solved pose (full 6-DOF)
    Mrecv = np.eye(4); Mrecv[:3,:3]=sh.euler_R(pose[3],pose[4],pose[5]); Mrecv[:3,3]=pose[:3]
    recv = apply(Mrecv, load("Receiver"))
    scene.append(("receiver", recv, (0.85,0.20,0.16,1.0)))

    balls={}
    for leg,(side,yp,kind) in LEG2BUILD.items():
        th,tip,B,rod,err = sol[leg]
        O = sh.SERVOS[leg][0]
        td = unit(tip - O)              # arm points toward solved tip
        scene.append((f"servo_{leg}", servo_box(O), (0.05,0.05,0.05,1.0)))
        if kind=="main":
            arm=load("SR6 臂"); M,bw=place_arm(arm, ARM_HUB, ARM_BALL, O, td)
            scene.append((f"arm_{leg}", apply(M,arm), (0.92,0.92,0.92,1.0)))
        else:
            parm=load("R-投手" if side=="R" else "L-投手")
            hub=RP_HUB if side=="R" else LP_HUB
            ball=RP_BALL if side=="R" else LP_BALL
            M,bw=place_arm(parm, hub, ball, O, td)
            scene.append((f"pitch_{leg}", apply(M,parm), (0.85,0.85,0.85,1.0)))
        balls[leg]=bw
        # rod: arm tip -> receiver pivot (length ~175 by construction)
        scene.append((f"rod_{leg}", make_rod(bw, B), (0.88,0.18,0.14,1.0)))
    return scene, balls

def render(scene, tag, views=None):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    if views is None:
        views=[("front_+Y",(0,-90)),("iso",(20,-60)),("right_+X",(0,0)),("top",(89,-90))]
    fig=plt.figure(figsize=(5*len(views),5))
    allv=np.vstack([m.vertices for _,m,_ in scene])
    c=allv.mean(0); r=(allv.max(0)-allv.min(0)).max()/2*1.05
    for i,(name,(el,az)) in enumerate(views):
        ax=fig.add_subplot(1,len(views),i+1,projection="3d")
        for nm,mesh,col in scene:
            f=mesh.faces; v=mesh.vertices; step=max(1,len(f)//7000)
            pc=Poly3DCollection(v[f[::step]],alpha=0.92)
            pc.set_facecolor(col); pc.set_edgecolor((0,0,0,0.06)); pc.set_linewidth(0.1)
            ax.add_collection3d(pc)
        ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=el,azim=az)
        ax.set_title(name); ax.set_xlabel("X"); ax.set_ylabel("Y")
    p=os.path.join(OUT,f"sr6_{tag}.png")
    plt.tight_layout(); plt.savefig(p,dpi=95); plt.close(); print("wrote",p)

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--tag",default="core_v3"); a=ap.parse_args()
    scene,balls=build()
    print("base  bbox", np.round(scene[0][1].bounds,1).tolist())
    print("Rframe bbox", np.round(scene[1][1].bounds,1).tolist())
    print("Lframe bbox", np.round(scene[2][1].bounds,1).tolist())
    for k,v in balls.items(): print(f"  ball {k:10s} = {np.round(v,1).tolist()}")
    render(scene, a.tag)
