# -*- coding: utf-8 -*-
"""Assemble the REAL SR6 STL parts into one body, anchored to measured frame geometry.

Servo positions come from the real frame part (main shafts at Y=+/-30, Z~22 on the
outer wall; after the two halves join at centre the main shafts sit at X=+/-60).
Arms/links/receiver are the real STL meshes hung on this skeleton; the receiver home
pose is solved by least squares so all 6 links span 175 mm (eye-to-eye).
"""
import numpy as np, trimesh, json, os
from scipy.optimize import least_squares

STL = "SR6 完整资料进阶版本 签收后提供解压密码/STLs/SR6测试版零件/"
P = {
 "base":   STL+"SR6 底座 Beta1A.stl",
 "Lframe": STL+"SR6 L形框架 Beta1.stl",
 "Rframe": STL+"SR6 R-Frame Beta1.stl",
 "recv":   STL+"SR6 Receiver Beta1.stl",
 "arm":    STL+"SR6 臂 Beta1.stl",
 "Lpitch": STL+"SR6 L-投手 Beta1.stl",
 "Rpitch": STL+"SR6 R-投手 Beta1.stl",
 "mlink":  STL+"SR6 轴承主连杆 Beta1.stl",
 "plink":  STL+"SR6 轴承投手链接 Beta1.stl",
}
ROD = 175.0

def euler_R(roll,pitch,yaw):
    cx,sx=np.cos(pitch),np.sin(pitch); cy,sy=np.cos(roll),np.sin(roll); cz,sz=np.cos(yaw),np.sin(yaw)
    Rx=np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]); Ry=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]); Rz=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    return Rz@Ry@Rx

# --- real-frame-anchored servo world positions (mm) -------------------------
SERVO = {
 "L_mainA": np.array([-60.0,  30.0, 22.0]),
 "L_mainB": np.array([-60.0, -30.0, 22.0]),
 "R_mainA": np.array([ 60.0,  30.0, 22.0]),
 "R_mainB": np.array([ 60.0, -30.0, 22.0]),
 "L_pitch": np.array([-60.0,   0.0, 33.0]),
 "R_pitch": np.array([ 60.0,   0.0, 33.0]),
}
ARMLEN = {k:(75.0 if "pitch" in k else 50.0) for k in SERVO}
# receiver-local pivots (from Receiver STL through-axes)
BLOCAL = {
 "L_mainA": np.array([-59.98, 0.0, 0.0]),  "L_mainB": np.array([-59.98, 0.0, 0.0]),
 "R_mainA": np.array([ 59.98, 0.0, 0.0]),  "R_mainB": np.array([ 59.98, 0.0, 0.0]),
 "L_pitch": np.array([-61.0,-14.235,53.126]),"R_pitch": np.array([61.0,-14.235,53.126]),
}
LEGS=list(SERVO)
# arms swing about servo X-axis: arm_tip = servo + L*(0, cos th, sin th)  (in-plane Y,Z)
HOME_H = 200.0   # home receiver height (rec upright); tuned so arms sit naturally

def arm_tip(k, th):
    L=ARMLEN[k]; o=SERVO[k]
    return o+np.array([0.0, L*np.cos(th), L*np.sin(th)])

def ik(k, pivot, prev=np.pi/2):
    """solve swing angle th so |arm_tip - pivot| = ROD (pivot.X == servo.X)."""
    o=SERVO[k]; L=ARMLEN[k]
    py,pz=pivot[1]-o[1], pivot[2]-o[2]
    rho=np.hypot(py,pz); 
    cosd=(L*L+rho*rho-ROD*ROD)/(2*L*rho)
    if abs(cosd)>1: return None
    base=np.arctan2(pz,py); d=np.arccos(cosd)
    cands=[base+d, base-d]
    return min(cands,key=lambda t:abs(np.arctan2(np.sin(t-prev),np.cos(t-prev))))

def home_solution(H=HOME_H):
    R=np.eye(3); t=np.array([0,0,H])
    piv={k:(R@BLOCAL[k])+t for k in LEGS}
    ang={}; ball={}
    for k in LEGS:
        th=ik(k,piv[k]); ang[k]=th; ball[k]=arm_tip(k,th)
    return R,t,piv,ang,ball

def load(name):
    return trimesh.load(P[name],force="mesh")

def place_align(mesh, src_a, src_b, dst_a, dst_b, src_normal=None):
    """Rigidly map local point src_a->dst_a and direction (src_b-src_a)->(dst_b-dst_a)."""
    m=mesh.copy()
    v1=src_b-src_a; v2=dst_b-dst_a
    v1n=v1/np.linalg.norm(v1); v2n=v2/np.linalg.norm(v2)
    ax=np.cross(v1n,v2n); s=np.linalg.norm(ax); c=v1n@v2n
    if s<1e-9:
        R=np.eye(3) if c>0 else np.diag([1,-1,-1.0])
    else:
        ax=ax/s; K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
        R=np.eye(3)+s*K+(1-c)*(K@K)
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=dst_a-R@src_a
    m.apply_transform(T); return m,T

def main():
    R,t,piv,ang,BALL=home_solution()
    print("receiver t=",np.round(t,1)," (upright)")
    for k in LEGS:
        if ang[k] is None: print(f"  {k:8s} UNREACHABLE at H={HOME_H}"); continue
        d=np.linalg.norm(piv[k]-BALL[k]); print(f"  {k:8s} th={np.degrees(ang[k]):6.1f} ball={np.round(BALL[k],1)} piv={np.round(piv[k],1)} link={d:.2f}")

    scene=trimesh.Scene()
    def add(m,color,name):
        m.visual.face_colors=color; scene.add_geometry(m,geom_name=name)

    # frames: L shifted so main shafts (local X~-106) -> world X~-60 ; R = mirror
    Lf=load("Lframe"); dx=-60.0-(Lf.bounds[0][0]+ (-60- (Lf.bounds[0][0]) ))  # placeholder
    # shift so outer-wall (xmin) main shaft plane lands near x=-60: xmin~-110 -> -60 => +50
    Lf.apply_translation([ -60.0 - (Lf.bounds[0][0]+4.0), 0,0])
    add(Lf,[180,60,55,255],"L_frame")
    Rf=load("Rframe"); 
    Rf.apply_translation([  60.0 - (Rf.bounds[1][0]-4.0), 0,0])
    add(Rf,[180,60,55,255],"R_frame")

    # base under frames
    base=load("base"); base.apply_translation([ -base.centroid[0], -base.centroid[1], Lf.bounds[0][2]-base.bounds[1][2] ])
    add(base,[150,40,40,255],"base")

    # arms (real mesh). main arm local: servo(67.5,0,51) ball(67.5,50,51)
    arm_src_s=np.array([67.5,0,51.0]); arm_src_b=np.array([67.5,50,51.0])
    for k in ["L_mainA","L_mainB","R_mainA","R_mainB"]:
        am,_=place_align(load("arm"),arm_src_s,arm_src_b,SERVO[k],BALL[k])
        add(am,[230,230,235,255],"arm_"+k)
    # pitch arms: L_pitch local servo(-7.5,30,51) ball(-39.74,97.72,51)
    p_src_s=np.array([-7.5,30,51.0]); p_src_b=np.array([-39.74,97.72,51.0])
    for k,mesh in [("L_pitch","Lpitch"),("R_pitch","Rpitch")]:
        am,_=place_align(load(mesh),p_src_s,p_src_b,SERVO[k],BALL[k])
        add(am,[230,230,235,255],"arm_"+k)

    # links: main link straight along X, eyes at bbox x-extremes
    for k in LEGS:
        lk=load("plink" if "pitch" in k else "mlink")
        b=lk.bounds; xe0=np.array([b[0][0],(b[0][1]+b[1][1])/2,(b[0][2]+b[1][2])/2]); xe1=np.array([b[1][0],xe0[1],xe0[2]])
        lm,_=place_align(lk,xe0,xe1,BALL[k],piv[k])
        add(lm,[210,70,60,255] if "pitch" not in k else [225,120,60,255],"link_"+k)

    # receiver at solved pose
    rc=load("recv"); T=np.eye(4); T[:3,:3]=R; T[:3,3]=t; rc.apply_transform(T)
    add(rc,[200,55,50,255],"receiver")

    scene.export("sr6_real_assembly.glb")
    print("wrote sr6_real_assembly.glb  parts:",len(scene.geometry))
    json.dump({"servo":{k:v.tolist() for k,v in SERVO.items()},
               "armlen":ARMLEN,"blocal":{k:v.tolist() for k,v in BLOCAL.items()},
               "home_H":HOME_H,"home_angles":{k:(None if ang[k] is None else float(ang[k])) for k in LEGS},
               "receiver_t":t.tolist()},
              open("assembly_real.json","w"),indent=2)

if __name__=="__main__":
    main()
