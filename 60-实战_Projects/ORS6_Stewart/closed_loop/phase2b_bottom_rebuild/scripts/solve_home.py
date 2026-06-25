#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Solve SR6 home pose with CORRECTED architecture (shafts ‖Y, 2 frames in depth).

World frame (= build_core.py): X=left/right, Y=fore/aft(depth), Z=up.
Servos (measured, build_core):
  main: (±30, ±28, 61) shaft‖Y, arm len 50 swings in XZ plane
  pitch:( 0,  ±28, 61) shaft‖Y, arm len 75
Receiver pivots (measured from Receiver STL, all bores ‖X):
  main shaft  (±60, 0,    0)   -> 2 coaxial balls offset along X (front/back rod)
  pitch       (±60, -14.2, 53.1) -> 1 ball per side
At home the receiver ring is ~horizontal (no big rotation).
"""
import numpy as np, math
from scipy.optimize import least_squares

ROD = 175.0
ARM_M, ARM_P = 50.0, 75.0
SZ = 61.0          # servo shaft Z (world), measured
SY = 28.0          # frame depth half (front +28 / back -28)
MAIN_SX = 30.0     # main servo |X|
DX = 5.0           # half-offset of the two coaxial balls on a main shaft

# servo world positions ; name: (O, axis, armlen, ('main'|'pitch'))
SERVOS = {
    "R_front": (np.array([ MAIN_SX,  SY, SZ]), "main",  +1),
    "R_back":  (np.array([ MAIN_SX, -SY, SZ]), "main",  +1),
    "L_front": (np.array([-MAIN_SX,  SY, SZ]), "main",  -1),
    "L_back":  (np.array([-MAIN_SX, -SY, SZ]), "main",  -1),
    "P_front": (np.array([0.0,  SY, SZ]),      "pitch", 0),
    "P_back":  (np.array([0.0, -SY, SZ]),      "pitch", 0),
}
# receiver-local ball centres ; front rod -> inner X, back rod -> outer X (avoid crossing)
B_LOCAL = {
    "R_front": np.array([ 60-DX, 0.0,   0.0]),
    "R_back":  np.array([ 60+DX, 0.0,   0.0]),
    "L_front": np.array([-60+DX, 0.0,   0.0]),
    "L_back":  np.array([-60-DX, 0.0,   0.0]),
    "P_front": np.array([ 60.0, -14.235, 53.126]),   # front pitch -> right pitch ball
    "P_back":  np.array([-60.0, -14.235, 53.126]),   # back  pitch -> left  pitch ball
}
SERVO_AXIS = np.array([0., 1., 0.])   # all shafts ‖Y; arms swing in XZ plane
MAIN_SERVOS = ["R_front","R_back","L_front","L_back"]
PITCH_SERVOS = ["P_front","P_back"]

def unit(v):
    n = np.linalg.norm(v); return v/n if n > 1e-12 else v

def euler_R(roll, pitch, yaw):
    cx, sx = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(roll),  math.sin(roll)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
    Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    return Rz @ Ry @ Rx

def b_world(name, pose):
    tx,ty,tz,ro,pi,yw = pose
    return euler_R(ro,pi,yw) @ B_LOCAL[name] + np.array([tx,ty,tz])

class Leg:
    def __init__(self, name):
        O, kind, _ = SERVOS[name]
        self.name=name; self.O=O.astype(float)
        self.L = ARM_M if kind=="main" else ARM_P
        self.a = unit(SERVO_AXIS)
        ref = np.array([1.,0,0]) if abs(self.a[0])<0.9 else np.array([0,1.,0])
        self.u0 = unit(ref - self.a*(self.a@ref))   # in-plane basis (here +X)
        self.w0 = np.cross(self.a, self.u0)         # (here +Z)
    def tip(self, th):
        return self.O + self.L*(math.cos(th)*self.u0 + math.sin(th)*self.w0)
    def reach(self, B, want_dir=None):
        """Solve arm angle so |tip-B|==ROD. want_dir = preferred arm direction
        (world) used to pick the elbow. returns (th, |rod|-ROD)."""
        d = B - self.O
        h = self.a @ d                      # out-of-plane (Y) offset
        proj = d - self.a*h
        rhs = ROD*ROD - h*h
        if rhs < 0: return None, 1e3
        rin = math.sqrt(rhs)
        pc = np.array([proj@self.u0, proj@self.w0])
        dpc = np.linalg.norm(pc)
        ca = (self.L*self.L + dpc*dpc - rin*rin)/(2*self.L*dpc+1e-12)
        reach_ok = abs(ca) <= 1.0
        ca = max(-1,min(1,ca))
        base = math.atan2(pc[1], pc[0]); alpha = math.acos(ca)
        cands = [base-alpha, base+alpha]
        if want_dir is not None:
            wd = np.array([want_dir@self.u0, want_dir@self.w0])
            def score(th): return np.array([math.cos(th),math.sin(th)])@wd
            th = max(cands, key=score)
        else:
            th = min(cands, key=lambda t: np.linalg.norm(self.tip(t)-B))
        err = np.linalg.norm(self.tip(th)-B) - ROD
        return th, err

LEGS = {n: Leg(n) for n in SERVOS}

def home_residual(p):
    pose = p
    res=[]
    for n in SERVOS:
        B = b_world(n, pose)
        th,e = LEGS[n].reach(B)
        res.append(e)
    return res

# preferred arm directions at home (world): mains horizontal outward, pitch up
WANT_DIR = {
    "R_front": np.array([ 1.,0,0]), "R_back": np.array([ 1.,0,0]),
    "L_front": np.array([-1.,0,0]), "L_back": np.array([-1.,0,0]),
    "P_front": np.array([0.,0,1.]), "P_back": np.array([0.,0,1.]),
}

def solve_home_at(pose):
    """Given receiver pose, per-leg IK with preferred elbow. Returns dict of
    name->(theta, tip, ball, rod, err)."""
    out={}
    for n in SERVOS:
        B=b_world(n,pose); th,e=LEGS[n].reach(B, WANT_DIR[n])
        tip=LEGS[n].tip(th); out[n]=(th,tip,B,np.linalg.norm(tip-B),e)
    return out

if __name__ == "__main__":
    import json
    # mechanism holds receiver flat at a RANGE of H (heave DOF). Pick H so main
    # arms are most horizontal (matches p24) while all 6 legs stay reachable.
    print("  H   | main-arm tilt(deg, mean abs) | max|rod-175| | reachable")
    best=None
    for H in range(200, 250, 2):
        sol=solve_home_at([0,0,H,0,0,0])
        tilts=[]; maxerr=0
        for n in MAIN_SERVOS:
            th,tip,B,rod,e=sol[n]
            tilts.append(abs(math.degrees(math.atan2(tip[2]-SZ, abs(tip[0])-SERVOS[n][0][0]))))
        for n in SERVOS: maxerr=max(maxerr,abs(sol[n][4]))
        ok = maxerr < 1e-3
        mt=np.mean(tilts)
        print(f"  {H:3d} | {mt:6.2f} | {maxerr:.4f} | {ok}")
        if ok and (best is None or mt<best[1]): best=(H,mt)
    H=best[0] if best else 224
    print(f"\nchosen H={H} (flattest main arms, all reachable)")
    sol=solve_home_at([0,0,H,0,0,0])
    out={"pose":[0,0,H,0,0,0],"legs":{}}
    print("  --- legs at home ---")
    for n in SERVOS:
        th,tip,B,rod,e=sol[n]
        print(f"   {n:8s} th={math.degrees(th):7.2f} tip={np.round(tip,1)} ball={np.round(B,1)} rod={rod:.3f}")
        out["legs"][n]={"theta_deg":math.degrees(th),"tip":tip.tolist(),"ball":B.tolist(),"rod":rod}
    json.dump(out, open("home_solution.json","w"), indent=2)
    print("wrote home_solution.json")
