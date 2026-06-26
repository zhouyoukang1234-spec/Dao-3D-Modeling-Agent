"""assemble_connected.py -- per-part PLACED assembly where the 6 link rods are
SNAPPED to real mesh features instead of firmware abstract anchor points.

Root cause (measured): firmware arm_tips sit ~35mm off the Arm horn mesh and
recv_mounts lie on a radius-97.7mm circle while the Receiver ring mesh only
reaches radius ~70mm -> rods splay PAST the ring. Fix: project each kinematic
anchor onto the nearest point of its real mesh, then draw links mesh->mesh so
they physically connect, like the real photo (converging cone onto the ring)."""
import os, sys, math
import numpy as np
import trimesh

sys.path.insert(0, "C:/Users/Administrator/repos/Dao-3D-Modeling-Agent/60-实战_Projects")
os.environ.setdefault("ORS6_STL_ROOT", "C:/c/Users/Administrator/sr6/full_stls")
from ORS6_Stewart import parts as P
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME

ARM_PIVOT = np.array([67.5, 0.0, 51.5])
SERVO_PIVOT_H = 46.0

def _hex(c): return "#%06x" % c
def _vf(name):
    m = trimesh.load(P.stl_path(name), process=False); return m.vertices.copy(), m.faces.copy()

def _nearest_on(mesh_v, pt):
    d = np.linalg.norm(mesh_v - pt, axis=1); i = int(np.argmin(d))
    return mesh_v[i].copy(), float(d[i])

def _rod(p1, p2, r=6.0):
    cyl = trimesh.creation.cylinder(radius=r, segment=[np.asarray(p1,float), np.asarray(p2,float)], sections=20)
    return cyl.vertices.copy(), cyl.faces.copy()

def build(verbose=True, snap=True):
    ik = StewartIK(); g = ik.compute_full_geometry(*TCODE_HOME)
    rp = ik.compute_receiver_pose(*TCODE_HOME)
    recv_T = np.array([rp[0], rp[1], rp[2]])
    parts = []      # (v,f,color)
    arm_meshes = {} # slot -> placed (v,f)

    for nm in ["Base", "L_Frame", "R_Frame", "Lid", "PowerBus"]:
        v, f = _vf(nm); parts.append((v, f, _hex(P.PARTS[nm][2])))

    arm_v0, arm_f0 = _vf("Arm")
    for sname, stype, sx, sy, sign in P.SERVO_SLOTS:
        if stype != "main": continue
        v = arm_v0.copy(); mirror = sx < 0
        if mirror: v = v * np.array([-1.,1,1]); f = arm_f0[:, ::-1]
        else: f = arm_f0
        piv = np.array([-ARM_PIVOT[0] if mirror else ARM_PIVOT[0], ARM_PIVOT[1], ARM_PIVOT[2]])
        shaft = np.array([sx, sy, SERVO_PIVOT_H])
        v = v + (shaft - piv)
        parts.append((v, f, _hex(P.PARTS["Arm"][2]))); arm_meshes[sname] = (v, f)

    for nm in ["L_Pitcher", "R_Pitcher"]:
        v, f = _vf(nm); parts.append((v, f, _hex(P.PARTS[nm][2])))

    rv, rf = _vf("Receiver"); rv = rv + recv_T
    parts.append((rv, rf, _hex(P.PARTS["Receiver"][2])))

    # links: snap firmware anchors onto real meshes
    link_info = []
    for sname, stype, sx, sy, sign in P.SERVO_SLOTS:
        tip = np.array(g["arm_tips"][sname], float)
        mnt = np.array(g["recv_mounts"][sname], float)
        if snap:
            if stype == "main" and sname in arm_meshes:
                tip2, dt = _nearest_on(arm_meshes[sname][0], tip)
            else:
                tip2, dt = tip, 0.0   # pitch horns from pitcher meshes; keep firmware tip
            mnt2, dm = _nearest_on(rv, mnt)
        else:
            tip2, mnt2, dt, dm = tip, mnt, 0.0, 0.0
        v, f = _rod(tip2, mnt2, r=6.0)
        parts.append((v, f, "#cc2b1d"))  # red link like photo
        link_info.append((sname, np.linalg.norm(mnt2 - tip2), dt, dm))

    if verbose:
        allv = np.vstack([v for v,f,c in parts])
        print("parts", len(parts), "verts", len(allv))
        print("bounds", np.round(allv.min(0),1), np.round(allv.max(0),1))
        for s,L,dt,dm in link_info:
            print(f"  link {s:11s} len={L:6.1f}  snap arm {dt:5.1f}  snap recv {dm:5.1f}")
    return parts

if __name__ == "__main__":
    build()
