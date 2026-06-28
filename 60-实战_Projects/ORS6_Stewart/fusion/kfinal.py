# -*- coding: utf-8 -*-
"""DEFINITIVE ORS6 assembler (vertical redesign).

Architecture (top-down, no trial-error):
  1. Body shell rigidly registered to Tripo (kfit_body.npz, flip resolved by kreg2).
     Tb maps the kinematics/assembly frame -> Tripo (the true source).
  2. Whole mechanism placed by EXACT assembly.py logic, parametrised by the 6
     firmware T-Codes (StewartIK). For ANY T-Code the 6 rods are 175mm BY
     CONSTRUCTION (tip<->mount are coplanar at Y=sy). No fitting of part shapes.
  3. Find the single T-Code pose whose mechanism, mapped through Tb, best matches
     Tripo (symmetric chamfer over the moving parts).  -> the real deployed pose.
  4. Render assembly + assembly-over-Tripo overlay; export colored GLB.
"""
import os, sys, math, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.dirname(ROOT)
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
sys.path.insert(0, PROJ); sys.path.insert(0, ROOT); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
from scipy.spatial import cKDTree
from scipy.optimize import minimize
from PIL import Image
import parts as P
from parts import SR6, HOME_H, RECV_PARTS, DEFAULT_HIDDEN, SERVO_SLOTS
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME
from overlay_check import render_fixed, _look_at

IK = StewartIK()
HOME = IK.compute_full_geometry(*TCODE_HOME)["arm_angles"]
ARM_PIVOT = np.array([67.5, 0.0, 51.5]); FRAME_X = 99.6
SPH = SR6["servoPivotH"]

# ---- colors (user spec) ----
C_BODY = (0.78, 0.10, 0.10); C_RECV = (0.74, 0.09, 0.09)
C_ARM = (0.93, 0.92, 0.88); C_ROD = (0.80, 0.80, 0.82); C_BALL = (0.62, 0.64, 0.68)

STATIC = ["Base", "L_Frame", "R_Frame", "Lid", "PowerBus"]
RECV_VIS = [n for n in ["Receiver", "Twist_Base", "Twist_Body", "Twist_Lid",
                        "RingGear", "ExchangeGear", "DriveGear"] if n not in DEFAULT_HIDDEN]
GEARS = {"RingGear", "ExchangeGear", "DriveGear"}


def Rx(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def Ry(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def Rz(a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def mesh(name):
    m = P.load_stl(name)
    if isinstance(m, trimesh.Scene):
        m = m.to_geometry()
    return np.asarray(m.vertices, float), np.asarray(m.faces, int)


# ---- body transform (assembly frame -> Tripo) ----
bf = np.load(os.path.join(WORK, "kfit_body.npz")); bR, bt = bf["R"], bf["t"]
def Tb(V):
    return (bR @ np.asarray(V, float).reshape(-1, 3).T).T + bt

# cache native meshes
MESH = {}
for n in STATIC + RECV_VIS + ["Arm", "L_Pitcher", "R_Pitcher"]:
    MESH[n] = mesh(n)
ARM_M = MESH["Arm"]
ARM_ML = (ARM_M[0] * np.array([-1, 1, 1.0]), ARM_M[1][:, ::-1])  # mirrored-left


def cyl(p0, p1, r, n=14):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    ax = p1 - p0; L = np.linalg.norm(ax) or 1.0; ax /= L
    t = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1, 0, 0.0])
    u = np.cross(ax, t); u /= np.linalg.norm(u) or 1; v = np.cross(ax, u)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.array([np.cos(t_) * u + np.sin(t_) * v for t_ in th]) * r
    V = np.vstack([p0 + ring, p1 + ring]); F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + i], [j, n + j, n + i]]
    return V, np.array(F)


def icosphere(c, r, sub=1):
    m = trimesh.creation.icosphere(subdivisions=sub, radius=r)
    return np.asarray(m.vertices) + np.asarray(c, float), np.asarray(m.faces)


def geom(tcode):
    """Return arm_angles, recv_pose, arm_tips, recv_mounts for a T-Code."""
    g = IK.compute_full_geometry(*tcode)
    rp = IK.compute_receiver_pose(*tcode)
    return g["arm_angles"], rp, g["arm_tips"], g["recv_mounts"]


def part_transforms(tcode):
    """List of (name, R(3x3), t(3,), is_arm) placing each native part in ASSEMBLY frame."""
    ang, (tx, ty, tz_stl, roll, pitch, twist), tips, mounts = geom(tcode)
    recv_dz = tz_stl - HOME_H
    out = []
    # static
    for n in STATIC:
        out.append((n, np.eye(3), np.zeros(3), "body"))
    # main arms
    for sname, stype, sx, sy, _s in SERVO_SLOTS:
        if stype != "main":
            continue
        left = sx < 0
        piv = np.array([-ARM_PIVOT[0] if left else ARM_PIVOT[0], ARM_PIVOT[1], ARM_PIVOT[2]])
        shaft = np.array([sx, sy, SPH])
        R = Ry(math.degrees(ang[sname] - HOME[sname]))
        t = shaft - R @ piv
        out.append(("Arm_L" if left else "Arm_R", R, t, "arm"))
    # pitchers
    for pname in ["L_Pitcher", "R_Pitcher"]:
        sname = "LeftPitch" if "L_" in pname else "RightPitch"
        sx = -FRAME_X if "L_" in pname else FRAME_X
        ctr = np.array([sx, 0, SPH])
        R = Ry(math.degrees(ang[sname] - HOME[sname]))
        t = ctr - R @ ctr
        out.append((pname, R, t, "arm"))
    # receiver group
    recv_rot = Rx(pitch) @ Ry(roll); base = np.array([tx, ty, HOME_H + recv_dz])
    for n in RECV_VIS:
        if n in GEARS:
            tw = 1 if n == "RingGear" else -1
            R = recv_rot @ Rz(tw * twist)
        else:
            R = recv_rot
        out.append((n, R, base, "recv"))
    return out, tips, mounts


# ---- pre-sample native parts for fast objective ----
def sample_native(V, F, k):
    m = trimesh.Trimesh(V, F, process=False)
    try:
        pts, _ = trimesh.sample.sample_surface(m, k)
        return np.asarray(pts, float)
    except Exception:
        return V[np.random.default_rng(0).choice(len(V), min(k, len(V)), replace=False)]

NATIVE_PTS = {n: sample_native(*MESH[n], 1500) for n in STATIC}
for n in RECV_VIS:
    NATIVE_PTS[n] = sample_native(*MESH[n], 1200)
NATIVE_PTS["Arm_R"] = sample_native(*ARM_M, 600)
NATIVE_PTS["Arm_L"] = sample_native(ARM_ML[0], ARM_ML[1], 600)
NATIVE_PTS["L_Pitcher"] = sample_native(*MESH["L_Pitcher"], 600)
NATIVE_PTS["R_Pitcher"] = sample_native(*MESH["R_Pitcher"], 600)

# fixed body points in Tripo frame
BODY_T = Tb(np.vstack([NATIVE_PTS[n] for n in STATIC]))

# Tripo target
_tp = trimesh.load(os.path.join(WORK, "tripo_mm.glb"), process=False)
if isinstance(_tp, trimesh.Scene):
    _tp = _tp.to_geometry()
VT = np.asarray(_tp.vertices, float); FT = np.asarray(_tp.faces, int)
rng = np.random.default_rng(0)
VT_S = VT[rng.choice(len(VT), 9000, replace=False)]
TREE_T = cKDTree(VT_S)


def moving_pts(tcode):
    tfs, tips, mounts = part_transforms(tcode)
    pts = []
    for n, R, t, kind in tfs:
        if kind == "body":
            continue
        pts.append((NATIVE_PTS[n] @ R.T) + t)
    # rods (line samples) + ball points
    for sname, *_ in SERVO_SLOTS:
        a = np.array(tips[sname]); b = np.array(mounts[sname])
        pts.append(a + (b - a) * np.linspace(0, 1, 16)[:, None])
    P_asm = np.vstack(pts)
    return Tb(P_asm), tips, mounts


def objective(x):
    tcode = tuple(int(np.clip(v, 0, 9999)) for v in x)
    try:
        mv, tips, mounts = moving_pts(tcode)
    except Exception:
        return 1e6
    A = np.vstack([mv, BODY_T])
    dA, _ = TREE_T.query(A)                       # assembly -> tripo
    treeA = cKDTree(A)
    dT, _ = treeA.query(VT_S)                      # tripo -> assembly
    return float(dA.mean() + dT.mean())


def _mapinv(val, a, b):
    return (val - a) / (b - a) * 9999.0


def pose_to_tcode(tx, ty, tz, roll_deg, pitch_deg, twist_deg=0.0):
    """Exact inverse of compute_receiver_pose -> firmware T-Codes."""
    L0 = _mapinv((tz - HOME_H) * 100.0, -6000, 6000)
    L1 = _mapinv(ty * 100.0, -3000, 3000)
    L2 = _mapinv(tx * 100.0, -3000, 3000)
    R1 = _mapinv(roll_deg * 100.0, -3000, 3000)
    R2 = _mapinv(pitch_deg * 100.0, -2500, 2500)
    twist_us = -twist_deg * (math.pi / 180.0) * SR6["msPerRad"]
    R0 = _mapinv(twist_us, 1000, -1000)
    return tuple(int(round(np.clip(v, 0, 9999))) for v in (L0, L1, L2, R0, R1, R2))


# ---- ring target in assembly frame ----
_rc = np.load(os.path.join(WORK, "ring_circle.npz"))
CR = bR.T @ (_rc["center"] - bt)
NR = bR.T @ _rc["normal"]; NR = NR / np.linalg.norm(NR)
if NR[2] < 0:
    NR = -NR
ROLL0 = math.degrees(math.asin(max(-1, min(1, NR[0]))))
PITCH0 = math.degrees(math.atan2(-NR[1], NR[2]))


def mount_geom(pose5):
    """pose5=(tx,ty,tz,roll,pitch) -> (mount_centroid, recv_axis) in assembly frame."""
    tc = pose_to_tcode(*pose5, 0.0)
    g = IK.compute_full_geometry(*tc)
    cen = np.array([g["recv_mounts"][s] for s in g["recv_mounts"]]).mean(0)
    axis = (Rx(pose5[4]) @ Ry(pose5[3])) @ np.array([0, 0, 1.0])
    return cen, axis


def optimize():
    """Place the mount-ring centroid + axis on the detected Tripo ring, optimising
    the 5 receiver-pose DOF inside the firmware-reachable range.  Twist then swept
    to best align rods/gears with Tripo."""
    bnds = [(-30, 30), (-30, 30), (148.5, 268.5), (-30, 30), (-25, 25)]

    def loss(v):
        cen, axis = mount_geom(v)
        return float(np.sum((cen - CR) ** 2) + 400.0 * np.sum((axis - NR) ** 2))

    best = None
    for tz0 in (200, 230, 255):
        x0 = [CR[0], np.clip(CR[1], -30, 30), tz0, ROLL0, PITCH0]
        r = minimize(loss, x0, method="L-BFGS-B", bounds=bnds,
                     options={"maxiter": 500, "eps": 0.5})
        if best is None or r.fun < best.fun:
            best = r
    tx, ty, tz, roll, pitch = best.x
    cen, axis = mount_geom(best.x)
    print("pose solve: centroid err", round(float(np.linalg.norm(cen - CR)), 2),
          "mm  axis err", round(float(np.degrees(np.arccos(np.clip(axis @ NR, -1, 1)))), 2), "deg")
    print("  recv_pose(tx,ty,tz,roll,pitch)", np.round(best.x, 2))
    bx = None; fb = 1e18
    for tw in np.linspace(-45, 45, 61):
        tc = pose_to_tcode(tx, ty, tz, roll, pitch, tw)
        f = objective(list(tc))
        if f < fb:
            fb = f; bx = tc
    print("twist-swept chamfer", round(fb, 2), "tcode", bx,
          "recv_pose", np.round(IK.compute_receiver_pose(*bx), 2))
    return bx, fb


def build_meshes(tcode):
    """Full colored mesh list [(V,F,rgb)] in TRIPO frame."""
    tfs, tips, mounts = part_transforms(tcode)
    parts = []
    for n, R, t, kind in tfs:
        V, F = (ARM_M if n == "Arm_R" else ARM_ML if n == "Arm_L" else MESH[n])
        Vt = Tb((V @ R.T) + t)
        col = C_BODY if kind == "body" else C_ARM if kind == "arm" else C_RECV
        parts.append((Vt, F, col))
    # rods + balls
    for sname, *_ in SERVO_SLOTS:
        a = np.array(tips[sname]); b = np.array(mounts[sname])
        Vc, Fc = cyl(a, b, 3.0, 16)
        parts.append((Tb(Vc), Fc, C_ROD))
        for p in (a, b):
            Vs, Fs = icosphere(p, 5.0, 1)
            parts.append((Tb(Vs), Fs, C_BALL))
    return parts, tips, mounts


def export_glb(parts, path):
    scene = trimesh.Scene()
    for i, (V, F, c) in enumerate(parts):
        m = trimesh.Trimesh(np.asarray(V), np.asarray(F), process=False)
        m.visual.vertex_colors = np.tile((np.array(c) * 255).astype(np.uint8), (len(V), 1))
        scene.add_geometry(m, geom_name=f"p{i}")
    scene.export(path)


def render_views(parts, tag):
    allV = np.vstack([p[0] for p in parts]); fix = (allV.min(0) + allV.max(0)) / 2
    VTc = np.tile([0.30, 0.55, 0.95], (len(VT), 1))
    # concat assembly
    Va = np.vstack([p[0] for p in parts]); Ca = np.vstack([np.tile(p[2], (len(p[0]), 1)) for p in parts])
    offs = np.cumsum([0] + [len(p[0]) for p in parts])
    Fa = np.vstack([p[1] + offs[i] for i, p in enumerate(parts)])
    rows = []
    for vd in [(1, -1, 0.5), (0, -1, 0.12), (1, 0, 0.12), (0, 0, 1)]:
        span = max(np.ptp(np.vstack([Va, VT]) @ _look_at(vd)[0]),
                   np.ptp(np.vstack([Va, VT]) @ _look_at(vd)[1])) * 1.1
        ia = render_fixed(Va, Fa, Ca, vd, fix, span, W=460, H=460)
        it = render_fixed(VT, FT, VTc, vd, fix, span, W=460, H=460)
        Vo = np.vstack([Va, VT]); Fo = np.vstack([Fa, FT + len(Va)])
        Co = np.vstack([Ca, np.tile([0.30, 0.55, 0.95], (len(VT), 1))])
        io = render_fixed(Vo, Fo, Co, vd, fix, span, W=460, H=460)
        rows.append(np.hstack([ia, it, io]))
    Image.fromarray(np.vstack(rows)).save(os.path.join(WORK, f"{tag}.png"))


def _ring_basis(center, normal, radius):
    n_ = np.asarray(normal, float); n_ /= np.linalg.norm(n_)
    t = np.array([1, 0, 0.0]) if abs(n_[0]) < 0.9 else np.array([0, 1, 0.0])
    u = np.cross(n_, t); u /= np.linalg.norm(u); v = np.cross(n_, u)
    return np.asarray(center, float), n_, u, v


def _ring_circle_pts(center, normal, radius, n=300, tube=7.0):
    c, n_, u, v = _ring_basis(center, normal, radius)
    pts = []
    for a in np.linspace(0, 2 * np.pi, n, endpoint=False):
        rad = np.cos(a) * u + np.sin(a) * v
        base = c + radius * rad
        for b in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            pts.append(base + tube * (np.cos(b) * rad + np.sin(b) * n_))
    return np.array(pts)


def _nearest_on_ring(p, center, normal, radius):
    c, n_, u, v = _ring_basis(center, normal, radius)
    d = np.asarray(p, float) - c
    proj = d - (d @ n_) * n_
    if np.linalg.norm(proj) < 1e-6:
        proj = u
    proj = proj / np.linalg.norm(proj) * radius
    return c + proj


def colored_tripo(tcode):
    """Color the TRUE-SOURCE Tripo mesh (it IS the photo->3D source: 1:1, watertight,
    zero floating/misalignment).  Segmentation = nearest labeled REFERENCE, using the
    BEST ground truth for each region:
      - body + frames : registered CAD body points (Tb)            -> red
      - servo arms    : registered CAD arm points (body-anchored)  -> white
      - receiver ring : the DETECTED Tripo ring (true location)    -> darker red
      - rods          : arm-tip(CAD) -> nearest-ring-point(Tripo)  -> silver
                        (correct on BOTH ends, so lies on the real rods)
      - ball joints   : small caps at each rod end                 -> gray
    Returns (V,F,vertex_colors)."""
    tfs, tips, mounts = part_transforms(tcode)
    refP = []; refC = []

    def add(pts, col):
        pts = np.asarray(pts, float).reshape(-1, 3)
        refP.append(pts); refC.append(np.tile(col, (len(pts), 1)))

    # body + arms from registered CAD (receiver CAD parts dropped: wrong shape/pose)
    for n, R, t, kind in tfs:
        if kind == "body":
            add(Tb((MESH[n][0] @ R.T) + t), C_BODY)
        elif kind == "arm":
            V = ARM_M[0] if n == "Arm_R" else ARM_ML[0] if n == "Arm_L" else MESH[n][0]
            add(Tb((V @ R.T) + t), C_ARM)

    # detected Tripo ring (true receiver location/size)
    rc = np.load(os.path.join(WORK, "ring_circle.npz"))
    rcen, rnrm, rrad = rc["center"], rc["normal"], float(rc["radius"])
    add(_ring_circle_pts(rcen, rnrm, rrad, 320, 7.0), C_RECV)

    # rods: arm tip (CAD, body-anchored) -> nearest point on detected ring.
    # skip the body-embedded start (t<0.18) so the body surface is not stolen.
    rodP = []
    for sname, *_ in SERVO_SLOTS:
        a = Tb(np.array(tips[sname]))[0]
        b = _nearest_on_ring(a, rcen, rnrm, rrad)
        seg = a + (b - a) * np.linspace(0.18, 1.0, 55)[:, None]
        rodP.append(seg)
        add(seg, C_ROD)
        add(b + (a - b) * np.linspace(0.0, 0.05, 5)[:, None], C_BALL)   # ring-end ball
    rodP = np.vstack(rodP)

    refP = np.vstack(refP); refC = np.vstack(refC)
    _, idx = cKDTree(refP).query(VT)
    cols = refC[idx]
    # majority-vote smoothing over 1-ring neighbours (3 passes) to remove speckle
    adj = trimesh.Trimesh(VT, FT, process=False).vertex_neighbors
    sm = cols.copy()
    for _ in range(3):
        nxt = sm.copy()
        for i, nb in enumerate(adj):
            if nb:
                allc = np.vstack([sm[i], sm[nb]])
                uq, inv = np.unique(allc, axis=0, return_inverse=True)
                nxt[i] = uq[np.bincount(inv).argmax()]
        sm = nxt
    return VT, FT, sm


def render_tripo_colored(V, F, C, tag):
    fix = (V.min(0) + V.max(0)) / 2
    rows = []
    for vd in [(1, -1, 0.5), (0, -1, 0.12), (1, 0, 0.12), (0, 0, 1)]:
        span = max(np.ptp(V @ _look_at(vd)[0]), np.ptp(V @ _look_at(vd)[1])) * 1.1
        rows.append(render_fixed(V, F, C, vd, fix, span, W=600, H=600))
    Image.fromarray(np.hstack(rows)).save(os.path.join(WORK, f"{tag}.png"))


def main():
    tcode, score = optimize()
    ang, rp, tips, mounts = geom(tcode)
    print("POSE tcode", tcode, "recv_pose", np.round(rp, 2))
    for s in tips:
        print("  rod", s, round(float(np.linalg.norm(np.array(tips[s]) - np.array(mounts[s]))), 2))
    parts, tips, mounts = build_meshes(tcode)
    export_glb(parts, os.path.join(WORK, "ORS6_final.glb"))
    render_views(parts, "kfinal")
    np.savez(os.path.join(WORK, "kfinal_pose.npz"), tcode=tcode, recv_pose=rp, score=score)
    print("saved ORS6_final.glb + kfinal.png  chamfer", round(score, 2))

    # --- primary deliverable: colored TRUE-SOURCE Tripo (1:1 by construction) ---
    Vt, Ft, Ct = colored_tripo(tcode)
    mt = trimesh.Trimesh(Vt, Ft, process=False)
    mt.visual.vertex_colors = np.hstack([(Ct * 255).astype(np.uint8),
                                         np.full((len(Vt), 1), 255, np.uint8)])
    mt.export(os.path.join(WORK, "ORS6_tripo_colored.glb"))
    render_tripo_colored(Vt, Ft, Ct, "ktripo_colored")
    print("saved ORS6_tripo_colored.glb + ktripo_colored.png")


if __name__ == "__main__":
    main()
