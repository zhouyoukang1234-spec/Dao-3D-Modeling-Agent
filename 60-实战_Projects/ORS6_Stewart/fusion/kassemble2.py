# -*- coding: utf-8 -*-
"""NEW architecture (v2): assemble the REAL STL parts onto the Tripo-fitted pose.

Difference from kassemble.py (which floated/used a torus receiver + wrong arm
seating):
  * Receiver, frames, body, gears: REAL STL parts, rigidly placed (no torus,
    no deformation).  Static parts via body transform Tb; receiver parts via the
    fitted receiver transform Rxf so their rod anchors land exactly on `mounts`.
  * Arms: REAL Arm / L_Pitcher / R_Pitcher STL seated by 2-feature alignment ---
    native long axis (+Y) -> (tip-pivot), native broad normal (+Z) -> servo shaft
    axis (CAD Y).  The horn swings in the X-Z plane about the Y shaft axis, exactly
    as the SR6 kinematics defines it; rods stay 175mm by the fitted solve.
  * Links: REAL MainLink / PitcherLink STL stretched tip<->mount.
Frame = Tripo mm, so the assembly overlays 1:1 on tripo_mm.glb.
"""
import os, sys, math, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
PROJ = os.path.dirname(ROOT)
for p in (PROJ, ROOT, WORK, os.path.dirname(os.path.abspath(__file__))):
    sys.path.insert(0, p)
import trimesh
from PIL import Image
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME, ARM_PIVOT_STL
from ORS6_Stewart.parts import SERVO_SLOTS, SR6, PARTS, RECV_PARTS, DEFAULT_HIDDEN, stl_path
from overlay_check import render_fixed, _look_at

PAL = {"body":[.78,.10,.10], "frame":[.80,.16,.16], "recv":[.74,.09,.09],
       "rod":[.80,.16,.16], "horn":[.93,.92,.88], "ball":[.62,.64,.68]}


def L(name):
    p = stl_path(name)
    if not os.path.exists(p):
        return None, None
    m = trimesh.load(p, force="mesh")
    return np.asarray(m.vertices, float), np.asarray(m.faces, int)


def place_2pt(V, hub, ballnat, shaft, balli, shaft_axis):
    """Rigid: native long(hub->ballnat) -> (shaft->balli); native broad normal
    (+Z) -> shaft_axis; translate hub -> shaft.  No scaling (real part size)."""
    Ln = ballnat - hub; Ln = Ln / (np.linalg.norm(Ln) or 1)
    Nn = np.array([0, 0, 1.0]); Nn = Nn - Ln * (Nn @ Ln); Nn = Nn / (np.linalg.norm(Nn) or 1)
    Tn = np.cross(Ln, Nn)
    Li = balli - shaft; Li = Li / (np.linalg.norm(Li) or 1)
    Ni = np.array(shaft_axis, float); Ni = Ni - Li * (Ni @ Li); Ni = Ni / (np.linalg.norm(Ni) or 1)
    Ti = np.cross(Li, Ni)
    Rm = np.column_stack([Li, Ni, Ti]) @ np.column_stack([Ln, Nn, Tn]).T
    return (V - hub) @ Rm.T + shaft


def rot_a2b(a, b):
    a = a / (np.linalg.norm(a) or 1); b = b / (np.linalg.norm(b) or 1)
    v = np.cross(a, b); c = float(a @ b)
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def place_link(name, p0, p1):
    V, F = L(name)
    if V is None:
        return None, None
    c = (V.min(0) + V.max(0)) / 2
    ea = np.array([c[0], V[:, 1].max(), c[2]]); eb = np.array([c[0], V[:, 1].min(), c[2]])
    al = eb - ea; Ll = np.linalg.norm(al) or 1
    at = np.array(p1) - np.array(p0); Tl = np.linalg.norm(at) or 1
    R = rot_a2b(al, at)
    return (V - ea) @ R.T * (Tl / Ll) + np.array(p0), F


def uvsphere(c, r=5.5, n=10):
    u = np.linspace(0, np.pi, n); v = np.linspace(0, 2 * np.pi, 2 * n)
    uu, vv = np.meshgrid(u, v)
    x = c[0] + r * np.sin(uu) * np.cos(vv); y = c[1] + r * np.sin(uu) * np.sin(vv); z = c[2] + r * np.cos(uu)
    V = np.stack([x.ravel(), y.ravel(), z.ravel()], 1)
    F = []; cols = 2 * n
    for i in range(cols - 1):
        for j in range(n - 1):
            a = i * n + j; b = a + n
            F += [[a, b, a + 1], [b, b + 1, a + 1]]
    return V, np.array(F)


def arm_anchors(V, lo_frac=0.18, hi_frac=0.18):
    """Detect hub (servo-bore end, low native Y) and ball (link end, high Y)."""
    y = V[:, 1]; ylo, yhi = y.min(), y.max(); span = yhi - ylo
    hub = V[y < ylo + lo_frac * span].mean(0)
    ball = V[y > yhi - hi_frac * span].mean(0)
    return hub, ball


def build():
    bf = np.load(os.path.join(WORK, "kfit_body.npz")); bR, bt = bf["R"], bf["t"]
    def Tb(P): return (bR @ np.asarray(P, float).T).T + bt
    pf = np.load(os.path.join(WORK, "kfit_pose.npz"))
    Rg, C0, tinit = pf["Rg"], pf["C0"], pf["tinit"]
    tips, mounts = pf["tips"], pf["mounts"]
    def Rxf(V): return (Rg @ (Tb(V) - C0).T).T + C0 + tinit
    shaft_axis = bR @ np.array([0, 1.0, 0])   # CAD Y shaft -> Tripo

    parts = []
    # --- static body / frames / lid ---
    static = [n for n in PARTS if n not in RECV_PARTS and n not in DEFAULT_HIDDEN
              and n not in ("Arm", "L_Pitcher", "R_Pitcher")]
    for nm in static:
        V, F = L(nm)
        if V is None:
            continue
        col = PAL["frame"] if nm in ("L_Frame", "R_Frame") else PAL["body"]
        parts.append((Tb(V), F, col))
    # --- real receiver parts at fitted pose ---
    for nm in RECV_PARTS:
        V, F = L(nm)
        if V is None:
            continue
        parts.append((Rxf(V), F, PAL["recv"]))
    # --- arms (real STL, feature-seated) ---
    Va, Fa = L("Arm")
    hub_a = np.array(ARM_PIVOT_STL)            # (67.5, 0, 51.5)
    ball_a = hub_a + np.array([0, 1.0, 0])     # +Y long axis (direction only)
    for i, (s, stype, sx, sy, _) in enumerate(SERVO_SLOTS):
        if stype == "main":
            V, F = Va.copy(), Fa
            hub, ball = hub_a.copy(), ball_a.copy()
            if sx < 0:                          # mirror geometry for left frame
                V = V * np.array([-1.0, 1, 1]); F = Fa[:, ::-1]
                hub = hub * np.array([-1.0, 1, 1]); ball = ball * np.array([-1.0, 1, 1])
        else:
            pn = "L_Pitcher" if sx < 0 else "R_Pitcher"
            V, F = L(pn)
            if V is None:
                continue
            hub, ball = arm_anchors(V)
        piv = Tb([sx, sy, SR6["servoPivotH"]])
        Vt = place_2pt(V, hub, ball, piv, np.array(tips[i]), shaft_axis)
        parts.append((Vt, F, PAL["horn"]))
    # --- links + ball joints ---
    for i, (s, stype, _, _, _) in enumerate(SERVO_SLOTS):
        link = "MainLink" if stype == "main" else "PitcherLink"
        V, F = place_link(link, tips[i], mounts[i])
        if V is not None:
            parts.append((V, F, PAL["rod"]))
        for pt in (tips[i], mounts[i]):
            Vs, Fs = uvsphere(pt); parts.append((Vs, Fs, PAL["ball"]))
    return parts


def main():
    parts = build()
    tp = trimesh.load(os.path.join(WORK, "tripo_mm.glb"), process=False)
    if isinstance(tp, trimesh.Scene):
        tp = tp.to_geometry()
    Vt = np.asarray(tp.vertices); Ft = np.asarray(tp.faces)
    allV = []; allF = []; allC = []; off = 0
    for V, F, c in parts:
        allV.append(V); allF.append(F + off); allC.append(np.tile(c, (len(V), 1))); off += len(V)
    Va = np.vstack(allV); Fa = np.vstack(allF); Ca = np.vstack(allC)
    Ct = np.tile([0.20, 0.55, 0.95], (len(Vt), 1))
    big = np.vstack([Va, Vt]); center = (big.min(0) + big.max(0)) / 2
    rows = []
    for vd in [(1, -1, 0.5), (0, -1, 0.12), (1, 0, 0.12), (0, 0, 1)]:
        span = max(np.ptp(big @ _look_at(vd)[0]), np.ptp(big @ _look_at(vd)[1])) * 1.1
        ia = render_fixed(Va, Fa, Ca, vd, center, span, W=460, H=460)
        Vo = np.vstack([Va, Vt]); Fo = np.vstack([Fa, Ft + len(Va)]); Co = np.vstack([Ca, Ct])
        io = render_fixed(Vo, Fo, Co, vd, center, span, W=460, H=460)
        rows.append(np.hstack([ia, io]))
    Image.fromarray(np.vstack(rows)).save(os.path.join(WORK, "kassembly2.png"))
    print("saved kassembly2.png")
    from scipy.spatial import cKDTree
    asm = trimesh.Trimesh(Va, Fa, process=False)
    Pa = asm.sample(60000); Pt = trimesh.Trimesh(Vt, Ft, process=False).sample(60000)
    ta = cKDTree(Pa); tt = cKDTree(Pt)
    da, _ = tt.query(Pa); db, _ = ta.query(Pt)
    print(f"chamfer A->T mean {da.mean():.2f} med {np.median(da):.2f}")
    print(f"chamfer T->A mean {db.mean():.2f} med {np.median(db):.2f}")
    print(f"symmetric mean {0.5*(da.mean()+db.mean()):.2f} median {0.5*(np.median(da)+np.median(db)):.2f}")
    # export colored glb
    allC8 = []; off = 0; vv = []; ff = []
    for V, F, c in parts:
        vv.append(V); ff.append(F + off)
        allC8.append(np.tile(np.r_[np.array(c) * 255, 255].astype(np.uint8), (len(V), 1))); off += len(V)
    m = trimesh.Trimesh(np.vstack(vv), np.vstack(ff), vertex_colors=np.vstack(allC8), process=False)
    out = os.path.join(WORK, "ORS6_fused_v2.glb"); m.export(out); print("saved", out)


if __name__ == "__main__":
    main()
