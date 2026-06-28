# -*- coding: utf-8 -*-
"""DECOUPLED, perception-grounded ORS6 assembler.

Why a new architecture (the firmware-IK route could NEVER match the photo):
  firmware IK forces all 6 receiver mounts to be COPLANAR (Y=sy) and solves ONE
  platform pose, so the receiver can only sit level/centred -> it can never tilt &
  offset like the real device in the photo/Tripo.  Every prior render was "完全错位"
  for exactly that reason.

New architecture (assemble REAL parts to the TRUE source, like a human):
  1. BODY  : the static frame, placed by the body registration Tb (CAD -> Tripo).
  2. RECEIVER: the real Receiver cradle placed RIGIDLY at the pose Tripo actually
     shows -- detected ring centre CR + ring axis NR (tilted!).  The 6 mounts are
     rigid on the cradle and tilt with it (the coplanar constraint is DROPPED).
  3. LEGS  : each servo horn is fixed on the body; solve its 1-DOF swing angle so
     its ball reaches the (now tilted) mount as close to 175mm as geometry allows,
     then drop the real link STL spanning horn-ball -> mount.  No global IK pose.
  4. Render assembly over Tripo (4 views) + export colored GLB.  Success = a human
     looking at it says "yes, that's the device" (gross structure), not exact coords.
"""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K          # reuse loaders, Tb, cyl, icosphere, render infra
import parts as P
import trimesh

WORK = K.WORK
IK, SR6, SLOTS = K.IK, K.SR6, K.SERVO_SLOTS
SPH, ROD = SR6["servoPivotH"], SR6["mainRod"]      # 46, 175
HOME, Tb = K.HOME, K.Tb
ARM_PIVOT, FRAME_X = K.ARM_PIVOT, K.FRAME_X
C_BODY, C_RECV, C_ARM, C_ROD, C_BALL = K.C_BODY, K.C_RECV, K.C_ARM, K.C_ROD, K.C_BALL

# ---------- rigid receiver mount geometry (home, assembly frame) ----------
GH = IK.compute_full_geometry(*K.TCODE_HOME)
MH = {s: np.array(GH["recv_mounts"][s]) for s in GH["recv_mounts"]}     # home mounts
CH = np.mean([MH[s] for s in MH], axis=0)                              # home centroid
ARMLEN = {s: (SR6["pitchArm"] if t == "pitch" else SR6["mainArm"]) for s, t, *_ in SLOTS}
SX = {s: sx for s, t, sx, sy, _ in SLOTS}
SY = {s: sy for s, t, sx, sy, _ in SLOTS}
SGN = {s: (1 if sx > 0 else -1) for s, t, sx, sy, _ in SLOTS}

# ---------- detected Tripo ring -> assembly frame (CR centre, NR axis) ----------
CR, NR = K.CR, K.NR        # already flipped so NR[2] >= 0

# ---------- link STLs (real dog-bone links) ----------
def load_link(name):
    m = P.load_stl(name)
    if isinstance(m, trimesh.Scene):
        m = m.to_geometry()
    V = np.asarray(m.vertices, float); F = np.asarray(m.faces, int)
    c = V.mean(0); X = V - c
    _, _, vt = np.linalg.svd(X, full_matrices=False)
    axis = vt[0]                                   # principal (long) axis
    proj = X @ axis
    e0 = c + axis * proj.min(); e1 = c + axis * proj.max()
    return V, F, e0, e1                            # endpoints = bore centres approx

LINK_MAIN = load_link("BearingMain")
LINK_PITCH = load_link("BearingPitch")


def Raxis(a, b):
    a = a / (np.linalg.norm(a) or 1); b = b / (np.linalg.norm(b) or 1)
    v = np.cross(a, b); c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def twist_R(axis, deg):
    k = axis / (np.linalg.norm(axis) or 1); a = math.radians(deg)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * (kx @ kx)


def recv_xform(twist_deg):
    """Rigid transform of the HOME-world receiver config -> tilted Tripo pose.
       X' = Rr @ (X - CH) + CR ,  Rr = twist(NR)·align(+z -> NR)."""
    Rr = twist_R(NR, twist_deg) @ Raxis(np.array([0, 0, 1.0]), NR)
    return Rr


def mount_world(s, Rr):
    return Rr @ (MH[s] - CH) + CR


def tip_of(s, theta):
    return np.array([SX[s] - SGN[s] * ARMLEN[s] * math.cos(theta),
                     SY[s], SPH + ARMLEN[s] * math.sin(theta)])


def solve_arm(s, M):
    """swing angle theta minimising |dist(tip,M) - 175| (closest reach to a free mount)."""
    L, sgn = ARMLEN[s], SGN[s]
    A = SX[s] - M[0]; dy = SY[s] - M[1]; dz0 = SPH - M[2]
    P_ = -2 * A * sgn * L; Q_ = 2 * dz0 * L
    rhs = ROD ** 2 - (A * A + dy * dy + dz0 * dz0 + L * L)
    amp = math.hypot(P_, Q_)
    if amp > 1e-9 and abs(rhs / amp) <= 1.0:
        phi = math.atan2(Q_, P_)
        cands = [phi + math.acos(rhs / amp), phi - math.acos(rhs / amp)]
        th = min(cands, key=lambda t: abs(t - HOME[s]))
    else:                                          # unreachable: aim straight at M
        th = min(np.linspace(math.radians(-100), math.radians(170), 400),
                 key=lambda t: abs(np.linalg.norm(tip_of(s, t) - M) - ROD))
    tip = tip_of(s, th)
    return th, tip, float(np.linalg.norm(tip - M))


def solve_pose(twist_deg):
    Rr = recv_xform(twist_deg)
    res = {}; tot = 0.0; swing = 0.0
    for s, *_ in SLOTS:
        M = mount_world(s, Rr)
        th, tip, d = solve_arm(s, M)
        res[s] = (th, tip, M, d)
        tot += abs(d - ROD); swing += abs(th - HOME[s])
    return Rr, res, tot, swing


def best_twist():
    best = None
    for tw in np.linspace(0, 360, 145, endpoint=False):
        Rr, res, tot, swing = solve_pose(tw)
        score = tot + 8.0 * math.degrees(swing) / 60.0
        if best is None or score < best[0]:
            best = (score, tw, Rr, res, tot, swing)
    return best


# ---------- arm placement (native STL) ----------
def arm_transform(s, theta):
    left = SX[s] < 0
    if ARMLEN[s] == SR6["mainArm"]:
        piv = np.array([-ARM_PIVOT[0] if left else ARM_PIVOT[0], ARM_PIVOT[1], ARM_PIVOT[2]])
        shaft = np.array([SX[s], SY[s], SPH])
        R = K.Ry(math.degrees(theta - HOME[s]))
        t = shaft - R @ piv
        V, F = (K.ARM_ML if left else K.ARM_M)
    else:
        ctr = np.array([-FRAME_X if left else FRAME_X, 0, SPH])
        R = K.Ry(math.degrees(theta - HOME[s]))
        t = ctr - R @ ctr
        V, F = K.MESH["L_Pitcher" if left else "R_Pitcher"]
    return V, F, R, t


def cylinder(a, b, r=2.4, n=10):
    a = np.asarray(a, float); b = np.asarray(b, float)
    ax = b - a; L = np.linalg.norm(ax) or 1.0; ax = ax / L
    ref = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(ax, ref); u /= (np.linalg.norm(u) or 1)
    v = np.cross(ax, u)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.array([math.cos(t) * u + math.sin(t) * v for t in th]) * r
    V = np.vstack([a + ring, b + ring])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + j], [i, n + j, n + i]]
    return V, np.array(F)


def place_link(s, a, b):
    """Place the real link STL spanning a->b. Returns (V,F,end_a,end_b) where end_a,
    end_b are the link's own bore centres after placement (true ball-joint spots)."""
    V, F, e0, e1 = (LINK_PITCH if ARMLEN[s] != SR6["mainArm"] else LINK_MAIN)
    nax = e1 - e0; ncen = (e0 + e1) / 2
    R = Raxis(nax, np.array(b) - np.array(a))
    cen = (np.array(a) + np.array(b)) / 2
    Vt = (V - ncen) @ R.T + cen
    ea = (e0 - ncen) @ R.T + cen
    eb = (e1 - ncen) @ R.T + cen
    if np.linalg.norm(ea - a) > np.linalg.norm(eb - a):
        ea, eb = eb, ea
    return Vt, F, ea, eb


def build():
    score, tw, Rr, res, tot, swing = best_twist()
    print(f"twist={tw:.1f}  rod_resid_sum={tot:.2f}mm  swing_sum={math.degrees(swing):.1f}deg")
    for s, *_ in SLOTS:
        th, tip, M, d = res[s]
        print(f"  {s:11s} angle={math.degrees(th):7.1f}  rod={d:6.1f}")

    parts = []
    # 1. body (static) via Tb
    for n in K.STATIC:
        V, F = K.MESH[n]
        parts.append((Tb(V), F, C_BODY))
    # 2. receiver group rigidly at tilted Tripo pose (home-world -> Rr·(·-CH)+CR)
    B0 = np.array([0, 0, K.HOME_H])
    for n in K.RECV_VIS:
        V, F = K.MESH[n]
        Vw = (V + B0)                              # home-world
        Vt = (Vw - CH) @ Rr.T + CR                 # tilted, assembly frame
        parts.append((Tb(Vt), F, C_RECV))
    # 3. legs: arms + real links + balls
    for s, *_ in SLOTS:
        th, tip, M, d = res[s]
        V, F, R, t = arm_transform(s, th)
        parts.append((Tb((V @ R.T) + t), F, C_ARM))
        Vl, Fl = place_link(s, tip, M)
        parts.append((Tb(Vl), Fl, C_ROD))
        for p in (tip, M):
            Vs, Fs = K.icosphere(p, 5.0, 1)
            parts.append((Tb(Vs), Fs, C_BALL))
    return parts


def main():
    parts = build()
    K.export_glb(parts, os.path.join(WORK, "ORS6_build.glb"))
    K.render_views(parts, "kbuild")
    print("saved ORS6_build.glb + kbuild.png")


if __name__ == "__main__":
    main()
