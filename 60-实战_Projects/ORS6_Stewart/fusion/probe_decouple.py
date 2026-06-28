# -*- coding: utf-8 -*-
"""Validate the DECOUPLED per-leg architecture:
 place receiver rigidly at the Tripo-detected ring pose, treat the 6 mounts as
 rigid on the receiver, then solve each arm's swing angle so |rod| == 175 exactly
 (allowing the out-of-plane component the firmware planar IK forbids)."""
import sys, math, os
sys.path.insert(0, ".")
import numpy as np
import kfinal as K

IK = K.IK
SR6 = K.SR6
SPH = SR6["servoPivotH"]
ROD = SR6["mainRod"]            # 175
SLOTS = K.SERVO_SLOTS

# --- rigid receiver mount geometry (local frame, receiver origin at (0,0,HOME_H)) ---
gh = IK.compute_full_geometry(*K.TCODE_HOME)
MOUNT_LOCAL = {s: np.array(gh["recv_mounts"][s]) - np.array([0, 0, K.HOME_H])
               for s in gh["recv_mounts"]}
HOME_ANG = gh["arm_angles"]
ARMLEN = {s: (SR6["pitchArm"] if t == "pitch" else SR6["mainArm"]) for s, t, *_ in SLOTS}
SX = {s: sx for s, t, sx, sy, _ in SLOTS}
SY = {s: sy for s, t, sx, sy, _ in SLOTS}
SIGNX = {s: (1 if sx > 0 else -1) for s, t, sx, sy, _ in SLOTS}

# --- Tripo ring in assembly frame ---
rc = np.load("data/ring_circle.npz")
CEN = K.bR.T @ (rc["center"] - K.bt)
NRM = K.bR.T @ rc["normal"]; NRM /= np.linalg.norm(NRM)


def Raxis(a, b):
    """rotation mapping unit a -> unit b."""
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def recv_pose(axis_sign, twist_deg):
    """R_recv, t_recv placing the receiver so its mount-ring axis -> axis_sign*NRM
    and centroid -> Tripo ring center, plus a twist about the ring axis."""
    target = axis_sign * NRM
    R0 = Raxis(np.array([0, 0, 1.0]), target)
    tw = math.radians(twist_deg)
    # twist about the target axis
    K_ = target
    Kx = np.array([[0, -K_[2], K_[1]], [K_[2], 0, -K_[0]], [-K_[1], K_[0], 0]])
    Rtw = np.eye(3) + math.sin(tw) * Kx + (1 - math.cos(tw)) * (Kx @ Kx)
    R = Rtw @ R0
    cen_local = np.array([0, 0, 4.7])         # mount centroid local
    t = CEN - R @ cen_local
    return R, t


def solve_arm(s, M):
    sx, sy, L, sgn = SX[s], SY[s], ARMLEN[s], SIGNX[s]
    A = sx - M[0]; dy = sy - M[1]; dz0 = SPH - M[2]
    P = -2 * A * sgn * L; Q = 2 * dz0 * L
    rhs = ROD ** 2 - (A * A + dy * dy + dz0 * dz0 + L * L)
    amp = math.hypot(P, Q)
    if amp < 1e-9:
        return None
    r = rhs / amp
    reach = abs(r) <= 1.0
    rr = max(-1.0, min(1.0, r))
    phi = math.atan2(Q, P)
    cands = [phi + math.acos(rr), phi - math.acos(rr)]
    # pick closest to home angle
    best = min(cands, key=lambda th: abs(th - HOME_ANG[s]))
    tip = np.array([sx - sgn * L * math.cos(best), sy, SPH + L * math.sin(best)])
    return best, tip, np.linalg.norm(tip - M), reach


def evaluate(axis_sign, twist_deg, verbose=False):
    R, t = recv_pose(axis_sign, twist_deg)
    tot = 0.0; nreach = 0; swing = 0.0
    rows = []
    for s, typ, *_ in SLOTS:
        M = R @ MOUNT_LOCAL[s] + t
        sol = solve_arm(s, M)
        if sol is None:
            tot += 1e3; continue
        ang, tip, rod, reach = sol
        err = abs(rod - ROD)
        tot += err
        nreach += int(reach)
        swing += abs(math.degrees(ang - HOME_ANG[s]))
        rows.append((s, round(math.degrees(ang), 1), round(rod, 1), reach))
    if verbose:
        for r in rows:
            print("   ", r)
    return tot, nreach, swing


if __name__ == "__main__":
    best = None
    for sign in (+1, -1):
        for tw in range(0, 360, 5):
            tot, nreach, swing = evaluate(sign, tw)
            score = (6 - nreach) * 1e4 + tot + 0.05 * swing
            if best is None or score < best[0]:
                best = (score, sign, tw, tot, nreach, swing)
    print("BEST axis_sign", best[1], "twist", best[2],
          "rod_resid_sum", round(best[3], 2), "reachable", best[4], "/6",
          "swing_sum_deg", round(best[5], 1))
    evaluate(best[1], best[2], verbose=True)
