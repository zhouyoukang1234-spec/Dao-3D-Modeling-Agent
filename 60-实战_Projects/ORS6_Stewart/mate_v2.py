#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ORS6_Stewart · mate_v2 — 全真实几何 + 物理对接 (道法自然 v2).

反者道之动 / 无为而无不为:
  不再臆测舵机轴朝向, 不再硬塞固件角. 一切从真实 STL 几何读出, 让装配"自然落位":

  1. 舵机: 从真实 frame 网格 12 颗 M3 安装孔成组 → 6 个舵机舱形心 + 轴向(mesh +Z).
     (tools/servo_frames.py 实测: ±76.4 处, 轴沿 mesh +Z, 臂在 mesh XY 面摆动.)
  2. receiver: 取其设计静止位姿 (居中, 沿 mesh +Z 方向被举起, 可调 standoff/tilt).
  3. 臂角 = 解出来的, 不是给定的: 每条腿的球头在以舵机轴为法线的圆上, 解臂角 ψ
     使 真实球头→真实 receiver 孔 的距离 = 175mm. 臂"自然"找到连接角.
  4. 收敛拓扑 (PDF p.31): 同侧 2 主臂的连杆汇聚到 receiver 同一下孔; 投手臂→上耳孔.

可调三参 (居中对称 ⇒ 只剩 3 自由度): receiver 的 y_r, z_r, tiltX.
按说明书实物图视觉收敛即可.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import trimesh
from scipy.optimize import brentq

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from ORS6_Stewart.parts import stl_path  # noqa: E402
from ORS6_Stewart import render_mate as RM  # noqa: E402

ROD_LEN = 175.0

# ── 真实舵机 (mesh 坐标, tools/servo_frames.py 实测) ─────────────────────────
# 每个: (腿名, 类型, 舵机轴心 P, 连接的 receiver 孔名)
#   轴向恒为 mesh +Z. L 在 -x, R 在 +x.
SERVOS = [
    ("L_main_b", "main", np.array([-76.40, -30.0, 18.27]), "Left"),
    ("L_main_f", "main", np.array([-76.40,   0.0, 18.27]), "Left"),
    ("L_pitch",  "pitch", np.array([-76.40, +30.0, 26.52]), "LeftPitch"),
    ("R_main_b", "main", np.array([+76.40, -30.0, 18.27]), "Right"),
    ("R_main_f", "main", np.array([+76.40,   0.0, 18.27]), "Right"),
    ("R_pitch",  "pitch", np.array([+76.40, +30.0, 26.52]), "RightPitch"),
]

# ── 真实臂接口 (臂局部, _dao_axis_v2 实测; 花键轴 = 局部 +Z) ──────────────────
ARM_SPLINE = np.array([67.5, -7.68, 55.0])
ARM_BALL = np.array([67.5, 50.0, 51.0])
PITCH_SPLINE = {"L": np.array([-7.5, 22.32, 54.25]), "R": np.array([7.5, 22.32, 54.25])}
PITCH_BALL = {"L": np.array([-39.74, 97.72, 50.25]), "R": np.array([39.74, 97.72, 50.25])}

# ── receiver 真实孔 (receiver 局部) — 收敛: 同侧 2 主腿共用 1 个下孔 ──────────
RECV_HOLE = {
    "Left":  np.array([-59.98, 0.0, 0.0]),
    "Right": np.array([59.98, 0.0, 0.0]),
    "LeftPitch":  np.array([-61.0, -14.24, 53.13]),
    "RightPitch": np.array([61.0, -14.24, 53.13]),
}

# 连杆真实球心 (pivot-to-pivot, 局部)
MAIN_LINK_PIVOTS = (np.array([-87.5, 0.0, 0.0]), np.array([87.5, 0.0, 0.0]))
PITCH_LINK_PIVOTS = (np.array([-82.2, 0.0, 0.0]), np.array([82.2, 0.0, 60.0]))


def _load(name: str) -> trimesh.Trimesh:
    return trimesh.load(stl_path(name), process=False)


def Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def recv_pose(y_r: float, z_r: float, tilt: float) -> np.ndarray:
    """receiver 世界位姿: 居中(x=0), 绕 X 倾 tilt, 平移 (0,y_r,z_r)."""
    T = np.eye(4)
    T[:3, :3] = Rx(tilt)
    T[:3, 3] = [0.0, y_r, z_r]
    return T


def ball_on_circle(P: np.ndarray, span: np.ndarray, psi: float) -> np.ndarray:
    """臂绕 mesh +Z 轴转, 球头在以 P 为心、轴为法线的圆上, 绝对 XY 角 = psi."""
    rho = float(np.linalg.norm(span[:2]))
    h = float(span[2])
    return P + np.array([rho * math.cos(psi), rho * math.sin(psi), h])


def solve_arm_angle(P: np.ndarray, span: np.ndarray, hole: np.ndarray,
                    prefer: float) -> float:
    """解臂角 psi 使 |ball(psi) - hole| = 175. 取靠近 prefer 朝向的解."""
    def f(psi):
        return float(np.linalg.norm(ball_on_circle(P, span, psi) - hole)) - ROD_LEN

    # 在 [prefer-pi, prefer+pi] 扫描找根
    best = None
    N = 720
    prev_psi = prefer - math.pi
    prev_f = f(prev_psi)
    for i in range(1, N + 1):
        psi = prefer - math.pi + 2 * math.pi * i / N
        fv = f(psi)
        if prev_f == 0:
            cand = prev_psi
        elif prev_f * fv < 0:
            cand = brentq(f, prev_psi, psi)
        else:
            prev_psi, prev_f = psi, fv
            continue
        # 选离 prefer 最近的根
        d = abs(((cand - prefer + math.pi) % (2 * math.pi)) - math.pi)
        if best is None or d < best[0]:
            best = (d, cand)
        prev_psi, prev_f = psi, fv
    if best is None:
        # 无解(连杆够不到): 取最小残差角
        psis = np.linspace(prefer - math.pi, prefer + math.pi, N)
        psi = min(psis, key=lambda p: abs(f(p)))
        return psi
    return best[1]


def arm_transform(spline_l: np.ndarray, ball_l: np.ndarray,
                  P: np.ndarray, psi: float) -> Tuple[np.ndarray, np.ndarray]:
    """臂网格世界变换 + 球头世界坐标. 花键局部+Z→mesh+Z, 球头 XY 角=psi."""
    span = ball_l - spline_l
    phi_local = math.atan2(span[1], span[0])
    R = Rz(psi - phi_local)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = P - R @ spline_l
    ball_w = P + R @ span
    return T, ball_w


def place_link(mesh, pivots, p_a, p_b, up=(0, 0, 1)) -> np.ndarray:
    A, B = pivots
    u = (B - A).astype(float); u /= np.linalg.norm(u)
    w = (p_b - p_a).astype(float); w /= np.linalg.norm(w)
    # 最小旋转 u→w
    c = float(np.dot(u, w))
    if c > 1 - 1e-12:
        R0 = np.eye(3)
    elif c < -1 + 1e-12:
        a = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
        ax = np.cross(u, a); ax /= np.linalg.norm(ax)
        R0 = _axis_rot(ax, math.pi)
    else:
        ax = np.cross(u, w); ax /= np.linalg.norm(ax)
        R0 = _axis_rot(ax, math.acos(c))
    T = np.eye(4)
    T[:3, :3] = R0
    T[:3, 3] = p_a - R0 @ A
    return T


def _axis_rot(ax: np.ndarray, ang: float) -> np.ndarray:
    x, y, z = ax
    s, C, cc = math.sin(ang), 1 - math.cos(ang), math.cos(ang)
    return np.array([
        [cc + x*x*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, cc + y*y*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, cc + z*z*C]])


def _all_rod_errs(y_r, z_r, tilt):
    """给定 receiver 三参, 每条腿解最优臂角, 返回 6 杆长误差."""
    Trecv = recv_pose(y_r, z_r, tilt)
    errs = []
    for name, typ, P, holekey in SERVOS:
        hole_w = Trecv[:3, :3] @ RECV_HOLE[holekey] + Trecv[:3, 3]
        if typ == "main":
            span = ARM_BALL - ARM_SPLINE
        else:
            side = "L" if P[0] < 0 else "R"
            span = PITCH_BALL[side] - PITCH_SPLINE[side]
        prefer = math.atan2(hole_w[1] - P[1], hole_w[0] - P[0])
        psi = solve_arm_angle(P, span, hole_w, prefer)
        rod = float(np.linalg.norm(ball_on_circle(P, span, psi) - hole_w))
        errs.append(rod - ROD_LEN)
    return np.array(errs)


def solve_recv(seed=(0.0, 150.0, 0.0)):
    """解 receiver 三参 (y_r,z_r,tilt) 使 6 杆同时 =175mm."""
    from scipy.optimize import least_squares
    sol = least_squares(lambda x: _all_rod_errs(*x), seed,
                        xtol=1e-12, ftol=1e-12, max_nfev=4000)
    return sol.x, _all_rod_errs(*sol.x)


@dataclass
class Result:
    placements: List[RM.Placement]
    Trecv: np.ndarray
    balls: Dict[str, np.ndarray]
    rods: Dict[str, float]
    angles: Dict[str, float]


def build(y_r=0.0, z_r=185.0, tilt=0.0, show_enclosure=True) -> Result:
    Trecv = recv_pose(y_r, z_r, tilt)
    pls: List[RM.Placement] = []
    balls: Dict[str, np.ndarray] = {}
    rods: Dict[str, float] = {}
    angles: Dict[str, float] = {}

    arm_mesh = _load("Arm")
    main_link = _load("BearingMain")
    pitch_link = _load("BearingPitch")

    def leg_local(name, typ):
        if typ == "main":
            return ARM_SPLINE, ARM_BALL
        side = "L" if name.startswith("L_") else "R"
        return PITCH_SPLINE[side], PITCH_BALL[side]

    # 每条腿用各自真实网格几何直接解臂角; 右腿以左腿镜像角为 prefer ⇒ 选对称分支.
    psi_L: Dict[str, float] = {}
    transforms: Dict[str, Tuple[np.ndarray, np.ndarray, str]] = {}
    for name, typ, P, holekey in SERVOS:
        hole_w = Trecv[:3, :3] @ RECV_HOLE[holekey] + Trecv[:3, 3]
        spline_l, ball_l = leg_local(name, typ)
        span = ball_l - spline_l
        if name.startswith("L_"):
            prefer = math.atan2(hole_w[1] - P[1], hole_w[0] - P[0])
        else:  # 右腿: 偏好 = 左对应腿角的镜像 (π - ψ_L)
            prefer = math.pi - psi_L["L_" + name[2:]]
        psi = solve_arm_angle(P, span, hole_w, prefer)
        if name.startswith("L_"):
            psi_L[name] = psi
        T, ball_w = arm_transform(spline_l, ball_l, P, psi)
        transforms[name] = (T, ball_w, typ)
        angles[name] = math.degrees(psi)
        balls[name] = ball_w
        rods[name] = float(np.linalg.norm(ball_w - hole_w))

    for name, typ, P, holekey in SERVOS:
        T, ball_w, typ = transforms[name]
        hole_w = Trecv[:3, :3] @ RECV_HOLE[holekey] + Trecv[:3, 3]
        if typ == "main":
            pls.append(RM.Placement(f"Arm_{name}", arm_mesh.copy(), T, (228, 224, 212)))
            lk, piv = main_link, MAIN_LINK_PIVOTS
        else:
            side = "L" if name.startswith("L_") else "R"
            pls.append(RM.Placement(f"Pitch_{name}", _load(f"{side}_Pitcher"), T, (228, 224, 212)))
            lk, piv = pitch_link, PITCH_LINK_PIVOTS
        TL = place_link(lk, piv, ball_w, hole_w)
        pls.append(RM.Placement(f"Link_{name}", lk.copy(), TL, (235, 235, 235)))
        for p in (ball_w, hole_w):
            sph = trimesh.creation.icosphere(subdivisions=2, radius=3.6)
            Ts = np.eye(4); Ts[:3, 3] = p
            pls.append(RM.Placement(f"Brg_{name}_{p[2]:.0f}_{p[0]:.0f}", sph, Ts, (90, 90, 95)))

    pls.append(RM.Placement("Receiver", _load("Receiver"), Trecv, (40, 70, 150), alpha=0.6))

    if show_enclosure:
        pls.append(RM.Placement("Base", _load("Base"), np.eye(4), (170, 45, 45), alpha=0.9))
        pls.append(RM.Placement("L_Frame", _load("L_Frame"), np.eye(4), (190, 60, 60), alpha=0.9))
        pls.append(RM.Placement("R_Frame", _load("R_Frame"), np.eye(4), (190, 60, 60), alpha=0.9))

    return Result(pls, Trecv, balls, rods, angles)


def main(out_prefix="output/sr6_v2", **kw):
    r = build(**kw)
    print("receiver:", np.round(r.Trecv[:3, 3], 1),
          "tiltX", round(math.degrees(math.atan2(r.Trecv[2, 1], r.Trecv[2, 2])), 1))
    print("arm angles (deg) & rod lengths:")
    for k in r.rods:
        print(f"  {k:9s} psi={r.angles[k]:+7.1f}  rod={r.rods[k]:8.3f} (err {r.rods[k]-ROD_LEN:+.3f})")
    op = os.path.join(_HERE, out_prefix)
    os.makedirs(os.path.dirname(op), exist_ok=True)
    paths = RM.render_views(r.placements, op, res=1100)
    RM.export_glb(r.placements, op + ".glb")
    print("rendered:", [os.path.basename(p) for p in paths])
    return r


if __name__ == "__main__":
    main()
