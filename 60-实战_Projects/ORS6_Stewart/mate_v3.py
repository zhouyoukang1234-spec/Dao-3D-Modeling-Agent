#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ORS6_Stewart · mate_v3 — 立式帐篷框架 + 任意轴向臂摆 (道法自然 v3).

相对 v2 的根本纠错:
  v2 把 6 个舵机轴当成 mesh +Z(框架平躺), 臂在水平面摆 —— 错。
  v3 用 frame_stand: 框架 45° 立成帐篷坐进 Base, 主舵机轴朝外+上、pitch 朝内。
  臂绕**真实舵机轴 n**(世界向量)摆动, 球头在以 n 为法线的圆上。
  仍解 receiver 6-DOF(居中对称 ⇒ y,z,tilt 三参), 使 6 杆同时 = 175mm。

拓扑(说明书 p31): 同侧 2 主连杆汇聚 receiver 同一下孔; 投手连杆→上耳孔。
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import trimesh
from scipy.optimize import brentq, least_squares

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from ORS6_Stewart.parts import stl_path  # noqa: E402
from ORS6_Stewart import render_mate as RM  # noqa: E402
from ORS6_Stewart import frame_stand as FS  # noqa: E402

ROD_LEN = 175.0

# 臂局部接口 (花键中心 / 球心, mesh; 花键轴 = mesh +Z)
ARM_SPLINE = np.array([67.5, -7.68, 55.0])
ARM_BALL = np.array([67.5, 50.0, 51.0])
PITCH_SPLINE = {"L": np.array([-7.5, 22.32, 54.25]), "R": np.array([7.5, 22.32, 54.25])}
PITCH_BALL = {"L": np.array([-39.74, 97.72, 50.25]), "R": np.array([39.74, 97.72, 50.25])}

# receiver 真实孔 (receiver 局部) — 收敛拓扑
RECV_HOLE = {
    "Left": np.array([-60.0, 0.0, 0.0]),
    "Right": np.array([60.0, 0.0, 0.0]),
    "LeftPitch": np.array([-61.0, -14.24, 53.13]),
    "RightPitch": np.array([61.0, -14.24, 53.13]),
}
# 连杆 (BearingMain/Pitch) 两端球心 pivot, 局部
MAIN_LINK_PIVOTS = (np.array([-87.5, 0.0, 0.0]), np.array([87.5, 0.0, 0.0]))
PITCH_LINK_PIVOTS = (np.array([-82.2, 0.0, 0.0]), np.array([82.2, 0.0, 60.0]))

# 腿 -> receiver 孔映射
LEG_HOLE = {
    "L_main_a": "Left", "L_main_b": "Left", "L_pitch": "LeftPitch",
    "R_main_a": "Right", "R_main_b": "Right", "R_pitch": "RightPitch",
}


def _load(name):
    return trimesh.load(stl_path(name), process=False)


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def axis_rot(ax, ang):
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    s, cc = math.sin(ang), math.cos(ang)
    C = 1 - cc
    return np.array([
        [cc + x*x*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, cc + y*y*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, cc + z*z*C]])


def align_z_to(n):
    """旋转矩阵: mesh +Z -> 单位向量 n."""
    z = np.array([0.0, 0, 1.0])
    v = np.cross(z, n)
    s = np.linalg.norm(v)
    c = float(np.dot(z, n))
    if s < 1e-9:
        return np.eye(3) if c > 0 else axis_rot(np.array([1.0, 0, 0]), math.pi)
    return axis_rot(v, math.atan2(s, c))


def servo_table():
    """6 舵机: name -> (type, centroid C(world), axle n(world), spline_l, ball_l)."""
    tab = {}
    for side in ("L", "R"):
        _, sv = FS.servos_world(side)
        for nm, c, n in sv:
            typ = "pitch" if "pitch" in nm else "main"
            if typ == "main":
                sl, bl = ARM_SPLINE, ARM_BALL
            else:
                sl, bl = PITCH_SPLINE[side], PITCH_BALL[side]
            tab[nm] = (typ, c, n, sl, bl)
    return tab


def arm_ball(C, n, R0, span_l, psi):
    return C + axis_rot(n, psi) @ (R0 @ span_l)


def solve_psi(C, n, R0, span_l, hole, prefer):
    def f(psi):
        return float(np.linalg.norm(arm_ball(C, n, R0, span_l, psi) - hole)) - ROD_LEN
    N = 720
    best = None
    prev = prefer - math.pi
    pf = f(prev)
    for i in range(1, N + 1):
        psi = prefer - math.pi + 2 * math.pi * i / N
        fv = f(psi)
        cand = None
        if pf == 0:
            cand = prev
        elif pf * fv < 0:
            cand = brentq(f, prev, psi)
        if cand is not None:
            d = abs(((cand - prefer + math.pi) % (2 * math.pi)) - math.pi)
            if best is None or d < best[0]:
                best = (d, cand)
        prev, pf = psi, fv
    if best is None:
        psis = np.linspace(prefer - math.pi, prefer + math.pi, N)
        return min(psis, key=lambda p: abs(f(p)))
    return best[1]


TAB = servo_table()
R0 = {nm: align_z_to(v[2]) for nm, v in TAB.items()}


def recv_pose(y_r, z_r, tilt):
    T = np.eye(4)
    T[:3, :3] = Rx(tilt)
    T[:3, 3] = [0.0, y_r, z_r]
    return T


def _legs(Trecv):
    """每腿解 psi, 返回 {name: (psi, ball_w, hole_w, rod)}."""
    out = {}
    psiL = {}
    for nm in ["L_main_a", "L_main_b", "L_pitch", "R_main_a", "R_main_b", "R_pitch"]:
        typ, C, n, sl, bl = TAB[nm]
        hole = Trecv[:3, :3] @ RECV_HOLE[LEG_HOLE[nm]] + Trecv[:3, 3]
        span = bl - sl
        if nm.startswith("L_"):
            prefer = 0.0
        else:
            prefer = -psiL.get("L_" + nm[2:], 0.0)
        psi = solve_psi(C, n, R0[nm], span, hole, prefer)
        if nm.startswith("L_"):
            psiL[nm] = psi
        ball = arm_ball(C, n, R0[nm], span, psi)
        out[nm] = (psi, ball, hole, float(np.linalg.norm(ball - hole)))
    return out


def _errs(y_r, z_r, tilt):
    legs = _legs(recv_pose(y_r, z_r, tilt))
    return np.array([v[3] - ROD_LEN for v in legs.values()])


def solve_recv(seed=(0.0, 230.0, 0.0)):
    sol = least_squares(lambda x: _errs(*x), seed, xtol=1e-12, ftol=1e-12, max_nfev=5000)
    return sol.x, _errs(*sol.x)


@dataclass
class Result:
    placements: List[RM.Placement]
    Trecv: np.ndarray
    legs: Dict[str, tuple]


def place_link(mesh, pivots, p_a, p_b):
    A, B = pivots
    u = (B - A).astype(float); u /= np.linalg.norm(u)
    w = (p_b - p_a).astype(float); w /= np.linalg.norm(w)
    c = float(np.dot(u, w))
    if c > 1 - 1e-12:
        R = np.eye(3)
    elif c < -1 + 1e-12:
        a = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
        R = axis_rot(np.cross(u, a), math.pi)
    else:
        R = axis_rot(np.cross(u, w), math.acos(c))
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p_a - R @ A
    return T


def build(seed=(0.0, 230.0, 0.0), show_enclosure=True):
    x, err = solve_recv(seed)
    y_r, z_r, tilt = x
    Trecv = recv_pose(y_r, z_r, tilt)
    legs = _legs(Trecv)
    pls: List[RM.Placement] = []

    arm_mesh = _load("Arm")
    main_link = _load("BearingMain")
    pitch_link = _load("BearingPitch")

    for nm in legs:
        typ, C, n, sl, bl = TAB[nm]
        span = bl - sl
        psi, ball, hole, rod = legs[nm]
        R = axis_rot(n, psi) @ R0[nm]
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = C - R @ sl
        if typ == "main":
            pls.append(RM.Placement(f"Arm_{nm}", arm_mesh.copy(), T, (228, 224, 212)))
            lk, piv = main_link, MAIN_LINK_PIVOTS
        else:
            side = "L" if nm.startswith("L_") else "R"
            pls.append(RM.Placement(f"Pitch_{nm}", _load(f"{side}_Pitcher"), T, (228, 224, 212)))
            lk, piv = pitch_link, PITCH_LINK_PIVOTS
        TL = place_link(lk, piv, ball, hole)
        pls.append(RM.Placement(f"Link_{nm}", lk.copy(), TL, (235, 235, 235)))

    pls.append(RM.Placement("Receiver", _load("Receiver"), Trecv, (40, 70, 150), alpha=0.6))
    if show_enclosure:
        base = _load("Base")
        pls.append(RM.Placement("Base", base, np.eye(4), (175, 60, 60), alpha=0.25))
        for side in ("L", "R"):
            nm = "L_Frame" if side == "L" else "R_Frame"
            pls.append(RM.Placement(nm, trimesh.load(stl_path(nm), process=True),
                                    FS.frame_transform(side), (180, 75, 75), alpha=0.8))
    return Result(pls, Trecv, legs), err


def main(out_prefix="output/sr6_v3", **kw):
    r, err = build(**kw)
    print("receiver pos:", np.round(r.Trecv[:3, 3], 1),
          "tiltX(deg)", round(math.degrees(math.atan2(r.Trecv[2, 1], r.Trecv[2, 2])), 1))
    print("rod residuals (mm):", np.round(err, 3))
    for nm, (psi, ball, hole, rod) in r.legs.items():
        print(f"  {nm:9s} psi={math.degrees(psi):+7.1f}  rod={rod:8.3f}")
    op = os.path.join(_HERE, out_prefix)
    os.makedirs(os.path.dirname(op), exist_ok=True)
    paths = RM.render_views(r.placements, op, res=1100)
    print("rendered:", [os.path.basename(p) for p in paths])
    return r


if __name__ == "__main__":
    main()
