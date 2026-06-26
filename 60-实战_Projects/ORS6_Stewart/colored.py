#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · colored — 逐件着色装配体 (单一真相源)

道生一: 31 个独立 STL = 唯一几何真相 (逐件加载, 绝不合并成单网格)
一生二: 已验证固件 IK (kinematics.py) 驱动运动件位置
二生三: 真实硬件配色 (红机体/红连杆 · 白舵机摇臂 · 铬球头关节)
三生万物: 任意 T-Code 姿态 → 多视角着色渲染 + 着色 GLB 导出

为什么上一轮是"一坨浆糊":
  1. 旧路径渲染**合并单网格**装配体 → 无逐件材质 → 整体一坨、无色彩区分;
  2. 在已处于装配坐标系的 STL 上**叠加运动学位姿** → 几何漂移错乱。
本模块从 0 重建: 静态件原位加载, 运动件仅由 IK 驱动, 逐件着色。
"""
from __future__ import annotations
import math
import os
from typing import Dict, List, Tuple

import numpy as np

from .render import Part, render, hex_rgb, cylinder, uvsphere
from .kinematics import StewartIK, TCODE_HOME, ARM_PIVOT_STL
from .parts import (PARTS, SR6, HOME_H, SERVO_SLOTS, RECV_PARTS,
                    DEFAULT_HIDDEN, stl_path)

FRAME_X = 99.6
_INSTANCED = {"Arm", "L_Pitcher", "R_Pitcher"}

# ── 真实硬件配色 (对照实物照片标定) ──
PALETTE = {
    "body":   hex_rgb(0xd62828),   # 机体结构件 (亮红)
    "frame":  hex_rgb(0xd62828),   # L/R 框架
    "recv":   hex_rgb(0xc81f1f),   # 接收器 (略深红)
    "rod":    hex_rgb(0xd62828),   # 连杆 — 实物为红色
    "horn":   hex_rgb(0xf0ece4),   # 舵机摇臂 / 枢纽块 — 白
    "ball":   hex_rgb(0xc2c6cc),   # 球头关节 — 铬
}

# 标准多视角 (一致取景)
VIEWS = {
    "iso":   (1, -1, 0.5),
    "front": (0, -1, 0.05),
    "side":  (1, 0, 0.05),
    "top":   (0, 0, 1),
}


def _load(name) -> Tuple[np.ndarray, np.ndarray]:
    import trimesh
    p = stl_path(name)
    if not os.path.exists(p):
        return None, None
    m = trimesh.load(p, force="mesh")
    return np.asarray(m.vertices, float), np.asarray(m.faces, int)


def _Ry(deg):
    t = math.radians(deg); c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rx(deg):
    t = math.radians(deg); c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def build_colored(pose: Tuple = TCODE_HOME) -> List[Part]:
    """Return a list[Part] = fully colored SR6 assembly at the given T-Code pose.

    Static structural parts load at native assembly coordinates; servo arms rotate
    by their IK angle-delta from home; the receiver elevates to HOME_H (+pose);
    the 6 push-rods are generated parametrically from arm-tip → receiver-mount.
    """
    ik = StewartIK()
    geom = ik.compute_full_geometry(*pose)
    home = ik.compute_full_geometry(*TCODE_HOME)
    tx, ty, tz, roll, pitch, twist = ik.compute_receiver_pose(*pose)
    recv_dz = tz - HOME_H

    parts: List[Part] = []

    # ── A. static structural (red), native coords ──
    static = [n for n in PARTS if n not in RECV_PARTS and n not in DEFAULT_HIDDEN
              and n not in _INSTANCED]
    for nm in static:
        V, F = _load(nm)
        if V is None:
            continue
        col = PALETTE["frame"] if nm in ("L_Frame", "R_Frame") else PALETTE["body"]
        parts.append(Part(V, F, col, nm))

    # ── B. 4 main servo arms (white horns), instanced + IK-rotated ──
    Varm, Farm = _load("Arm")
    if Varm is not None:
        for sname, stype, sx, sy, _sign in SERVO_SLOTS:
            if stype != "main":
                continue
            is_left = sx < 0
            V = Varm.copy()
            if is_left:
                V = V * np.array([-1, 1, 1.0])
                F = Farm[:, ::-1]
                piv = np.array([-ARM_PIVOT_STL[0], ARM_PIVOT_STL[1], ARM_PIVOT_STL[2]])
            else:
                F = Farm
                piv = np.array(ARM_PIVOT_STL)
            shaft = np.array([sx, sy, SR6["servoPivotH"]])
            delta = math.degrees(geom["arm_angles"][sname] - home["arm_angles"][sname])
            Vt = (V - piv) @ _Ry(delta).T + shaft
            parts.append(Part(Vt, F, PALETTE["horn"], f"Arm_{sname}"))

    # ── B2. pitch horns (white) ──
    for pname in ("L_Pitcher", "R_Pitcher"):
        V, F = _load(pname)
        if V is None:
            continue
        sname = "LeftPitch" if pname.startswith("L_") else "RightPitch"
        delta = math.degrees(geom["arm_angles"][sname] - home["arm_angles"][sname])
        if abs(delta) > 0.01:
            sx = -FRAME_X if pname.startswith("L_") else FRAME_X
            piv = np.array([sx, 0, SR6["servoPivotH"]])
            V = (V - piv) @ _Ry(delta).T + piv
        parts.append(Part(V, F, PALETTE["horn"], pname))

    # ── C. receiver + T-wist head (red), elevated to HOME_H + pose ──
    for nm in (n for n in RECV_PARTS if n not in DEFAULT_HIDDEN):
        V, F = _load(nm)
        if V is None:
            continue
        if abs(roll) > 0.01 or abs(pitch) > 0.01:
            V = V @ _Rx(pitch).T @ _Ry(roll).T
        V = V + np.array([tx, ty, HOME_H + recv_dz])
        parts.append(Part(V, F, PALETTE["recv"], nm))

    # ── D. 6 push-rods (red) + chrome ball joints, from verified IK ──
    for sname, _stype, _sx, _sy, _sign in SERVO_SLOTS:
        tip = geom["arm_tips"][sname]
        mount = geom["recv_mounts"][sname]
        V, F = cylinder(tip, mount, r=3.0)
        parts.append(Part(V, F, PALETTE["rod"], f"Rod_{sname}"))
        for pt in (tip, mount):
            Vs, Fs = uvsphere(pt, r=5.0)
            parts.append(Part(Vs, Fs, PALETTE["ball"], f"Ball_{sname}"))

    return parts


def assembly_bounds(parts: List[Part]):
    allv = np.vstack([p.V for p in parts if len(p.V)])
    return allv.min(0), allv.max(0)


def render_views(pose=TCODE_HOME, out_dir="output/renders", label="home",
                 views=None, W=900, H=900):
    """Render the colored assembly from each view to PNG. Returns list of paths."""
    from PIL import Image
    parts = build_colored(pose)
    bounds = assembly_bounds(parts)
    views = views or VIEWS
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for vn, vd in views.items():
        img = render(parts, view_dir=vd, W=W, H=H, bounds=bounds)
        p = os.path.join(out_dir, f"ORS6_{label}_{vn}.png")
        Image.fromarray(img).save(p)
        paths.append(p)
    return paths


def export_glb(pose=TCODE_HOME, out_path="output/ORS6_home_colored.glb"):
    """Export the colored assembly as a single GLB scene (per-part face colors)."""
    import trimesh
    parts = build_colored(pose)
    scene = trimesh.Scene()
    for p in parts:
        if len(p.F) == 0:
            continue
        m = trimesh.Trimesh(vertices=p.V, faces=p.F, process=False)
        col = (np.array([*p.color, 1.0]) * 255).astype(np.uint8)
        m.visual.face_colors = np.tile(col, (len(p.F), 1))
        scene.add_geometry(m, geom_name=p.name or "part")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    scene.export(out_path)
    return out_path
