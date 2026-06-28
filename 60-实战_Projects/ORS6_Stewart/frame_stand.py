#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SR6 立式框架（帐篷）落位 — 道法自然 / 物理接地。

根本纠错（自检闭环抓到）:
  之前框架停在打印平躺姿态、6 个舵机轴朝 +Z, 臂在水平面摆 —— 全错。
  说明书 p22 实物 + Base 真实几何证实:
    · Base 是外壳箱体, 其内部中央有一对 **45° 斜面**(法向 (±0.707,0,0.707), 面积最大),
      就是两半框架的落座面。
    · 两半框架各自倾 45° 立着坐进 Base, 在顶部中央相聚成"帐篷"(∧),
      脚分开撑向 Base 两侧; 舵机装在外侧面, 主舵机轴朝外+上、pitch 朝内。

本模块只做一件事: 给出 L/R 框架在 Base 世界系下的 4×4 落位变换 +
真实舵机(形心/输出轴向)的世界坐标, 供上层机构求解使用。
"""
from __future__ import annotations
import math
import numpy as np
import trimesh

from ORS6_Stewart.parts import stl_path

# Base 内部落座斜面中心 (从 Base 网格最大角面提取) ─ 左右对称
SEAT_LEAN_DEG = 45.0
# 舵机舱形心 (各自框架 mesh 局部, 来自 M3 安装孔成组提取)
SERVO_CENTROIDS_MESH = {
    "main_a": np.array([-76.40, -30.0, 18.27]),
    "main_b": np.array([-76.40, 0.0, 18.27]),
    "pitch": np.array([-76.40, 30.0, 26.52]),
}


def Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def frame_transform(side: str) -> np.ndarray:
    """L/R 框架世界落位: 绕 Y 倾 ∓45° + 平移使脚坐到 Base 底、脊向中心。"""
    name = "L_Frame" if side == "L" else "R_Frame"
    ang = -SEAT_LEAN_DEG if side == "L" else SEAT_LEAN_DEG
    fr = trimesh.load(stl_path(name), process=True)
    R = Ry(math.radians(ang))
    v = (R @ fr.vertices.T).T
    tz = 7.0 - v[:, 2].min()                       # 脚坐到 Base 内底 (~z7)
    if side == "L":
        tx = -2.0 - v[:, 0].max()                  # 内脊推向中心 x≈0
    else:
        tx = 2.0 - v[:, 0].min()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, 0.0, tz]
    return T


def servos_world(side: str):
    """返回该侧 3 个舵机的 (名称, 形心_世界, 输出轴_世界单位向量)。

    舵机轴 = 框架安装面法向 (mesh +Z) 经落位变换后的世界方向。
    """
    T = frame_transform(side)
    R = T[:3, :3]
    axle = R @ np.array([0.0, 0.0, 1.0])           # 主舵机: 朝外+上 45°
    out = []
    for key, c in SERVO_CENTROIDS_MESH.items():
        cm = c.copy()
        if side == "R":
            cm = cm * np.array([-1.0, 1.0, 1.0])   # R 框架镜像
        cw = R @ cm + T[:3, 3]
        n = axle.copy()                            # 主舵机轴: 朝外+上 45° (旋转已含左右)
        # pitch 舵机朝内: 取轴的水平分量反向 (装在垂直邻面)
        if key == "pitch":
            n = np.array([-n[0], n[1], n[2]])
        out.append((f"{side}_{key}", cw, n / np.linalg.norm(n)))
    return T, out


if __name__ == "__main__":
    from ORS6_Stewart import render_mate as RM

    base = trimesh.load(stl_path("Base"), process=False)
    pls = [RM.Placement("Base", base, np.eye(4), (180, 80, 80), alpha=0.28)]
    for side, col in (("L", (60, 120, 200)), ("R", (60, 180, 90))):
        name = "L_Frame" if side == "L" else "R_Frame"
        fr = trimesh.load(stl_path(name), process=True)
        T, sv = servos_world(side)
        pls.append(RM.Placement(side, fr, T, col, alpha=0.9))
        for nm, c, n in sv:
            print(f"{nm}: centroid={np.round(c, 1)} axle={np.round(n, 2)}")
    RM.render_views(pls, "/tmp/tent", res=820)
    print("rendered /tmp/tent_*.png")
