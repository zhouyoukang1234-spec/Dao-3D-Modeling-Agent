#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从真实 frame 网格里提取 6 个舵机舱: 4 颗 M3 安装螺孔成组 → 形心 + 轴向(孔轴).

道法自然: 舵机输出轴 = 安装法兰 4 螺孔所在平面之法向 = 螺孔轴向.
不臆测, 全部从真实 STL 几何读出.
"""
from __future__ import annotations
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
import numpy as np
from ORS6_Stewart.tools._dao_axis_v2 import find_holes  # noqa
from ORS6_Stewart.parts import stl_path  # noqa
import trimesh


def servo_groups(name: str):
    m = trimesh.load(stl_path(name))  # 默认处理(合并顶点)以便邻接面分析
    holes = find_holes(m, r_min=1.3, r_max=2.1)  # M3 ≈ R1.6-1.8
    # 只保留轴向 ~ +Z 的孔 (舵机螺孔)
    zser = [h for h in holes if abs(h["axis_dir"][2]) > 0.9]
    pts = np.array([h["axis_midpoint"] for h in zser])
    # 合并同一螺孔的上下两段 (xy 几乎重合)
    merged = []
    used = [False] * len(pts)
    for i in range(len(pts)):
        if used[i]:
            continue
        grp = [i]
        used[i] = True
        for j in range(i + 1, len(pts)):
            if not used[j] and np.linalg.norm(pts[i][:2] - pts[j][:2]) < 3:
                grp.append(j); used[j] = True
        c = pts[grp].mean(axis=0)
        merged.append(c)
    merged = np.array(merged)
    # 按 y 分 3 组 (3 颗舵机沿 y 排列), 每组 4 孔
    order = np.argsort(merged[:, 1])
    merged = merged[order]
    print(f"\n=== {name}: {len(merged)} 螺孔 (合并后) ===")
    for p in merged:
        print(f"   ({p[0]:+7.2f},{p[1]:+7.2f},{p[2]:+7.2f})")
    # 用 KMeans 风格: 按 y 聚成 3 簇
    ys = merged[:, 1]
    # 简单按 y 间隔切 3 段
    ymin, ymax = ys.min(), ys.max()
    servos = []
    for k in range(3):
        lo = ymin + (ymax - ymin) * k / 3 - 1
        hi = ymin + (ymax - ymin) * (k + 1) / 3 + 1
        sel = merged[(ys >= lo) & (ys <= hi)]
        if len(sel) >= 2:
            servos.append(sel.mean(axis=0))
    print(f"  -> {len(servos)} 舵机形心:")
    for s in servos:
        print(f"     centroid ({s[0]:+7.2f},{s[1]:+7.2f},{s[2]:+7.2f})  axle≈+Z(mesh)")
    return servos


if __name__ == "__main__":
    for n in ["L_Frame", "R_Frame"]:
        servo_groups(n)
