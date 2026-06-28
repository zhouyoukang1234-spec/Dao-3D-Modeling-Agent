# -*- coding: utf-8 -*-
"""native_assembly.features — 认知层: 从零件自身几何自动识别咬合特征.

核心: 圆孔/圆柱座(螺栓孔/轴承眼/舵机轴座)的**圆心+轴线**。这些就是装配的
咬合锚点——球铰中心、舵机转轴、连杆两端枢轴。检测出来即可**取代全部手写魔法坐标**。

算法 (霍夫式圆心累加, 对三角网鲁棒, 无需 fcl/RANSAC 库):
  给定候选轴向 a, 取所有"侧壁面"(法向⊥a)。圆孔/圆台的侧壁法向沿半径方向,
  其所在直线(过面心、沿法向)必过圆心 → 在 a⊥平面内对这些直线做 2D 累加,
  峰即圆心; 峰处侧壁面的 a 向坐标均值即圆心沿轴位置; 半径由面心到圆心中位距给出。
"""
from __future__ import annotations

from typing import List, Tuple

import numpy as np
import trimesh


class Hole:
    __slots__ = ("center", "axis", "radius", "votes", "n_faces")

    def __init__(self, center, axis, radius, votes, n_faces):
        self.center = np.asarray(center, float)
        self.axis = np.asarray(axis, float)
        self.radius = float(radius)
        self.votes = int(votes)
        self.n_faces = int(n_faces)

    def __repr__(self):
        c = self.center.round(2)
        return f"Hole(c={c.tolist()}, r={self.radius:.1f}, votes={self.votes})"


def _basis(a: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, t)
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    return u, v


def detect_holes(mesh: trimesh.Trimesh, axis, grid: float = 0.75,
                 lateral_tol: float = 0.25, merge_r: float = 8.0,
                 peak_frac: float = 0.5, min_radius: float = 1.2,
                 max_radius: float = 16.0) -> List[Hole]:
    """检测沿 `axis` 的圆孔/圆台, 返回按票数降序的 Hole 列表。"""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    u, v = _basis(a)
    fc = mesh.triangles_center
    fn = mesh.face_normals
    lat = np.abs(fn @ a) < lateral_tol
    if lat.sum() < 6:
        return []
    C = fc[lat]
    N = fn[lat]
    P2 = np.c_[C @ u, C @ v]
    D2 = np.c_[N @ u, N @ v]
    nrm = np.linalg.norm(D2, axis=1)
    ok = nrm > 1e-6
    P2, D2, C = P2[ok], D2[ok] / nrm[ok, None], C[ok]
    lo = P2.min(0) - 2
    hi = P2.max(0) + 2
    nx = int((hi[0] - lo[0]) / grid) + 1
    ny = int((hi[1] - lo[1]) / grid) + 1
    acc = np.zeros((nx, ny))
    span = max(hi[0] - lo[0], hi[1] - lo[1])
    ts = np.arange(-span, span, grid * 0.6)
    for p, d in zip(P2, D2):
        pts = p + ts[:, None] * d
        ix = ((pts[:, 0] - lo[0]) / grid).astype(int)
        iy = ((pts[:, 1] - lo[1]) / grid).astype(int)
        g = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        acc[ix[g], iy[g]] += 1
    thr = acc.max() * peak_frac
    cand = np.argwhere(acc >= thr)
    order = sorted(range(len(cand)), key=lambda i: -acc[cand[i][0], cand[i][1]])
    holes: List[Hole] = []
    taken: List[np.ndarray] = []
    for oi in order:
        i, j = cand[oi]
        c2 = np.array([lo[0] + i * grid, lo[1] + j * grid])
        if any(np.linalg.norm(c2 - t) < merge_r for t in taken):
            continue
        rad = np.linalg.norm(P2 - c2, axis=1)
        sel = rad < max_radius
        if sel.sum() < 4:
            continue
        rmed = float(np.median(rad[(rad < max_radius) & (rad > 0.3)])) if sel.any() else 0.0
        if rmed < min_radius or rmed > max_radius:
            continue
        taken.append(c2)
        center = c2[0] * u + c2[1] * v + float((C[sel] @ a).mean()) * a
        holes.append(Hole(center, a, rmed, int(acc[i, j]), int(sel.sum())))
    return holes


def best_axis_holes(mesh: trimesh.Trimesh, **kw) -> Tuple[np.ndarray, List[Hole]]:
    """在 X/Y/Z 三主轴中选检出圆孔最"干净"(票数最高)的轴。"""
    best = (None, [])
    best_score = -1.0
    for ax in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        hs = detect_holes(mesh, ax, **kw)
        if not hs:
            continue
        score = sum(h.votes for h in hs[:3])
        if score > best_score:
            best_score = score
            best = (ax, hs)
    return best


def largest_hole(holes: List[Hole]) -> Hole:
    """票数最高的孔 (通常是直径最大的承座, 如舵机轴座)。"""
    return max(holes, key=lambda h: h.votes)


def all_holes(mesh: trimesh.Trimesh, dedup_r: float = 6.0, **kw) -> List[Hole]:
    """跨 X/Y/Z 三轴检测并去重: 球铰孔/枢轴孔的轴向可能各不相同, 单轴会漏检。
    同一物理孔在不同候选轴会重复检出, 按空间距离去重并保留票数最高者。"""
    pool: List[Hole] = []
    for ax in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        pool.extend(detect_holes(mesh, ax, **kw))
    pool.sort(key=lambda h: -h.votes)
    kept: List[Hole] = []
    for h in pool:
        if any(np.linalg.norm(h.center - k.center) < dedup_r for k in kept):
            continue
        kept.append(h)
    return kept


def end_holes(mesh: trimesh.Trimesh, n: int = 2, **kw) -> List[Hole]:
    """连杆两端枢轴: 跨轴检测小孔(M4级), 取相距最远的一对/若干。"""
    import itertools
    hs = [h for h in all_holes(mesh, **kw) if h.radius < 6.0]
    if len(hs) < 2:
        return hs
    pair = max(itertools.combinations(hs, 2),
               key=lambda ab: np.linalg.norm(ab[0].center - ab[1].center))
    return list(pair)
