#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · render — 第一性原理软件渲染器 (numpy z-buffer)

无任何外部渲染依赖 (不需要 OpenGL / CadQuery / matplotlib)。
给定若干 (顶点, 面, 颜色) 部件 + 相机方向, 做正交投影 + z-buffer 光栅化
+ 漫反射着色, 输出 RGB 图像 (HxWx3 uint8)。

道法自然: 渲染即"以模型反观实物"——逐件着色, 而非合并成一坨单网格。
"""
from __future__ import annotations
import numpy as np


def look_at_basis(view_dir, up=(0, 0, 1)):
    """Return (right, true_up, forward) orthonormal basis looking along view_dir."""
    f = np.asarray(view_dir, float)
    f = f / (np.linalg.norm(f) + 1e-12)
    up = np.asarray(up, float)
    if abs(np.dot(f, up)) > 0.99:          # degenerate: view nearly parallel to up
        up = np.array([0.0, 1.0, 0.0])
    r = np.cross(up, f); r /= (np.linalg.norm(r) + 1e-12)
    u = np.cross(f, r)
    return r, u, f


class Part:
    """A renderable mesh part: vertices (N,3), triangle faces (M,3), rgb color 0..1."""
    __slots__ = ("V", "F", "color", "name")

    def __init__(self, V, F, color, name=""):
        self.V = np.asarray(V, np.float64)
        self.F = np.asarray(F, np.int64)
        self.color = np.asarray(color, np.float64)
        self.name = name


def render(parts, view_dir=(1, -1, 0.6), up=(0, 0, 1),
           W=900, H=900, light_dir=(0.4, -0.7, 0.8),
           bg=(1, 1, 1), ambient=0.32, margin=0.06, bounds=None):
    """Render parts (list[Part]) to an HxWx3 uint8 image via orthographic z-buffer.

    bounds: optional (cmin(3), cmax(3)) world AABB to frame consistently across views.
    """
    right, tup, fwd = look_at_basis(view_dir, up)
    R = np.stack([right, tup, fwd], axis=0)        # world->camera rows
    L = np.asarray(light_dir, float); L /= (np.linalg.norm(L) + 1e-12)

    if bounds is None:
        allv = np.vstack([p.V for p in parts if len(p.V)])
        cmin, cmax = allv.min(0), allv.max(0)
    else:
        cmin, cmax = np.asarray(bounds[0], float), np.asarray(bounds[1], float)
    corners = np.array([[x, y, z] for x in (cmin[0], cmax[0])
                        for y in (cmin[1], cmax[1]) for z in (cmin[2], cmax[2])])
    cc = corners @ R.T
    umin, umax = cc[:, 0].min(), cc[:, 0].max()
    vmin, vmax = cc[:, 1].min(), cc[:, 1].max()
    span = max(umax - umin, vmax - vmin) * (1 + 2 * margin)
    uc, vc = (umin + umax) / 2, (vmin + vmax) / 2
    scale = (min(W, H)) / span
    ucen, vcen = W / 2, H / 2

    img = np.empty((H, W, 3), np.float64); img[:] = np.asarray(bg, float)
    zbuf = np.full((H, W), -1e30, np.float64)

    for p in parts:
        if len(p.F) == 0:
            continue
        cam = p.V @ R.T
        sx = (cam[:, 0] - uc) * scale + ucen
        sy = vcen - (cam[:, 1] - vc) * scale
        depth = cam[:, 2]
        tri = p.F
        v0 = p.V[tri[:, 0]]; v1 = p.V[tri[:, 1]]; v2 = p.V[tri[:, 2]]
        n = np.cross(v1 - v0, v2 - v0)
        nl = np.linalg.norm(n, axis=1); nz = nl > 1e-12
        n[nz] /= nl[nz][:, None]
        shade = ambient + (1 - ambient) * np.clip(np.abs(n @ L), 0, 1)
        P0 = np.stack([sx[tri[:, 0]], sy[tri[:, 0]], depth[tri[:, 0]]], 1)
        P1 = np.stack([sx[tri[:, 1]], sy[tri[:, 1]], depth[tri[:, 1]]], 1)
        P2 = np.stack([sx[tri[:, 2]], sy[tri[:, 2]], depth[tri[:, 2]]], 1)
        _raster(img, zbuf, P0, P1, P2, shade, p.color)

    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def _raster(img, zbuf, P0, P1, P2, shade, base):
    H, W = zbuf.shape
    x0, y0 = P0[:, 0], P0[:, 1]
    x1, y1 = P1[:, 0], P1[:, 1]
    x2, y2 = P2[:, 0], P2[:, 1]
    area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    keep = np.abs(area) > 1e-9
    idx = np.nonzero(keep)[0]
    minx = np.floor(np.minimum(np.minimum(x0, x1), x2)).astype(int)
    maxx = np.ceil(np.maximum(np.maximum(x0, x1), x2)).astype(int)
    miny = np.floor(np.minimum(np.minimum(y0, y1), y2)).astype(int)
    maxy = np.ceil(np.maximum(np.maximum(y0, y1), y2)).astype(int)
    for t in idx:
        ax0, ax1 = max(0, minx[t]), min(W - 1, maxx[t])
        ay0, ay1 = max(0, miny[t]), min(H - 1, maxy[t])
        if ax0 > ax1 or ay0 > ay1:
            continue
        xs = np.arange(ax0, ax1 + 1)
        ys = np.arange(ay0, ay1 + 1)
        gx, gy = np.meshgrid(xs + 0.5, ys + 0.5)
        ar = area[t]
        w0 = ((x1[t] - gx) * (y2[t] - gy) - (x2[t] - gx) * (y1[t] - gy)) / ar
        w1 = ((x2[t] - gx) * (y0[t] - gy) - (x0[t] - gx) * (y2[t] - gy)) / ar
        w2 = 1.0 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        z = w0 * P0[t, 2] + w1 * P1[t, 2] + w2 * P2[t, 2]
        sub = zbuf[ay0:ay1 + 1, ax0:ax1 + 1]
        better = inside & (z > sub)
        if not better.any():
            continue
        sub[better] = z[better]
        col = base * shade[t]
        isub = img[ay0:ay1 + 1, ax0:ax1 + 1]
        isub[better] = col


def hex_rgb(h):
    """0xRRGGBB int -> rgb float triple 0..1."""
    return np.array([((h >> 16) & 255) / 255, ((h >> 8) & 255) / 255, (h & 255) / 255])


# --- primitive mesh builders (for parametric rods + ball joints) ---

def cylinder(p1, p2, r=3.0, n=16):
    """Triangulated cylinder between p1 and p2 with radius r."""
    p1 = np.asarray(p1, float); p2 = np.asarray(p2, float)
    axis = p2 - p1; Llen = np.linalg.norm(axis)
    if Llen < 1e-6:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    d = axis / Llen
    a = np.array([1.0, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(d, a); u /= np.linalg.norm(u)
    v = np.cross(d, u)
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.array([np.cos(t) * u + np.sin(t) * v for t in ang]) * r
    V = np.vstack([p1 + ring, p2 + ring])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + i], [j, n + j, n + i]]
    cb = len(V); V = np.vstack([V, p1, p2])
    for i in range(n):
        j = (i + 1) % n
        F += [[cb, j, i], [cb + 1, n + i, n + j]]
    return V, np.asarray(F, int)


def uvsphere(c, r=5.0, nu=10, nv=10):
    """Triangulated UV sphere centred at c with radius r."""
    c = np.asarray(c, float)
    us = np.linspace(0, np.pi, nu); vs = np.linspace(0, 2 * np.pi, nv, endpoint=False)
    V = []
    for a in us:
        for b in vs:
            V.append([r * np.sin(a) * np.cos(b), r * np.sin(a) * np.sin(b), r * np.cos(a)])
    V = np.asarray(V) + c
    F = []
    for i in range(nu - 1):
        for j in range(nv):
            j2 = (j + 1) % nv
            a = i * nv + j; b = i * nv + j2
            cc = (i + 1) * nv + j; dd = (i + 1) * nv + j2
            F += [[a, b, cc], [b, dd, cc]]
    return V, np.asarray(F, int)
