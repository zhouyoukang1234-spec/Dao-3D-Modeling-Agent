# -*- coding: utf-8 -*-
"""高速位姿拟合: 点云溅射代替逐面光栅 (~100x), 用于大网格 (ORS6_home 21万面)
对照实物照片搜最佳相机位姿轮廓 IoU. 道.感.校 的高速前向."""
from __future__ import annotations
import os, sys
import numpy as np
import trimesh

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import dao_jiao as DJ            # noqa: E402
import dao_perception as dp      # noqa: E402


class FastFitter(DJ.PoseFitter):
    """与 PoseFitter 同接口, 但 render_mask 用 splat_mask (点云溅射)."""

    def __init__(self, pts):
        self.P = np.asarray(pts, float)
        self.C = self.P.mean(0)
        self.R = float(np.linalg.norm(self.P.max(0) - self.P.min(0))) * 1.05
        self.fc = None
        self.vcol = None

    def render_mask(self, az, el, W=170, H=170):
        cam = dp.camera_orbit(self.C, self.R, az, el, width=W, height=H, fov_deg=35)
        m = dp.splat_mask(self.P, cam, close=max(2, W // 80))

        class _RR:  # mimic RenderResult.mask interface
            pass
        rr = _RR()
        rr.mask = m
        return rr

    def render_rgb(self, az, el, W=560, H=560):
        cam = dp.camera_orbit(self.C, self.R, az, el, width=W, height=H, fov_deg=35)
        m = dp.splat_mask(self.P, cam, close=max(2, W // 80))
        img = np.ones((H, W, 3))
        img[m] = np.array([0.80, 0.15, 0.12])
        return img, m


def sample_stl(path, n=40000):
    m = trimesh.load(path, process=False)
    if isinstance(m, trimesh.Scene):
        m = m.to_geometry()
    pts, _ = trimesh.sample.sample_surface(m, n)
    return np.asarray(pts, float)


def sample_mesh(V, F, n=40000):
    m = trimesh.Trimesh(vertices=np.asarray(V, float), faces=np.asarray(F, int), process=False)
    pts, _ = trimesh.sample.sample_surface(m, n)
    return np.asarray(pts, float)
