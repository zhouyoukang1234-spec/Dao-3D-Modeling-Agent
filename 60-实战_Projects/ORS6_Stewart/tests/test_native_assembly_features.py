# -*- coding: utf-8 -*-
"""native_assembly.features 自检 — 认知层 (圆孔/枢轴自动识别)。

需本机真实 STL (经 DAO Bridge 取回到 STLs/); 无 STL 时整组跳过, 不影响离线 CI。
锁定行为: 从零件自身几何测出的关键枢轴必须与已知物理量一致 (无任何魔法坐标输入)。
"""
import os

import numpy as np
import pytest

pytest.importorskip("trimesh")
import trimesh  # noqa: E402

from ORS6_Stewart.parts import stl_path  # noqa: E402
from ORS6_Stewart.native_assembly import features as FT  # noqa: E402

_HAVE_ARM = os.path.exists(stl_path("Arm"))
_HAVE_LINK = os.path.exists(stl_path("MainLink"))


@pytest.mark.skipif(not _HAVE_ARM, reason="real STL not mounted")
def test_arm_hub_and_ball():
    m = trimesh.load(stl_path("Arm"), force="mesh")
    hs = FT.all_holes(m)
    assert len(hs) >= 2
    hub = FT.largest_hole(hs)
    ball = max(hs, key=lambda h: float(np.linalg.norm(h.center - hub.center)))
    # 舵机轴座是最大孔, 半径明显大于球铰端
    assert hub.radius > ball.radius
    # 臂枢轴长 (hub->ball) 在物理量级 (firmware mainArm=50, 实测含球座偏置 ~50-55)
    pivot = float(np.linalg.norm(ball.center - hub.center))
    assert 45.0 < pivot < 60.0


@pytest.mark.skipif(not _HAVE_LINK, reason="real STL not mounted")
def test_mainlink_length_matches_firmware():
    """主连杆枢轴-枢轴长度应≈ firmware 规格 175mm — 由纯几何复现, 验证认知层。"""
    m = trimesh.load(stl_path("MainLink"), force="mesh")
    ends = FT.end_holes(m)
    assert len(ends) == 2
    length = float(np.linalg.norm(ends[0].center - ends[1].center))
    assert abs(length - 175.0) < 3.0, f"detected {length:.2f} != 175"
