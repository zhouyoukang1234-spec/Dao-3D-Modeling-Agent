# -*- coding: utf-8 -*-
"""native_assembly.validate 自检 — 离线 (用仓库内 GLB, 不依赖远程 STL)。

这些断言只验证**体检器本身可靠**(能跑、schema 完整、能抓到已知的物理不一致),
而非断言装配通过——当前装配本就 FAIL,这正是反馈层的价值所在。
"""
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
GLB = os.path.normpath(os.path.join(HERE, "..", "assets", "ORS6_assembled.glb"))

pytest.importorskip("trimesh")
pytest.importorskip("rtree")


def _report():
    from ORS6_Stewart.native_assembly import validate
    return validate.run(GLB)


@pytest.mark.skipif(not os.path.exists(GLB), reason="committed GLB missing")
def test_report_schema():
    r = _report()
    assert r["n_parts"] >= 20
    for k in ("rods", "seating", "penetration", "pass"):
        assert k in r
    assert set(r["pass"]) == {"rods", "seating", "penetration", "all"}


@pytest.mark.skipif(not os.path.exists(GLB), reason="committed GLB missing")
def test_six_rods_measured():
    r = _report()
    for leg in ("LowerLeft", "UpperLeft", "LeftPitch", "RightPitch",
                "UpperRight", "LowerRight"):
        assert "length_mm" in r["rods"][leg], f"{leg} length not measured"


@pytest.mark.skipif(not os.path.exists(GLB), reason="committed GLB missing")
def test_detects_pitch_asymmetry():
    """已知缺陷: 两根 pitch 连杆不等长 (手册 p26 要求等长) — 体检器须能抓到。"""
    r = _report()
    assert r["rods"]["_summary"]["pitch_equal_pass"] is False
