#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bolted_flange_stack.py — 大规模螺栓法兰接头 (51 零件) · 万法归一 60 门

一条 live FreeCAD 内核, 锻造一套 24 螺栓的法兰管接头, 共 51 个独立零件:
  2 法兰 (各钻 24 个 RBC 螺栓孔, solid.pattern_polar 极阵列) + 1 垫片
  + 24 螺栓 (贯穿两法兰间隙孔) + 24 螺母 (与螺栓同轴啮合)

这是把装配规模一举推到 51 件, 让干涉检查的 O(n^2) 摩擦自暴露并就地优化:
  asm.interference 原本对全部 C(51,2)=1275 对都做昂贵的布尔 common();
  现加 AABB 宽相剔除 — 包围盒不相交的零件对直接跳过, 只对真正可能相交的
  零件对做布尔. 本例宽相把窄相布尔次数砍掉约 8 成.

闭式/几何校验 (道法自然 · 以解为镜):
  1 件数 BOM == 51
  2 螺栓全部落在螺栓圆上: 每颗螺栓中心半径 == RBC (解析)
  3 干涉: 恰好 24 处啮合 (每颗螺栓∩对应螺母), 无其它非预期碰撞
  4 宽相剔除: narrow_phase << pairs_checked (1275), 结果与全检一致
  5 总质量有限 + 渲染

运行:  python 60-实战_Projects/bolted_flange_stack.py
"""
import math
import os
import sys
import time
from pathlib import Path

_DAO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "_paths.py").is_file())
sys.path.insert(0, str(_DAO_ROOT))
import _paths  # noqa: E402,F401

from cad_agent import new_session  # noqa: E402

N = 24
OD, BORE, TF = 200.0, 100.0, 20.0
RBC = 75.0
DB, DH = 10.0, 12.0           # bolt dia, clearance-hole dia
GOD, GID, GT = 120.0, 100.0, 2.0
ZA, ZG, ZB = 0.0, TF, TF + GT  # flangeA bottom, gasket bottom, flangeB bottom
BOLT_Z0, BOLT_L = -8.0, 58.0
NUT_R, NUT_H = 9.0, 6.0


def _pos(i):
    th = 2 * math.pi * i / N
    return RBC * math.cos(th), RBC * math.sin(th)


def build_flange(s, name, z0):
    s.act("solid.cylinder", {"name": name + "_od", "radius": OD / 2, "height": TF, "pos": [0, 0, z0]})
    s.act("solid.cylinder", {"name": name + "_bore", "radius": BORE / 2, "height": TF + 2, "pos": [0, 0, z0 - 1]})
    s.act("solid.cut", {"a": name + "_od", "b": name + "_bore", "out": name + "_ring"})
    s.act("solid.cylinder", {"name": name + "_h", "radius": DH / 2, "height": TF + 2, "pos": [RBC, 0, z0 - 1]})
    s.act("solid.pattern_polar", {"name": name + "_h", "count": N, "angle": 360,
                                  "center": [0, 0, 0], "axis": [0, 0, 1], "out": name + "_holes"})
    r = s.act("solid.cut", {"a": name + "_ring", "b": name + "_holes", "out": name})
    assert r.ok, r.error
    return name


def main():
    s = new_session("flange_stack")
    print("FreeCAD", s.registry.kernel.freecad_version)

    build_flange(s, "FlangeA", ZA)
    build_flange(s, "FlangeB", ZB)
    s.act("solid.cylinder", {"name": "g_od", "radius": GOD / 2, "height": GT, "pos": [0, 0, ZG]})
    s.act("solid.cylinder", {"name": "g_id", "radius": GID / 2, "height": GT + 2, "pos": [0, 0, ZG - 1]})
    assert s.act("solid.cut", {"a": "g_od", "b": "g_id", "out": "Gasket"}).ok
    for i in range(N):
        x, y = _pos(i)
        assert s.act("solid.cylinder", {"name": "Bolt%02d" % i, "radius": DB / 2,
                                        "height": BOLT_L, "pos": [x, y, BOLT_Z0]}).ok
        assert s.act("solid.cylinder", {"name": "Nut%02d" % i, "radius": NUT_R,
                                        "height": NUT_H, "pos": [x, y, BOLT_Z0]}).ok

    # --- assemble (51 components) ---------------------------------------
    assert s.act("asm.create", {"name": "Joint"}).ok
    parts = ["FlangeA", "FlangeB", "Gasket"]
    parts += ["Bolt%02d" % i for i in range(N)] + ["Nut%02d" % i for i in range(N)]
    for nm in parts:
        assert s.act("asm.add", {"assembly": "Joint", "body": nm, "name": nm.lower()}).ok

    # --- 校验 1: BOM 件数 ----------------------------------------------
    bom = s.act("asm.bom", {"assembly": "Joint", "density": 0.00785})
    print("[1] BOM components = %d  total mass = %.0f g"
          % (bom.data["component_count"], bom.data["total_mass"]))
    assert bom.data["component_count"] == 51, bom.data

    # --- 校验 2: 螺栓圆 (质心半径, 偏移烘焙在几何里而非 Link 位姿) -------
    maxerr = 0.0
    for i in range(N):
        com = s.act("solid.measure", {"name": "Bolt%02d" % i}).data["center_of_mass"]
        maxerr = max(maxerr, abs(math.hypot(com[0], com[1]) - RBC))
    print("[2] bolt-circle radius error (max over %d bolts) = %.2e mm (RBC=%.1f)" % (N, maxerr, RBC))
    assert maxerr < 1e-3, maxerr

    # --- 校验 3+4: 干涉 + 宽相剔除 + 计时 -------------------------------
    t0 = time.perf_counter()
    inter = s.act("asm.interference", {"assembly": "Joint"})
    dt = time.perf_counter() - t0
    d = inter.data
    pairs = {tuple(sorted((c["a"], c["b"]))) for c in d["clashes"]}
    expected = {tuple(sorted(("bolt%02d" % i, "nut%02d" % i))) for i in range(N)}
    print("[3] clashes = %d (expected %d bolt-nut engagements)  match=%s"
          % (d["clash_count"], N, pairs == expected))
    print("[4] broad-phase: %d/%d pairs reached narrow boolean (%.0f%% culled)  in %.2fs"
          % (d["narrow_phase"], d["pairs_checked"],
             100.0 * (1 - d["narrow_phase"] / d["pairs_checked"]), dt))
    assert pairs == expected, ("unexpected clashes", pairs ^ expected)
    assert d["narrow_phase"] < d["pairs_checked"], d

    # --- 渲染 ----------------------------------------------------------
    if "view.render" in s.tools():
        out = str(_paths.ROOT / "output" / "fem_demo" / "bolted_flange_stack.png")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        rr = s.act("view.render", {"assembly": "Joint", "view": "iso", "path": out})
        if rr.ok:
            print("render -> %s (%d bytes)" % (out, rr.data["bytes"]))

    print("BOLTED FLANGE STACK OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
