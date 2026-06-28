#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""flanged_coupling.py — 大规模复杂装配体实战 · 万法归一 60 门

一条 live FreeCAD 内核, 端到端锻造一套 8 螺栓法兰联轴器 (10 个零件):
  · 两片法兰: 参数化 pad 圆盘 + 中心镗孔 + 单个螺栓孔 → 极阵列 (pattern_polar)
  · 8 根螺栓: 贯穿两片法兰的螺栓圈

全程以闭式/精确几何校验 (道法自然 · 以解为镜):
  1 极阵列恰好 N=8 孔 (PartDesign PolarPattern occurrences 精确)
  2 装配 BOM 件数 == 2 + N == 10
  3 BOM 总质量 == 解析体积×密度  (<2%)
  4 干涉检查: 间隙孔 → 螺栓不啃法兰, 两法兰面接触不互穿
  5 法兰 FEM: 夹底面 + 轴向拉载, 报告安全系数

运行:  python 60-实战_Projects/flanged_coupling.py
"""
import math
import os
import sys
from pathlib import Path

_DAO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "_paths.py").is_file())
sys.path.insert(0, str(_DAO_ROOT))
import _paths  # noqa: E402,F401  五层 sys.path 注入

from cad_agent import new_session  # noqa: E402

# --- 参数 (mm) ----------------------------------------------------------------
OD_R = 60.0       # 法兰外半径  (Ø120)
BORE_R = 20.0     # 中心镗孔半径 (Ø40)
HOLE_R = 6.5      # 螺栓孔半径   (Ø13 间隙孔, 配 M12)
BOLT_R = 6.0      # 螺栓半径     (M12 公称)
BC_R = 45.0       # 螺栓圈半径
T = 12.0          # 单片法兰厚
N = 8             # 螺栓数
H_BOLT = 2 * T    # 螺栓长 = 贯穿两片
RHO = 0.00785     # 钢密度 g/mm^3


def build_flange(s, name):
    assert s.act("param.body", {"name": name}).ok
    assert s.act("param.pad", {"body": name, "feature": "Disk",
                               "profile": {"circle": OD_R, "at": [0, 0]}, "length": T}).ok
    assert s.act("param.pocket", {"body": name, "feature": "Bore",
                                  "profile": {"circle": BORE_R, "at": [0, 0]},
                                  "length": T, "through": True}).ok
    assert s.act("param.pocket", {"body": name, "feature": "Hole0",
                                  "profile": {"circle": HOLE_R, "at": [BC_R, 0]},
                                  "length": T, "through": True}).ok
    pp = s.act("param.pattern_polar", {"body": name, "feature": "BoltHoles",
                                       "originals": ["Hole0"], "axis": "Z",
                                       "count": N, "angle": 360})
    assert pp.ok, pp.error
    assert pp.data["occurrences"] == N, pp.data
    return pp


def main():
    s = new_session("coupling")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- 锻造两片法兰 (极阵列恰好 N 个螺栓孔) --------------------------------
    for nm in ("FlangeA", "FlangeB"):
        build_flange(s, nm)
    print("flange polar pattern: %d bolt holes each (exact)" % N)

    # --- 锻造 8 根螺栓 ------------------------------------------------------
    for i in range(N):
        assert s.act("solid.cylinder", {"name": "Bolt%d" % i,
                                        "radius": BOLT_R, "height": H_BOLT}).ok

    # --- 装配 --------------------------------------------------------------
    assert s.act("asm.create", {"name": "Coupling"}).ok
    assert s.act("asm.add", {"assembly": "Coupling", "body": "FlangeA",
                             "name": "flangea", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "Coupling", "body": "FlangeB", "name": "flangeb"}).ok
    # FlangeB 面对面叠在 FlangeA 之上
    assert s.act("asm.place", {"name": "flangeb", "pos": [0, 0, T]}).ok
    # 8 根螺栓置于螺栓圈, 贯穿两片
    for i in range(N):
        th = 2 * math.pi * i / N
        assert s.act("asm.add", {"assembly": "Coupling", "body": "Bolt%d" % i,
                                 "name": "bolt%d" % i}).ok
        assert s.act("asm.place", {"name": "bolt%d" % i,
                                   "pos": [BC_R * math.cos(th), BC_R * math.sin(th), 0]}).ok

    # --- 校验 2: BOM 件数 == 2 + N -----------------------------------------
    bom = s.act("asm.bom", {"assembly": "Coupling", "density": RHO})
    assert bom.ok, bom.error
    assert bom.data["component_count"] == 2 + N, bom.data
    # --- 校验 3: 总质量 == 解析 -------------------------------------------
    flange_vol = math.pi * (OD_R ** 2 - BORE_R ** 2) * T - N * math.pi * HOLE_R ** 2 * T
    bolt_vol = math.pi * BOLT_R ** 2 * H_BOLT
    analytic_mass = RHO * (2 * flange_vol + N * bolt_vol)
    err = abs(bom.data["total_mass"] / analytic_mass - 1.0)
    print("BOM: %d parts  mass=%.1f g  analytic=%.1f g  err=%.2f%%"
          % (bom.data["component_count"], bom.data["total_mass"], analytic_mass, err * 100))
    assert err < 0.02, ("mass mismatch", bom.data["total_mass"], analytic_mass)

    # --- 校验 4: 干涉 (间隙孔 → 螺栓不啃法兰, 法兰面接触不互穿) -------------
    clashes = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"]
               for c in s.act("asm.interference", {"assembly": "Coupling"}).data["clashes"]}
    bad = {k: v for k, v in clashes.items() if v > 1.0}
    assert not bad, ("unexpected interference (bolts should clear clearance holes)", bad)
    print("interference: %d significant clashes (clearance holes clear bolts)" % len(bad))

    # --- 校验 5: 法兰 FEM ---------------------------------------------------
    if "fem.solve" in s.tools():
        assert s.act("fem.setup", {"target": "FlangeA", "material": "steel",
                                   "mesh_size": 8}).ok
        assert s.act("fem.fix", {"select": {"axis": "z", "side": "min"}}).ok
        assert s.act("fem.load", {"select": {"axis": "z", "side": "max"},
                                  "kind": "force", "value": 20000,
                                  "direction": [0, 0, 1]}).ok
        fr = s.act("fem.solve", {"allowable_mpa": 250})
        assert fr.ok, fr.error
        print("flange FEM: max vM=%.2f MPa  SF=%.2f  passed=%s"
              % (fr.data["max_von_mises_mpa"], fr.data["safety_factor"], fr.data["passed"]))

    # --- 渲染存证 -----------------------------------------------------------
    if "view.render" in s.tools():
        out = str(_paths.ROOT / "output" / "fem_demo" / "flanged_coupling.png")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        rr = s.act("view.render", {"assembly": "Coupling", "view": "iso", "path": out})
        if rr.ok:
            print("render -> %s (%d bytes)" % (out, rr.data["bytes"]))

    print("FLANGED COUPLING OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
