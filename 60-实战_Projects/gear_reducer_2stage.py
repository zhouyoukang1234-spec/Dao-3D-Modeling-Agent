#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gear_reducer_2stage.py — 两级复合齿轮减速箱 · 万法归一 60 门

一条 live FreeCAD 内核, 端到端锻造一套两级减速箱 (7 零件):
  级1: 输入小齿 z1 啮合中间大齿 z2     (m1=2,  z1=18, z2=36)
  级2: 中间小齿 z3 啮合输出大齿 z4     (m2=2.5, z3=18, z4=45)
  z2/z3 同轴刚性 (中间轴), 三轴: 输入/中间/输出

闭式/几何校验 (道法自然 · 以解为镜):
  1 中心距 a = m(z_a+z_b)/2 精确 (级1=54, 级2=78.75)
  2 传动比 i = (z2/z1)(z4/z3) = 2 x 2.5 = 5 (齿数闭式)
  3 级1啮合: meshing_phase_deg 相位 → 啮合 (~0 干涉); 错半齿 → 卡死
  4 级2啮合: 中间轴已被级1定相 → 复合定相摩擦点;
            实践搜索出可啮合定相 (无为而无不为), 并验证错半齿卡死

运行:  python 60-实战_Projects/gear_reducer_2stage.py
"""
import os
import sys
from pathlib import Path

_DAO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "_paths.py").is_file())
sys.path.insert(0, str(_DAO_ROOT))
import _paths  # noqa: E402,F401

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M1, Z1, Z2 = 2.0, 18, 36
M2, Z3, Z4 = 2.5, 18, 45
TH = 12.0          # 齿宽
LVL2 = 20.0        # 级2 轴向标高
A1 = M1 * (Z1 + Z2) / 2.0      # 54
A2 = M2 * (Z3 + Z4) / 2.0      # 78.75


def gear(s, name, m, z, zpos):
    assert s.act("param.body", {"name": name}).ok
    assert s.act("param.pad", {"body": name, "feature": name + "f",
                               "profile": {"gear": {"module": m, "teeth": z}},
                               "length": TH}).ok
    return name


def overlaps(s, asm):
    return {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"]
            for c in s.act("asm.interference", {"assembly": asm}).data["clashes"]}


def main():
    s = new_session("reducer")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- 校验 1: 中心距精确 ----------------------------------------------
    assert abs(A1 - 54.0) < 1e-9 and abs(A2 - 78.75) < 1e-9, (A1, A2)
    # --- 校验 2: 传动比闭式 ----------------------------------------------
    ratio = (Z2 / Z1) * (Z4 / Z3)
    print("center distances: stage1 a1=%.2f  stage2 a2=%.2f" % (A1, A2))
    print("ratio i = (z2/z1)(z4/z3) = %.1f x %.1f = %.3f" % (Z2 / Z1, Z4 / Z3, ratio))
    assert abs(ratio - 5.0) < 1e-9, ratio

    # --- 锻造 4 齿轮 + 3 轴 ----------------------------------------------
    gear(s, "Z1", M1, Z1, 0.0)
    gear(s, "Z2", M1, Z2, 0.0)
    gear(s, "Z3", M2, Z3, LVL2)
    gear(s, "Z4", M2, Z4, LVL2)
    for nm, h in (("ShaftIn", 40), ("ShaftMid", 60), ("ShaftOut", 60)):
        assert s.act("solid.cylinder", {"name": nm, "radius": 6, "height": h}).ok

    # --- 装配 ------------------------------------------------------------
    assert s.act("asm.create", {"name": "Reducer"}).ok
    for nm in ("Z1", "Z2", "Z3", "Z4", "ShaftIn", "ShaftMid", "ShaftOut"):
        assert s.act("asm.add", {"assembly": "Reducer", "body": nm, "name": nm.lower()}).ok
    # 三轴座标: 输入(0,0) 中间(A1,0) 输出(A1+A2,0)
    assert s.act("asm.place", {"name": "z1", "pos": [0, 0, 0]}).ok
    assert s.act("asm.place", {"name": "z2", "pos": [A1, 0, 0]}).ok
    assert s.act("asm.place", {"name": "z3", "pos": [A1, 0, LVL2]}).ok
    assert s.act("asm.place", {"name": "z4", "pos": [A1 + A2, 0, LVL2]}).ok
    assert s.act("asm.place", {"name": "shaftin", "pos": [0, 0, -10]}).ok
    assert s.act("asm.place", {"name": "shaftmid", "pos": [A1, 0, -10]}).ok
    assert s.act("asm.place", {"name": "shaftout", "pos": [A1 + A2, 0, -10]}).ok

    # --- 校验 3: 级1啮合 (相位 → 啮合; 错半齿 → 卡死) --------------------
    phi2 = meshing_phase_deg(0.0, Z1, Z2)
    assert s.act("asm.rotate", {"name": "z2", "axis": [0, 0, 1], "angle": phi2, "at": [A1, 0, 0]}).ok
    # z3 与 z2 同轴刚性: 施加同一相位
    assert s.act("asm.rotate", {"name": "z3", "axis": [0, 0, 1], "angle": phi2, "at": [A1, 0, LVL2]}).ok
    mesh1 = overlaps(s, "Reducer").get(("z1", "z2"), 0.0)
    print("stage1 phased mesh z1<->z2 overlap = %.2f mm^3 (phi=%.3f deg)" % (mesh1, phi2))
    assert mesh1 < 12.0, ("stage1 jams", mesh1)
    assert s.act("asm.rotate", {"name": "z2", "axis": [0, 0, 1], "angle": 180.0 / Z2, "at": [A1, 0, 0]}).ok
    jam1 = overlaps(s, "Reducer").get(("z1", "z2"), 0.0)
    assert jam1 > 40.0, ("stage1 mis-phase should jam", jam1)
    print("stage1 mis-phase jam = %.1f mm^3 (engagement real)" % jam1)
    assert s.act("asm.rotate", {"name": "z2", "axis": [0, 0, 1], "angle": -180.0 / Z2, "at": [A1, 0, 0]}).ok

    # --- 校验 4: 级2啮合 (复合定相摩擦 → 实践搜索可啮合定相) --------------
    pitch4 = 360.0 / Z4
    best_phi, best_ov = 0.0, 1e9
    steps = 24
    for k in range(steps):
        phi = k * pitch4 / steps
        s.act("asm.rotate", {"name": "z4", "axis": [0, 0, 1], "angle": phi, "at": [A1 + A2, 0, LVL2]})
        ov = overlaps(s, "Reducer").get(("z3", "z4"), 0.0)
        if ov < best_ov:
            best_ov, best_phi = ov, phi
        s.act("asm.rotate", {"name": "z4", "axis": [0, 0, 1], "angle": -phi, "at": [A1 + A2, 0, LVL2]})
    assert s.act("asm.rotate", {"name": "z4", "axis": [0, 0, 1], "angle": best_phi, "at": [A1 + A2, 0, LVL2]}).ok
    print("stage2 searched mesh z3<->z4 overlap = %.2f mm^3 (phi=%.3f deg)" % (best_ov, best_phi))
    assert best_ov < 15.0, ("stage2 no mesh found (compound clocking friction)", best_ov)
    # 错半齿应卡死
    s.act("asm.rotate", {"name": "z4", "axis": [0, 0, 1], "angle": pitch4 / 2.0, "at": [A1 + A2, 0, LVL2]})
    jam2 = overlaps(s, "Reducer").get(("z3", "z4"), 0.0)
    assert jam2 > 40.0, ("stage2 mis-phase should jam", jam2)
    print("stage2 mis-phase jam = %.1f mm^3 (engagement real)" % jam2)
    s.act("asm.rotate", {"name": "z4", "axis": [0, 0, 1], "angle": -pitch4 / 2.0, "at": [A1 + A2, 0, LVL2]})

    # --- BOM + 渲染 ------------------------------------------------------
    bom = s.act("asm.bom", {"assembly": "Reducer", "density": 0.00785})
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))
    assert bom.data["component_count"] == 7, bom.data

    if "view.render" in s.tools():
        out = str(_paths.ROOT / "output" / "fem_demo" / "gear_reducer_2stage.png")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        rr = s.act("view.render", {"assembly": "Reducer", "view": "top", "path": out})
        if rr.ok:
            print("render -> %s (%d bytes)" % (out, rr.data["bytes"]))

    print("GEAR REDUCER 2-STAGE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
