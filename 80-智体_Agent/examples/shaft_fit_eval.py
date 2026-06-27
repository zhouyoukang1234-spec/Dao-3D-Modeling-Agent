#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/shaft_fit_eval.py — 智体亲手造形并【工程评价】 (轴-轴承座配合)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为. 此非"为他人造工具", 而是【智体用自己的工具在底层 BREP 引擎上
亲手造件, 并对其作工程评价】, 边造边 perceive→evaluate→verify, 在真实使用中发现并修缺陷.

本例 (闭环 11) 与前各轮不同: 从"造形(act)"深化到"工程评价(evaluate)".
此前度量仅几何 (体积/包围盒); 真实工程需问: 这件多重? 重心在哪? 装进座里干涉否? 间隙多少?
→ 补 solid.inspect (质量/质心/主惯性矩, 按材料密度) 与 solid.interference (干涉/间隙检测).
作其回归: 钢轴 + 两种孔径轴承座 (间隙配合 / 过盈配合), 评价质量并校核配合.

用法 (须可见 freecadcmd):
    python examples/shaft_fit_eval.py [--out 输出目录] [--png]
退出码 0 = 设计意图 + 工程评价全部符合预期.
"""
from __future__ import annotations
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session
from cad_agent.session import Check

# ── 设计意图 (参数化, 单位 mm) ────────────────────────────────────────
SHAFT_R, SHAFT_L = 10.0, 80.0
HOUS = (50.0, 50.0, 30.0)
BORE_CLEAR, BORE_PRESS = 10.25, 9.8   # 间隙配合 / 过盈配合 孔半径
STEEL, ALU = 7.85, 2.70               # 密度 g/cm³


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("fit", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        if not r.ok:
            print("  [FAIL] %s: %s" % (tool, r.error))
        return r

    print("· 钢轴 Ø%g×%g" % (2 * SHAFT_R, SHAFT_L))
    act("solid.cylinder", radius=SHAFT_R, height=SHAFT_L, center=[0, 0, SHAFT_L / 2], name="shaft")

    print("· 两种轴承座 (间隙孔 Ø%g / 过盈孔 Ø%g)" % (2 * BORE_CLEAR, 2 * BORE_PRESS))
    for nm, br in (("hous_clear", BORE_CLEAR), ("hous_press", BORE_PRESS)):
        act("solid.box", x=HOUS[0], y=HOUS[1], z=HOUS[2], center=[0, 0, HOUS[2] / 2], name=nm)
        act("solid.cylinder", radius=br, height=HOUS[2] + 6, center=[0, 0, HOUS[2] / 2], name=nm + "_bore")
        act("solid.boolean", op="difference", a=nm, b=nm + "_bore", result=nm, consume=True)

    print("· 工程评价 (inspect 质量特性)")
    sh = act("solid.inspect", name="shaft", density=STEEL).data
    print("  钢轴: 质量 %.1f g, 质心 %s, 回转半径 %s mm" %
          (sh["mass_g"], sh["center_of_mass"], sh.get("radius_of_gyration_mm")))
    hc = act("solid.inspect", name="hous_clear", density=ALU).data
    print("  铝座(间隙): 质量 %.1f g, 质心 %s" % (hc["mass_g"], hc["center_of_mass"]))

    print("· 配合校核 (interference 干涉/间隙)")
    fc = act("solid.interference", a="shaft", b="hous_clear").data
    fp = act("solid.interference", a="shaft", b="hous_press").data
    print("  轴⇔间隙座: 干涉=%s, 间隙=%s mm" % (fc["interfering"], fc.get("min_clearance_mm")))
    print("  轴⇔过盈座: 干涉=%s, 过盈重叠=%s mm³" % (fp["interfering"], fp.get("overlap_volume_mm3")))

    # 理论核对: 钢轴 V=πr²L → 质量; 间隙=孔半径-轴半径; 过盈重叠=环面积×座高
    m_theo = math.pi * SHAFT_R ** 2 * SHAFT_L / 1000.0 * STEEL
    clr_theo = BORE_CLEAR - SHAFT_R
    ov_theo = math.pi * (SHAFT_R ** 2 - BORE_PRESS ** 2) * HOUS[2]

    def near(a, b, tol):
        return a is not None and abs(a - b) <= tol

    print("· 验证设计意图 + 工程评价")
    rep = s.verify([
        Check(kind="exists", obj="shaft"),
        Check(kind="watertight", obj="shaft"),
        Check(kind="custom", label="钢轴质量≈%.1fg (理论)" % m_theo,
              fn=lambda ws: (near(sh["mass_g"], m_theo, 0.5), "实得 %.1f g" % sh["mass_g"])),
        Check(kind="custom", label="质心居轴几何中心 (0,0,%.0f)" % (SHAFT_L / 2),
              fn=lambda ws: (near(sh["center_of_mass"][2], SHAFT_L / 2, 1e-3)
                             and abs(sh["center_of_mass"][0]) < 1e-6, "z=%.3f" % sh["center_of_mass"][2])),
        Check(kind="custom", label="间隙配合: 不干涉且间隙≈%.2fmm" % clr_theo,
              fn=lambda ws: ((not fc["interfering"]) and near(fc.get("min_clearance_mm"), clr_theo, 0.02),
                             "干涉=%s 间隙=%s" % (fc["interfering"], fc.get("min_clearance_mm")))),
        Check(kind="custom", label="过盈配合: 干涉且重叠≈%.0fmm³" % ov_theo,
              fn=lambda ws: (fp["interfering"] and near(fp.get("overlap_volume_mm3"), ov_theo, ov_theo * 0.05),
                             "干涉=%s 重叠=%s" % (fp["interfering"], fp.get("overlap_volume_mm3")))),
    ])
    print(rep.render())

    if save_png:
        s.act("solid.perceive", {"name": "hous_clear", "resolution": 288, "out_dir": out_dir, "save_png": True})

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "shaft_fit_eval_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形 + 工程评价: 轴-轴承座配合 (inspect/interference, BREP 直连) ═══")
    return build(args.out, args.png)


if __name__ == "__main__":
    raise SystemExit(main())
