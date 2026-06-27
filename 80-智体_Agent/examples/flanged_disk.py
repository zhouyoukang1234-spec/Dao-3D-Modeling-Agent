#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/flanged_disk.py — 智体亲手造形 (带毂法兰盘)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为. 此非"为他人造工具", 而是【智体用自己的工具在底层 BREP 引擎上
亲手造一个真实可制造的零件】, 边造边 perceive→verify, 在真实使用中暴露并修复缺陷.

本例是闭环 3 的回归, 顺带覆盖两项在真实造形中补出的能力:
    · solid.pattern_polar — 单 bolt 孔 → 环形阵列 ×N (取代手写循环, 即真实 CAD 的阵列)
    · solid.fillet 的 near/within 定向选棱 — 只倒 hub 根那一条圆棱, 不殃及其余
并作为感知渲染的回归: 高面数 (>1.2 万面) 实体经 perceive 多视角渲染须【无空洞/无麻点】.

用法 (须可见 freecadcmd):
    python examples/flanged_disk.py [--out 输出目录] [--png]
退出码 0 = 设计意图全部验证通过.
"""
from __future__ import annotations
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session
from cad_agent.session import Check

# ── 设计意图 (参数化, 单位 mm) ────────────────────────────────────────
OD, T = 120.0, 14.0           # 盘外径 / 盘厚
HUB_D, HUB_H = 60.0, 20.0     # 凸毂直径 / 高 (总高 = T + HUB_H = 34)
BORE = 20.0                   # 中心通孔半径 (Ø40)
BOLT_R, BOLT_C, NB = 7.0, 45.0, 6   # 螺栓孔半径 / 分布圆半径 / 数
FILLET_R = 4.0                # hub 根倒圆半径


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("flange", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        tag = a.get("result") or a.get("name") or tool
        d = r.data or {}
        flag = "" if r.ok else "  [FAIL] " + str(r.error)
        print("  %-13s %-8s V=%s 水密=%s%s" %
              (tool.split(".")[-1], tag, d.get("volume"), d.get("watertight", d.get("closed")), flag))
        return r

    print("· 盘 ∪ 毂")
    act("solid.cylinder", radius=OD / 2, height=T, center=[0, 0, T / 2], name="disc")
    act("solid.cylinder", radius=HUB_D / 2, height=HUB_H, center=[0, 0, T + HUB_H / 2], name="hub")
    act("solid.boolean", op="union", a="disc", b="hub", result="flange", consume=True)

    print("· 中心镗通孔")
    act("solid.cylinder", radius=BORE, height=(T + HUB_H) * 2, center=[0, 0, (T + HUB_H) / 2], name="bore")
    act("solid.boolean", op="difference", a="flange", b="bore", result="flange", consume=True)

    print("· 单 bolt 孔 → 环形阵列 ×%d (pattern_polar)" % NB)
    act("solid.cylinder", radius=BOLT_R, height=T * 3, center=[BOLT_C, 0, T / 2], name="bolt")
    act("solid.pattern_polar", name="bolt", count=NB, angle=360, axis=[0, 0, 1],
        center=[0, 0, 0], result="bolts", consume=True)
    act("solid.boolean", op="difference", a="flange", b="bolts", result="flange", consume=True)

    print("· 定向倒圆 hub 根 (near 取在根圆上, within 只圈那一条棱)")
    act("solid.fillet", name="flange", radius=FILLET_R, near=[HUB_D / 2, 0, T], within=2.0)

    print("· 感知")
    r = s.act("solid.perceive", {"name": "flange", "resolution": 288,
                                 "out_dir": out_dir, "save_png": save_png})
    if r.ok:
        print("  " + r.data["summary"].replace("\n", " "))

    print("· 验证设计意图")
    rep = s.verify([
        Check(kind="exists", obj="flange"),
        Check(kind="watertight", obj="flange"),
        Check(kind="extent", obj="flange", axis=0, lo=OD - 0.1, hi=OD + 0.1, label="OD≈120"),
        Check(kind="extent", obj="flange", axis=2, lo=T + HUB_H - 0.1, hi=T + HUB_H + 0.1, label="H≈34"),
        Check(kind="count", value=1, label="仅余 flange"),
    ])
    print(rep.render())

    e = s.act("solid.export", {"name": "flange", "path": os.path.join(out_dir, "flanged_disk.step")})
    print("· STEP: " + (str(e.data.get("path")) if e.ok else "FAIL " + str(e.error)))

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "flanged_disk_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形: 带毂法兰盘 (环形阵列 + 定向倒圆, BREP 直连) ═══")
    return build(args.out, args.png)


if __name__ == "__main__":
    raise SystemExit(main())
