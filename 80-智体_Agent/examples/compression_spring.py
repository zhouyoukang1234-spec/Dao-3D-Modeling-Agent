#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/compression_spring.py — 智体亲手造形 (压缩弹簧 / coil spring)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为. 此非"为他人造工具", 而是【智体用自己的工具在底层 BREP 引擎上
亲手造一个真实可制造的零件】, 边造边 perceive→verify, 在真实使用中暴露并修复缺陷.

本例 (闭环 7) 暴露的缺口: 此前无"螺旋扫掠 (helix)", 弹簧/线圈/螺纹这类螺旋体造不出.
→ 补 solid.helix (Part.makeHelix 生螺旋线 + makePipeShell 扫圆截面), 作其回归:
    主螺旋线圈本体 (节距/丝径参数化) + 两端磨平端面 (压缩弹簧典型工艺) 用布尔切平.

用法 (须可见 freecadcmd):
    python examples/compression_spring.py [--out 输出目录] [--png]
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
COIL_R = 18.0          # 螺旋半径 (中心线)
WIRE_R = 2.6           # 丝径 (圆截面半径)
PITCH = 9.0            # 节距
FREE_H = 72.0          # 自由高度 (螺旋总高)
OD = 2 * (COIL_R + WIRE_R)   # 外径


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("spring", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        tag = a.get("result") or a.get("name") or tool
        d = r.data or {}
        flag = "" if r.ok else "  [FAIL] " + str(r.error)
        print("  %-13s %-8s V=%s 水密=%s%s" %
              (tool.split(".")[-1], tag, d.get("volume"), d.get("watertight", d.get("closed")), flag))
        return r

    print("· 主螺旋线圈 (helix, 节距 %g / 丝径 Ø%g)" % (PITCH, 2 * WIRE_R))
    act("solid.helix", name="spring", pitch=PITCH, height=FREE_H, radius=COIL_R, wire_radius=WIRE_R)

    print("· 两端磨平 (布尔切去顶/底超出自由高度的部分)")
    pad = 4 * WIRE_R
    act("solid.box", x=OD + 20, y=OD + 20, z=pad, center=[0, 0, -pad / 2 + 0.4], name="grind_lo")
    act("solid.boolean", op="difference", a="spring", b="grind_lo", result="spring", consume=True)
    act("solid.box", x=OD + 20, y=OD + 20, z=pad, center=[0, 0, FREE_H + pad / 2 - 0.4], name="grind_hi")
    act("solid.boolean", op="difference", a="spring", b="grind_hi", result="spring", consume=True)

    print("· 感知")
    r = s.act("solid.perceive", {"name": "spring", "resolution": 288,
                                 "out_dir": out_dir, "save_png": save_png})
    if r.ok:
        print("  " + r.data["summary"].replace("\n", " "))

    print("· 验证设计意图")
    rep = s.verify([
        Check(kind="exists", obj="spring"),
        Check(kind="watertight", obj="spring"),
        Check(kind="count", value=1, label="单连通簧体"),
        Check(kind="extent", obj="spring", axis=0, lo=OD - 0.5, hi=OD + 0.5, label="外径≈%.1f" % OD),
        Check(kind="extent", obj="spring", axis=2, lo=FREE_H - 1.2, hi=FREE_H + 0.2, label="自由高≈%.0f" % FREE_H),
    ])
    print(rep.render())

    e = s.act("solid.export", {"name": "spring", "path": os.path.join(out_dir, "compression_spring.step")})
    print("· STEP: " + (str(e.data.get("path")) if e.ok else "FAIL " + str(e.error)))

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "compression_spring_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形: 压缩弹簧 (螺旋扫掠 helix, BREP 直连) ═══")
    return build(args.out, args.png)


if __name__ == "__main__":
    raise SystemExit(main())
