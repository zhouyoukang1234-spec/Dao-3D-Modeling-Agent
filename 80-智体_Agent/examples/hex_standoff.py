#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/hex_standoff.py — 智体亲手造形 (M10 六角隔离柱)
═══════════════════════════════════════════════════════════════════════════════
闭环 2 的产物: 造六角柱时撞上"原有图元仅 box/cyl/sphere/cone/torus, 无任何多边形棱柱"
之能力缺口, 遂给自己补 solid.extrude (任意 2D 闭合轮廓 → 拉伸棱柱). 本例即用之.
兼作 solid.extrude 的回归: 六角拉伸 → 钻孔 → 倒角, 须得水密单实体.

用法 (须可见 freecadcmd):
    python examples/hex_standoff.py [--out 目录] [--png]
退出码 0 = 验证通过.
"""
from __future__ import annotations
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session
from cad_agent.session import Check

W = 17.0              # 对边宽 (across-flats), M10 标准
H = 40.0              # 高
BORE = 5.5            # M10 过孔半径
R = W / math.sqrt(3)  # 外接圆半径


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("hex_standoff", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        d = r.data or {}
        flag = "" if r.ok else "  [FAIL] " + str(r.error)
        print("  %-9s V=%s 水密=%s%s" %
              (tool.split(".")[-1], d.get("volume"), d.get("watertight", d.get("closed")), flag))
        return r

    # 对边竖直的正六边形顶点 (偏 30°)
    hexpts = [[R * math.cos(math.radians(30 + 60 * i)),
               R * math.sin(math.radians(30 + 60 * i))] for i in range(6)]

    print("· 六角体 (extrude) 对边宽%g 高%g" % (W, H))
    act("solid.extrude", points=hexpts, height=H, center=[0, 0, H / 2], name="hex")
    print("· 通孔 Ø%g" % (BORE * 2))
    act("solid.cylinder", radius=BORE, height=H * 1.5, center=[0, 0, H / 2], name="bore")
    act("solid.boolean", op="difference", a="hex", b="bore", result="hex", consume=True)
    print("· 两端/棱倒角")
    act("solid.chamfer", name="hex", distance=1.0)

    r = s.act("solid.perceive", {"name": "hex", "resolution": 256,
                                 "out_dir": out_dir, "save_png": save_png})
    if r.ok:
        print("  " + r.data["summary"].replace("\n", " "))

    rep = s.verify([
        Check(kind="exists", obj="hex"),
        Check(kind="watertight", obj="hex"),
        Check(kind="extent", obj="hex", axis=2, lo=H - 0.1, hi=H + 0.1, label="Z≈40"),
        Check(kind="extent", obj="hex", axis=0, lo=W - 1.5, hi=2 * R + 0.5, label="X∈[对边,对角]"),
        Check(kind="count", value=1, label="仅余 hex"),
    ])
    print(rep.render())
    e = s.act("solid.export", {"name": "hex", "path": os.path.join(out_dir, "hex_standoff.step")})
    print("· STEP: " + (str(e.data.get("path")) if e.ok else "FAIL " + str(e.error)))

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "hex_standoff_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形: M10 六角隔离柱 (extrude 直连) ═══")
    return build(args.out, args.png)


if __name__ == "__main__":
    sys.exit(main())
