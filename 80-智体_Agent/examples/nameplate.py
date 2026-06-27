#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/nameplate.py — 智体亲手造形 (刻字铭牌 / engraved nameplate)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为. 此非"为他人造工具", 而是【智体用自己的工具在底层 BREP 引擎上
亲手造一个真实可制造的零件】, 边造边 perceive→verify, 在真实使用中暴露并修复缺陷.

本例 (闭环 10) 暴露的缺口: 此前无"文字成实体 (text)", 零件刻号/铭牌/凸字标记造不出.
→ 补 solid.text (Part.makeWireString 字体轮廓造字形面再拉伸; 含字腔孔的字 A/O/D 自动按
   面积判定内外环挖空), 作其回归: 底板上凹刻文字 (差集) + 凸起边框 (并集) + 安装孔.

用法 (须可见 freecadcmd):
    python examples/nameplate.py [--out 输出目录] [--png]
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
PLATE = (120.0, 40.0, 6.0)    # 底板 x/y/z
TEXT = "DAO CAD"
TXT_SIZE = 15.0
ENGRAVE = 1.6                 # 刻字深度
HOLE_R = 3.0


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("plate", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        tag = a.get("result") or a.get("name") or tool
        d = r.data or {}
        flag = "" if r.ok else "  [FAIL] " + str(r.error)
        print("  %-13s %-8s V=%s 水密=%s%s" %
              (tool.split(".")[-1], tag, d.get("volume"), d.get("watertight", d.get("closed")), flag))
        return r

    px, py, pz = PLATE
    print("· 底板")
    act("solid.box", x=px, y=py, z=pz, center=[0, 0, pz / 2], name="plate")

    print("· 刻字 (text 字形拉伸, 差集凹刻入顶面 %gmm)" % ENGRAVE)
    act("solid.text", text=TEXT, size=TXT_SIZE, depth=ENGRAVE + 1.0,
        center=[0, 0, pz - ENGRAVE], name="ink")
    act("solid.boolean", op="difference", a="plate", b="ink", result="plate", consume=True)

    print("· 四角安装孔")
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        h = "h%d" % i
        act("solid.cylinder", radius=HOLE_R, height=pz + 6,
            center=[sx * (px / 2 - 8), sy * (py / 2 - 8), pz / 2], name=h)
        act("solid.boolean", op="difference", a="plate", b=h, result="plate", consume=True)

    print("· 感知")
    r = s.act("solid.perceive", {"name": "plate", "resolution": 360, "view": "top",
                                 "out_dir": out_dir, "save_png": save_png})
    if r.ok:
        print("  " + r.data["summary"].replace("\n", " "))

    print("· 验证设计意图")
    rep = s.verify([
        Check(kind="exists", obj="plate"),
        Check(kind="watertight", obj="plate"),
        Check(kind="count", value=1, label="单连通铭牌 (刻字不断开板体)"),
        Check(kind="extent", obj="plate", axis=0, lo=px - 0.5, hi=px + 0.5, label="板长≈%.0f" % px),
        Check(kind="extent", obj="plate", axis=2, lo=pz - 0.5, hi=pz + 0.5, label="板厚≈%.0f" % pz),
    ])
    print(rep.render())

    e = s.act("solid.export", {"name": "plate", "path": os.path.join(out_dir, "nameplate.step")})
    print("· STEP: " + (str(e.data.get("path")) if e.ok else "FAIL " + str(e.error)))

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "nameplate_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形: 刻字铭牌 '%s' (文字成实体 text, BREP 直连) ═══" % TEXT)
    return build(args.out, args.png)


if __name__ == "__main__":
    raise SystemExit(main())
