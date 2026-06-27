#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/motor_mount.py — 智体亲手造形 (NEMA17 电机 L 支座)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为. 此非"为他人造工具", 而是【智体用自己的工具在底层 BREP 引擎上
亲手造一个真实可制造的零件】, 边造边 perceive→verify, 在真实使用中暴露并修复缺陷.

本例顺带成为 robust 倒角的回归: 一个带孔/带筋的复杂实体上 solid.chamfer(默认 auto 选棱)
须产出【水密单实体】(跳过孔口圆棱; 整批失败则贪心累加且每步校验, 决不破面).

用法 (须可见 freecadcmd):
    python examples/motor_mount.py [--out 输出目录] [--png]
退出码 0 = 设计意图全部验证通过.
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
BX, BY, BZ = 90.0, 70.0, 8.0          # 底板
VX, VY, VZ = 90.0, 8.0, 60.0          # 立板
M6 = 3.3                               # 底板 M6 过孔半径
BORE_R = 11.0                          # NEMA17 定位止口 Ø22
NEMA = 15.5                            # NEMA17 螺栓中心距 (±15.5 方阵)
M3 = 1.7                               # M3 过孔半径
BORE_Z = 33.0                          # 镗孔中心高 (留足上下边距)
CORNER = [(36, 26), (36, -26), (-36, 26), (-36, -26)]
WALL_YC = -BY / 2 + VY / 2             # 立板中心 y = -31
WALL_FRONT = -BY / 2 + VY             # 立板前面 y = -27
GUSSET_T, GUSSET = 8.0, 26.0          # 加强筋厚 / 直角边


def build(out_dir: str, save_png: bool) -> int:
    s = new_session("motor_mount", engine="freecad")

    def act(tool, **a):
        r = s.act(tool, a)
        tag = a.get("result") or a.get("name") or tool
        d = r.data or {}
        flag = "" if r.ok else "  [FAIL] " + str(r.error)
        print("  %-9s %-8s V=%s 水密=%s%s" %
              (tool.split(".")[-1], tag, d.get("volume"), d.get("watertight", d.get("closed")), flag))
        return r

    def cut(target, tool):
        act("solid.boolean", op="difference", a=target, b=tool, result=target, consume=True)

    def hole_Y(name, cx, cz, r, depth):
        c = [cx, WALL_YC, cz]
        act("solid.cylinder", radius=r, height=depth, center=c, name=name)
        act("solid.rotate", name=name, angle_deg=90, axis=[1, 0, 0], center=c)

    print("· 底板 + 4 角 M6 孔")
    act("solid.box", x=BX, y=BY, z=BZ, center=[0, 0, BZ / 2], name="base")
    for i, (cx, cy) in enumerate(CORNER):
        act("solid.cylinder", radius=M6, height=BZ * 3, center=[cx, cy, BZ / 2], name="m6_%d" % i)
        cut("base", "m6_%d" % i)

    print("· 立板 union")
    act("solid.box", x=VX, y=VY, z=VZ, center=[0, WALL_YC, VZ / 2], name="wall")
    act("solid.boolean", op="union", a="base", b="wall", result="bracket", consume=True)

    print("· 加强筋 ×2 (矩形 − 斜切盒 = 三角)")
    Hm_y = WALL_FRONT + GUSSET / 2
    Hm_z = BZ + GUSSET / 2
    push = 40.0 * math.sqrt(2) / 2  # 沿斜面法向 (0,1,1)/√2 推移
    for k, gx in ((1, 30.0), (2, -30.0)):
        cb, ck = "gb_%d" % k, "gc_%d" % k
        act("solid.box", x=GUSSET_T, y=GUSSET, z=GUSSET,
            center=[gx, WALL_FRONT + GUSSET / 2, BZ + GUSSET / 2], name=cb)
        act("solid.box", x=GUSSET_T + 6, y=80, z=80, center=[gx, Hm_y, Hm_z], name=ck)
        act("solid.rotate", name=ck, angle_deg=-45, axis=[1, 0, 0], center=[gx, Hm_y, Hm_z])
        act("solid.translate", name=ck, dx=0, dy=push, dz=push)
        cut(cb, ck)
        act("solid.boolean", op="union", a="bracket", b=cb, result="bracket", consume=True)

    print("· NEMA17 止口 + 方阵 4×M3")
    hole_Y("bore", 0, BORE_Z, BORE_R, depth=VY * 4)
    cut("bracket", "bore")
    for i, (sx, sz) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        hole_Y("n17_%d" % i, sx * NEMA, BORE_Z + sz * NEMA, M3, depth=VY * 4)
        cut("bracket", "n17_%d" % i)

    print("· 外棱倒角 (auto: 仅两侧皆平面的硬棱, 跳过孔口圆棱)")
    act("solid.chamfer", name="bracket", distance=1.0)

    print("· 感知")
    r = s.act("solid.perceive", {"name": "bracket", "resolution": 256,
                                 "out_dir": out_dir, "save_png": save_png})
    if r.ok:
        print("  " + r.data["summary"].replace("\n", " "))

    print("· 验证设计意图")
    rep = s.verify([
        Check(kind="exists", obj="bracket"),
        Check(kind="watertight", obj="bracket"),
        Check(kind="extent", obj="bracket", axis=0, lo=BX - 0.1, hi=BX + 0.1, label="X≈90"),
        Check(kind="extent", obj="bracket", axis=2, lo=VZ - 0.1, hi=VZ + 0.1, label="Z≈60"),
        Check(kind="count", value=1, label="仅余 bracket"),
    ])
    print(rep.render())

    e = s.act("solid.export", {"name": "bracket", "path": os.path.join(out_dir, "motor_mount.step")})
    print("· STEP: " + (str(e.data.get("path")) if e.ok else "FAIL " + str(e.error)))

    try:
        s.registry.freecad_kernel.close()
    except Exception:
        pass
    return 0 if rep.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "motor_mount_out"))
    ap.add_argument("--png", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    print("═══ 智体亲手造形: NEMA17 电机 L 支座 (BREP 直连) ═══")
    return build(args.out, args.png)


if __name__ == "__main__":
    sys.exit(main())
