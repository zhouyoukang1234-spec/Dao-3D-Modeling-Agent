# -*- coding: utf-8 -*-
"""native_assembly.solve — 实践层: 用**实测咬合特征**(零件自身几何)拼装, 零魔法坐标.

与 truth_assembly 的根本区别:
  truth_assembly 把 ARM_HUB / ARM_BALL / ANCH_MAIN / PITCH_ANCH 等当**输入**手填;
  本模块把它们当**输出**——由 features.py 从 STL 自动测出:
    - 臂: 最大圆孔=舵机轴座(hub), 离 hub 最远的孔=球铰端(ball);
    - 主连杆: 两端最远的一对小孔=两枢轴, 长度由几何给出(实测≈175, 印证 firmware);
    - 受接器: 每侧 X≈±66/Y≈0 处的三层安装孔, 上/下主腿各取一层(不再共用一个锚点)。
  这直接修掉了体检发现的两类缺陷: 关节悬空(魔法球铰偏 3-7mm) 与 连杆穿入受接器体
  (魔法锚点 Z 偏低 ~13mm、且上下腿被错误地塌缩到同一个孔)。

本阶段落地 4 条主腿(几何唯一可定)。2 条 pitch 腿用的是带偏置的折弯连杆, 需要 pitch
连杆的非共面枢轴模型 + 受接器上部锚点语义先验, 列为下一步。
"""
from __future__ import annotations

import os
from typing import List, Optional

import numpy as np
import trimesh

from ..parts import (PARTS, SR6, HOME_H, SERVO_SLOTS, RECV_PARTS,
                     DEFAULT_HIDDEN, stl_path)
from ..render import Part, uvsphere
from ..truth_assembly import place_2pt, place_link, solve_alpha, PAL, _load
from . import features as FT

H = SR6["servoPivotH"]
RECV_LIFT = np.array([0.0, 0.0, HOME_H])
BALL_R = 5.0


def arm_features():
    """臂的舵机轴座(hub)与球铰端(ball), 取自 Arm STL 自身几何。"""
    m = trimesh.load(stl_path("Arm"), force="mesh")
    hs = FT.all_holes(m)
    hub = FT.largest_hole(hs).center
    ball = max(hs, key=lambda h: float(np.linalg.norm(h.center - hub))).center
    return hub, ball


def receiver_main_mounts():
    """受接器两侧主连杆安装孔: X≈±66, Y≈0, 大孔, 按 Z 升序返回 (left, right)。"""
    m = trimesh.load(stl_path("Receiver"), force="mesh")
    hs = FT.all_holes(m, dedup_r=8.0)

    def side(sign):
        c = [h for h in hs if (h.center[0] * sign) > 55
             and abs(h.center[1]) < 8 and h.radius > 9]
        c.sort(key=lambda h: h.center[2])
        return c
    return side(-1), side(1)


def pitcher_features(name):
    """投手臂的舵机轴座(hub)与球铰端(ball), 取自 STL 自身几何。"""
    m = trimesh.load(stl_path(name), force="mesh")
    hs = FT.all_holes(m)
    hub = FT.largest_hole(hs).center
    ball = max(hs, key=lambda h: float(np.linalg.norm(h.center - hub))).center
    return hub, ball


def add_pitch(parts: List[Part]) -> List[Part]:
    """追加 2 条 pitch 腿: 投手臂 + 175mm 连杆 + 球铰, 锚点为受接器上部安装孔(实测)。

    现状(WIP): 球-球连杆已确证等长=175(手册 p26), 受接器锚点取实测 M4 孔
    (X=0, |Y|≈52.5, Z_local≈-10.5); 但 pitcher 臂的 home 摆角尚未用固件 IK 标定,
    导致 pitcher 臂与相邻主臂存在 ~5mm 重叠(手册 p32: 仅连杆"tab"应重叠, 臂体不应)。
    下一步用固件 pitch IK 求 home 摆角后即可消除。
    """
    for pname, sx, ysign in (("L_Pitcher", -99.6, 1.0),
                             ("R_Pitcher", 99.6, -1.0)):
        V, F = _load(pname)
        if V is None:
            continue
        hub, ball = pitcher_features(pname)
        sax = 1.0 if sx > 0 else -1.0
        shaft = np.array([sx, 0.0, H])
        M = np.array([0.0, ysign * 52.5, HOME_H - 10.5])
        Lp = float(np.linalg.norm(ball - hub))
        sol = solve_alpha(sx, 0.0, Lp, M, prefer_y_sign=(1 if sx < 0 else -1))
        if sol is None:
            continue
        _a, by, bz = sol
        balli = np.array([sx, by, bz])
        Vt = place_2pt(V, hub, ball, shaft, balli, [sax, 0, 0])
        parts.append(Part(Vt, F, PAL["horn"], pname))
        Vr, Fr = place_link("MainLink", balli, M)
        sn = "LeftPitch" if sx < 0 else "RightPitch"
        parts.append(Part(Vr, Fr, PAL["rod"], f"Rod_{sn}"))
        for pt in (balli, M):
            Vs, Fs = uvsphere(pt, r=BALL_R)
            parts.append(Part(Vs, Fs, PAL["ball"], "Ball"))
    return parts


def build_full() -> Optional[List[Part]]:
    """完整 6 腿(4 主 + 2 pitch), 全部由实测特征驱动。pitch 腿见 add_pitch 现状说明。"""
    parts = build_main()
    if parts is None:
        return None
    return add_pitch(parts)


def build_main() -> Optional[List[Part]]:
    """静态机架 + 抬升受接器 + 4 条主腿(臂+175连杆+球铰), 全部由实测特征驱动。"""
    if not os.path.exists(stl_path("Arm")):
        return None
    parts: List[Part] = []

    static = [n for n in PARTS if n not in RECV_PARTS and n not in DEFAULT_HIDDEN
              and n not in ("Arm", "L_Pitcher", "R_Pitcher")]
    for nm in static:
        V, F = _load(nm)
        if V is None:
            continue
        col = PAL["frame"] if nm in ("L_Frame", "R_Frame") else PAL["body"]
        parts.append(Part(V, F, col, nm))

    V, F = _load("Receiver")
    parts.append(Part(V + RECV_LIFT, F, PAL["recv"], "Receiver"))

    HUB, BALL = arm_features()
    arm_len = float(np.linalg.norm(BALL - HUB))
    left, right = receiver_main_mounts()
    # 上腿接最高孔(idx 2), 下腿接中孔(idx 1) — 由体检最小化座缝/穿模搜出的最优配置
    mount_idx = {"Lower": 1, "Upper": 2}

    Varm, Farm = _load("Arm")
    for sname, stype, sx, sy, _sign in SERVO_SLOTS:
        if stype != "main":
            continue
        sax = 1.0 if sx > 0 else -1.0
        shaft = np.array([sx, sy, H])
        mounts = right if sx > 0 else left
        idx = mount_idx["Lower" if sname.startswith("Lower") else "Upper"]
        M = mounts[idx].center + RECV_LIFT
        sol = solve_alpha(sx, sy, arm_len, M, prefer_y_sign=(1 if sy > 0 else -1))
        if sol is None:
            continue
        _a, by, bz = sol
        ball = np.array([sx, by, bz])
        if sx < 0:
            Vsrc = Varm * np.array([-1.0, 1.0, 1.0])
            Fsrc = Farm[:, ::-1]
            hub = HUB * np.array([-1.0, 1.0, 1.0])
            bn = BALL * np.array([-1.0, 1.0, 1.0])
        else:
            Vsrc, Fsrc, hub, bn = Varm, Farm, HUB.copy(), BALL.copy()
        Vt = place_2pt(Vsrc, hub, bn, shaft, ball, [sax, 0, 0])
        parts.append(Part(Vt, Fsrc, PAL["horn"], f"Arm_{sname}"))
        Vr, Fr = place_link("MainLink", ball, M)
        parts.append(Part(Vr, Fr, PAL["rod"], f"Rod_{sname}"))
        for pt in (ball, M):
            Vs, Fs = uvsphere(pt, r=BALL_R)
            parts.append(Part(Vs, Fs, PAL["ball"], "Ball"))
    return parts


def main():
    from ..truth_assembly import export_glb
    parts = build_main()
    if parts is None:
        print("[solve] 未读到 STL — 请先经 DAO Bridge 取回零件到 STLs/。")
        return
    outdir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..",
                                          "output", "native_assembly"))
    os.makedirs(outdir, exist_ok=True)
    glb = os.path.join(outdir, "ORS6_native_main.glb")
    export_glb(parts, glb)
    print("[solve] 写出", glb, "parts", len(parts))
    full = build_full()
    glb6 = os.path.join(outdir, "ORS6_native_full.glb")
    export_glb(full, glb6)
    print("[solve] 写出", glb6, "parts", len(full))


if __name__ == "__main__":
    main()
