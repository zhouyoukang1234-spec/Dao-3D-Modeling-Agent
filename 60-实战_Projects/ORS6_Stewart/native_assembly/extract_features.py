# -*- coding: utf-8 -*-
"""native_assembly.extract_features — 认知层产出: 把每个零件的咬合特征(圆心/轴线/
枢轴长度)从其**自身几何**里测出来, 并与 truth_assembly 里的手写魔法坐标逐一对照,
量化"哪几个魔法数是错的、错多少"。这是用「测量」取代「试凑」的桥梁产物。

需要本机真实 STL (经 DAO Bridge 取回到 STLs/)。离线则跳过并提示。
运行: python -m ORS6_Stewart.native_assembly.extract_features
"""
from __future__ import annotations

import json
import os
from typing import Optional

import numpy as np
import trimesh

from ..parts import stl_path, SR6
from . import features as FT

# truth_assembly.py 中的魔法常量 (待被测量取代)
MAGIC = {
    "Arm.hub": [67.5, 0.0, 51.0],
    "Arm.ball": [67.5, 50.0, 51.0],
    "Arm.pivot_len": SR6["mainArm"],          # 50.0
    "L_Pitcher.hub": [-7.5, 30.0, 51.75],
    "L_Pitcher.ball": [-39.74, 97.72, 50.25],
    "R_Pitcher.hub": [7.5, 30.0, 51.75],
    "R_Pitcher.ball": [39.74, 97.72, 50.25],
    "Pitcher.pivot_len": SR6["pitchArm"],     # 75.0
    "MainLink.length": SR6["mainRod"],        # 175.0 (firmware IK)
}


def _load(name) -> Optional[trimesh.Trimesh]:
    p = stl_path(name)
    if not os.path.exists(p):
        return None
    return trimesh.load(p, force="mesh")


def _arm_like(name):
    """臂/投手: 最大孔=舵机轴座(hub), 离 hub 最远的孔=球铰端(ball)。"""
    m = _load(name)
    if m is None:
        return None
    hs = FT.all_holes(m)
    if len(hs) < 2:
        return None
    hub = FT.largest_hole(hs)
    ball = max(hs, key=lambda h: float(np.linalg.norm(h.center - hub.center)))
    return {
        "hub": hub.center.tolist(),
        "hub_axis": hub.axis.tolist(),
        "hub_r": round(hub.radius, 2),
        "ball": ball.center.tolist(),
        "ball_axis": ball.axis.tolist(),
        "ball_r": round(ball.radius, 2),
        "pivot_len": round(float(np.linalg.norm(ball.center - hub.center)), 2),
    }


def _link(name):
    """连杆: 两端最远的一对小孔 = 两枢轴; 距离 = 枢轴-枢轴长度。"""
    m = _load(name)
    if m is None:
        return None
    ends = FT.end_holes(m)
    if len(ends) < 2:
        return None
    p, q = ends[0].center, ends[1].center
    return {
        "pivot_a": p.tolist(),
        "pivot_b": q.tolist(),
        "length": round(float(np.linalg.norm(p - q)), 2),
    }


def run() -> dict:
    out = {"parts": {}, "compare": []}

    for nm in ("Arm", "L_Pitcher", "R_Pitcher"):
        f = _arm_like(nm)
        if f:
            out["parts"][nm] = f

    for nm in ("MainLink", "PitcherLink", "BearingMain", "BearingPitch"):
        f = _link(nm)
        if f:
            out["parts"][nm] = f

    rec = _load("Receiver")
    if rec is not None:
        hs = FT.all_holes(rec, dedup_r=8.0)
        out["parts"]["Receiver"] = {
            "n_candidate_holes": len(hs),
            "top_holes": [
                {"c": h.center.round(2).tolist(), "r": round(h.radius, 2),
                 "axis": h.axis.astype(int).tolist(), "votes": h.votes}
                for h in sorted(hs, key=lambda h: -h.votes)[:12]
            ],
            "note": "6 个连杆安装孔需结合 SR6 装配手册语义先验消歧 (纯几何不足以唯一确定)",
        }

    def cmp(key, detected):
        magic = MAGIC.get(key)
        if magic is None or detected is None:
            return
        d = np.asarray(detected, float)
        mg = np.asarray(magic, float)
        delta = float(np.linalg.norm(d - mg)) if d.shape == mg.shape else abs(
            float(d) - float(mg))
        out["compare"].append({
            "feature": key,
            "magic": np.round(mg, 2).tolist() if mg.ndim else round(float(mg), 2),
            "detected": np.round(d, 2).tolist() if d.ndim else round(float(d), 2),
            "delta": round(delta, 2),
        })

    P = out["parts"]
    if "Arm" in P:
        cmp("Arm.hub", P["Arm"]["hub"])
        cmp("Arm.ball", P["Arm"]["ball"])
        cmp("Arm.pivot_len", P["Arm"]["pivot_len"])
    if "L_Pitcher" in P:
        cmp("L_Pitcher.hub", P["L_Pitcher"]["hub"])
        cmp("L_Pitcher.ball", P["L_Pitcher"]["ball"])
    if "R_Pitcher" in P:
        cmp("R_Pitcher.hub", P["R_Pitcher"]["hub"])
        cmp("R_Pitcher.ball", P["R_Pitcher"]["ball"])
    if "MainLink" in P:
        cmp("MainLink.length", P["MainLink"]["length"])
    return out


def main():
    rep = run()
    if not rep["parts"]:
        print("[features] 未读到 STL — 请先经 DAO Bridge 取回零件到 STLs/。")
        return
    outdir = os.path.join(os.path.dirname(__file__), "..", "output", "native_assembly")
    outdir = os.path.abspath(outdir)
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, "features.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(rep, fh, ensure_ascii=False, indent=2)

    print("=== 认知层: 实测咬合特征 (取自零件自身几何) ===")
    for nm, f in rep["parts"].items():
        if "pivot_len" in f:
            print(f"  {nm:12s} hub={np.round(f['hub'], 1).tolist()} "
                  f"ball={np.round(f['ball'], 1).tolist()} 枢轴长={f['pivot_len']}")
        elif "length" in f:
            print(f"  {nm:12s} 枢轴-枢轴长度={f['length']}")
        else:
            print(f"  {nm:12s} 候选孔={f['n_candidate_holes']} ({f['note']})")
    print("\n=== 魔法坐标 vs 实测 (delta=偏差 mm) ===")
    for c in rep["compare"]:
        print(f"  {c['feature']:20s} magic={c['magic']}  detected={c['detected']}  Δ={c['delta']}")
    print(f"\n[features] 写出 {path}")


if __name__ == "__main__":
    main()
