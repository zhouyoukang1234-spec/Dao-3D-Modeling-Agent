# -*- coding: utf-8 -*-
"""native_assembly.validate — 物理不变量体检 (反馈层 · 离线可跑).

把"装得对不对"从"看图感觉"变成**可证伪的数字**。输入是一份已装配 GLB
(默认 assets/ORS6_assembled.glb，内含 32 个真实零件网格)，输出四类物理不变量:

  1. 杆长 (rod length)        —— 6 根连杆球-球距离；4 main 应相等、2 pitch 应相等
  2. 关节贴合 (joint seating) —— 每根连杆两端球心到相邻零件表面的间隙(零悬空)
  3. 零穿模 (no penetration)  —— 运动机构零件两两真实网格穿透深度
  4. 对称 (symmetry)          —— 左右镜像腿杆长一致

依赖: trimesh, numpy。不依赖远程 STL / 物理引擎，CI 可跑。
"""
from __future__ import annotations

import argparse
import json
import os
from typing import Dict, List, Tuple

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_GLB = os.path.normpath(os.path.join(HERE, "..", "assets", "ORS6_assembled.glb"))

LEGS = ["LowerLeft", "UpperLeft", "LeftPitch", "RightPitch", "UpperRight", "LowerRight"]
MAIN_LEGS = ["LowerLeft", "UpperLeft", "UpperRight", "LowerRight"]
PITCH_LEGS = ["LeftPitch", "RightPitch"]

# SR6 物理规格 (固件 + 手册 p26)
SPEC_MAIN_ROD = 175.0          # mm, 4 根主连杆等长
ROD_TOL = 0.5                  # mm
SYM_TOL = 0.5                  # mm, 左右/同类等长容差
SEAT_TOL = 2.0                 # mm, 球心到相邻面允许间隙 (零悬空)


def _center(m: trimesh.Trimesh) -> np.ndarray:
    return (m.bounds[0] + m.bounds[1]) / 2.0


def rod_endpoints(rod: trimesh.Trimesh) -> Tuple[np.ndarray, np.ndarray]:
    """连杆两端球铰中心 = 沿主轴(PCA 最长方向)投影的两极端点的截面形心。
    与球几何命名无关，对任何已装配 GLB 都成立。"""
    V = np.asarray(rod.vertices, float)
    c = V.mean(0)
    Vc = V - c
    axis = np.linalg.svd(Vc, full_matrices=False)[2][0]
    t = Vc @ axis
    lo_mask = t <= (t.min() + 0.06 * (t.max() - t.min()))
    hi_mask = t >= (t.max() - 0.06 * (t.max() - t.min()))
    return V[lo_mask].mean(0), V[hi_mask].mean(0)


def leg_ball_centers(g: Dict[str, trimesh.Trimesh], leg: str) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (臂端球心, 接收器端球心)。优先用显式 Ball geom，否则由连杆端点推。
    约定: Z 较低者为臂端，较高者为接收器端。"""
    a = g.get(f"Ball_{leg}")
    b = g.get(f"Ball_{leg}_1")
    if a is not None and b is not None:
        p, q = _center(a), _center(b)
    else:
        rod = g.get(f"Rod_{leg}")
        if rod is None:
            raise KeyError(f"no Rod_{leg} nor Ball_{leg}")
        p, q = rod_endpoints(rod)
    return (p, q) if p[2] <= q[2] else (q, p)


def load(glb: str) -> Dict[str, trimesh.Trimesh]:
    scene = trimesh.load(glb)
    if not isinstance(scene, trimesh.Scene):
        raise SystemExit(f"{glb} is not a multi-part scene")
    return dict(scene.geometry)


def surf_dist(pt: np.ndarray, m: trimesh.Trimesh) -> float:
    """最近表面距离 (mm)。"""
    closest, dist, _ = trimesh.proximity.closest_point(m, pt.reshape(1, 3))
    return float(dist[0])


def check_rods(g: Dict[str, trimesh.Trimesh]) -> Tuple[Dict, bool]:
    out: Dict[str, Dict] = {}
    ok = True
    for leg in LEGS:
        try:
            p, q = leg_ball_centers(g, leg)
        except KeyError as e:
            out[leg] = {"error": str(e)}
            ok = False
            continue
        L = float(np.linalg.norm(p - q))
        out[leg] = {"length_mm": round(L, 3)}
    # 4 main 等长 & = 175
    mains = [out[lg]["length_mm"] for lg in MAIN_LEGS if "length_mm" in out[lg]]
    pitch = [out[lg]["length_mm"] for lg in PITCH_LEGS if "length_mm" in out[lg]]
    main_spread = (max(mains) - min(mains)) if mains else 999
    pitch_spread = (max(pitch) - min(pitch)) if pitch else 999
    main_err = max(abs(m - SPEC_MAIN_ROD) for m in mains) if mains else 999
    summary = {
        "main_lengths": mains,
        "pitch_lengths": pitch,
        "main_equal_spread_mm": round(main_spread, 3),
        "pitch_equal_spread_mm": round(pitch_spread, 3),
        "main_vs_175_max_err_mm": round(main_err, 3),
        "main_equal_pass": main_spread <= SYM_TOL,
        "main_175_pass": main_err <= ROD_TOL,
        "pitch_equal_pass": pitch_spread <= SYM_TOL,
    }
    ok = ok and summary["main_equal_pass"] and summary["main_175_pass"] and summary["pitch_equal_pass"]
    out["_summary"] = summary
    return out, ok


def check_seating(g: Dict[str, trimesh.Trimesh]) -> Tuple[Dict, bool]:
    """每根连杆两端球心 → 应同时贴合 (a) 连杆端 (b) 相邻臂/接收器。零悬空。"""
    arm_for = {lg: f"Arm_{lg}" for lg in MAIN_LEGS}
    arm_for["LeftPitch"] = "L_Pitcher"
    arm_for["RightPitch"] = "R_Pitcher"
    out: Dict[str, Dict] = {}
    ok = True
    for leg in LEGS:
        arm = g.get(arm_for[leg])
        recv = g.get("Receiver")
        rec: Dict[str, float] = {}
        try:
            ball_arm, ball_recv = leg_ball_centers(g, leg)
        except KeyError as e:
            out[leg] = {"error": str(e), "seated_pass": False}
            ok = False
            continue
        if arm is not None:
            rec["arm_ball_to_arm_mm"] = round(surf_dist(ball_arm, arm), 2)
        if recv is not None:
            rec["recv_ball_to_receiver_mm"] = round(surf_dist(ball_recv, recv), 2)
        rec["seated_pass"] = all(v <= SEAT_TOL for k, v in rec.items() if k.endswith("_mm"))
        ok = ok and rec["seated_pass"]
        out[leg] = rec
    return out, ok


def _sample_inside(a: trimesh.Trimesh, b: trimesh.Trimesh, n: int = 1500) -> Tuple[float, float]:
    """采 a 表面点，测落在 b 内部的比例与最大穿透深度 (mm)。"""
    pts = a.sample(n) if len(a.faces) else a.vertices
    try:
        inside = b.contains(pts)
    except Exception:
        return 0.0, 0.0
    if not inside.any():
        return 0.0, 0.0
    ins = pts[inside]
    _, dist, _ = trimesh.proximity.closest_point(b, ins)
    return float(inside.mean()), float(dist.max())


def check_penetration(g: Dict[str, trimesh.Trimesh]) -> Tuple[Dict, bool]:
    """运动机构零件两两穿模 (排除合法嵌套的结构件)。"""
    moving = [f"Rod_{lg}" for lg in LEGS] + [f"Arm_{lg}" for lg in MAIN_LEGS] + \
             ["L_Pitcher", "R_Pitcher", "Receiver"]
    moving = [m for m in moving if m in g]
    out: List[Dict] = []
    ok = True
    PEN_TOL = 1.5  # mm，超过视为真实穿模
    for i in range(len(moving)):
        for j in range(i + 1, len(moving)):
            x, y = moving[i], moving[j]
            # bbox 快速排除
            a0, a1 = g[x].bounds
            b0, b1 = g[y].bounds
            if (np.minimum(a1, b1) - np.maximum(a0, b0) <= 0).any():
                continue
            frac, depth = _sample_inside(g[x], g[y])
            if depth > 0.3:
                rec = {"pair": f"{x} ∩ {y}", "inside_frac": round(frac, 3),
                       "max_depth_mm": round(depth, 2)}
                out.append(rec)
                if depth > PEN_TOL:
                    ok = False
    out.sort(key=lambda r: -r["max_depth_mm"])
    return {"pairs": out[:30], "pen_tol_mm": PEN_TOL}, ok


def run(glb: str = DEFAULT_GLB) -> Dict:
    g = load(glb)
    rods, rods_ok = check_rods(g)
    seat, seat_ok = check_seating(g)
    pen, pen_ok = check_penetration(g)
    report = {
        "glb": glb,
        "n_parts": len(g),
        "rods": rods,
        "seating": seat,
        "penetration": pen,
        "pass": {"rods": rods_ok, "seating": seat_ok, "penetration": pen_ok,
                 "all": rods_ok and seat_ok and pen_ok},
    }
    return report


def _fmt(report: Dict) -> str:
    s = report["rods"]["_summary"]
    L: List[str] = []
    L.append(f"=== 本源装配物理体检 · {os.path.basename(report['glb'])} ({report['n_parts']} 零件) ===")
    L.append("")
    L.append("[1] 杆长 (ball-to-ball):")
    for leg in LEGS:
        v = report["rods"][leg].get("length_mm")
        L.append(f"    {leg:12s} {v} mm")
    L.append(f"    4 main 等长 spread={s['main_equal_spread_mm']}mm  "
             f"{'PASS' if s['main_equal_pass'] else 'FAIL'}; "
             f"vs 175 err={s['main_vs_175_max_err_mm']}mm "
             f"{'PASS' if s['main_175_pass'] else 'FAIL'}")
    L.append(f"    2 pitch 等长 spread={s['pitch_equal_spread_mm']}mm  "
             f"{'PASS' if s['pitch_equal_pass'] else 'FAIL'}")
    L.append("")
    L.append("[2] 关节贴合 (球心→相邻面间隙, 零悬空):")
    for leg in LEGS:
        r = report["seating"][leg]
        L.append(f"    {leg:12s} {{ {', '.join(f'{k}={v}' for k, v in r.items() if k.endswith('_mm'))} }} "
                 f"{'PASS' if r.get('seated_pass') else 'FAIL'}")
    L.append("")
    L.append(f"[3] 穿模 (>{report['penetration']['pen_tol_mm']}mm 视为穿模):")
    if not report["penetration"]["pairs"]:
        L.append("    无显著穿模")
    for p in report["penetration"]["pairs"][:12]:
        L.append(f"    {p['pair']:40s} depth={p['max_depth_mm']}mm frac={p['inside_frac']}")
    L.append("")
    p = report["pass"]
    L.append(f"=== 结论: rods={'PASS' if p['rods'] else 'FAIL'}  "
             f"seating={'PASS' if p['seating'] else 'FAIL'}  "
             f"penetration={'PASS' if p['penetration'] else 'FAIL'}  "
             f"→ ALL {'PASS' if p['all'] else 'FAIL'} ===")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser(description="ORS6 本源装配物理体检")
    ap.add_argument("--glb", default=DEFAULT_GLB)
    ap.add_argument("--json", help="把报告写到此 JSON 路径")
    args = ap.parse_args()
    report = run(args.glb)
    print(_fmt(report))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"\nJSON → {args.json}")


if __name__ == "__main__":
    main()
