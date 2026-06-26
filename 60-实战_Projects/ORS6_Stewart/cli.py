#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · cli — 统一命令行入口

Usage:
    python -m ORS6_Stewart <command> [args...]

Commands:
    info                          — STL_ROOT / PARTS 数量 / HOME_H 概览
    health                        — 完整健康诊断 (STL + verify + IK + tools)
    verify                        — V1-V8 数值验证
    ik-verify                     — IK standalone 11 项检查
    rebuild-bounds                — 重建 _stl_bounds.json
    overview                      — 31 零件概览 (按 Z 排序)
    servo                         — 舵机槽位提取 (Z=46mm)
    section <Z>                   — 任意 Z 截面分析
    info-part <Name>              — 单零件 trimesh 数值信息
    mass [Part] [material]        — 质量属性 (默认 pla)
    quality [Part]                — 质量检查
    workspace [res]               — IK 工作空间 (默认 res=10)
    clearance                     — 装配间距
    assembly                      — 装配整体统计
    ik-forward L0 L1 L2 R0 R1 R2  — 正运动学 (0-1 归一化)
    collision P1 P2               — 两零件碰撞
    rods [L0 L1 L2 R0 R1 R2]      — 杆几何 (默认 home)
    render [L0...R2] [label] [dir]— 逐件着色软件渲染 (多视角PNG, 无外部引擎)
    glb [L0...R2] [path]          — 导出逐件着色 GLB (可旋转查看)
    build [L0...R2] [label]       — CadQuery 构建一姿态
    build-fc [L0...R2] [label]    — FreeCAD 构建 (需 FreeCAD 环境)
    motion [cadquery|freecad]     — 15 姿态动画序列
    serve [port]                  — 启动 Viewer HTTP 服务 (默认 :8871)
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent       # ORS6_Stewart/
PROJECTS_DIR = HERE.parent                    # 60-实战_Projects/
if str(PROJECTS_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECTS_DIR))

import ORS6_Stewart as S  # noqa: E402


def cmd_info():
    print(f"ORS6_Stewart v{S.__version__}")
    print(f"  STL_ROOT: {S.STL_ROOT}")
    print(f"  BOUNDS_FILE: {S.BOUNDS_FILE}")
    print(f"  PARTS: {len(S.PARTS)}")
    print(f"  RECV_PARTS: {len(S.RECV_PARTS)}")
    print(f"  DEFAULT_HIDDEN: {len(S.DEFAULT_HIDDEN)}")
    print(f"  HOME_H: {S.HOME_H}mm")
    print(f"  SR6: {S.SR6}")
    missing = [n for n in S.PARTS if not os.path.exists(S.stl_path(n))]
    print(f"  Missing STL: {len(missing)} {missing if missing else '(all found)'}")


def cmd_health():
    """Comprehensive health: STL + verify + IK + tool detection."""
    t0 = time.time()
    score, total = 0, 0

    # 1. STL Parts
    print("=== STL Parts ===")
    missing = [n for n in S.PARTS if not os.path.exists(S.stl_path(n))]
    total += 1
    if not missing:
        score += 1
        print(f"  [OK] {len(S.PARTS)}/{len(S.PARTS)} STL files exist")
    else:
        print(f"  [FAIL] missing {len(missing)}: {missing}")

    # 2. verify_assembly (V1-V8)
    print("\n=== Assembly Verify ===")
    total += 1
    try:
        r = S.verify_assembly()
        n_pass = sum(1 for v in r.values() if v == "PASS")
        if n_pass == len(r):
            score += 1
            print(f"  [OK] verify: {n_pass}/{len(r)} PASS")
        else:
            print(f"  [FAIL] verify: {n_pass}/{len(r)}")
            for k, v in r.items():
                if v != "PASS":
                    print(f"    - {k}: {v}")
    except Exception as e:
        print(f"  [FAIL] verify exception: {e}")

    # 3. IK standalone
    print("\n=== IK Verify ===")
    total += 1
    try:
        checks = S.verify_ik_standalone()
        n_pass = sum(1 for _, ok, _ in checks if ok)
        if n_pass == len(checks):
            score += 1
            print(f"  [OK] IK: {n_pass}/{len(checks)} PASS")
        else:
            print(f"  [FAIL] IK: {n_pass}/{len(checks)}")
    except Exception as e:
        print(f"  [FAIL] IK exception: {e}")

    # 4. Tool availability
    print("\n=== Tools ===")
    tools = {}
    for pkg, symbol in [("trimesh", "trimesh"), ("numpy", "numpy"),
                        ("cadquery", "cadquery"), ("build123d", "build123d"),
                        ("OCP.StlAPI", "OCP.StlAPI"), ("scipy", "scipy")]:
        total += 1
        try:
            __import__(symbol)
            score += 1
            tools[pkg] = "OK"
            print(f"  [OK] {pkg}")
        except ImportError:
            tools[pkg] = "MISSING"
            print(f"  [--] {pkg} (optional)")
    # trimesh/numpy ARE required, so fail if missing
    for req in ("trimesh", "numpy"):
        if tools.get(req) == "MISSING":
            print(f"  [WARN] {req} required for analysis functions!")

    # 5. FreeCAD (optional)
    total += 1
    freecad_candidates = [
        r"D:\安装的软件\FreeCAD 1.0\bin\FreeCADCmd.exe",
        r"D:\安装的软件\FreeCAD 0.21\bin\FreeCADCmd.exe",
        r"C:\Program Files\FreeCAD 1.0\bin\FreeCADCmd.exe",
    ]
    freecad_found = next((p for p in freecad_candidates if os.path.exists(p)), None)
    if freecad_found:
        score += 1
        print(f"  [OK] FreeCAD: {freecad_found}")
    else:
        print(f"  [--] FreeCAD not found in common paths (optional)")

    # Grade
    dt = time.time() - t0
    pct = round(100 * score / total) if total > 0 else 0
    grade = ("S" if pct == 100 else "A" if pct >= 90 else "B"
             if pct >= 75 else "C" if pct >= 60 else "F")
    print(f"\n{'='*50}")
    print(f"Health: {score}/{total} ({pct}%) Grade {grade} [{dt:.1f}s]")


def cmd_rods(pose_args):
    pose = tuple(int(a) for a in pose_args) if len(pose_args) == 6 else S.TCODE_HOME
    rods = S.compute_rods(pose)
    for r in rods:
        print(f"  {r['servo']:12s} arm={r['arm_angle_deg']:+7.2f}°  "
              f"2D={r['rod_2d_mm']:6.2f}mm  3D={r['rod_3d_mm']:6.2f}mm  "
              f"bay={r['bay_offset_mm']:4.1f}mm  stress={r['stress_pct']:.2f}%")


def main():
    args = sys.argv[1:]
    cmd = args[0] if args else "info"

    if cmd == "info":
        cmd_info()
    elif cmd == "health":
        cmd_health()
    elif cmd == "verify":
        r = S.verify_assembly()
        for k, v in r.items():
            print(f"  {k}: {v}")
        n = sum(1 for v in r.values() if v == "PASS")
        print(f"\n{n}/{len(r)} PASS")
    elif cmd == "ik-verify":
        checks = S.verify_ik_standalone()
        for name, ok, detail in checks:
            print(f"  [{'OK' if ok else 'FAIL'}] {name}: {detail}")
        n = sum(1 for _, ok, _ in checks if ok)
        print(f"\n{n}/{len(checks)} PASS")
    elif cmd == "rebuild-bounds":
        r = S.rebuild_bounds()
        print(json.dumps(r, indent=2, ensure_ascii=False))
    elif cmd == "overview":
        print(S.overview())
    elif cmd == "servo":
        print(json.dumps(S.extract_servo_slots(), indent=2, ensure_ascii=False, default=str))
    elif cmd == "section":
        z = float(args[1]) if len(args) > 1 else 46
        for n in ["L_Frame", "R_Frame"]:
            r = S.section_at_z(n, z)
            if r:
                print(f"{n} Z={z}: X={r['x_range']} Y={r['y_range']} ({r['width_x']:.1f}×{r['depth_y']:.1f}mm)")
    elif cmd == "info-part":
        name = args[1] if len(args) > 1 else "Base"
        print(json.dumps(S.part_info(name), indent=2, ensure_ascii=False, default=str))
    elif cmd == "mass":
        part = args[1] if len(args) > 1 else None
        mat = args[2] if len(args) > 2 else S.DEFAULT_MATERIAL
        r = S.mass_properties(part, mat) if part else S.mass_properties_all(mat)
        print(json.dumps(r, indent=2, ensure_ascii=False, default=str))
    elif cmd == "quality":
        part = args[1] if len(args) > 1 else None
        r = S.quality_check(part) if part else S.quality_check_all()
        print(json.dumps(r, indent=2, ensure_ascii=False, default=str))
    elif cmd == "workspace":
        res = int(args[1]) if len(args) > 1 else 10
        print(json.dumps(S.workspace_analysis(res), indent=2, ensure_ascii=False, default=str))
    elif cmd == "clearance":
        print(json.dumps(S.clearance_analysis(), indent=2, ensure_ascii=False, default=str))
    elif cmd == "assembly":
        print(json.dumps(S.assembly_stats(), indent=2, ensure_ascii=False, default=str))
    elif cmd == "ik-forward":
        vals = [float(a) for a in args[1:7]]
        while len(vals) < 6:
            vals.append(0.5)
        print(json.dumps(S.ik_forward(*vals), indent=2, ensure_ascii=False, default=str))
    elif cmd == "collision":
        p1 = args[1] if len(args) > 1 else "Base"
        p2 = args[2] if len(args) > 2 else "L_Frame"
        print(json.dumps(S.collision_check(p1, p2), indent=2, ensure_ascii=False, default=str))
    elif cmd == "rods":
        cmd_rods(args[1:7])
    elif cmd == "render":
        pose_args = args[1:7]
        pose = tuple(int(a) for a in pose_args) if len(pose_args) == 6 else S.TCODE_HOME
        label = args[7] if len(args) > 7 else "home"
        out_dir = args[8] if len(args) > 8 else "output/renders"
        paths = S.render_views(pose=pose, out_dir=out_dir, label=label)
        for p in paths:
            print(f"  {p}")
        print(f"\n{len(paths)} views rendered → {out_dir}")
    elif cmd == "glb":
        pose_args = args[1:7]
        pose = tuple(int(a) for a in pose_args) if len(pose_args) == 6 else S.TCODE_HOME
        out_path = args[7] if len(args) > 7 else "output/ORS6_home_colored.glb"
        p = S.export_glb(pose=pose, out_path=out_path)
        print(f"GLB exported: {p} ({os.path.getsize(p)//1024}KB)")
    elif cmd == "build":
        pose_args = args[1:7]
        pose = tuple(int(a) for a in pose_args) if len(pose_args) == 6 else S.TCODE_HOME
        label = args[7] if len(args) > 7 else "home"
        r = S.build_cadquery(pose=pose, label=label)
        print(f"Built: {r['step_path']} ({r['step_kb']}KB), "
              f"{r['stl_path']} ({r['stl_kb']}KB)")
    elif cmd == "build-fc":
        pose_args = args[1:7]
        pose = tuple(int(a) for a in pose_args) if len(pose_args) == 6 else S.TCODE_HOME
        label = args[7] if len(args) > 7 else "home"
        r = S.build_freecad(pose=pose, label=label)
        print(f"FCStd: {r.get('fcstd_path')} STEP: {r.get('step_path')}")
    elif cmd == "motion":
        engine = args[1] if len(args) > 1 else "cadquery"
        r = S.motion_sequence(engine=engine)
        print(f"\n{r['poses_built']}/{r['poses_total']} poses built in {r['elapsed_s']}s")
        print(f"Report: {r['report_path']}")
    elif cmd == "serve":
        # Launch viewer server
        if len(args) > 1:
            sys.argv = ["server.py", args[1]]
        else:
            sys.argv = ["server.py"]
        from ORS6_Stewart.viewer.server import main as serve_main
        serve_main()
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
