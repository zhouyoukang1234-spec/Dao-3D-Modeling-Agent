#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · parts — 零件注册表 · 唯一数据真相

31 STL (30 TempestMAx Beta + 1 ESP32_Mount 自制) + 6 舵机槽位 +
IK 常数 + 变体组 + 隐藏规则 + HOME_H.

此模块零外部依赖 (除 os/json), 可在任何环境 import.
trimesh 延迟加载 (load_stl / part_info / section_at_z 首次调用时).

STL_ROOT 解析顺序:
  1. 环境变量 ORS6_STL_ROOT (若存在且目录存在)
  2. ORS6-VAM饮料摇匀器/SR6资料/STLs (若存在, 默认位置)
  3. 60-实战_Projects/ORS6_Stewart/STLs (若存在, 本项目 symlink 位置)
  4. raise RuntimeError
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ══════════════════════════════════════════════════════════════════════════════
# STL_ROOT 解析 — 道法自然: 先看环境, 再看近邻, 再看本地
# ══════════════════════════════════════════════════════════════════════════════

def _locate_stl_root() -> str:
    """按优先级定位 STL 根目录."""
    # 1. 环境变量
    env = os.environ.get("ORS6_STL_ROOT")
    if env and os.path.isdir(env):
        return env

    # 2. 默认: 3D建模Agent/../ORS6-VAM饮料摇匀器/SR6资料.../STLs
    #   parents[0]=60-实战_Projects, [1]=3D建模Agent, [2]=一生二
    workspace_root = Path(SCRIPT_DIR).parents[2]  # 一生二
    ors6_default = (workspace_root / "ORS6-VAM饮料摇匀器"
                    / "SR6资料，签收后提供解压密码"
                    / "SR6 完整资料进阶版本 签收后提供解压密码" / "STLs")
    if ors6_default.is_dir():
        return str(ors6_default)

    # 3. 本地 symlink
    local = Path(SCRIPT_DIR) / "STLs"
    if local.is_dir():
        return str(local)

    # 道法自然: 未挂载 STL 时不在 import 期硬崩 — 退回默认路径并告警，
    # 让"不读 STL"的离线流程(如 native_assembly 物理体检 / CI)仍可 import。
    # 真正去 load_stl 的调用会自然抛 FileNotFoundError, 信息同样清晰。
    import warnings
    warnings.warn(
        "ORS6 STL root not found; using placeholder. Set ORS6_STL_ROOT to load "
        f"geometry. (looked under {ors6_default} and {local})",
        RuntimeWarning,
        stacklevel=2,
    )
    return str(ors6_default)


STL_ROOT: str = _locate_stl_root()
BOUNDS_FILE: str = os.path.join(SCRIPT_DIR, "_stl_bounds.json")


# ══════════════════════════════════════════════════════════════════════════════
# SR6 IK 常数 — firmware-verified (ESP32 .ino + PDF p.26 交叉验证)
# ══════════════════════════════════════════════════════════════════════════════

SR6: Dict[str, float] = dict(
    baseH=162.48,      # firmware 16248/100
    mainArm=50.0,      # firmware 2a=100 → a=50mm
    mainRod=175.0,     # √30625 = √(28125+2500), PDF p.26 "175mm apart"
    pitchArm=75.0,     # firmware 2a=150 → a=75mm
    pitchOff=55.0,     # firmware 5500/100
    pitchAng=15.0,     # firmware 0.2618 rad
    msPerRad=637,      # standard servo μs/rad
    servoPivotH=46.0,  # STL-verified: Arm Z_min=46mm
)

# HOME_H: 接收器归位时 Z 坐标 (receiver bottom plane)
HOME_H: float = SR6["servoPivotH"] + SR6["baseH"]  # 208.48mm


# ══════════════════════════════════════════════════════════════════════════════
# PARTS Registry — 31 STL 零件 (name → (subfolder, filename, color_hex, group))
# ══════════════════════════════════════════════════════════════════════════════

PARTS: Dict[str, Tuple[str, str, int, str]] = {
    # ─── Core structural (11) ───
    "Base":         ("SR6测试版零件", "SR6 底座 Beta1A.stl",          0xbb1a1a, "core"),
    "L_Frame":      ("SR6测试版零件", "SR6 L形框架 Beta1.stl",        0xcc2020, "core"),
    "R_Frame":      ("SR6测试版零件", "SR6 R-Frame Beta1.stl",       0xcc2020, "core"),
    "L_Pitcher":    ("SR6测试版零件", "SR6 L-投手 Beta1.stl",         0xcc2020, "core"),
    "R_Pitcher":    ("SR6测试版零件", "SR6 R-投手 Beta1.stl",         0xcc2020, "core"),
    "Arm":          ("SR6测试版零件", "SR6 臂 Beta1.stl",             0xe0ddd8, "core"),
    "Receiver":     ("SR6测试版零件", "SR6 Receiver Beta1.stl",      0x2a3a6a, "core"),
    "Lid":          ("SR6测试版零件", "SR6 盖子 Beta1.stl",           0xcc2020, "core"),
    "WindowLid":    ("SR6备用零件",   "SR6 Window Lid Beta1.stl",    0xcc2020, "core"),
    "PowerBus":     ("SR6测试版零件", "SR6 电源总线支架 Beta1.stl",    0xe0ddd8, "core"),
    "Spacer":       ("SR6测试版零件", "SR6 4x3mm 垫片 Beta1.stl",    0xaaaaaa, "core"),

    # ─── Linkage (4) ───
    "BearingMain":  ("SR6测试版零件", "SR6 轴承主连杆 Beta1.stl",      0xf0ede8, "linkage"),
    "BearingPitch": ("SR6测试版零件", "SR6 轴承投手链接 Beta1.stl",    0xf0ede8, "linkage"),
    "MainLink":     ("SR6备用零件",   "SR6 Main Link Alpha1.stl",    0xf0ede8, "linkage"),
    "PitcherLink":  ("SR6备用零件",   "SR6 Pitcher Link Alpha1.stl", 0xf0ede8, "linkage"),

    # ─── Shield (3) ───
    "Shield":       ("SR6测试版防护罩", "SR6 Shield 40mm Fan.stl",   0xcc2020, "shield"),
    "Shield_OLED":  ("SR6测试版防护罩", "SR6 Shield 40mm Fan + OLED Display.stl", 0xcc2020, "shield"),
    "Shield_Alt":   ("SR6测试版防护罩", "SR6 Shield 40mm Fan + OLED Display(alternate dimensions).stl", 0xcc2020, "shield"),

    # ─── Tray (3) ───
    "Tray":         ("SR6测试版托盘",   "SR6 Tray Standard Beta1.stl", 0xe0ddd8, "tray"),
    "Tray_ScrewJack": ("SR6测试版托盘", "SR6 Tray Screw Jack Beta1.stl", 0xe0ddd8, "tray"),
    "Tray_XT60":    ("SR6测试版托盘",   "SR6 Tray XT60E1-M Beta1.stl",   0xe0ddd8, "tray"),

    # ─── T-wist4 (9) ───
    "Twist_Base":   ("SR6 T-wist4", "T-wist4 SR6 Base Beta1.stl",  0xcc2020, "twist"),
    "Twist_Body":   ("SR6 T-wist4", "T-wist4 SR6 Body Beta1.stl",  0xcc2020, "twist"),
    "Twist_Lid":    ("SR6 T-wist4", "T-wist4 Lid Beta1.stl",       0xe0ddd8, "twist"),
    "RingGear":     ("SR6 T-wist4", "T-wist Clip Ring Gear Beta4.stl", 0x444444, "twist"),
    "ExchangeGear": ("SR6 T-wist4", "T-wist Exchange Gear Beta1.stl",  0x444444, "twist"),
    "DriveGear":    ("SR6 T-wist4", "T-wist4 Drive Beta1.stl",         0x444444, "twist"),
    "GrommetLink":  ("SR6 T-wist4", "SR6 Grommet Pitcher Link Beta1.stl", 0xf0ede8, "twist"),
    "L_AngleLink":  ("SR6 T-wist4", "SR6 L-Pitcher Angle Link Beta1.stl", 0xf0ede8, "twist"),
    "R_AngleLink":  ("SR6 T-wist4", "SR6 R-Pitcher Angle Link Beta1.stl", 0xf0ede8, "twist"),

    # ─── Custom (1) ───
    # Custom parts live in ORS6-VAM饮料摇匀器/custom_parts (sibling of STL_ROOT's grandparent)
    "ESP32_Mount":  ("__custom__", "ESP32_Mount.stl",              0x2288aa, "custom"),
}


# ══════════════════════════════════════════════════════════════════════════════
# Variant Groups & Default Hidden
# ══════════════════════════════════════════════════════════════════════════════

VARIANT_GROUPS: Dict[str, Dict[str, Any]] = {
    "lid":        {"parts": ["Lid", "WindowLid"],                         "default": "Lid"},
    "shield":     {"parts": ["Shield", "Shield_OLED", "Shield_Alt"],      "default": "Shield"},
    "tray":       {"parts": ["Tray", "Tray_ScrewJack", "Tray_XT60"],      "default": "Tray"},
    "main_link":  {"parts": ["MainLink", "BearingMain"],                  "default": "BearingMain"},
    "pitch_link": {"parts": ["PitcherLink", "BearingPitch", "GrommetLink"], "default": "BearingPitch"},
}

# Hidden by default in viewer: non-default variants + origin-positioned links + tiny parts
DEFAULT_HIDDEN: Set[str] = set()
for _vg in VARIANT_GROUPS.values():
    for _p in _vg["parts"]:
        if _p != _vg["default"]:
            DEFAULT_HIDDEN.add(_p)
DEFAULT_HIDDEN.update({"MainLink", "BearingMain", "PitcherLink", "BearingPitch",
                       "GrommetLink", "L_AngleLink", "R_AngleLink"})
DEFAULT_HIDDEN.update({"Spacer", "ESP32_Mount"})

# Parts that are elevated to HOME_H (receiver + twist module)
RECV_PARTS: Set[str] = {"Receiver", "Twist_Base", "Twist_Body", "Twist_Lid",
                        "RingGear", "ExchangeGear", "DriveGear",
                        "GrommetLink", "L_AngleLink", "R_AngleLink"}

# 6 servo slots (STL coords, Z-up, mm) — 物理布局 · 固件 out1-out6 顺序
# (name, type, x_mm, y_mm, sign_to_receiver)
SERVO_SLOTS: List[Tuple[str, str, float, float, int]] = [
    ("LowerLeft",  "main",  -99.6, +37.0, -1),
    ("UpperLeft",  "main",  -99.6, -37.0, -1),
    ("LeftPitch",  "pitch", -99.6,   0.0, -1),
    ("RightPitch", "pitch", +99.6,   0.0, +1),
    ("UpperRight", "main",  +99.6, -37.0, +1),
    ("LowerRight", "main",  +99.6, +37.0, +1),
]


# ══════════════════════════════════════════════════════════════════════════════
# STL 路径解析 + trimesh 延迟加载
# ══════════════════════════════════════════════════════════════════════════════

def stl_path(name: str) -> str:
    """Absolute path to a named STL part."""
    if name not in PARTS:
        raise KeyError(f"Unknown part: {name!r}. Valid: {sorted(PARTS)}")
    sub, fn, _, _ = PARTS[name]
    if sub == "__custom__":
        # ESP32_Mount lives in ORS6-VAM饮料摇匀器/custom_parts (sibling of 3D建模Agent)
        workspace_root = Path(SCRIPT_DIR).parents[2]  # 一生二
        p = workspace_root / "ORS6-VAM饮料摇匀器" / "custom_parts" / fn
        return str(p)
    return os.path.join(STL_ROOT, sub, fn)


def load_stl(name: str):
    """Load named STL as trimesh mesh (lazy import)."""
    import trimesh
    return trimesh.load(stl_path(name))


def part_info(name: str) -> Dict[str, Any]:
    """Numerical info (vertices/faces/bbox/volume/watertight) for a part."""
    m = load_stl(name)
    b = m.bounds
    return {
        "name": name,
        "vertices": len(m.vertices),
        "faces": len(m.faces),
        "center_mm": [(b[0][i] + b[1][i]) / 2 for i in range(3)],
        "size_mm": [float(b[1][i] - b[0][i]) for i in range(3)],
        "min_mm": [float(b[0][i]) for i in range(3)],
        "max_mm": [float(b[1][i]) for i in range(3)],
        "watertight": bool(m.is_watertight),
        "volume_cm3": round(float(m.volume) / 1000, 2) if m.is_watertight else None,
    }


def section_at_z(name: str, z_mm: float) -> Optional[Dict[str, Any]]:
    """Cross-section a part at given Z height. Returns bounds dict or None."""
    m = load_stl(name)
    sec = m.section(plane_origin=[0, 0, z_mm], plane_normal=[0, 0, 1])
    if sec is None:
        return None
    b = sec.bounds
    return {
        "z_mm": z_mm,
        "x_range": [float(b[0][0]), float(b[1][0])],
        "y_range": [float(b[0][1]), float(b[1][1])],
        "width_x": float(b[1][0] - b[0][0]),
        "depth_y": float(b[1][1] - b[0][1]),
        "vertices": len(sec.vertices),
    }


def extract_servo_slots() -> Dict[str, Any]:
    """Extract real servo slot positions from frame STLs at Z=servoPivotH.
    Used to validate the 199.2mm rectangular frame layout."""
    results: Dict[str, Any] = {}
    pivot_z = SR6["servoPivotH"]
    for name in ["L_Frame", "R_Frame"]:
        sec = section_at_z(name, pivot_z)
        if sec:
            cx = sum(sec["x_range"]) / 2
            cy = sum(sec["y_range"]) / 2
            results[name] = {
                "center_x": round(cx, 1),
                "center_y": round(cy, 1),
                "x_range": [round(v, 1) for v in sec["x_range"]],
                "y_range": [round(v, 1) for v in sec["y_range"]],
            }
    # Pitcher arm positions (reference points, not servo slots)
    for name in ["L_Pitcher", "R_Pitcher"]:
        info = part_info(name)
        results[name] = {
            "center_x": round(info["center_mm"][0], 1),
            "center_y": round(info["center_mm"][1], 1),
        }
    arm_info = part_info("Arm")
    results["Arm_position"] = {
        "center_x": round(arm_info["center_mm"][0], 1),
        "center_y": round(arm_info["center_mm"][1], 1),
        "center_z": round(arm_info["center_mm"][2], 1),
    }
    if "L_Frame" in results and "R_Frame" in results:
        gap = results["R_Frame"]["center_x"] - results["L_Frame"]["center_x"]
        results["frame_spacing_mm"] = round(gap, 1)
        results["layout"] = "rectangular"
        results["note"] = f"Frame spacing={gap:.1f}mm. NOT circular BASE_R=90mm."
    return results


# ══════════════════════════════════════════════════════════════════════════════
# Bounds cache — rebuild from STL, persist to _stl_bounds.json
# ══════════════════════════════════════════════════════════════════════════════

def rebuild_bounds(output: Optional[str] = None) -> Dict[str, Any]:
    """Compute bounds for all 31 STL parts and write to BOUNDS_FILE."""
    import trimesh  # noqa: F401 (used via load_stl)
    output = output or BOUNDS_FILE
    bounds: Dict[str, Dict[str, Any]] = {}
    for name in PARTS:
        try:
            info = part_info(name)
            bounds[name] = {
                "center": info["center_mm"],
                "size": info["size_mm"],
                "min": info["min_mm"],
                "max": info["max_mm"],
                "vertices": info["vertices"],
                "faces": info["faces"],
                "watertight": info["watertight"],
                "volume_cm3": info["volume_cm3"],
            }
        except Exception as e:
            bounds[name] = {"error": str(e)}
    with open(output, "w", encoding="utf-8") as f:
        json.dump(bounds, f, indent=2, ensure_ascii=False)
    return {"output": output, "count": len(bounds), "errors": [k for k, v in bounds.items() if "error" in v]}


def overview() -> str:
    """Compact overview of all parts (from BOUNDS_FILE)."""
    if not os.path.exists(BOUNDS_FILE):
        rebuild_bounds()
    bounds = json.load(open(BOUNDS_FILE, encoding="utf-8"))
    lines: List[str] = []
    for name, b in sorted(bounds.items(),
                          key=lambda x: x[1].get("center", [0, 0, 0])[2]):
        if "error" in b:
            lines.append(f"  {name[:40]:40s} ERROR: {b['error']}")
            continue
        c = b["center"]
        s = b.get("size", [0, 0, 0])
        lines.append(f"  {name[:40]:40s} center=({c[0]:6.1f},{c[1]:6.1f},{c[2]:6.1f}) "
                     f"size=({s[0]:5.1f}x{s[1]:5.1f}x{s[2]:5.1f})")
    return "\n".join(lines)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "info"
    if cmd == "info":
        print(f"STL_ROOT: {STL_ROOT}")
        print(f"BOUNDS_FILE: {BOUNDS_FILE}")
        print(f"PARTS: {len(PARTS)}")
        print(f"RECV_PARTS: {len(RECV_PARTS)}")
        print(f"DEFAULT_HIDDEN: {len(DEFAULT_HIDDEN)}")
        print(f"HOME_H: {HOME_H}mm")
        missing = [n for n in PARTS if not os.path.exists(stl_path(n))]
        print(f"Missing STL files: {len(missing)} {missing if missing else ''}")
    elif cmd == "rebuild":
        r = rebuild_bounds()
        print(json.dumps(r, indent=2, ensure_ascii=False))
    elif cmd == "overview":
        print(overview())
    elif cmd == "servo":
        print(json.dumps(extract_servo_slots(), indent=2, ensure_ascii=False))
    elif cmd == "section":
        z = float(sys.argv[2]) if len(sys.argv) > 2 else 46
        for n in ["L_Frame", "R_Frame"]:
            r = section_at_z(n, z)
            if r:
                print(f"{n} Z={z}: X={r['x_range']} Y={r['y_range']}")
    else:
        print(__doc__)
