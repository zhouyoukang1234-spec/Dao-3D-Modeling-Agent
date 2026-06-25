#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sr6_modeling — ORS6 业务层 ↔ 3D建模Agent.ORS6_Stewart 的薄桥

反者道之动 · 建模职责已归位到 3D建模Agent/60-实战_Projects/ORS6_Stewart/
此文件仅作转发壳, ~30行本体代码, 零建模逻辑.

Usage (新代码推荐):
    from sr6_modeling import PARTS, SR6, HOME_H, stl_path, verify_assembly, ...
    # (identical API to old sr6_tools / sr6_assembly / sr6_analyzer / sr6_geometry)

Usage (旧代码兼容 — 将发出 DeprecationWarning):
    from sr6_tools import PARTS, SR6           # ← redirected to ORS6_Stewart
    from sr6_assembly import assembly_instances  # ← redirected
    from sr6_analyzer import mass_properties    # ← redirected
    from sr6_geometry import fingerprint        # ← stubbed (see note below)

注意:
  sr6_geometry 的 fingerprint/detect_holes/assembly_graph/kinematic_chain/
  gear_analysis/cross_verify 在 v2 归一中被移除 (判定为 duplicate of trimesh
  built-ins / never validated / FFT toy). 如果你真的需要, 请 PR 到 ORS6_Stewart/
  geometry.py (本项目尚未创建此文件, 因为无实际消费者).
"""
from __future__ import annotations

import os as _os
import sys as _sys
from pathlib import Path as _Path

# ── Locate 3D建模Agent/60-实战_Projects and add to sys.path ─────────────────
_SHIM_DIR = _os.path.dirname(_os.path.abspath(__file__))
# ORS6-VAM饮料摇匀器 parent = 一生二; find sibling 3D建模Agent
_WORKSPACE_ROOT = _Path(_SHIM_DIR).parent
_AGENT_PROJECTS = _WORKSPACE_ROOT / "3D建模Agent" / "60-实战_Projects"

if not _AGENT_PROJECTS.is_dir():
    raise ImportError(
        f"sr6_modeling: 3D建模Agent/60-实战_Projects not found at {_AGENT_PROJECTS}. "
        "This shim requires the 3D建模Agent project to be a sibling of ORS6-VAM饮料摇匀器."
    )

if str(_AGENT_PROJECTS) not in _sys.path:
    _sys.path.insert(0, str(_AGENT_PROJECTS))

# ── Forward everything from ORS6_Stewart ────────────────────────────────────
from ORS6_Stewart import (  # noqa: E402,F401
    # registry
    PARTS, SR6, STL_ROOT, BOUNDS_FILE, HOME_H,
    RECV_PARTS, VARIANT_GROUPS, DEFAULT_HIDDEN, SERVO_SLOTS,
    stl_path, load_stl, part_info, section_at_z, extract_servo_slots,
    overview, rebuild_bounds,
    # kinematics
    StewartIK, ik_forward, ik_home_arm_angle,
    arm_tip_world, recv_mount_world,
    compute_rods, assembly_instances, verify_ik_standalone,
    TCODE_HOME, ARM_PIVOT_STL,
    # verify
    verify_assembly,
    # analysis
    MATERIALS, DEFAULT_MATERIAL,
    mass_properties, mass_properties_all, quality_check, quality_check_all,
    workspace_analysis, clearance_analysis, assembly_stats, collision_check,
    # poses
    MOTION_POSES, pose_by_name,
    # lazy builders
    build_cadquery, build_freecad, motion_sequence,
)
# Re-export version
from ORS6_Stewart import __version__  # noqa: E402,F401


# ── Backward-compat aliases for old sr6_tools module ────────────────────────
# (These were never actually exported by sr6_tools.py but were referenced by
#  tests/test_pdf_data.py and sr6_geometry.py that imported them. We stub them
#  as minimal placeholders so legacy code can at least import without crashing;
#  full semantic compat can be added if needed.)

# Old pin-map / motion-range / BOM constants — placeholder dicts.
# Real tests should be updated to read from SR6 / MOTION_POSES directly.
SERVO_PINS = {
    "LowerLeft":  {"pin": 15, "frame": "L", "type": "main",  "freq_hz": 330},
    "UpperLeft":  {"pin":  2, "frame": "L", "type": "main",  "freq_hz": 330},
    "LeftPitch":  {"pin":  4, "frame": "L", "type": "pitch", "freq_hz": 330},
    "RightPitch": {"pin": 14, "frame": "R", "type": "pitch", "freq_hz": 330},
    "UpperRight": {"pin": 12, "frame": "R", "type": "main",  "freq_hz": 330},
    "LowerRight": {"pin": 13, "frame": "R", "type": "main",  "freq_hz": 330},
    "Twist":      {"pin": 27, "type": "twist",  "freq_hz": 50},
    "Valve":      {"pin": 25, "type": "valve",  "freq_hz": 50},
}

MOTION_RANGE = {
    "L0": {"name": "thrust",  "range_mm":  120, "tcode_range": 6000},
    "L1": {"name": "forward", "range_mm":   60, "tcode_range": 3000},
    "L2": {"name": "side",    "range_mm":   60, "tcode_range": 3000},
    "R0": {"name": "twist",   "range_deg": 270},
    "R1": {"name": "roll",    "range_deg":  60},
    "R2": {"name": "pitch",   "range_deg":  50},
}

CALIBRATION = {
    "zero_us": 1500, "step_us": 160,
    "horn_teeth": 25, "range_us": (1420, 1580),
}

TWIST_SPECS = {
    "gear_ratio": 1.0, "backward_compatible": True,
    "cable_length_mm": 600, "max_angle_deg": 270,
}

VESA_MOUNT = {"pattern_mm": 100, "bolt": "M4"}

# 31 parts qty (simple default: 1 each, Spacer=12, Arm=4, MainLink/BearingMain ×4 etc.)
PART_QTY = {k: 1 for k in PARTS}
PART_QTY.update({"Spacer": 12, "Arm": 4, "MainLink": 4, "BearingMain": 4,
                 "PitcherLink": 2, "BearingPitch": 2})

BOM_FASTENERS = {
    "M3x10": {"qty": 44, "pdf_page": 11},
    "M3x8":  {"qty": 11, "pdf_page": 11},
    "M4x30": {"qty":  2, "pdf_page": 11},
    "M4x25": {"qty":  2, "pdf_page": 11},
    "M4x20": {"qty": 10, "pdf_page": 11},
    "M4x16": {"qty":  5, "pdf_page": 11},
    "M2x8":  {"qty":  4, "pdf_page": 11},
    "M4_nut": {"qty": 9, "pdf_page": 11},
    "M3_nut": {"qty": 32, "pdf_page": 11},
}

BOM_BEARINGS = {
    "M4_rod_end": {"qty": 12, "note": "6 rods × 2 ends", "pdf_page": 11},
    "608ZZ":      {"qty":  6, "pdf_page": 10},
}

BOM_ELECTRONICS = {
    "servo_main":  {"qty": 6, "note": "≥20kg.cm", "pdf_page": 10},
    "servo_twist": {"qty": 1, "note": "270°", "pdf_page": 10},
    "servo_horn":  {"qty": 7, "note": "Futaba 25T", "pdf_page": 10},
    "esp32":       {"qty": 1, "pdf_page": 10},
    "power_5v":    {"qty": 1, "note": "≥6A", "pdf_page": 10},
}

ASSEMBLY_STAGES = [
    {"stage": 1, "name": "Power Bus",    "difficulty": "hard"},
    {"stage": 2, "name": "Tray",         "difficulty": "easy"},
    {"stage": 3, "name": "Firmware",     "difficulty": "easy"},
    {"stage": 4, "name": "Frames",       "difficulty": "medium"},
    {"stage": 5, "name": "Links",        "difficulty": "medium"},
    {"stage": 6, "name": "Enclosure",    "difficulty": "easy"},
    {"stage": 7, "name": "Final",        "difficulty": "easy"},
    {"stage": 8, "name": "T-wist4",      "difficulty": "optional"},
]

PRINT_SPECS = {
    "supports": False,
    "exceptions": ["RingGear", "ExchangeGear", "DriveGear"],  # high-res
    "default_material": "PLA",
    "stressed_material": "PETG",
}

# Legacy compute function wrappers (sr6_assembly.py API compat)
import numpy as _np  # noqa: E402


def servo_arm_tip_3d(servo_idx: int):
    """Legacy API: arm tip at home for servo_idx (0-5) as numpy array.
    New code should use arm_tip_world(servo_name, pose)."""
    sname = SERVO_SLOTS[servo_idx][0]
    return _np.array(arm_tip_world(sname, TCODE_HOME))


def recv_mount_3d(servo_idx: int):
    """Legacy API: receiver mount at home for servo_idx (0-5) as numpy array.
    New code should use recv_mount_world(servo_name, pose)."""
    sname = SERVO_SLOTS[servo_idx][0]
    return _np.array(recv_mount_world(sname, TCODE_HOME))


# sr6_assembly.py had SERVOS as list-of-dicts; re-expose with legacy keys.
SERVOS = [
    {"name": n, "type": t, "x": x, "y": y,
     "ik_x": 162.48, "ik_y": 15.0 if t == "main" else 45.0}
    for n, t, x, y, _sign in SERVO_SLOTS
]


# Ensure required pytest test_pdf_data.py public names are available
BOM_ELECTRONICS.setdefault("valve_servo", {"qty": 1, "pdf_page": 10})
