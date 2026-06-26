#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart — Stewart 六轴平台一等项目 (3D建模Agent · 60-实战_Projects)

道生一: 31 STL = 唯一几何真相
一生二: firmware IK + trimesh 数值分析
二生三: CadQuery(OCP) + FreeCAD 两大装配路径
三生万物: 15 姿态动画 + 质量/工作空间/间距/碰撞 + Web 查看器

反者道之动 — 此项目原散居于 ORS6-VAM饮料摇匀器, 今归位于万法本源.
ORS6 业务层 (TCode/VaM/Funscript) 通过 `sr6_modeling` shim 反向 import 此处.

对外 API (稳定):
    from ORS6_Stewart import (
        PARTS, SR6, STL_ROOT, HOME_H, RECV_PARTS,
        VARIANT_GROUPS, DEFAULT_HIDDEN,
        stl_path, load_stl, part_info, section_at_z, extract_servo_slots,
        StewartIK, ik_forward, compute_rods, arm_tip_world, recv_mount_world,
        verify_assembly, verify_ik_standalone,
        mass_properties, mass_properties_all, quality_check, quality_check_all,
        workspace_analysis, clearance_analysis, assembly_stats, collision_check,
        build_cadquery, build_freecad, motion_sequence,
        MOTION_POSES,
    )

CLI: python -m ORS6_Stewart {health|verify|build|pose L0 L1 L2 R0 R1 R2|motion|analyze|serve}
"""
from __future__ import annotations

# Ensure 3D建模Agent _paths is bootstrapped (for dao_kinematics import chain)
import sys as _sys
from pathlib import Path as _Path
_DAO_ROOT = next(p for p in _Path(__file__).resolve().parents if (p / "_paths.py").is_file())
if str(_DAO_ROOT) not in _sys.path:
    _sys.path.insert(0, str(_DAO_ROOT))
try:
    import _paths  # noqa: F401
except Exception:
    pass

# ── Re-export the public API ────────────────────────────────────────────────
from .parts import (
    PARTS, SR6, STL_ROOT, BOUNDS_FILE, HOME_H,
    RECV_PARTS, VARIANT_GROUPS, DEFAULT_HIDDEN, SERVO_SLOTS,
    stl_path, load_stl, part_info, section_at_z, extract_servo_slots,
    overview, rebuild_bounds,
)
from .kinematics import (
    StewartIK, ik_forward, ik_home_arm_angle, arm_tip_world, recv_mount_world,
    compute_rods, assembly_instances, verify_ik_standalone,
    TCODE_HOME, ARM_PIVOT_STL,
)
from .verify import verify_assembly
from .analysis import (
    MATERIALS, DEFAULT_MATERIAL,
    mass_properties, mass_properties_all, quality_check, quality_check_all,
    workspace_analysis, clearance_analysis, assembly_stats, collision_check,
)
from .poses import MOTION_POSES, pose_by_name
from .colored import (
    build_colored, render_views, export_glb, PALETTE, VIEWS,
)
from .render import Part, render, hex_rgb

__version__ = "2.0.0"
__all__ = [
    # parts
    "PARTS", "SR6", "STL_ROOT", "BOUNDS_FILE", "HOME_H",
    "RECV_PARTS", "VARIANT_GROUPS", "DEFAULT_HIDDEN", "SERVO_SLOTS",
    "stl_path", "load_stl", "part_info", "section_at_z",
    "extract_servo_slots", "overview", "rebuild_bounds",
    # kinematics
    "StewartIK", "ik_forward", "ik_home_arm_angle",
    "arm_tip_world", "recv_mount_world", "compute_rods",
    "assembly_instances", "verify_ik_standalone",
    "TCODE_HOME", "ARM_PIVOT_STL",
    # verify
    "verify_assembly",
    # analysis
    "MATERIALS", "DEFAULT_MATERIAL",
    "mass_properties", "mass_properties_all",
    "quality_check", "quality_check_all",
    "workspace_analysis", "clearance_analysis",
    "assembly_stats", "collision_check",
    # poses
    "MOTION_POSES", "pose_by_name",
    # colored render
    "build_colored", "render_views", "export_glb", "PALETTE", "VIEWS",
    "Part", "render", "hex_rgb",
]


# Lazy assembly builders (CadQuery/FreeCAD heavy imports on demand)
def build_cadquery(*args, **kwargs):
    """Build Stewart assembly via CadQuery+OCP. Lazy import."""
    from .assembly import build_cadquery as _impl
    return _impl(*args, **kwargs)


def build_freecad(*args, **kwargs):
    """Build Stewart assembly inside FreeCAD. Lazy import."""
    from .assembly import build_freecad as _impl
    return _impl(*args, **kwargs)


def motion_sequence(*args, **kwargs):
    """Run the 15-pose motion sequence. Lazy import."""
    from .assembly import motion_sequence as _impl
    return _impl(*args, **kwargs)
