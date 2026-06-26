# -*- coding: utf-8 -*-
"""真零件位姿装配器 (trimesh, 无 CadQuery) — 1:1 复刻 assembly.build_cadquery 的刚体变换.
把真实 STL 零件按固件运动学(StewartIK)放到任意舵机位姿, 6 根杆=固件 arm_tip→recv_mount,
保证零件本体接触、无悬空. 用于 道.感.校 位姿搜索与三向证据图.
"""
from __future__ import annotations
import os, sys, math
import numpy as np
import trimesh
from trimesh import transformations as TF

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # ORS6_Stewart
sys.path.insert(0, os.path.dirname(_PKG))                            # 60-实战_Projects
from ORS6_Stewart.parts import (PARTS, HOME_H, RECV_PARTS, DEFAULT_HIDDEN,  # noqa: E402
                                 SR6, SERVO_SLOTS, stl_path)
from ORS6_Stewart.kinematics import StewartIK, TCODE_HOME             # noqa: E402

ARM_PIVOT = (67.5, 0.0, 51.5)
FRAME_X = 99.6
_INSTANCED = {"Arm", "L_Pitcher", "R_Pitcher"}
ROD_BODY_D = 6.0
ROD_END_D = 10.0
_PIVH = SR6["servoPivotH"]


def _T(v):
    return TF.translation_matrix(v)


def _R(angle_deg, axis, point=(0, 0, 0)):
    return TF.rotation_matrix(math.radians(angle_deg), axis, point)


def _mirror_x():
    M = np.eye(4); M[0, 0] = -1.0
    return M


def _load(name):
    p = stl_path(name)
    if not os.path.exists(p):
        return None
    try:
        m = trimesh.load(p, process=False)
        if isinstance(m, trimesh.Scene):
            m = m.to_geometry()
        return m
    except Exception:
        return None


def _rod(p1, p2):
    p1 = np.asarray(p1, float); p2 = np.asarray(p2, float)
    L = float(np.linalg.norm(p2 - p1))
    parts = [trimesh.creation.icosphere(subdivisions=1, radius=ROD_END_D / 2).apply_translation(p1),
             trimesh.creation.icosphere(subdivisions=1, radius=ROD_END_D / 2).apply_translation(p2)]
    if L > 0.1:
        parts.append(trimesh.creation.cylinder(radius=ROD_BODY_D / 2, segment=[p1, p2], sections=12))
    return trimesh.util.concatenate(parts)


def _compute_ik(pose):
    ik = StewartIK()
    geom = ik.compute_full_geometry(*pose)
    home = ik.compute_full_geometry(*TCODE_HOME)
    recv = ik.compute_receiver_pose(*pose)
    return geom, home, recv


def build(pose=TCODE_HOME, include_hidden=False, drop_parts=()):
    """Return (combined_trimesh, info). Replicates build_cadquery geometry."""
    geom, home, recv = _compute_ik(pose)
    tx, ty, tz_stl, roll_deg, pitch_deg, twist_deg = recv
    recv_dz = tz_stl - HOME_H
    meshes = []

    # A. static structural parts
    static = [n for n in PARTS if n not in RECV_PARTS and n not in DEFAULT_HIDDEN
              and n not in _INSTANCED and n not in drop_parts]
    for n in static:
        m = _load(n)
        if m is not None:
            meshes.append(m)

    # B. 4 main arms
    arm = _load("Arm")
    if arm is not None:
        for sname, stype, sx, sy, _s in SERVO_SLOTS:
            if stype != "main":
                continue
            is_left = sx < 0
            piv = (-ARM_PIVOT[0] if is_left else ARM_PIVOT[0], ARM_PIVOT[1], ARM_PIVOT[2])
            shaft = (sx, sy, _PIVH)
            delta = math.degrees(geom["arm_angles"][sname] - home["arm_angles"][sname])
            M = _T(shaft) @ _R(delta, [0, 1, 0]) @ _T([-piv[0], -piv[1], -piv[2]])
            if is_left:
                M = M @ _mirror_x()
            meshes.append(arm.copy().apply_transform(M))

    # B2. pitchers
    for pname in ("L_Pitcher", "R_Pitcher"):
        if pname in drop_parts:
            continue
        m = _load(pname)
        if m is None:
            continue
        sname = "LeftPitch" if "L_" in pname else "RightPitch"
        delta = math.degrees(geom["arm_angles"][sname] - home["arm_angles"][sname])
        if abs(delta) > 0.01:
            sx = -FRAME_X if "L_" in pname else FRAME_X
            m = m.copy().apply_transform(_R(delta, [0, 1, 0], point=[sx, 0, _PIVH]))
        meshes.append(m)

    # C. receiver group
    recv_vis = [n for n in RECV_PARTS if n not in DEFAULT_HIDDEN and n not in drop_parts]
    for pname in recv_vis:
        m = _load(pname)
        if m is None:
            continue
        M = _T([tx, ty, HOME_H + recv_dz])
        if abs(roll_deg) > 0.01 or abs(pitch_deg) > 0.01:
            M = M @ _R(pitch_deg, [1, 0, 0]) @ _R(roll_deg, [0, 1, 0])
        if pname in ("RingGear", "ExchangeGear", "DriveGear"):
            tw_sign = 1 if pname == "RingGear" else -1
            M = M @ _R(tw_sign * twist_deg, [0, 0, 1])
        meshes.append(m.copy().apply_transform(M))

    # D. 6 rods
    rod_lens = []
    for sname, stype, _sx, _sy, _s in SERVO_SLOTS:
        tip = geom["arm_tips"][sname]
        mount = geom["recv_mounts"][sname]
        meshes.append(_rod(tip, mount))
        rod_lens.append(float(np.linalg.norm(np.asarray(tip) - np.asarray(mount))))

    combined = trimesh.util.concatenate(meshes)
    info = {"rod_lens": rod_lens, "recv": (tx, ty, tz_stl, roll_deg, pitch_deg, twist_deg),
            "n_meshes": len(meshes)}
    return combined, info


def sample(pose=TCODE_HOME, n=45000, **kw):
    m, info = build(pose, **kw)
    pts, _ = trimesh.sample.sample_surface(m, n)
    return np.asarray(pts, float), info
