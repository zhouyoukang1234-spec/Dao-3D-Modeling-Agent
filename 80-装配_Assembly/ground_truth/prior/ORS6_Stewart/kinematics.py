#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · kinematics — Stewart 平台运动学的唯一实现

反者道之动: 此前分散于
  - ors6_freecad_build.py::StewartIK (866行, 全功能)
  - ors6_cq_build.py (import ↑)
  - sr6_assembly.py::ik_home_arm_angle (仅 home)
  - sr6_config.js (JS 重写)
  - sr6_analyzer.py::ik_forward (近似)

今归一于此. 所有上游 import this.

固件来源: SR6-Alpha4-ESP32.ino (TempestMAx, 2022).
PDF 交叉验证: SR6 Build Instructions.pdf p.26.

核心 API:
  StewartIK.compute_servo_outputs(L0,L1,L2,R0,R1,R2) → {servo_name: μs_delta}
  StewartIK.compute_receiver_pose(...)  → (tx, ty, tz, roll°, pitch°, twist°)
  StewartIK.compute_full_geometry(...)   → {arm_tips, recv_mounts, arm_angles}
  arm_tip_world(servo_name, pose)        → (x, y, z) in STL coords
  recv_mount_world(servo_name, pose)     → (x, y, z) in STL coords
  compute_rods(pose=TCODE_HOME)          → [{servo, arm_angle_deg, rod_3d_mm, ...}]
  ik_home_arm_angle(servo_name)          → radians  (convenience for home pose)
  ik_forward(L0,...,R2)                  → high-level dict (receiver pose + physical)
  verify_ik_standalone()                 → 10-check IK verification
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from .parts import SR6, HOME_H, SERVO_SLOTS

Vec3 = Tuple[float, float, float]
Pose6 = Tuple[int, int, int, int, int, int]

TCODE_HOME: Pose6 = (5000, 5000, 5000, 5000, 5000, 5000)

# 固件 magic 常数:
#   1500      = home μs for main servo target y (thrust=0)
#   16248     = baseH in 1/100mm (162.48mm)
#   4500      = home μs-like target y for pitch servo (at thrust=0)
#   5500      = pitchOff in 1/100mm
#   0.2618    = 15° in radians (pitchAng)
#   28125     = mainRod² - mainArm² = 175² - 50²
#   36250     = pitchArm² + pitchRod² = 75² + 175²
#   5625      = pitchArm² = 75²
_RAD_PER_CDEG = 0.0001745  # 1/100° → radians (approx π/18000)


# ══════════════════════════════════════════════════════════════════════════════
# StewartIK — 主 API (firmware 1:1)
# ══════════════════════════════════════════════════════════════════════════════

class StewartIK:
    """Stewart platform inverse kinematics — firmware-verified 1:1 port."""

    def __init__(self, sr6: Optional[Dict[str, float]] = None):
        self.sr6 = sr6 or SR6

    # ── T-Code → physical units (firmware map()) ──
    def tcode_to_physical(self, L0=5000, L1=5000, L2=5000,
                          R0=5000, R1=5000, R2=5000) -> Dict[str, int]:
        thrust = self._map(L0, 0, 9999, -6000, 6000)   # 1/100mm
        fwd    = self._map(L1, 0, 9999, -3000, 3000)
        side   = self._map(L2, 0, 9999, -3000, 3000)
        twist  = self._map(R0, 0, 9999, 1000, -1000)   # μs offset
        roll   = self._map(R1, 0, 9999, -3000, 3000)
        pitch  = self._map(R2, 0, 9999, -2500, 2500)
        return {"thrust": thrust, "fwd": fwd, "side": side,
                "twist": twist, "roll": roll, "pitch": pitch}

    # ── Core servo output (μs) — firmware 1:1 ──
    def compute_servo_outputs(self, L0=5000, L1=5000, L2=5000,
                              R0=5000, R1=5000, R2=5000) -> Dict[str, float]:
        p = self.tcode_to_physical(L0, L1, L2, R0, R1, R2)
        thrust, fwd, side = p["thrust"], p["fwd"], p["side"]
        roll, pitch = p["roll"], p["pitch"]

        # Main servos (firmware out1-out2, out5-out6)
        out1 = self._set_main_servo(16248 - fwd, 1500 + thrust + roll)  # LowerLeft
        out2 = self._set_main_servo(16248 - fwd, 1500 - thrust - roll)  # UpperLeft
        out5 = self._set_main_servo(16248 - fwd, 1500 - thrust + roll)  # UpperRight
        out6 = self._set_main_servo(16248 - fwd, 1500 + thrust - roll)  # LowerRight

        # Pitch servos (firmware out3, out4)
        out3 = self._set_pitch_servo(16248 - fwd, 4500 - thrust,
                                      side - 1.5 * roll, -pitch)
        out4 = self._set_pitch_servo(16248 - fwd, 4500 - thrust,
                                      -side + 1.5 * roll, -pitch)
        return {
            "LowerLeft": out1, "UpperLeft": out2, "LeftPitch": out3,
            "RightPitch": out4, "UpperRight": out5, "LowerRight": out6,
            "twist_us": p["twist"],
            "physical": p,
        }

    # ── Receiver pose from T-Code ──
    def compute_receiver_pose(self, L0=5000, L1=5000, L2=5000,
                              R0=5000, R1=5000, R2=5000) -> Tuple[float, float, float, float, float, float]:
        """Returns (tx, ty, tz, roll_deg, pitch_deg, twist_deg) in STL coords."""
        p = self.tcode_to_physical(L0, L1, L2, R0, R1, R2)
        tz = HOME_H + p["thrust"] / 100.0  # 1/100mm → mm
        ty = p["fwd"] / 100.0
        tx = p["side"] / 100.0
        roll_deg  = p["roll"] / 100.0      # 1/100° → deg
        pitch_deg = p["pitch"] / 100.0
        twist_deg = -(p["twist"] / self.sr6["msPerRad"]) * (180.0 / math.pi)
        return (tx, ty, tz, roll_deg, pitch_deg, twist_deg)

    # ── Full geometry: arm_tips + recv_mounts + arm_angles ──
    def compute_full_geometry(self, L0=5000, L1=5000, L2=5000,
                              R0=5000, R1=5000, R2=5000) -> Dict[str, Dict[str, Any]]:
        """Compute arm_tips, recv_mounts, arm_angles (radians) for all 6 servos.

        Arm angle is computed DIRECTLY from IK geometry (gamma − beta),
        matching sr6_assembly.py::ik_home_arm_angle.
        """
        p = self.tcode_to_physical(L0, L1, L2, R0, R1, R2)
        thrust, fwd, side = p["thrust"], p["fwd"], p["side"]
        roll, pitch = p["roll"], p["pitch"]
        c = self.sr6

        servo_ik = {
            "LowerLeft":  {"x_fw": 16248 - fwd, "y_fw": 1500 + thrust + roll,
                           "z_fw": 0, "pitch_fw": 0, "type": "main"},
            "UpperLeft":  {"x_fw": 16248 - fwd, "y_fw": 1500 - thrust - roll,
                           "z_fw": 0, "pitch_fw": 0, "type": "main"},
            "LeftPitch":  {"x_fw": 16248 - fwd, "y_fw": 4500 - thrust,
                           "z_fw": side - 1.5 * roll, "pitch_fw": -pitch,
                           "type": "pitch"},
            "RightPitch": {"x_fw": 16248 - fwd, "y_fw": 4500 - thrust,
                           "z_fw": -side + 1.5 * roll, "pitch_fw": -pitch,
                           "type": "pitch"},
            "UpperRight": {"x_fw": 16248 - fwd, "y_fw": 1500 - thrust + roll,
                           "z_fw": 0, "pitch_fw": 0, "type": "main"},
            "LowerRight": {"x_fw": 16248 - fwd, "y_fw": 1500 + thrust - roll,
                           "z_fw": 0, "pitch_fw": 0, "type": "main"},
        }

        arm_tips: Dict[str, Vec3] = {}
        recv_mounts: Dict[str, Vec3] = {}
        arm_angles: Dict[str, float] = {}

        for sname, stype, sx, sy, _sign in SERVO_SLOTS:
            ik = servo_ik[sname]
            sign_x = 1 if sx > 0 else -1
            arm_len = c["pitchArm"] if stype == "pitch" else c["mainArm"]

            if stype == "main":
                x = ik["x_fw"] / 100.0
                y = ik["y_fw"] / 100.0
                gamma = math.atan2(x, y)
                csq = x * x + y * y
                cc = math.sqrt(csq) if csq > 0 else 1e-9
                cos_beta = (csq + arm_len ** 2 - c["mainRod"] ** 2) / (2 * arm_len * cc)
                cos_beta = max(-1.0, min(1.0, cos_beta))
                beta = math.acos(cos_beta)
                arm_angle = gamma - beta
                # 反者道之动: firmware 2D IK 在 arm plane (Y=sy) 内, mount 必共面 → Y=sy (非 0)
                mount = (sx - sign_x * y, sy, c["servoPivotH"] + x)
            else:  # pitch
                x = ik["x_fw"]
                y = ik["y_fw"]
                z = ik["z_fw"]
                pit = ik["pitch_fw"]
                pit_rad = pit * _RAD_PER_CDEG
                x += 5500 * math.sin(0.2618 + pit_rad)
                y -= 5500 * math.cos(0.2618 + pit_rad)
                x /= 100.0
                y /= 100.0
                z /= 100.0
                bsq = 36250 - (75 + z) ** 2  # pitch rod²·dynamic
                gamma = math.atan2(x, y)
                csq = x * x + y * y
                cc = math.sqrt(csq) if csq > 0 else 1e-9
                cos_beta = (csq + 5625 - bsq) / (150 * cc)
                cos_beta = max(-1.0, min(1.0, cos_beta))
                beta = math.acos(cos_beta)
                arm_angle = gamma - beta
                # pitch servo sy=0, 巧合等价, 但语义上仍应为 sy 以保共面
                mount = (sx - sign_x * y, sy, c["servoPivotH"] + x)

            tip_h = arm_len * math.cos(arm_angle)
            tip_v = arm_len * math.sin(arm_angle)
            tip = (sx - sign_x * tip_h, sy, c["servoPivotH"] + tip_v)

            arm_tips[sname] = tip
            recv_mounts[sname] = mount
            arm_angles[sname] = arm_angle

        return {"arm_tips": arm_tips, "recv_mounts": recv_mounts, "arm_angles": arm_angles}

    # ── firmware 1:1 internals ──
    def _set_main_servo(self, x: float, y: float) -> int:
        """SetMainServo — firmware line 837-844"""
        x /= 100.0
        y /= 100.0
        gamma = math.atan2(x, y)
        csq = x * x + y * y
        c = math.sqrt(csq) if csq > 0 else 1e-9
        arg = (csq - 28125) / (100 * c)
        arg = max(-1.0, min(1.0, arg))
        beta = math.acos(arg)
        return int(self.sr6["msPerRad"] * (gamma + beta - math.pi))

    def _set_pitch_servo(self, x: float, y: float, z: float, pitch: float) -> int:
        """SetPitchServo — firmware line 851-862"""
        pitch *= _RAD_PER_CDEG
        x += 5500 * math.sin(0.2618 + pitch)
        y -= 5500 * math.cos(0.2618 + pitch)
        x /= 100.0
        y /= 100.0
        z /= 100.0
        bsq = 36250 - (75 + z) ** 2
        gamma = math.atan2(x, y)
        csq = x * x + y * y
        c = math.sqrt(csq) if csq > 0 else 1e-9
        arg = (csq + 5625 - bsq) / (150 * c)
        arg = max(-1.0, min(1.0, arg))
        beta = math.acos(arg)
        return int(self.sr6["msPerRad"] * (gamma + beta - math.pi))

    @staticmethod
    def _map(x: float, in_min: float, in_max: float,
             out_min: float, out_max: float) -> int:
        """Arduino map()."""
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


# Singleton default IK for convenience API
from typing import Any  # moved here to satisfy forward reference
_DEFAULT_IK = StewartIK()


# ══════════════════════════════════════════════════════════════════════════════
# High-level convenience API
# ══════════════════════════════════════════════════════════════════════════════

def arm_tip_world(servo_name: str,
                  pose: Pose6 = TCODE_HOME,
                  ik: Optional[StewartIK] = None) -> Vec3:
    """Arm tip position in STL coords (mm) for given T-Code pose."""
    ik = ik or _DEFAULT_IK
    return ik.compute_full_geometry(*pose)["arm_tips"][servo_name]


def recv_mount_world(servo_name: str,
                     pose: Pose6 = TCODE_HOME,
                     ik: Optional[StewartIK] = None) -> Vec3:
    """Receiver mount position in STL coords (mm) for given T-Code pose."""
    ik = ik or _DEFAULT_IK
    return ik.compute_full_geometry(*pose)["recv_mounts"][servo_name]


def ik_home_arm_angle(servo_name: str, ik: Optional[StewartIK] = None) -> float:
    """Physical arm angle (radians) at home pose. Negative = below horizontal."""
    ik = ik or _DEFAULT_IK
    return ik.compute_full_geometry(*TCODE_HOME)["arm_angles"][servo_name]


# Arm STL hub center (trimesh-verified spline bore center, STL coords)
ARM_PIVOT_STL: Vec3 = (67.5, 0.0, 51.5)


def assembly_instances(pose: Pose6 = TCODE_HOME,
                       ik: Optional[StewartIK] = None) -> Dict[str, Any]:
    """Compute assembly instance placements (for viewer multi-instance rendering).

    Returns dict with:
      arms:         4 main arm instances (servo, shaft, translate, mirror, angle)
      links:        6 rod connections (arm_tip → recv_mount, for parametric rods)
      pitcher_arms: 2 pitcher arm positions (L/R, separate STLs)
      home_h:       208.48mm
      arm_pivot_stl: (67.5, 0, 51.5)
    """
    ik = ik or _DEFAULT_IK
    geom = ik.compute_full_geometry(*pose)
    home_geom = ik.compute_full_geometry(*TCODE_HOME)

    arms: List[Dict[str, Any]] = []
    pitcher_arms: List[Dict[str, Any]] = []
    for idx, (sname, stype, sx, sy, _sign) in enumerate(SERVO_SLOTS):
        shaft = (sx, sy, SR6["servoPivotH"])
        angle_deg = round(math.degrees(geom["arm_angles"][sname]), 2)
        if stype == "main":
            mirror_x = sx < 0
            piv = ((-ARM_PIVOT_STL[0] if mirror_x else ARM_PIVOT_STL[0]),
                   ARM_PIVOT_STL[1], ARM_PIVOT_STL[2])
            translate = (shaft[0] - piv[0], shaft[1] - piv[1], shaft[2] - piv[2])
            arms.append({
                "servo": sname, "servo_idx": idx,
                "shaft": list(shaft),
                "translate": [round(v, 2) for v in translate],
                "mirror_x": mirror_x,
                "arm_angle_deg": angle_deg,
                "angle_delta_deg": round(math.degrees(
                    geom["arm_angles"][sname] - home_geom["arm_angles"][sname]), 2),
            })
        else:
            pitcher_arms.append({
                "servo": sname,
                "stl": "L_Pitcher" if sx < 0 else "R_Pitcher",
                "shaft": list(shaft),
                "arm_angle_deg": angle_deg,
            })

    links: List[Dict[str, Any]] = []
    rods = compute_rods(pose, ik)
    for rod in rods:
        links.append({
            "servo": rod["servo"],
            "type": rod["type"],
            "arm_tip": rod["tip"],
            "recv_mount": rod["mount"],
            "rod_3d_mm": rod["rod_3d_mm"],
            "link_stl": "MainLink" if rod["type"] == "main" else "PitcherLink",
        })

    return {
        "arms": arms,
        "links": links,
        "pitcher_arms": pitcher_arms,
        "home_h": HOME_H,
        "arm_pivot_stl": list(ARM_PIVOT_STL),
    }


def compute_rods(pose: Pose6 = TCODE_HOME,
                 ik: Optional[StewartIK] = None) -> List[Dict[str, Any]]:
    """Compute the 6 rod geometries for a given pose."""
    ik = ik or _DEFAULT_IK
    geom = ik.compute_full_geometry(*pose)
    ROD_NOM = SR6["mainRod"]
    results: List[Dict[str, Any]] = []
    for sname, stype, _sx, _sy, _sign in SERVO_SLOTS:
        tip = geom["arm_tips"][sname]
        mount = geom["recv_mounts"][sname]
        dx = mount[0] - tip[0]
        dy = mount[1] - tip[1]
        dz = mount[2] - tip[2]
        rod_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        rod_2d = math.sqrt(dx * dx + dz * dz)  # in-servo-plane
        bay_off = abs(dy)
        # 反者道之动: mount Y=sy 后, 所有 rod 在 arm plane 内 3D=2D=175mm (firmware)
        nominal = 175.0
        results.append({
            "servo": sname,
            "type": stype,
            "arm_angle_deg": round(math.degrees(geom["arm_angles"][sname]), 2),
            "tip": [round(v, 2) for v in tip],
            "mount": [round(v, 2) for v in mount],
            "rod_2d_mm": round(rod_2d, 2),
            "bay_offset_mm": round(bay_off, 2),
            "rod_3d_mm": round(rod_3d, 2),
            "rod_nominal_mm": nominal,
            "stress_pct": round(abs(rod_3d - nominal) / max(nominal, 1.0) * 100, 2),
        })
    return results


def ik_forward(L0_pos: float = 0.5, L1_pos: float = 0.5, L2_pos: float = 0.5,
               R0_pos: float = 0.5, R1_pos: float = 0.5, R2_pos: float = 0.5,
               ik: Optional[StewartIK] = None) -> Dict[str, Any]:
    """SR6 "forward" kinematics (normalized 0-1 → pose + physical).

    (Same as compute_receiver_pose but with 0-1 normalized input.)"""
    ik = ik or _DEFAULT_IK

    def _to_tcode(p: float) -> int:
        return max(0, min(9999, int(round(p * 9999))))

    L0, L1, L2 = _to_tcode(L0_pos), _to_tcode(L1_pos), _to_tcode(L2_pos)
    R0, R1, R2 = _to_tcode(R0_pos), _to_tcode(R1_pos), _to_tcode(R2_pos)

    tx, ty, tz, roll_deg, pitch_deg, twist_deg = ik.compute_receiver_pose(
        L0, L1, L2, R0, R1, R2)
    phys = ik.tcode_to_physical(L0, L1, L2, R0, R1, R2)
    return {
        "tcode": {"L0": L0, "L1": L1, "L2": L2, "R0": R0, "R1": R1, "R2": R2},
        "receiver_position_mm": {"x": round(tx, 2), "y": round(ty, 2), "z": round(tz, 2)},
        "orientation_deg": {
            "twist": round(twist_deg, 2),
            "roll": round(roll_deg, 2),
            "pitch": round(pitch_deg, 2),
        },
        "physical": {
            "thrust_mm": round(phys["thrust"] / 100.0, 2),
            "surge_mm": round(phys["fwd"] / 100.0, 2),
            "sway_mm": round(phys["side"] / 100.0, 2),
        },
    }


# ══════════════════════════════════════════════════════════════════════════════
# IK standalone verification (10 checks)
# ══════════════════════════════════════════════════════════════════════════════

def verify_ik_standalone(ik: Optional[StewartIK] = None) -> List[Tuple[str, bool, str]]:
    """Verify IK engine correctness without FreeCAD. Returns list of (name, ok, detail)."""
    ik = ik or _DEFAULT_IK
    checks: List[Tuple[str, bool, str]] = []

    # V1: Home — main servos ≈ 0μs, pitch servos have geometry offset
    out = ik.compute_servo_outputs(*TCODE_HOME)
    main_max = max(abs(out[k]) for k in ["LowerLeft", "UpperLeft", "UpperRight", "LowerRight"])
    pitch_max = max(abs(out[k]) for k in ["LeftPitch", "RightPitch"])
    ok = main_max < 5 and pitch_max < 150
    checks.append(("V1_home_zero", ok, f"main_max={main_max}μs pitch_max={pitch_max}μs"))

    # V2: Symmetry — left/right mirror magnitude
    for key_l, key_r in [("LowerLeft", "LowerRight"), ("UpperLeft", "UpperRight")]:
        diff = abs(abs(out[key_l]) - abs(out[key_r]))
        checks.append((f"V2_symmetry_{key_l[:5]}", diff < 5, f"diff={diff}μs"))

    # V3: Thrust up activates all main servos
    out_up = ik.compute_servo_outputs(9999, 5000, 5000, 5000, 5000, 5000)
    thrust_active = all(abs(out_up[k]) > 50
                        for k in ["LowerLeft", "UpperLeft", "UpperRight", "LowerRight"])
    checks.append(("V3_thrust_response", thrust_active, "all mains active"))

    # V4: Pure roll breaks symmetry
    out_roll = ik.compute_servo_outputs(5000, 5000, 5000, 5000, 9999, 5000)
    roll_asymm = abs(abs(out_roll["LowerLeft"]) - abs(out_roll["LowerRight"])) > 1
    checks.append(("V4_roll_asymmetry", roll_asymm, "left≠right"))

    # V5: Home Z = HOME_H (208.48mm)
    pose = ik.compute_receiver_pose(*TCODE_HOME)
    checks.append(("V5_home_z_208.48", abs(pose[2] - HOME_H) < 0.1, f"z={pose[2]:.2f}"))

    # V6: Thrust range = 120mm
    pose_up = ik.compute_receiver_pose(9999, 5000, 5000, 5000, 5000, 5000)
    pose_dn = ik.compute_receiver_pose(0, 5000, 5000, 5000, 5000, 5000)
    rng = pose_up[2] - pose_dn[2]
    checks.append(("V6_thrust_120mm", abs(rng - 120.0) < 1.0, f"range={rng:.1f}mm"))

    # V7: Forward range = 60mm
    pose_f = ik.compute_receiver_pose(5000, 9999, 5000, 5000, 5000, 5000)
    pose_b = ik.compute_receiver_pose(5000, 0, 5000, 5000, 5000, 5000)
    rng = pose_f[1] - pose_b[1]
    checks.append(("V7_fwd_60mm", abs(rng - 60.0) < 1.0, f"range={rng:.1f}mm"))

    # V8: Side range = 60mm
    pose_r = ik.compute_receiver_pose(5000, 5000, 9999, 5000, 5000, 5000)
    pose_l = ik.compute_receiver_pose(5000, 5000, 0, 5000, 5000, 5000)
    rng = pose_r[0] - pose_l[0]
    checks.append(("V8_side_60mm", abs(rng - 60.0) < 1.0, f"range={rng:.1f}mm"))

    # V9: firmware constant 28125 = mainRod² − mainArm²
    rod_sq = SR6["mainRod"] ** 2 - SR6["mainArm"] ** 2
    checks.append(("V9_28125_constant", abs(rod_sq - 28125) < 0.1, f"rod²-arm²={rod_sq}"))

    # V10: firmware constant 36250 = pitchArm² + pitchRod²
    pitch_bsq = SR6["pitchArm"] ** 2 + SR6["mainRod"] ** 2  # pitchRod = mainRod
    checks.append(("V10_36250_constant", abs(pitch_bsq - 36250) < 0.1,
                   f"pitchArm²+rod²={pitch_bsq}"))

    return checks


if __name__ == "__main__":
    import json
    import sys

    cmd = sys.argv[1] if len(sys.argv) > 1 else "verify"
    if cmd == "verify":
        checks = verify_ik_standalone()
        passed = 0
        for name, ok, detail in checks:
            status = "✅" if ok else "❌"
            print(f"  {status} {name}: {detail}")
            if ok:
                passed += 1
        print(f"\n{passed}/{len(checks)} PASS — Grade "
              f"{'SSS' if passed == len(checks) else 'S' if passed >= len(checks)-1 else 'A'}")
    elif cmd == "home":
        print(json.dumps(compute_rods(TCODE_HOME), indent=2, ensure_ascii=False))
    elif cmd == "pose":
        args = sys.argv[2:8]
        pose = tuple(int(a) for a in args) if len(args) == 6 else TCODE_HOME
        print(json.dumps(compute_rods(pose), indent=2, ensure_ascii=False))
    elif cmd == "forward":
        args = [float(a) for a in sys.argv[2:8]]
        while len(args) < 6:
            args.append(0.5)
        print(json.dumps(ik_forward(*args), indent=2, ensure_ascii=False))
    else:
        print(__doc__)
