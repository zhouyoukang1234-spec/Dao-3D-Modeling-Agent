#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ORS6_Stewart · verify — 数值验证 (V1-V10)

用 STL bounds + IK 常数做硬断言, 每项通过=PASS, 否则=FAIL.
所有检查无 trimesh 依赖 (读 BOUNDS_FILE json), 可快速运行.
"""
from __future__ import annotations

import json
import os
from typing import Dict

from .parts import PARTS, BOUNDS_FILE, HOME_H, SR6, rebuild_bounds


def _ensure_bounds() -> Dict:
    if not os.path.exists(BOUNDS_FILE):
        rebuild_bounds()
    return json.load(open(BOUNDS_FILE, encoding="utf-8"))


def verify_assembly() -> Dict[str, str]:
    """Run 8 numerical verifications on the STL assembly. Returns {name: PASS/FAIL}."""
    bounds = _ensure_bounds()
    results = []

    # V1: Coordinate system consistency — all part XY centers within ±200mm
    ok = all(abs(b.get("center", [0, 0, 0])[i]) < 200
             for b in bounds.values() if "center" in b
             for i in range(2))
    results.append(("V1_coord_consistency", ok))

    # V2: Frame symmetry — L_Frame and R_Frame X centers are mirror-symmetric
    lf = bounds.get("L_Frame", {}).get("center", [0, 0, 0])[0]
    rf = bounds.get("R_Frame", {}).get("center", [0, 0, 0])[0]
    results.append(("V2_frame_symmetry", abs(lf + rf) < 1.0))

    # V3: Arm Z_min = servoPivotH (46mm)
    arm_z = bounds.get("Arm", {}).get("min", [0, 0, 0])[2]
    results.append(("V3_arm_pivot", abs(arm_z - SR6["servoPivotH"]) < 1.0))

    # V4: Receiver centered at X=0 (Y may have asymmetric lugs)
    recv = bounds.get("Receiver", {}).get("center", [999, 999, 0])
    results.append(("V4_receiver_center", abs(recv[0]) < 1.0))

    # V5: Frame rectangular spacing ≈ 157.4mm (bbox-center separation)
    results.append(("V5_rect_spacing", abs(rf - lf - 157.4) < 1.0))

    # V6: Part count = PARTS registry size (31)
    results.append(("V6_part_count", len(bounds) == len(PARTS)))

    # V7: HOME_H consistency
    results.append(("V7_home_height", abs(HOME_H - 208.48) < 0.01))

    # V8: IK constants (firmware magic numbers)
    rod_sq = SR6["mainRod"] ** 2 - SR6["mainArm"] ** 2
    pitch_sq = SR6["pitchArm"] ** 2 + SR6["mainRod"] ** 2
    results.append(("V8_ik_constants", abs(rod_sq - 28125) < 0.1 and abs(pitch_sq - 36250) < 0.1))

    return {name: ("PASS" if ok else "FAIL") for name, ok in results}


if __name__ == "__main__":
    r = verify_assembly()
    for k, v in r.items():
        print(f"  {k}: {v}")
    total = sum(1 for v in r.values() if v == "PASS")
    print(f"\n{'='*40}\n{total}/{len(r)} PASS")
