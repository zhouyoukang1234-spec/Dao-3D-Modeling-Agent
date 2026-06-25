#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anchors/sr6/constants.py — SR6 ground-truth kinematic skeleton.

Every number here is traced to a primary source: the canonical OSR/SR6 firmware
(SR6-Alpha4_ESP32.ino, by TempestMAx) and/or directly MEASURED from the real STL
meshes by uam/perceive.py. Nothing is guessed.

The firmware IK is the contract the physical mechanism must satisfy:

  SetMainServo(x, y):                       # x,y in 1/100 mm
    gamma = atan2(x, y)
    csq   = x^2 + y^2
    beta  = acos((csq - 28125) / (100 * sqrt(csq)))
    out   = 637 * (gamma + beta - pi)        # microseconds from neutral

  SetPitchServo(x, y, z, pitch):
    x += 5500 * sin(0.2618 + pitch_rad)      # 55mm @ 15 deg offset to upper pivot
    y -= 5500 * cos(0.2618 + pitch_rad)
    bsq   = 36250 - (75 + z)^2               # effective rod^2 w/ side offset z
    beta  = acos((csq + 5625 - bsq) / (150 * c))
    out   = 637 * (gamma + beta - pi)

Decoding the law-of-cosines (beta = acos((c^2 + a^2 - b^2)/(2 a c))):
  main:  2a = 100  -> a = 50 (mainArm);  a^2 - b^2 = -28125 -> b^2 = 30625 -> b = 175 (mainRod)
  pitch: 2a = 150  -> a = 75 (pitchArm); a^2 + (-bsq)+... -> b = 175 at z=0 (pitchRod)

MEASURED CONFIRMATION (uam/perceive.py on real STLs):
  Arm.stl            hole-center span = 50.0  mm  == mainArm     OK
  LPitcher/RPitcher  hole-center span = 75.0  mm  == pitchArm    OK
  MainLink_Alpha     hole-center span = 175.0 mm  == mainRod     OK
  Receiver           pivot-pivot span = 55.0  mm  == pitch offset OK
  BearingMainLink    NO rod_pivot pair (only M4 bracket holes)  <-- VARIANT TRAP: a
                     look-alike bracket, NOT the 175mm main rod (see ROOT_CAUSE / affordance)
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field

MM = 1.0  # all lengths in mm

# ── Servo signal law (firmware) ────────────────────────────────────────────
MS_PER_RAD = 637          # #define ms_per_rad — microseconds per radian (standard servo)
SERVO_ZERO_US = 1500      # all *Servo_ZERO
PI = math.pi

# ── Kinematic skeleton (firmware-derived, mesh-confirmed) ──────────────────
MAIN_ARM   = 50.0   # servo horn pivot -> rod-end pivot (Arm.stl: 50.0)
MAIN_ROD   = 175.0  # main link pivot-pivot (MainLink_Alpha.stl: 175.0)
PITCH_ARM  = 75.0   # pitcher arm pivot-pivot (L/RPitcher.stl: 75.0)
PITCH_ROD_EFF = 175.0   # EFFECTIVE rod in planar IK at z=0 (firmware bsq@z0 = 30625 -> 175)
PITCH_ROD_PHYS = 185.0  # PHYSICAL PitcherLink_Alpha hole span (measured 184-186); differs
                        # from eff because the 55mm@15deg upper-pivot offset + side-offset z
                        # fold the 3D link into a shorter planar projection. Do NOT conflate.
PITCH_OFF  = 55.0   # lower->upper receiver pivot offset (Receiver.stl: 55.0)
PITCH_ANG  = 0.2618 # 15 deg, rad — direction of the 55mm offset
BASE_X     = 162.48 # 16248/100 — servo-plane x of receiver lower pivot at home
LOWER_PIV_Y = 15.0  # 1500/100 — home y of main (lower) receiver pivots
UPPER_PIV_Y = 45.0  # 4500/100 — home y of pitch (upper) receiver pivots

# ── Servo layout (hub.html 3D + firmware pin map) ──────────────────────────
# angle = azimuth around device vertical axis (deg); type main|pitch; frame L|R
SERVOS = {
    "LowerLeft":  {"pin": 15, "frame": "L", "type": "main",  "angle_deg": 150, "out": "out1"},
    "UpperLeft":  {"pin":  2, "frame": "L", "type": "main",  "angle_deg": 210, "out": "out2"},
    "LeftPitch":  {"pin":  4, "frame": "L", "type": "pitch", "angle_deg": 120, "out": "out3"},
    "RightPitch": {"pin": 14, "frame": "R", "type": "pitch", "angle_deg":  60, "out": "out4"},
    "UpperRight": {"pin": 12, "frame": "R", "type": "main",  "angle_deg": 330, "out": "out5"},
    "LowerRight": {"pin": 13, "frame": "R", "type": "main",  "angle_deg":  30, "out": "out6"},
    "Twist":      {"pin": 27, "type": "twist", "freq_hz": 50},
    "Valve":      {"pin": 25, "type": "valve", "freq_hz": 50},
}

# ── T-Code axis registration (firmware RegisterAxis) ───────────────────────
TCODE_AXES = {
    "L0": "Up", "L1": "Forward", "L2": "Left",
    "R0": "Twist", "R1": "Roll", "R2": "Pitch",
    "V0": "Vibe1", "V1": "Vibe2", "A0": "Valve", "A1": "Suck",
}
TCODE_HOME = {k: 5000 for k in ("L0", "L1", "L2", "R0", "R1", "R2")}

# ── Motion mapping (firmware loop, SR6 branch) ─────────────────────────────
# map(in,0,9999,-X,X) — units are 1/100 mm or 1/100 deg
MOTION_MAP = {
    "roll":   ("R1", 3000), "pitch": ("R2", 2500), "fwd":   ("L1", 3000),
    "thrust": ("L0", 6000), "side":  ("L2", 3000),
}


def set_main_servo(x100: float, y100: float) -> int:
    """Exact firmware SetMainServo. x100,y100 in 1/100 mm. Returns us-from-neutral."""
    x, y = x100 / 100.0, y100 / 100.0
    gamma = math.atan2(x, y)
    csq = x * x + y * y
    c = math.sqrt(csq)
    beta = math.acos((csq - 28125) / (100 * c))
    return int(MS_PER_RAD * (gamma + beta - PI))


def set_pitch_servo(x100: float, y100: float, z100: float, pitch100: float) -> int:
    """Exact firmware SetPitchServo. pitch100 in 1/100 deg."""
    pitch = pitch100 * 0.0001745
    x = x100 + 5500 * math.sin(0.2618 + pitch)
    y = y100 - 5500 * math.cos(0.2618 + pitch)
    x, y, z = x / 100.0, y / 100.0, z100 / 100.0
    bsq = 36250 - (75 + z) ** 2
    gamma = math.atan2(x, y)
    csq = x * x + y * y
    c = math.sqrt(csq)
    beta = math.acos((csq + 5625 - bsq) / (150 * c))
    return int(MS_PER_RAD * (gamma + beta - PI))


# Self-check at home (matches prior verified IK: out~0 at neutral geometry)
if __name__ == "__main__":
    print("main@home  us:", set_main_servo(16248, 1500))
    print("pitch@home us:", set_pitch_servo(16248, 4500, 0, 0))
