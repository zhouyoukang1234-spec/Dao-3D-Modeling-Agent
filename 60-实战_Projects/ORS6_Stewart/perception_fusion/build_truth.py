# -*- coding: utf-8 -*-
"""真值锚点装配器 — truth-anchored Stewart assembly (trimesh, no cadquery).

反者道之动: 放弃固件 2D-IK 的"悬空杆端", 改以真实 STL 测得的球铰锚点 + 175mm 杆
长公理 (firmware) 反推装配. 三向归一:
  - 真实 STL 零件 (方向二) 提供锚点与零件本体
  - Tripo 视觉网格 (方向一) 提供接收环半径 (r=59.3) 与展开位姿, 与 STL r=59.98 互证
  - 道.感.校 (方向三) 对照实物照片打分

核心发现: 4 main 臂以"球铰朝内汇聚"取向 + 接收环 r=59.98 时, 接收环置于 Z=221mm
则 6 main 杆恰为 175.0mm (firmware 公理), 几何自洽 — 无需任何悬空点.

输出: output/ORS6_truth_<label>.stl  +  几何报告.
"""
from __future__ import annotations
import os, sys, json, math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJ = os.path.normpath(os.path.join(_HERE, ".."))
if _PROJ not in sys.path:
    sys.path.insert(0, _PROJ)

import trimesh
import parts as P  # noqa: E402

# ── 真实 STL 锚点 (HALLUCINATION_MAP §2, trimesh 圆柱孔 axis 实测) ──────────────
ARM_HORN = np.array([67.5, 0.0, 51.0])     # 舵机花键孔中心 (Arm 局部, axis‖Z)
ARM_BALL = np.array([67.5, 50.0, 51.0])    # M4 鱼眼球铰 (horn→ball=50=mainArm ✓)
RECV_MAIN_R = 59.98                          # 接收环 main mount 半径 (Y=0, Z=0)
RECV_PITCH = np.array([61.0, -14.24, 53.13]) # 接收环 pitch mount (局部)
PITCH_HORN = np.array([-7.5, 30.0, 51.75])   # L_Pitcher 花键 (R 镜像 X)
PITCH_BALL = np.array([-39.74, 97.72, 50.25])# L_Pitcher 球铰 (horn→ball=75=pitchArm ✓)

# 反推所得: 接收环 main-mount 平面世界高度 (使 6 main 杆=175mm)
RECV_Z_SOLVED = 221.0
ROD_NOM = 175.0
ROD_BODY_D = 6.0
ROD_BALL_D = 10.0

# main 舵机槽 (name, sx, sy) + 汇聚翻转配置 (lower 翻 Y 使球铰朝内)
MAIN_SLOTS = [("LowerLeft", -99.6, 37.0, True),
              ("UpperLeft", -99.6, -37.0, False),
              ("UpperRight", 99.6, -37.0, False),
              ("LowerRight", 99.6, 37.0, True)]

FRAME_X = 99.6          # pitch 舵机摆动 pivot 的 X
SERVO_PIVOT_H = 46.0    # pitch pivot 高度 (servoPivotH)
# pitch 舵机槽 (name, stl, x_sign): L 用原始 STL, R 镜像 X
PITCH_SLOTS = [("LeftPitch", "L_Pitcher", -1), ("RightPitch", "R_Pitcher", 1)]

COL = {  # 真实配色 (实物照片): 框架/杆=红, 舵机臂=米白, 球铰=铬, 接收=红
    "frame": (0.83, 0.13, 0.10), "arm": (0.93, 0.92, 0.90),
    "rod": (0.80, 0.12, 0.10), "ball": (0.75, 0.77, 0.80),
    "recv": (0.80, 0.13, 0.11), "gear": (0.27, 0.27, 0.27),
}


# ── 变换基元 ───────────────────────────────────────────────────────────────────
def _T(t):
    M = np.eye(4); M[:3, 3] = t; return M

def _Rz(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[0, 0] = c; M[0, 1] = -s; M[1, 0] = s; M[1, 1] = c; return M

def _Rx(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[1, 1] = c; M[1, 2] = -s; M[2, 1] = s; M[2, 2] = c; return M

def _Ry(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[0, 0] = c; M[0, 2] = s; M[2, 0] = -s; M[2, 2] = c; return M

def _MirX():
    M = np.eye(4); M[0, 0] = -1; return M

def _apply(M, p):
    p = np.asarray(p, float)
    return (M[:3, :3] @ p) + M[:3, 3]


# ── 臂取向: 镜像X(左) + 绕花键Z翻180(汇聚) + 平移花键到槽位 ──────────────────────
def main_arm_transform(sx, sy, flip_y):
    horn = ARM_HORN.copy()
    M = np.eye(4)
    if sx < 0:
        M = _MirX() @ M
    if flip_y:                       # 绕 (镜像后) 花键的竖直 Z 轴转 180° → 球铰朝 -Y
        h = _apply(M, ARM_HORN)
        M = _T(h) @ _Rz(180) @ _T(-h) @ M
    h = _apply(M, ARM_HORN)
    M = _T([sx - h[0], sy - h[1], 0.0]) @ M
    return M

def main_ball_world(sx, sy, flip_y):
    return _apply(main_arm_transform(sx, sy, flip_y), ARM_BALL)


def solve_rot(pivot, ball0, mount, axis, rod=ROD_NOM):
    """舵机臂绕 `axis`(过 `pivot`)旋转 θ, 使球铰→mount 距离 = rod(175).
    通用 rotary-servo IK: 主臂 axis=Z(水平摆), pitch 臂 axis=Y(竖直摆).
    返回 (θ_deg, ball_world, reachable)."""
    pivot = np.asarray(pivot, float); ball0 = np.asarray(ball0, float)
    mount = np.asarray(mount, float)
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    v = ball0 - pivot
    w = mount - pivot
    v_par = (v @ a) * a
    v_perp = v - v_par
    A = float(w @ v_perp)
    B = float(w @ np.cross(a, v_perp))
    K = (float(v @ v) + float(w @ w) - rod * rod) / 2.0 - float(w @ v_par)
    R = math.hypot(A, B)
    reach = True
    if R < 1e-9:
        th = 0.0
    else:
        s = K / R
        if abs(s) > 1.0:
            reach = False
            s = max(-1.0, min(1.0, s))
        phi = math.atan2(A, B)
        cand = [math.asin(s) - phi, math.pi - math.asin(s) - phi]
        th = min(cand, key=lambda t: abs((t + math.pi) % (2 * math.pi) - math.pi))
    ct, st = math.cos(th), math.sin(th)
    v_rot = v_par + ct * v_perp + st * np.cross(a, v_perp)
    return math.degrees(th), pivot + v_rot, reach


# ── 装配 ────────────────────────────────────────────────────────────────────────
def _load(name):
    p = P.stl_path(name)
    if not os.path.exists(p):
        return None
    return trimesh.load(p, process=False, force="mesh")

def _rod(p1, p2):
    p1 = np.asarray(p1, float); p2 = np.asarray(p2, float)
    seg = trimesh.creation.cylinder(radius=ROD_BODY_D / 2, segment=[p1, p2], sections=12)
    b1 = trimesh.creation.icosphere(subdivisions=1, radius=ROD_BALL_D / 2); b1.apply_translation(p1)
    b2 = trimesh.creation.icosphere(subdivisions=1, radius=ROD_BALL_D / 2); b2.apply_translation(p2)
    return trimesh.util.concatenate([seg, b1, b2])


def assemble(recv_z=RECV_Z_SOLVED, tx=0.0, ty=0.0, roll=0.0, pitch=0.0, twist=0.0,
             label="home", with_pitchers=True, export=True):
    meshes, cols = [], []

    def add(m, rgb):
        meshes.append(m)
        cols.append(np.tile(np.array(rgb), (len(m.vertices), 1)))

    # A. 静态结构件 (原位)
    for n in ["Base", "L_Frame", "R_Frame", "PowerBus"]:
        m = _load(n)
        if m is not None:
            add(m, COL["frame"])

    # B. 接收 + twist 组 (平移到反推高度 + 位姿) — 先定平台, 臂随后 IK 跟上
    Mr = _T([tx, ty, recv_z]) @ _Rx(pitch) @ _Ry(roll) @ _Rz(twist)
    recv_main_R = _apply(Mr, [RECV_MAIN_R, 0, 0])
    recv_main_L = _apply(Mr, [-RECV_MAIN_R, 0, 0])
    recv_pitch_R = _apply(Mr, RECV_PITCH)
    recv_pitch_L = _apply(Mr, RECV_PITCH * [-1, 1, 1])
    for n in ["Receiver", "Twist_Base", "Twist_Body", "Twist_Lid",
              "RingGear", "ExchangeGear", "DriveGear"]:
        m = _load(n)
        if m is None:
            continue
        m.apply_transform(Mr)
        add(m, COL["gear"] if "Gear" in n else COL["recv"])

    # C. 4 main 臂 (真值取向 + rotary-servo IK, 绕竖直花键 Z 轴摆, 杆=175 撑到平台)
    rods, balls = [], {}
    AXIS_Z = np.array([0.0, 0.0, 1.0])
    AXIS_Y = np.array([0.0, 1.0, 0.0])
    for name, sx, sy, flip in MAIN_SLOTS:
        M0 = main_arm_transform(sx, sy, flip)
        Hw = _apply(M0, ARM_HORN)
        B0w = _apply(M0, ARM_BALL)
        mount = recv_main_R if Hw[0] > 0 else recv_main_L
        th, ball, reach = solve_rot(Hw, B0w, mount, AXIS_Z)
        M = _T(Hw) @ _Rz(th) @ _T(-Hw) @ M0
        m = _load("Arm"); m.apply_transform(M)
        add(m, COL["arm"])
        balls[name] = ball
        add(_rod(ball, mount), COL["rod"])
        rods.append((name, float(np.linalg.norm(ball - mount)), reach))

    # C2. 2 pitch 臂 (绕 Y 轴竖直摆 pivot(±FRAME_X,0,servoPivotH), 杆=175 → pitch mount)
    if with_pitchers:
        for name, stl, sgn in PITCH_SLOTS:
            pivot = np.array([sgn * FRAME_X, 0.0, SERVO_PIVOT_H])
            ball0 = PITCH_BALL * [sgn, 1, 1]      # L 原始, R 镜像 X
            mount = recv_pitch_R if sgn > 0 else recv_pitch_L
            th, ball, reach = solve_rot(pivot, ball0, mount, AXIS_Y)
            m = _load(stl)
            if m is not None:
                m.apply_transform(_T(pivot) @ _Ry(th) @ _T(-pivot))
                add(m, COL["arm"])
            balls[name] = ball
            add(_rod(ball, mount), COL["rod"])
            rods.append((name, float(np.linalg.norm(ball - mount)), reach))

    # 球铰高亮 (铬球)
    for name in balls:
        s = trimesh.creation.icosphere(subdivisions=1, radius=ROD_BALL_D / 2)
        s.apply_translation(balls[name]); add(s, COL["ball"])

    V = np.vstack([m.vertices for m in meshes])
    Foff, off = [], 0
    for m in meshes:
        Foff.append(np.asarray(m.faces) + off); off += len(m.vertices)
    F = np.vstack(Foff)
    VC = np.vstack(cols)

    report = {"label": label, "recv_z": recv_z, "recv_pose": [tx, ty, roll, pitch, twist],
              "rods_mm": {n: round(L, 2) for n, L, _ in rods},
              "rod_max_err_mm": round(max(abs(L - ROD_NOM) for _, L, _ in rods), 2),
              "unreachable": [n for n, _, rc in rods if not rc],
              "n_verts": int(len(V)), "n_faces": int(len(F))}

    if export:
        out = os.path.join(_PROJ, "output", f"ORS6_truth_{label}.stl")
        trimesh.Trimesh(vertices=V, faces=F, process=False).export(out)
        report["stl"] = out
    return V, F, VC, report


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    V, F, VC, rep = assemble()
    print(json.dumps(rep, ensure_ascii=False, indent=2))
