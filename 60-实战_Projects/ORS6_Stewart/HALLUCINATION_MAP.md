# 幻觉地图 v∞ · 反者道之动

> 道, 可道也, 非恒道也。 —— 帛书《老子》
>
> 本文档以 firmware (`SR6-Alpha4_ESP32.ino`) + STL trimesh 实测 + PDF 标注 为三大不可质疑本源,
> 反向解构所有 ORS6_Stewart 数值的"公理 / 推论 / 幻觉"身份, 供后续 agent 阅读避免再创造新幻觉。

## 0. 真本源三足

| 本源 | 内容 | 不可质疑性 |
|------|------|------|
| **L0 firmware** | `SR6-Alpha4_ESP32.ino:837-863` `SetMainServo`/`SetPitchServo` | 极强 (实物 ESP32 烧录运行) |
| **L1 STL geometry** | 31 STL 几何 (圆柱孔 axis SVD + bbox) | 极强 (设计师交付件) |
| **L2 PDF instructions** | `SR6 Build Instructions.pdf` (TempestMAx, 2022) | 强 (装配权威文档) |

**至关重要**: firmware 仅在每个 servo 的 **2D 局部平面**做 IK, 完全**不引用世界坐标**。
所有"world coord"的数值都是我们 (kinematics.py) 在二者间架的桥梁, 是**推论**而非公理。

## 1. 公理 (firmware 给定)

| # | 数值 | 出处 | 用途 |
|---|------|------|------|
| 1 | `mainArm = 50.0 mm` | firmware `28125 = mainRod² − mainArm² = 175² − 50²` | main servo 摇臂长 |
| 2 | `mainRod = 175.0 mm` | firmware + PDF p.26 "175mm apart" | 连杆长 (M4 rod end bearing 间距) |
| 3 | `pitchArm = 75.0 mm` | firmware `5625 = pitchArm² = 75²` | pitch servo 摇臂长 |
| 4 | `pitchOff = 55.0 mm` | firmware `5500 = 55 mm * 100` | pitcher arm 在 pitch 平面内的偏移 |
| 5 | `pitchAng = 15° = 0.2618 rad` | firmware `0.2618` | pitch servo arm 静态角偏 |
| 6 | `msPerRad = 637 μs/rad` | firmware `ms_per_rad` | 标准舵机 μs/rad 系数 |

## 2. 推论 (STL trimesh 实测真值)

> 算法: `tools/_dao_axis_v2.py` — 圆柱孔法向 SVD 找 great-circle 法向 = 孔 axis。
> 输出: `tools/_dao_axis_v2.json`

### 2.1 Arm STL (单一镜像源)

```
horn 真值 = (67.5, 0.0, 51.0)    axis‖Z   R≈M3-M5 cluster
ball 真值 = (67.5, 50.0, 51.0)   axis‖Z   R=1.89mm (M4 rod end)
horn → ball 距离 = √(0² + 50² + 0²) = 50.0 mm   ✓ 完美等于 mainArm
```
- 当前 viewer `ARM_HORN_STL=(67.5, 0, 51)` ✓
- 当前 viewer `ARM_BALL_STL=(67.5, 50, 51)` ✓

### 2.2 L_Pitcher / R_Pitcher STL

```
L horn 真值 = (-7.5, 30.0, 51.75)
L ball 真值 = (-39.74, 97.72, 50.25)
L horn → ball 距离 = √(32.24² + 67.72² + 1.5²) = √(1039+4586+2.25) = 75.00 mm  ✓ 完美等于 pitchArm

R 镜像同 (X 取 +)
```
- 当前 viewer `PITCHER_HORN_STL=(±7.5, 30, 51.75)` ✓
- 当前 viewer `PITCHER_BALL_STL=(±41.47, 99.98, 50.25)` ❌ 偏 ~3mm (用 trimesh tip-cluster, 非真 ball axis)
- **kinematics.py:272 `PITCHER_PIVOT_STL` Y=9.0 是 STL bbox Y_min, 非真 horn axis**, 应改为 30.0

### 2.3 Receiver STL (本地坐标, 不含 HOME_H 平移)

```
main mount  = (±59.98, 0.0,    0.0)    axis‖X  R=1.79mm  L=13.3mm  (M4 rod end pin)
pitch mount = (±61.0, -14.24, 53.13)   axis‖X  R=1.79mm  L=12.0mm  (M4 rod end pin)
```
- main rod axle: 仅 2 个 mount hole (左右各 1), 但 4 main rod 共享 (PDF p.31: "two links on each bolt")
- pitch rod axle: 2 个 mount hole, 各服务一个 pitcher rod

### 2.4 L_Frame STL (servo 安装真值)

trimesh 实测 24 个 axis‖Z 的 R=1.74mm M3 通孔, 聚类得 6 组 (= 3 servo × 4 mount holes/servo, 但每组 4 hole 非全可见, 部分被 lug 几何遮挡, 归为 6 个 Y 中心):

```
servo 1 (LowerLeft main): Y=+30 (mount holes Y=+25, +35)  X=-51.9..-100.9 (49mm)
servo 2 (LeftPitch):       Y= 0 (mount holes Y= -5, +5)   X=-51.9..-100.9 (49mm)
servo 3 (UpperLeft main):  Y=-30 (mount holes Y=-25, -35) X=-51.9..-100.9 (49mm)
```

**servo body 矩形 49 × 10 mm 完美吻合标准 RC servo (Hitec HS-485HB / Futaba S3003) mount lug spec**.

axle 真位置 (按 PDF p.22 "main outward, pitch inward" + 标准 servo lug 5-10mm 偏置):

| servo | Y 真值 | X 真值 (推算) | 当前 SERVO_SLOTS X | 偏差 |
|-------|--------|----------------|---------------------|------|
| LowerLeft  | +30 | -106 ~ -111 | -99.6 | +6 ~ +11 mm |
| UpperLeft  | -30 | -106 ~ -111 | -99.6 | +6 ~ +11 mm |
| LeftPitch  | 0   | -42 ~ -47   | -99.6 | **+53 ~ +58 mm 巨错** |

**main Y=±37 (旧) 与真值 ±30 偏 7mm**

### 2.5 servoPivotH / HOME_H

- servoPivotH=46 ✓ — mount hole Z=22.30 (frame 内壁底) + servo 高度 24 = 46 (axle 在 servo 顶面)
- HOME_H = 46 + baseH(162.48) = 208.48 mm — 推论, 是我们的"servoPivotH 上方 162.48mm 处 = receiver home Z"几何假设, 与 firmware `16248` 单位互译

## 3. 当前 4 大错位本源

### 错位 #1 · split brain: PITCHER_PIVOT_STL Y 双源不一致 (★ 最严重)
- `kinematics.py:272` 返回 `pivot=[-7.5, 9.0, 51.75]` 给 viewer API
- `viewer/index.html:1130` 硬编码 `PITCHER_HORN_STL=(-7.5, 30.0, 51.75)`, **完全忽略** API 的 pivot 字段
- trimesh 实测真值 = 30.0
- **修复**: 让 `kinematics.py` 的 `PITCHER_PIVOT_STL` Y 改 30.0, 并删除 viewer 硬编码改用 API

### 错位 #2 · viewer PITCHER_BALL_STL 用 tip-cluster 而非真 ball axis
- `viewer/index.html:1134` 硬编码 `(±41.47, 99.98, 50.25)`, 来源是早期 trimesh "tip cluster bbox center"
- trimesh 圆柱 axis 真值 = `(±39.74, 97.72, 50.25)`, 偏差 ~3mm
- **修复**: 改为真值, horn-ball 距离会从 78mm → 75mm, 与 firmware pitchArm=75 完美吻合

### 错位 #3 · SERVO_SLOTS main Y=±37 偏 ±30 真值 7mm
- `parts.py:198-204` 硬编码 main Y=±37
- trimesh 实测 mount hole 中心 Y=±30
- 影响: viewer 中 main arm 整体 Y 方向偏 7mm
- **修复**: 改 ±30。需小心: kinematics.py 的 mount Y=sy 也跟着变, IK 数值会变 (mount Y 不再 ±37)

### 错位 #4 · SERVO_SLOTS pitch X=±99.6 偏 ±45 真值 53mm 🔥
- `parts.py:201,202` 硬编码 LeftPitch X=-99.6, RightPitch X=+99.6
- trimesh + 标准 servo lug 推算 真值 ≈ ±45 (pitch axle 朝内, 离 frame 内壁 5-10mm)
- 影响: pitcher arm 视觉起点 大错位 53mm, 但 firmware IK 不感知 (它只用 servo local frame)
- **修复风险高**: 改后视觉位置改, kinematics.py:163 `mount = (sx ± y, sy, ...)` 中 sx 也变, IK mount 位置改, rod_3d 仍能保持 175mm (因 IK 自洽), 但 视觉与 firmware 物理一致性 验证有难度

## 4. 反者审视: 哪些是"伪真值"

> 上士闻道, 堇而行之；中士闻道, 若存若亡；下士闻道, 大笑之。 —— 帛书《老子》

以下数值之前被各 agent 用作"truth", 实际只是 STL 表面统计:

| 假真值 | 来源 | 真值 |
|--------|------|------|
| Arm bbox center (67.5, 21.5, 51.5) | trimesh bbox/2 | horn axis (67.5, 0, 51), 二者皆有用但不同语义 |
| L_Pitcher bbox center (-16.6, 56.9, 51.8) | trimesh bbox/2 | horn axis (-7.5, 30, 51.75) |
| `PITCHER_PIVOT_STL Y=9` | STL bbox Y_min | **错误**, 真 horn axis Y=30 |
| `PITCHER_BALL Y=99.98` | trimesh tip cluster | **轻微错**, 真 ball axis Y=97.72 |
| frame_spacing=199.2mm | parts.py 硬编码 X 的 2 倍 | **真**只能从 servo axle X 推, mount-pattern 推算 211.8mm (非 199.2) |

## 5. 不可定 (需物理实测才能确认)

- servo lug 离 axle 真距离 (5mm? 10mm? 不同型号差异): 决定 servo axle X 真位置
- main servo "outward" 是 axle 朝外侧还是朝内侧 (PDF p.22 文字 vs 图片有歧义)
- receiver home Z 真值 (firmware 数学 = 208.48mm, 物理实测如何待校)

## 6. 反者建议: 单源真理化

当前**多源不一致**是幻觉根源:
- `kinematics.py:265, 272` 硬编码 ARM_PIVOT, PITCHER_PIVOT
- `viewer/index.html:984, 1130, 1134` 硬编码同语义不同值
- `parts.py:198-205` 硬编码 SERVO_SLOTS

应统一为:
1. **STL trimesh 自动提取真值 → JSON 缓存** (build-time once)
2. **Python (kinematics.py) 与 JS (viewer) 都从 JSON 读** (run-time 单源)
3. **测试文件锁死 JSON 真值** (CI 防漂移)

具体: 用 `tools/_dao_axis_v2.py` 输出 `_stl_axis_truth.json`, `parts.py` 加载, viewer API `/api/parts` 返回, viewer/index.html 删除所有硬编码改读 API。

## 7. closed_loop/true_kinematics.py 根治 (2026-06-25 · 反者道之动)

> 前文 (§3,§4) 是旧 `viewer/kinematics.py` 的幻觉。`closed_loop/true_kinematics.py`
> 是从根本重建的 3D 并联运动学, 本节记录它的**最后一处幻觉**及其根治。

### 7.1 幻觉: 主舵机 world X = ±85 (再加 ±99.6/±108 等)
- 旧 `closed_loop/true_kinematics.py` 把主舵机轴猜在 `X=±85`, 俯仰在 `X=±84.4`。
- 由此凭空得出每腿 **~25mm 平面外偏移** → "不可消除差 ~1.78mm/腿"。
- 这个 1.78mm 被当成"固件 2D vs 3D 刚体的真实差"写进闭环报告 —— **实为猜测坐标的产物**。

### 7.2 真值: 舵机与其球铰**同 X 平面** (X=±60 主 / ±61 俯仰)
- 实测 `SR6 臂` STL 是平面件 (hub 与 tip 同 X=67.5); 装在 ‖X 舵机轴上, 臂尖恒在该 X 平面内。
- 连杆从臂尖连到接收器主销 (§2.3 实测 **X=±59.98**)。固件 `SetMainServo` 是**纯平面 IK**,
  其成立前提 = "杆落在摆动平面内" = home 时臂尖 X == 主销 X ⇒ **舵机轴必在 X=±60**。
- 面内 (Y,Z) 仍由固件 home 给定: `SetMainServo(16248,1500)` ⇒ (竖直 162.48mm, 水平 15mm)
  ⇒ Z=HOME_H−162.48=30.52, Y=±15。俯仰同理在 X=±61 平面 (§2.3 pitch mount X=±61)。

### 7.3 根治后的诚实闭环 (实跑数, `python closure_report.py`)
```
(A) 刚体自洽:      worst dt=1.3e-10mm   rod-err=2.8e-14mm   (机器精度)
(B) 固件平面差:    home=0.000mm   主腿worst=1.141mm   俯仰worst=1.525mm
```
- **home 平面外偏移 = 0 ⇒ 6 杆精确 175.000mm, home 差 = 0** (旧的 1.78mm 假象消失)。
- 纯 thrust / fwd / pitch-rotation 等**面内**运动: 差 = 0 (固件 2D-IK 精确)。
- 仅 side / roll 等**真正离面**运动: 差 = √(175²+h²)−175, h = 接收器离面位移 (side 20mm⇒h=20mm⇒1.14mm)。
  这才是固件 2D 控制 vs 3D 刚体的**真实(且很小)差**, 非标定失败、非自指恒等式、非猜测坐标产物。

### 7.4 拓扑确认 (与 §2.3 一致)
- 主销 **2 个** (左右各 1, X=±60), 4 条主腿两两共享 ⇒ `true_kinematics.B_LOCAL` 中 LL/UL 同点、LR/UR 同点 ✓。
- `closed_loop/assembly/assemble.py` 仍按旧的"4 个独立主球(X=60/33)"渲染, 是**遗留渲染件**,
  与已根治的运动学拓扑不一致, 列为后续对齐项 (不影响 `true_kinematics` 闭环)。

---

**版本**: v∞ · 2026-05-12 · 反者审视
**依据**: SR6-Alpha4_ESP32.ino + 31 STL trimesh + SR6 Build Instructions.pdf p.1-45
**工具**: tools/_dao_axis_v2.py · tools/_dao_servo_axle.py · tools/_dao_full_audit.py
