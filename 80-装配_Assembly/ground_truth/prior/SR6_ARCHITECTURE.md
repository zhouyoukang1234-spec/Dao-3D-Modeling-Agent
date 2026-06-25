# SR6 Complete Architecture — PDF逆向解构

> 源: SR6 Build Instructions.pdf (45页, June 2022, by TempestMAx)
> 固件: SR6-Alpha4-ESP32.ino (TCode v0.3)
> 3D验证: hub.html (18 STL零件, IK运动学)

## 一、平台身份

**SR6 = Stroker Robot, 6-Axis** — 改良Stewart平台
- 6自由度: 上下(L0) + 左右(L2) + 前后(L1) + 横滚(R1) + 俯仰(R2) + 扭转(R0)
- 第7轴: T-wist4模块, 齿轮传动±135° (270°)
- 设计者: TempestMAx (Alpha 2020.12, Beta 2022.01, June'22修订)
- 安装: VESA 100mm (4×M4), 适配显示器支架/桌夹

## 二、运动学真相 (固件逆向 + PDF交叉验证)

### 核心常数

| 参数 | 符号 | 值 | 固件来源 | PDF验证 |
|------|------|------|---------|---------|
| 基座高度 | baseH | 162.48mm | `16248/100` | - |
| 舵机臂 | mainArm | 50mm | `2a=100 → a=50` | Futaba 25T M3 horn |
| 主连杆 | mainRod | **175mm** | `√(28125+2500)=√30625` | p.26 "175mm apart" |
| 俯仰臂 | pitchArm | 75mm | `2a=150 → a=75` | - |
| 俯仰连杆 | pitchRod | 175mm | `√(36250-5625)=√30625` | 同主连杆长度 |
| 俯仰偏移 | pitchOff | 55mm | `5500/100` | - |
| 俯仰角 | pitchAng | 15° | `0.2618 rad` | - |
| μs/弧度 | msPerRad | 637 | `#define` (标准舵机) | - |

### ⚠️ 167.7mm谬误

`√28125 ≈ 167.7mm` 是对固件常数的**误解**:
- 28125 = mainRod² − mainArm² = 175² − 50² = 30625 − 2500
- 这是余弦定理中的 (b²−a²) 项, **不是**任何零件的实际长度
- 已在 `ors6_hub.py` 和 `_sr6_reverse.py` 中修正

### 运动范围

| 轴 | T-Code | 固件映射 | 物理范围 | PDF确认 |
|----|--------|---------|---------|---------|
| 上下 | L0 (thrust) | ±6000 (1/100mm) | **120mm** | p.5 "120mm up-down" |
| 前后 | L1 (fwd) | ±3000 | **60mm** | p.5 "60mm forward-backward" |
| 左右 | L2 (side) | ±3000 | **60mm** | p.5 "60mm left-right" |
| 扭转 | R0 (twist) | 0-9999→±1000μs | **270°** (T-wist4) | p.33 "180° or 270°" |
| 横滚 | R1 (roll) | ±3000 | **~±30°** | p.5 "approximately ±30°" |
| 俯仰 | R2 (pitch) | ±2500 | **~±25°** | p.5 同上 |

### IK求解 (SetMainServo)

```
输入: (x, y) = 接收器枢轴位置 (1/100mm)
  x = 垂直方向 (baseH ± fwd)
  y = 水平方向 (1500 ± thrust ± roll, 即15mm偏移)

gamma = atan2(x, y)           // 枢轴方向角
c² = x² + y²                  // 枢轴距离²
beta = acos((c² - 28125) / (100·c))  // 余弦定理: (c² + arm² - rod²) / (2·arm·c)
output = msPerRad × (gamma + beta - π)  // 舵机脉宽偏移
```

### IK求解 (SetPitchServo)

```
输入: (x, y, z, pitch) — z=侧向偏移, pitch=1/100度

// 俯仰偏移: 接收器上枢轴相对下枢轴偏移55mm@15°
x += 5500·sin(0.2618 + pitch×0.0001745)
y -= 5500·cos(0.2618 + pitch×0.0001745)

// 等效臂长(考虑z偏移)
bsq = 36250 - (75 + z)²
// 当z=0: bsq = 30625 = 175² (同主连杆)

beta = acos((c² + 5625 - bsq) / (150·c))  // pitchArm=75, 75²=5625
```

### 舵机布局 (固件 + PDF p.23)

| 舵机 | 引脚 | 位置 | 方向 | 频率 | IK函数 |
|------|------|------|------|------|--------|
| Lower Left | D15 | 左框架下 | 向外 | 330Hz | SetMainServo(16248-fwd, 1500+thrust+roll) |
| Upper Left | D2 | 左框架上 | 向外 | 330Hz | SetMainServo(16248-fwd, 1500-thrust-roll) |
| Left Pitch | D4 | 左框架顶 | 向内 | 330Hz | SetPitchServo(16248-fwd, 4500-thrust, side-1.5·roll, -pitch) |
| Lower Right | D13 | 右框架下 | 向外 | 330Hz | SetMainServo(16248-fwd, 1500+thrust-roll) |
| Upper Right | D12 | 右框架上 | 向外 | 330Hz | SetMainServo(16248-fwd, 1500-thrust+roll) |
| Right Pitch | D14 | 右框架顶 | 向内 | 330Hz | SetPitchServo(16248-fwd, 4500-thrust, -side+1.5·roll, -pitch) |
| Twist | D27 | 接收器上 | - | 50Hz | 直接映射 R0 |
| Valve | D25 | 接收器上 | - | 50Hz | 速度跟踪+吸力控制 |

### hub.html 3D中角度布局

```
LowerLeft:  150° (左下)    UpperLeft:  210° (左上)
LeftPitch:  120° (左俯仰)  RightPitch: 60°  (右俯仰)
UpperRight: 330° (右上)    LowerRight: 30°  (右下)
```

## 三、机械结构 (BOM全解)

### 3D打印件 (12种核心 + 9种T-wist4)

#### 核心结构

| # | 零件 | 数量 | 功能 | STL文件 | hub.html |
|---|------|------|------|---------|----------|
| 1 | Frame L | 1 | 左框架, 容纳3舵机 | SR6 L形框架 Beta1.stl | ✅ 静态 |
| 2 | Frame R | 1 | 右框架, 容纳3舵机 | SR6 R-Frame Beta1.stl | ✅ 静态 |
| 3 | Base | 1 | 底座外壳, VESA接口 | SR6 底座 Beta1A.stl | ✅ 静态 |
| 4 | Lid | 1 | 顶盖, 开关孔 | SR6 盖子 Beta1.stl | ✅ 静态 |
| 5 | Receiver | 1 | 接收器, 持握玩具 | SR6 Receiver Beta1.stl | ✅ 动态(topGroup) |
| 6 | Main Arm | 4 | 主臂, Futaba 25T horn | SR6 臂 Beta1.stl | ✅ 替换horn primitive |
| 7 | L-Pitcher Arm | 1 | 左俯仰臂, 有弯折 | SR6 L-投手 Beta1.stl | ✅ 静态 |
| 8 | R-Pitcher Arm | 1 | 右俯仰臂, 有弯折 | SR6 R-投手 Beta1.stl | ✅ 静态 |
| 9 | Main Link | 4 | 主连杆175mm, 球头轴承 | SR6 Main Link Alpha1.stl | ✅ 动态(rod) |
| 10 | Pitcher Link | 2 | 俯仰连杆175mm | SR6 Pitcher Link Alpha1.stl | ✅ 动态(rod) |
| 11 | Power Bus Holder | 1 | 电源总线支架 | SR6 电源总线支架 Beta1.stl | ✅ 静态 |
| 12 | Tray | 1 | ESP32+电源座 | SR6 Tray Standard Beta1.stl | ❌ 内部不可见 |
| 13 | 4×3mm Spacer | 12 | 球头轴承垫片 | SR6 4x3mm 垫片 Beta1.stl | ❌ 微小零件 |

#### T-wist4 扭转模块

| # | 零件 | 数量 | 功能 | STL文件 | hub.html |
|---|------|------|------|---------|----------|
| 14 | Receiver Base | 1 | 接收器底座 | T-wist4 SR6 Base Beta1.stl | ✅ 静态 |
| 15 | Receiver Body | 1 | 接收器主体 | T-wist4 SR6 Body Beta1.stl | ✅ 静态 |
| 16 | Receiver Lid | 1 | 接收器盖子 | T-wist4 Lid Beta1.stl | ✅ 静态 |
| 17 | Ring Gear | 1 | 环形齿轮(旋转件) | T-wist Clip Ring Gear Beta4.stl | ✅ 动态(R0) |
| 18 | Transfer Gear | 1 | 传动齿轮(中间) | T-wist Exchange Gear Beta1.stl | ✅ 静态 |
| 19 | Drive Gear | 1 | 驱动齿轮(舵机上) | T-wist4 Drive Beta1.stl | ✅ 静态 |

**覆盖率**: hub.html加载 18/21 种STL = **86%** (缺失: Tray/Spacer/Bearing Links → 用Alpha Links替代)

### 紧固件 (完整BOM)

| 规格 | 数量 | 用途 |
|------|------|------|
| M4×30 | 2 | 主连杆→接收器下部 |
| M4×25 | 2 | 俯仰连杆→接收器上部 |
| M4×20 | 6+4* | 连杆→臂 + VESA安装* |
| M4×16 | 5 | 框架→底座(4) + 框架互连(1) |
| M3×10 | 44 | 舵机固定(24) + 托盘(4) + 顶盖(4) + T-wist(12) |
| M3×8 | 4+7 | 舵机horn(4/6) + T-wist(7) |
| M2×8 | 4 | ESP32固定(2) + 电源总线(2) |
| M4 nut | 9+bearing | 底座(8) + 框架(1) + 连杆轴承 |
| M3 nut | 32 | 框架内槽(28) + 底座(4) |
| M4 rod end bearing | 12 | 6连杆×2端 (或12×6mm橡胶垫圈替代) |

### 电子系统

| 组件 | 规格 | 用途 |
|------|------|------|
| ESP32 DevKit v1 | DOIT | 主控, USB串口, WiFi/BLE(未使用) |
| 电源 | 5-6V ≥6A(推荐10A) | 舵机供电 |
| 标准舵机 | ≥20kg.cm × 6 | 主驱动 |
| 扭转舵机 | 270° 5-10kg.cm × 1 | T-wist4驱动 |
| 震动电机 | × 2 (可选) | V0/V1通道, 需MOSFET |
| Futaba 25T M3 horn | × 7 | 金属舵机摆臂 |

### 电源总线 (手工焊接)

```
电源供应 → 桶形插座 → [红线30cm] → 拨动开关 → [红线30cm] → 铜洞洞板(+排)
                        [黑线10cm]                              → 铜洞洞板(-排)
                                                                → [细黑线7cm+crimp] → ESP32 GND
```
- 0.1"铜洞洞板, 2排×12孔(10舵机+2预留)
- 0.1"排针焊接, **两排必须电气隔离!**
- 舵机信号线从连接器拆出, 单独插ESP32数字引脚

## 四、装配架构 (7阶段)

```
Stage 1: 电源总线 ──────── 焊接(最难, ~30分钟)
Stage 2: 托盘 ──────────── ESP32+电源总线安装
Stage 3: 固件 ──────────── Arduino IDE上传
Stage 4: 框架 ──────────── L/R框架+6舵机+6臂+horn对齐+零位调整
Stage 5: 连杆 ──────────── 6×连杆(175mm)+轴承/垫圈
Stage 6: 外壳 ──────────── 底座+框架+托盘+顶盖
Stage 7: 最终装配 ──────── 接收器+连杆连接
Stage 8: T-wist4 (可选) ── 齿轮机构+twist接收器+600mm延长线
```

### 舵机零位校准 (Stage 4关键)

- 默认1500μs = 中心
- ±160μs = horn旋转1/25圈 (一个齿位)
- 调整范围: 1420~1580 (理论上超出=horn未安在最近齿位)
- 臂应水平: horn孔与臂端孔连线平行于框架底部

### T-wist4齿轮系 (Stage 8)

```
Twist Servo → Drive Gear(舵机horn上) → Transfer Gear(M3×25销轴) → Ring Gear(旋转环)
```
- 传动比 1:1 (齿数相同)
- 润滑: 凡士林涂抹接触面 → 大幅降低噪音
- Ring Gear向后兼容T-wist3

## 五、hub.html 3D仿真忠实度审计

| 维度 | 符合度 | 说明 |
|------|--------|------|
| IK运动学常数 | ✅ 100% | baseH/mainArm/mainRod/pitchArm/pitchOff/pitchAng全部正确 |
| 舵机布局 | ✅ 100% | 6个角度位置匹配固件 out1-out6 |
| STL零件覆盖 | ✅ 86% | 18/21种(缺Tray/Spacer/BearingLink) |
| 连杆长度 | ✅ 175mm | 主连杆和俯仰连杆均正确 |
| T-wist4齿轮 | ✅ 100% | Ring Gear动态跟随R0轴 |
| 材质区分 | ✅ | 框架灰/舵机黑/接收器蓝/齿轮金/连杆按应力变色 |
| 运动范围 | ✅ | thrust±60mm, fwd/side±30mm, roll/pitch±30° |

### 已修正的错误

| 文件 | 错误 | 修正 |
|------|------|------|
| `ors6_hub.py` L56 | 注释"167.7mm" | → "rod²-arm²=175²-50²" |
| `_sr6_reverse.py` L28 | `main_arm_mm: 167.7` | → `main_arm_mm: 50.0, main_rod_mm: 175.0` |

## 六、T-Code协议

```
格式: {TYPE}{CHANNEL}{VALUE}[{EXT}{EXT_VALUE}]
类型: L(线性) R(旋转) V(振动) A(辅助) D(设备) $(设置)

示例:
  L05000      → L0轴移到中点(5000/9999)
  L09999I1000 → L0移到最高, 1000ms到达
  R00000S500  → R0移到最低, 速度500
  DSTOP       → 全部停止
  D0          → 返回固件ID
  D1          → 返回TCode版本
  D2          → 列出所有轴
```

### 注册轴 (SR6模式)

| T-Code | 名称 | 功能 |
|--------|------|------|
| L0 | Up | 上下行程 (主轴) |
| L1 | Forward | 前后 |
| L2 | Left | 左右 |
| R0 | Twist | 扭转 (T-wist) |
| R1 | Roll | 横滚 |
| R2 | Pitch | 俯仰 |
| V0 | Vibe1 | 振动电机1 |
| V1 | Vibe2 | 振动电机2 (或润滑泵) |
| A0 | Valve | 气阀位置 |
| A1 | Suck | 吸力级别 |
| A2 | Lube | 润滑泵 (可选) |

## 七、完整零件图谱 (STL文件树)

```
STLs/
├── SR6测试版零件/ (12件核心)
│   ├── SR6 底座 Beta1A.stl ········ Base外壳 (263KB, VESA接口)
│   ├── SR6 L形框架 Beta1.stl ······ Left Frame (288KB, 3舵机槽)
│   ├── SR6 R-Frame Beta1.stl ······ Right Frame (304KB, 3舵机槽)
│   ├── SR6 Receiver Beta1.stl ····· 接收器 (406KB, 持握Fleshlight)
│   ├── SR6 盖子 Beta1.stl ········· Lid顶盖 (227KB, 开关孔)
│   ├── SR6 臂 Beta1.stl ··········· Main Arm×4 (72KB, Futaba horn槽)
│   ├── SR6 L-投手 Beta1.stl ······· Left Pitcher (102KB, 弯折臂)
│   ├── SR6 R-投手 Beta1.stl ······· Right Pitcher (102KB, 弯折臂)
│   ├── SR6 轴承主连杆 Beta1.stl ··· Bearing Main Link×4 (100KB)
│   ├── SR6 轴承投手链接 Beta1.stl · Bearing Pitcher Link×2 (120KB)
│   ├── SR6 电源总线支架 Beta1.stl · Power Bus Holder (24KB)
│   └── SR6 4x3mm 垫片 Beta1.stl ·· Spacer×12 (20KB)
│
├── SR6备用零件/ (Alpha兼容)
│   ├── SR6 Main Link Alpha1.stl ··· 橡胶垫圈版主连杆 (43KB) ← hub.html使用
│   ├── SR6 Pitcher Link Alpha1.stl  橡胶垫圈版俯仰连杆 (51KB) ← hub.html使用
│   └── SR6 Window Lid Beta1.stl ··· 窗口版顶盖 (282KB, Shield用)
│
├── SR6测试版托盘/ (3种+1 STEP)
│   ├── SR6 Tray Standard Beta1.stl  标准ESP32托盘 (138KB)
│   ├── SR6 Tray Screw Jack Beta1.stl 螺丝固定版 (127KB)
│   └── SR6 Tray XT60E1-M Beta1.stl  XT60电源版 (134KB)
│
├── SR6测试版防护罩/ (3种+1 STEP)
│   ├── SR6 Shield 40mm Fan.stl ···· 40mm风扇 (125KB) ← hub.html使用
│   ├── SR6 Shield 40mm Fan + OLED Display.stl (158KB)
│   └── SR6 Shield 40mm Fan + OLED Display(alternate).stl (167KB)
│
└── SR6 T-wist4/ (9件扭转模块)
    ├── T-wist4 SR6 Base Beta1.stl · 接收器底座 (275KB)
    ├── T-wist4 SR6 Body Beta1.stl · 接收器主体 (843KB, 最大单件)
    ├── T-wist4 Lid Beta1.stl ······ 接收器盖 (53KB)
    ├── T-wist Clip Ring Gear Beta4.stl 环形齿轮 (1.4MB, 最复杂)
    ├── T-wist Exchange Gear Beta1.stl  传动齿轮 (638KB)
    ├── T-wist4 Drive Beta1.stl ···· 驱动齿轮 (1.1MB)
    ├── SR6 Grommet Pitcher Link Beta1.stl (92KB)
    ├── SR6 L-Pitcher Angle Link Beta1.stl (108KB)
    └── SR6 R-Pitcher Angle Link Beta1.stl (77KB)

总计: 30个STL + 2个STEP = 32文件, ~7.48MB, ~156K三角面
```

---
*Generated 2026-03-19 from PDF reverse engineering + firmware analysis + 3D cross-validation*
