# ORS6 第一性原理 · v11 (归一后)

> 道可道，非常道。名可名，非常名。
> 反者道之动. 弱者道之用.

## 零、本文档定位

**v11 归一重写**: 原 467 行 `FIRST_PRINCIPLES.md` 中 80% 内容是建模层第一性原理 (STL/IK/装配/trimesh 等), 现已随建模代码一并归位到 `../3D建模Agent/60-实战_Projects/ORS6_Stewart/_AGENTS.md`.

本文档仅保留控制层 (ORS6 项目本身) 的第一性原理.

## 一、身份

ORS6-VAM饮料摇匀器 = **TCode 实时控制 + VaM 实时联动 + Funscript 播放 + 视频同步 + 多平台 Extension** 的业务系统.

它 **不是**:
- CAD 系统
- 3D 模型库
- 建模工具链宿主

建模相关一切职责归 **3D建模Agent/60-实战_Projects/ORS6_Stewart/**.

## 二、层次

```
┌────────────────────────────────────────────────────┐
│  浏览器 hub.html (TCode 控制面板 + Three.js 可视化) │
└────────────────────┬───────────────────────────────┘
                     │ HTTP+WebSocket
┌────────────────────▼───────────────────────────────┐
│  ors6_hub.py :41927  — 协议中枢                     │
│    ├── tcode/          TCode 协议编解码             │
│    ├── device_bridge   9 种设备适配器               │
│    ├── funscript/      Funscript 播放器             │
│    ├── vam_bridge/     VaM 实时桥                   │
│    ├── video_sync/     视频同步 + Stash/XBVR        │
│    └── extension/      Chrome 扩展                  │
└────────────────────┬───────────────────────────────┘
                     │ Serial/WiFi/BLE
┌────────────────────▼───────────────────────────────┐
│  ESP32 + 6 舵机 + T-wist4                          │
└────────────────────────────────────────────────────┘

建模 (几何/IK/装配/分析/查看器) ⟶ sr6_modeling shim ⟶
                                  3D建模Agent/60-实战_Projects/ORS6_Stewart
```

## 三、铁律 (控制层)

1. **控制与建模分离**: `ors6_hub.py` 不 import 任何建模代码; 真要用走 `sr6_modeling`.
2. **hub.html 仅做控制面板**: 不在 HTML 里做 CAD.
3. **TCode 位置 0-9999** (5000=中位): 全链路统一, 不转义.
4. **D 命令语义**: Hub 虚拟设备 (D0 停止, D1 归位) ≠ TCode 规范 (不要混用).
5. **测试约束**: 改 `tcode/` / `device_bridge.py` / `video_sync/` / `funscript/` 前先 `pytest tests/ -v`.

## 四、建模 API (via shim)

```python
# 旧: from sr6_tools import PARTS      ← 不可用, 文件已删
# 新: from sr6_modeling import PARTS   ← 经 3D建模Agent 转发

from sr6_modeling import (
    PARTS, SR6, HOME_H,                # 数据
    StewartIK, compute_rods,           # 运动学
    verify_assembly,                    # 数值验证
)
```

完整 API 见 `../3D建模Agent/60-实战_Projects/ORS6_Stewart/README.md`.

## 五、水位

| 层 | 内容 | 状态 |
|---|---|---|
| W0 | STL 坐标系 + trimesh 基础设施 | ✅ (归 ORS6_Stewart) |
| W1 | IK 运动学 (firmware 1:1) | ✅ 11/11 PASS |
| W2 | STL 装配精确性 | ✅ 8/8 PASS |
| W3 | 3D 查看器 (HDRI+IK+Explode+Analyze) | ✅ (归 ORS6_Stewart/viewer/) |
| W4 | TCode 控制链路 | ✅ (本项目) |
| W5 | 视频同步 + 设备互联 | ✅ (本项目) |
| W6 | 数值分析引擎 (mass/workspace/clearance) | ✅ (归 ORS6_Stewart/analysis.py) |
| W7 | Hub Connect + 音频驱动 | ✅ (本项目) |
| W8 | 跨项目集成 (pytest + ModelHub) | ✅ |
| W11 | forge_bridge 跨项目归一 (v1) | ✅ 被 v2 超越 |
| **W12** | **ORS6_Stewart 本源归一 (v2)** | **✅ 本次完成** |

## 六、V12 归一成果

- 删除 ORS6 端 ~3500 行建模代码 (sr6_tools/assembly/geometry/analyzer/studio + ors6_freecad_build/ors6_cq_build/freecad_assembly/forge_bridge/sr6_config.js/_fc_*.py)
- 清理 freecad_output/ 1966 MB 冗余输出 (~2GB 归零)
- IK 从三处实现归一为一处 (StewartIK @ ORS6_Stewart/kinematics.py)
- 测试回归: **76/76 pytest PASS** + **8/8 verify_assembly** + **11/11 ik verify**

## 七、向下引用

- **建模本源**: `../3D建模Agent/60-实战_Projects/ORS6_Stewart/_AGENTS.md`
- **建模 API**: `../3D建模Agent/60-实战_Projects/ORS6_Stewart/README.md`
- **SR6 硬件架构 (PDF 逆向)**: `SR6_ARCHITECTURE.md`
- **控制 API 手册**: `_AGENT_GUIDE.md`

---
*v11 归一 · 2026-04-18 · 反者道之动 · 万物并育而不相害*
