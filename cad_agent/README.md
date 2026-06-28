# cad_agent — FreeCAD live-kernel 闭环引擎 (整合自 dao-freecad-agent)

> 道法自然 · 以闭式解为镜. 一条常驻 FreeCAD 内核, 感知→执行→校验闭环, 模型树始终参数化、可再编辑.

整合进万法归一后, 归属 **10-反笙_FreeCAD** 能力域 (live-kernel + 仿真), 与既有
`freecad_backend`/`fc_*` 并存、互不相害. 经 `_paths.py` 五门 sys.path 注入,
任意门下脚本均可 `from cad_agent import new_session`.

## 本源能力 (92 工具)

- **几何/参数**: `solid.*` (box/cylinder/cut/union/common…)、`param.*`
  (body/pad/pocket/revolve/loft/sweep/helical/fillet/chamfer/shell/
  pattern_polar/pattern_linear/mirror…)
- **装配**: `asm.*` (create/add/place/rotate/coaxial/interference/bom/export…)
- **感知/渲染**: `view.*`、`solid.measure`
- **多物理 FEM (CalculiX)** — 每个对解析闭式解校验, 非肉眼:
  | 工具 | 物理 | 闭式解 | 误差 |
  |------|------|--------|------|
  | `fem.spin` | 旋转圆盘离心 | (3+ν)/8·ρω²R² | 0.25% |
  | `fem.buckle` | Euler 屈曲 | π²EI/(KL)² | 0.12% |
  | `fem.thermal` | 受约束杆热应力 | E·α·ΔT | 0.00% |
  | `fem.solve` | 悬臂弯曲屈服 | 6FL/bH² | 0.46% |
  | `fem.modal` | 悬臂一阶固有频率 | (1.875²/2π)·√(EI/ρAL⁴) | 0.36% |

## 入口

```bash
# 33/33 live 套件 (真实内核; 含 modal/高阶建模/CAM挖槽钻孔/多视图剖视图/草图约束/装配质心)
python verify_agent.py

# 五物理验道 (闭式解校验)
python 30-验证_Verify/_verify_fem.py        # 5/5 PASS

# 闭环演示 (四物理 + 应力云图 → output/fem_demo/)
python 50-演示_Demo/demo_fem_closure.py

# 大规模复杂装配实战
python 60-实战_Projects/flanged_coupling.py      # 8 螺栓法兰联轴器 (10 零件, 极阵列 + BOM + FEM)
python 60-实战_Projects/gear_reducer_2stage.py   # 两级齿轮减速箱 (7 零件, 传动比5.0 闭式)
python 60-实战_Projects/spring_in_housing.py     # 弹簧-缸套子装配 (螺旋扫掠 + 间隙装配)
python 60-实战_Projects/full_pipeline.py         # 全链路: 设计→STEP/STL/图纸→CAM刀路→FEM→BOM
python 60-实战_Projects/bolted_flange_stack.py   # 24螺栓法兰接头 (51 零件, 宽相剔除 93%)
python 60-实战_Projects/motion_clearance_sweep.py # 机构运动扫描间隙校核 (全360°碰撞谱 vs 闭式)
```

需真实 FreeCAD 1.0 (`FREECADCMD` 指向 `freecadcmd.exe`); CalculiX `ccx` 随 FreeCAD 自带.
`DAO_MOCK=1` 可在无 FreeCAD 环境跑解析 mock 单测 (`pytest tests/`).
