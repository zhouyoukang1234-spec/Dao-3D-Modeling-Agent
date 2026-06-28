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

## 入口

```bash
# 26/26 live 套件 (真实内核)
python verify_agent.py

# 四物理验道 (闭式解校验)
python 30-验证_Verify/_verify_fem.py        # 4/4 PASS

# 闭环演示 (四物理 + 应力云图 → output/fem_demo/)
python 50-演示_Demo/demo_fem_closure.py

# 大规模复杂装配实战: 8 螺栓法兰联轴器 (10 零件, 极阵列 + BOM + FEM)
python 60-实战_Projects/flanged_coupling.py
```

需真实 FreeCAD 1.0 (`FREECADCMD` 指向 `freecadcmd.exe`); CalculiX `ccx` 随 FreeCAD 自带.
`DAO_MOCK=1` 可在无 FreeCAD 环境跑解析 mock 单测 (`pytest tests/`).
