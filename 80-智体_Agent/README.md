# 80-智体_Agent · AI + CAD 通用智体层

> 反者道之动 — 不从「替人把某个零件建完」出发，从「像 AI 写代码那样，全程参与三维建模」出发。
> 道法自然 · 无为而无不为。

## 这一层是什么

把「AI 全程参与三维建模」做成一套**通用、引擎无关**的体系——和 `VS Code + Agent 插件 / Cursor`
在代码领域走过的路**同构**：

| AI 编程的演化 | 本层对应物 |
|---|---|
| 源码 → AST / 类型 / 诊断（语言服务器让 agent「看懂」代码） | `perception` 几何 → 多视角渲染 + 结构报告（让 AI「看懂」几何） |
| `read_file / write_file / edit / run`（对文本工作区的标准动作） | `tools` 引擎无关的 CAD 标准动作（图元 / 变换 / 布尔 / 度量 / 感知 / IO） |
| agent 会话：读上下文→改→测→再改 | `session` 智体会话：perceive→act→verify→再 act |
| MCP 工具协议（让任意外部驱动即插即用） | `mcp_server` stdio JSON-RPC，把整套工具暴露给外部驱动器（IDE 插件 / LLM 运行时） |

不锁定任何 CAD 软件。`mesh` 后端是**零外部软件依赖**的参考实现（纯 numpy 软件光栅器 +
trimesh + manifold3d 布尔），在**没有装 FreeCAD / SolidWorks 的环境**里也能跑通完整闭环；
其它引擎只需注册**同名同义**的工具即可被 agent 无差别驱动。

## 结构

```
80-智体_Agent/
├── cad_agent/
│   ├── __init__.py        build_default_registry() / new_session()
│   ├── perception.py      三维感知本源：相机 + 软件光栅器 + describe/perceive  ← AI 的「眼」
│   ├── tools.py           工具协议：Tool / ToolParam / Workspace / ToolRegistry ← MCP-for-CAD
│   ├── session.py         智体会话：AgentSession / Check / VerifyReport         ← AI 的「神」
│   ├── mcp_server.py      stdio JSON-RPC 暴露（MCP 精简子集）                    ← 外部驱动接入
│   └── backends/
│       ├── mesh_backend.py    mesh 引擎后端（trimesh）：零外部软件依赖的参考实现 ← AI 的「手」
│       ├── freecad_kernel.py  FreeCAD 无头几何内核（运行于 freecadcmd 自带 python）
│       └── freecad_backend.py FreeCAD 后端：把真实 BREP 实体能力注入同一工具协议 ← 另一只「手」
├── verify_agent.py             端到端自检（mesh 后端，无外部 CAD，✅/❌ + 退出码）
└── test_freecad_integration.py FreeCAD 后端端到端自检（看→动→验→撤销→导出）
```

## 从 FreeCAD 演化：成熟引擎做「手」，上层一字不改

> 道法自然 — 如 Cursor 之于 VS Code：站在成熟的 FreeCAD 之上演化，而非从零另起。

`freecad` 后端把 FreeCAD 降格为一只**纯函数式 BREP 几何内核**：经 `freecadcmd`
以子进程拉起 `freecad_kernel.py`，stdin/stdout 收发 JSON 行——

```
请求  {"op": "boolean", "args": {"op":"difference", "shapes":{"a": <brep>, "b": <brep>}}}
应答  __FCR__ {"ok": true, "data": {"brep": <新形状>, "mesh": {...}, "metrics": {...}}}
```

**内核不持有状态**：输入形状以 BREP 字符串随调用传入，输出形状以 BREP 字符串返回；
引擎无关的 `Workspace` 全权拥有状态——故快照 / 撤销 / 对比天然成立，FreeCAD 只是
一只可随时替换的「手」。工具命名空间 `solid.*` 与 mesh 后端 `mesh.*` **同义**，
`AgentSession` 自动择取（`solid.*` 优先），于是同一套 perceive→act→verify 闭环
**一字不改**即可从 mesh 切到真实 FreeCAD BREP：

```python
import _paths, cad_agent
s = cad_agent.new_session("demo", engine="freecad")   # ← 仅此一处改动
s.act("solid.box",      {"x": 60, "y": 40, "z": 6, "name": "base"})
s.act("solid.cylinder", {"radius": 3, "height": 20, "center": [20, 12, 3], "name": "h1"})
s.act("solid.boolean",  {"op": "difference", "a": "base", "b": "h1",
                         "result": "flange", "consume": True})
s.act("solid.fillet",   {"name": "flange2", "radius": 2})   # BREP 特有：倒圆/倒角
print(s.perceive("flange").data["summary"])                # 同一感知层「看见」FreeCAD 几何
s.act("solid.export",   {"name": "flange", "path": "flange.step"})  # 落地真实 STEP
```

需系统已安装 FreeCAD（`freecadcmd` 可见，或设环境变量 `FREECADCMD`）。
未安装时 `test_freecad_integration.py` 优雅跳过；mesh 后端仍是零依赖参考实现。

## 快速上手

```python
import _paths            # 注册五层路径，使 cad_agent 可被 import
import cad_agent
from cad_agent.session import Check

s = cad_agent.new_session("demo")          # 装载默认工具集的智体会话

# act：像 AI 调工具一样建一块「带孔法兰板」
s.act("mesh.box",      {"x": 40, "y": 30, "z": 6, "name": "plate"})
s.act("mesh.cylinder", {"radius": 5, "height": 20, "name": "drill"})
s.act("mesh.boolean",  {"op": "difference", "a": "plate", "b": "drill",
                         "result": "flange", "consume": True})

# perceive：让 AI「看懂」结果（结构报告 + 多视角渲染 + 自然语言摘要）
print(s.perceive("flange").data["summary"])

# verify：声明式断言，出 ✅/⚠️/❌
print(s.verify([
    Check("watertight", obj="flange"),
    Check("volume", obj="flange", lo=6000, hi=7000),
]).render())

s.undo()   # 每个变更前自动快照，可撤销
```

## 作为 MCP 工具被外部驱动

```bash
# 自检（内置回环，不走 stdio）
python "80-智体_Agent/cad_agent/mcp_server.py" --selftest

# 作为子进程被外部驱动（stdio JSON-RPC，一行一帧）
python "80-智体_Agent/cad_agent/mcp_server.py"
```

```jsonc
→ {"jsonrpc":"2.0","id":1,"method":"initialize"}
→ {"jsonrpc":"2.0","id":2,"method":"tools/list"}
→ {"jsonrpc":"2.0","id":3,"method":"tools/call",
    "params":{"name":"mesh.box","arguments":{"x":10,"y":10,"z":10}}}
→ {"jsonrpc":"2.0","id":4,"method":"perceive","params":{"name":"box1"}}
→ {"jsonrpc":"2.0","id":5,"method":"session/verify",
    "params":{"checks":[{"kind":"watertight","obj":"box1"}]}}
```

## 端到端自检

```bash
python "80-智体_Agent/verify_agent.py"               # mesh 后端，全过 → 退出码 0
python "80-智体_Agent/test_freecad_integration.py"   # FreeCAD 后端，全过 → 退出码 0
```

覆盖：感知（尺寸/体积/水密/多视角覆盖率）、工具协议（schema 完整性）、
会话闭环（plan 执行 + 声明式 verify）、撤销语义、失败工具不污染状态。

## 依赖

- 必需：`numpy`、`trimesh`
- 布尔（CSG）：`manifold3d`
- 可选：`Pillow`（渲染落 PNG）

```bash
pip install numpy trimesh manifold3d Pillow
```

## 设计取舍（承接上一对话的教训）

- **不**再为某个特定型号（SR6/OSR6）搞通一条写死的全链路；先把**通用起点**立起来。
- **不**外包给图生 3D / 云服务；以本机可得的几何能力，建**自足、可验证**的 perceive→act→verify 闭环。
- **不**重复造轮子：复用仓内既有的五层路径（`_paths.py`）、闭环（`dao_loop`）、验证风格（`✅/⚠️/❌`）。
- 步步为营：感知 → 工具协议 → 会话 → MCP 暴露，逐层可独立验证，再逐层叠加。
