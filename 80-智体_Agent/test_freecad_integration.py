#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_freecad_integration.py — FreeCAD 后端端到端自检 · 看→动→验 闭环之证
═══════════════════════════════════════════════════════════════════════════════
道法自然 — 如 Cursor 之于 VS Code: 站在成熟的 FreeCAD 之上演化, 而非从零另起.

本自检证明: 同一套 AgentSession (perceive→act→verify→undo) 把引擎从 mesh 换成
真实 FreeCAD BREP 内核后, 一字不改即可驱动 —— 此即 "万法归一, 引擎可换".

链路:
    act     用 solid.* 工具在 FreeCAD 里建一个法兰板 (底板 ∪ 立筋 − 两孔)
    verify  以 BREP 精确度量断言 体积/水密/尺寸/孔间距
    perceive用 perception 软渲染 "看见" FreeCAD 造的几何 (证明跨引擎互通)
    undo    回退快照, 证明撤销语义与 mesh 后端一致
    export  落地 STEP, 证明可交付真实 CAD 格式

退出码 0 = 全过; 非 0 = 有失败.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cad_agent import build_freecad_registry  # noqa: E402
from cad_agent.session import AgentSession, Check  # noqa: E402
from cad_agent.backends.freecad_backend import find_freecadcmd  # noqa: E402

OUT = r"C:\Users\Administrator\fc_work"
PASS, FAIL = "✅", "❌"
_fails = []


def expect(cond: bool, msg: str) -> None:
    print(f"  {PASS if cond else FAIL} {msg}")
    if not cond:
        _fails.append(msg)


def main() -> int:
    exe = find_freecadcmd()
    print("freecadcmd:", exe)
    if not exe:
        print(f"{FAIL} 未找到 FreeCAD; 跳过 (本机未安装)")
        return 0  # 无 FreeCAD 的环境不算失败 (mesh 后端仍是零依赖参考实现)

    os.makedirs(OUT, exist_ok=True)
    reg = build_freecad_registry()
    s = AgentSession("freecad_e2e", registry=reg)
    print("引擎无关择取 →  measure:", s._measure_tool, " perceive:", s._perceive_tool)
    expect(s._measure_tool == "solid.measure", "会话自动择取 solid.measure")
    expect(s._perceive_tool == "solid.perceive", "会话自动择取 solid.perceive")

    # ── act: 法兰 = 底板 ∪ 立筋 − 两孔 ─────────────────────────────────────
    print("\n[act] 在 FreeCAD 里建法兰板 …")
    plan = [
        {"tool": "solid.box", "args": {"x": 60, "y": 40, "z": 6, "center": [0, 0, 3], "name": "base"}},
        {"tool": "solid.box", "args": {"x": 6, "y": 40, "z": 30, "center": [-27, 0, 15], "name": "rib"}},
        {"tool": "solid.boolean", "args": {"op": "union", "a": "base", "b": "rib",
                                           "result": "flange", "consume": True}},
        {"tool": "solid.cylinder", "args": {"radius": 3, "height": 20, "center": [20, 12, 3], "name": "h1"}},
        {"tool": "solid.cylinder", "args": {"radius": 3, "height": 20, "center": [20, -12, 3], "name": "h2"}},
        {"tool": "solid.boolean", "args": {"op": "difference", "a": "flange", "b": "h1",
                                           "result": "flange", "consume": True}},
        {"tool": "solid.boolean", "args": {"op": "difference", "a": "flange", "b": "h2",
                                           "result": "flange", "consume": True}},
    ]
    out = s.run(plan)
    for o in out["outcomes"]:
        print(("  OK  " if o["ok"] else "  ERR ") + o["tool"] + ("" if o["ok"] else f"  -> {o['error']}"))
    expect(out["ok"], "建模计划全部成功")
    expect(s.workspace.names() == ["flange"], "工作区仅余 flange (consume 生效)")

    # ── verify: BREP 精确断言 ─────────────────────────────────────────────
    print("\n[verify] BREP 精确度量断言 …")
    # 底板 60*40*6=14400, 立筋 6*40*24=5760 (扣去与底板重叠 6*40*6), 两孔 2*π*9*6≈339
    vr = s.verify([
        Check("exists", obj="flange"),
        Check("count", value=1),
        Check("watertight", obj="flange"),
        Check("volume", obj="flange", lo=18000, hi=20500),
        Check("extent", obj="flange", axis=0, lo=59.9, hi=60.1),
        Check("extent", obj="flange", axis=2, lo=29.9, hi=30.1),
    ])
    print(vr.render())
    expect(vr.ok, "全部验证通过")

    # ── perceive: 软渲染 "看见" FreeCAD 几何 (跨引擎互通) ──────────────────
    print("\n[perceive] perception 软渲染 FreeCAD 几何 …")
    pr = s.perceive("flange", resolution=192, out_dir=OUT, save_png=True)
    expect(pr.ok, "感知成功")
    if pr.ok:
        rep = pr.data["report"]
        print("  摘要:", pr.data["summary"].replace("\n", " ")[:200])
        print("  BREP精确: 体积=%s 面积=%s 水密=%s" % (
            rep.get("brep_volume"), rep.get("brep_area"), rep.get("brep_closed")))
        print("  渲染:", list(pr.data["renders"].keys()))
        expect(rep.get("brep_closed") is True, "perception 报告含 BREP 精确水密=真")
        expect(len(pr.data["renders"]) >= 3, "至少 3 个视角渲染")

    # ── undo: 撤销语义 ────────────────────────────────────────────────────
    print("\n[undo] 撤销语义自检 …")
    s.act("solid.box", {"x": 10, "y": 10, "z": 10, "name": "scratch"})
    expect(s.workspace.has("scratch"), "新增 scratch 后存在")
    n_before = len(s.workspace)
    s.undo()
    expect(not s.workspace.has("scratch"), "undo 后 scratch 消失")
    expect(len(s.workspace) == n_before - 1, "undo 后对象数 -1")
    expect(s.workspace.has("flange"), "undo 不伤及 flange")

    # ── feature: 倒角/倒圆 (BREP 特有, 网格难为) — 简单实体上演示 ────────────
    print("\n[feature] BREP 倒圆/倒角 (独立简单实体) …")
    s.act("solid.box", {"x": 20, "y": 20, "z": 20, "name": "cube"})
    rf = s.act("solid.fillet", {"name": "cube", "radius": 3.0})
    expect(rf.ok, "倒圆成功")
    if rf.ok:
        expect(rf.data["volume"] < 8000, "倒圆后体积 < 原立方体 8000")
    s.act("solid.delete", {"name": "cube"})

    # ── export: 落地 STEP ─────────────────────────────────────────────────
    print("\n[export] 导出 STEP …")
    step = os.path.join(OUT, "flange_e2e.step")
    re = s.act("solid.export", {"name": "flange", "path": step})
    expect(re.ok and os.path.exists(step) and os.path.getsize(step) > 0, f"STEP 已落地: {step}")

    # ── 收尾 ──────────────────────────────────────────────────────────────
    reg.freecad_kernel.close()
    print("\n" + ("=" * 60))
    if _fails:
        print(f"{FAIL} 失败 {len(_fails)} 项:")
        for m in _fails:
            print("   -", m)
        return 1
    print(f"{PASS} FreeCAD 后端端到端全过 — 看→动→验→撤销→导出 闭环成立")
    print("   同一 AgentSession 逻辑, 引擎从 mesh 换为 FreeCAD, 一字未改.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
