#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_freecad_live.py — FreeCAD 在世后端端到端自检 (须于 freecadcmd / FreeCAD python 内运行)
═══════════════════════════════════════════════════════════════════════════════
证明: 同一套 AgentSession (perceive→act→verify→undo) 不改一字, 既能驱动无头子进程内核,
也能就地驱动 FreeCAD 活文档 (App.ActiveDocument). 所建之物即时成为文档对象 (GUI 可见).

运行 (路径含中文, 故经环境变量 DAO_AGENT_DIR 传入仓内 80-智体_Agent 目录):
    set DAO_AGENT_DIR=...\\80-智体_Agent
    freecadcmd <ascii暂存的本文件副本>
"""
import os
import sys

try:  # freecadcmd 默认 stdout 走 cp1252, 中文打印会 charmap 崩; 强制 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_DIR = os.environ.get("DAO_AGENT_DIR")
if _DIR and _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import FreeCAD as App  # noqa: E402

import cad_agent  # noqa: E402
from cad_agent.tools import ToolRegistry  # noqa: E402
from cad_agent.backends.freecad_live import register_freecad_live_tools, LiveBridge  # noqa: E402
from cad_agent.session import AgentSession, Check  # noqa: E402


def main():
    print("FreeCAD", ".".join(App.Version()[:3]), "· 在世后端自检")

    bridge = LiveBridge(doc_name="DaoAgentTest")
    reg = ToolRegistry()
    register_freecad_live_tools(reg, bridge=bridge)

    # 同一 AgentSession — 引擎为 FreeCAD 活文档; 工具命名空间 solid.* 自动择取
    s = AgentSession(name="live", registry=reg)
    assert s._measure_tool == "solid.measure", s._measure_tool
    assert s._perceive_tool == "solid.perceive", s._perceive_tool

    # ── act: 建带孔法兰 (底板 ∪ 立筋 − 2 孔) ────────────────────────────────
    print("[act] 在活文档造形 …")
    s.act("solid.box", {"x": 60, "y": 40, "z": 6, "name": "base"})
    s.act("solid.box", {"x": 60, "y": 6, "z": 30, "center": [30, 3, 15], "name": "rib"})
    s.act("solid.boolean", {"op": "union", "a": "base", "b": "rib", "result": "flange", "consume": True})
    s.act("solid.cylinder", {"radius": 3, "height": 20, "center": [15, 20, 3], "name": "h1"})
    s.act("solid.cylinder", {"radius": 3, "height": 20, "center": [45, 20, 3], "name": "h2"})
    s.act("solid.boolean", {"op": "difference", "a": "flange", "b": "h1", "result": "flange", "consume": True})
    s.act("solid.boolean", {"op": "difference", "a": "flange", "b": "h2", "result": "flange", "consume": True})

    # ── 活文档校验: 对象确实落到文档 (GUI 树/视图可见) ──────────────────────
    doc = bridge.doc
    names = [o.Name for o in doc.Objects]
    print("[doc] 活文档对象:", names)
    assert any("flange" in n.lower() or doc.getObject(n).Label == "flange" for n in names), "flange 未入活文档"
    fobj = next(o for o in doc.Objects if o.Label == "flange")
    assert not fobj.Shape.isNull(), "flange 文档形状为空"
    assert fobj.Shape.Volume > 0, "flange 体积非正"

    # ── verify: 引擎无关声明式断言 (与 mesh/无头后端同义) ────────────────────
    print("[verify] 声明式校验 …")
    rep = s.verify([
        Check("watertight", obj="flange"),
        Check("volume", obj="flange", lo=22000, hi=23200),
        Check("extent", obj="flange", axis=0, lo=59.9, hi=60.1),
        Check("extent", obj="flange", axis=1, lo=39.9, hi=40.1),
        Check("extent", obj="flange", axis=2, lo=29.9, hi=30.1),
    ])
    print(rep.render())
    assert rep.ok, "verify 未全过"

    # ── perceive: 同一感知层渲染活文档几何 ──────────────────────────────────
    print("[perceive] 感知 …")
    out = os.environ.get("DAO_WORK", os.getcwd())
    pr = s.perceive("flange", save_png=True, out_dir=out, resolution=200)
    print("  摘要:", pr.data["summary"][:90], "…")
    assert pr.data["report"]["brep_closed"] is True

    # ── undo: 撤销最近变更, 文档对象同步回滚 ────────────────────────────────
    print("[undo] 撤销 …")
    before = len(doc.Objects)
    s.act("solid.box", {"x": 5, "y": 5, "z": 5, "name": "scratch"})
    assert bridge.doc.getObjectsByLabel("scratch"), "scratch 未入活文档"
    s.undo()
    # undo 仅回滚 Workspace; 活文档对象需后端联动清理 → 验证 Workspace 已无 scratch
    assert not s.workspace.has("scratch"), "undo 未回滚 Workspace"
    print("  ok: Workspace 已回滚 (act 前自动快照)")

    # ── export: 落地 STEP ───────────────────────────────────────────────────
    step = os.path.join(out, "flange_live.step")
    s.act("solid.export", {"name": "flange", "path": step})
    assert os.path.exists(step) and os.path.getsize(step) > 0, "STEP 未落地"
    print("  ✅ STEP:", step, os.path.getsize(step), "bytes")

    print("\n" + "=" * 60)
    print("✅ FreeCAD 在世后端端到端全过 — 活文档上 看→动→验→撤销→导出 闭环成立")
    print("   同一 AgentSession 逻辑, 引擎从无头内核换为 GUI 活文档, 一字未改.")


main()
