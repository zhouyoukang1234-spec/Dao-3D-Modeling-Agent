#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
session.py — 智体建模会话 · perceive→act→verify 闭环本源
═══════════════════════════════════════════════════════════════════════════════
反者道之动 — 不追求 "一次生成完美模型", 从 "看→动→验→再看" 的螺旋收敛出发.
无为而无不为 — 每个变更前自动快照 (可撤销); 每步留痕 (可回放/审计).

这与代码 agent 的会话同构:
    代码: 读上下文 → 编辑/运行 → 测试/诊断 → 再编辑
    几何: perceive   → act(工具)  → verify(断言) → 再 act

AgentSession 是 LLM (或脚本、或 MCP 外部驱动) 的统一操作面:
    s.perceive(name)            看懂某对象
    s.act(tool, args)           执行一个工具 (变更自动快照)
    s.verify([checks...])       对当前工作区跑断言, 出 ✅/⚠️/❌
    s.undo()                    回到上一个快照
    s.run(plan)                 顺序执行一串步骤, 自动 verify
    s.trajectory                全过程留痕
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from .tools import ToolRegistry, ToolResult, Workspace

__all__ = ["AgentSession", "Check", "VerifyReport"]

# 验证标记 (与仓内 dao_verifier 风格一致)
PASS, WARN, FAIL = "✅", "⚠️", "❌"


# ═══════════════════════════════════════════════════════════════════════════
# 验证: 声明式断言, 对当前工作区求值
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class Check:
    """一条声明式断言. kind 决定语义:
        exists       obj 存在
        not_exists   obj 不存在
        watertight   obj 水密
        volume       obj 体积 ∈ [lo, hi]
        extent       obj 某轴尺寸 ∈ [lo, hi]   (需 axis: 0/1/2)
        count        工作区对象数 == value
        min_distance obj 到 other 的最小间距 ∈ [lo, hi]
        custom       自定义 fn(workspace) -> (ok: bool, msg: str)
    """
    kind: str
    obj: Optional[str] = None
    other: Optional[str] = None
    axis: Optional[int] = None
    lo: Optional[float] = None
    hi: Optional[float] = None
    value: Any = None
    tol: float = 1e-6
    fn: Optional[Callable[[Workspace], Tuple[bool, str]]] = None
    label: str = ""

    def describe(self) -> str:
        if self.label:
            return self.label
        if self.kind in ("exists", "not_exists", "watertight"):
            return f"{self.kind}({self.obj})"
        if self.kind == "volume":
            return f"volume({self.obj})∈[{self.lo},{self.hi}]"
        if self.kind == "extent":
            return f"extent({self.obj},axis={self.axis})∈[{self.lo},{self.hi}]"
        if self.kind == "count":
            return f"count=={self.value}"
        if self.kind == "min_distance":
            return f"min_distance({self.obj},{self.other})∈[{self.lo},{self.hi}]"
        return self.kind


@dataclass
class VerifyReport:
    results: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def passed(self) -> int:
        return sum(1 for r in self.results if r["mark"] == PASS)

    @property
    def failed(self) -> int:
        return sum(1 for r in self.results if r["mark"] == FAIL)

    @property
    def ok(self) -> bool:
        return self.failed == 0

    def render(self) -> str:
        lines = [f"{r['mark']} {r['check']}" + (f" — {r['detail']}" if r.get("detail") else "")
                 for r in self.results]
        lines.append(f"— {self.passed}/{len(self.results)} 通过"
                     + ("" if self.ok else f", {self.failed} 失败"))
        return "\n".join(lines)


# ═══════════════════════════════════════════════════════════════════════════
# 会话
# ═══════════════════════════════════════════════════════════════════════════
class AgentSession:
    def __init__(self, name: str = "session", registry: Optional[ToolRegistry] = None,
                 workspace: Optional[Workspace] = None, max_undo: int = 64,
                 measure_tool: Optional[str] = None,
                 perceive_tool: Optional[str] = None) -> None:
        self.name = name
        self.registry = registry or ToolRegistry()
        self.workspace = workspace or Workspace(name=name + "_ws")
        self.trajectory: List[Dict[str, Any]] = []
        self._undo: List[Dict[str, Any]] = []
        self._max_undo = max_undo
        self._t0 = time.time()
        # 引擎无关: 验证/感知所用的工具名按 registry 自动择取 (solid.* 优先于 mesh.*),
        # 也可显式指定. 这样同一会话逻辑既能驱动 mesh 后端, 也能驱动 FreeCAD 后端.
        self._measure_tool = measure_tool or self._pick("measure")
        self._perceive_tool = perceive_tool or self._pick("perceive")

    def _pick(self, verb: str) -> str:
        for ns in ("solid", "mesh"):
            name = f"{ns}.{verb}"
            if self.registry.has(name):
                return name
        return f"mesh.{verb}"  # 缺省回退

    # —— 能力发现 ——
    def tools(self) -> List[Dict[str, Any]]:
        return self.registry.schemas()

    # —— act: 执行一个工具 ——
    def act(self, tool: str, args: Optional[Dict[str, Any]] = None,
            *, record: bool = True) -> ToolResult:
        is_mut = self.registry.has(tool) and self.registry.get(tool).mutates
        if is_mut:
            self._undo.append(self.workspace.snapshot())
            if len(self._undo) > self._max_undo:
                self._undo.pop(0)
        res = self.registry.call(tool, args or {}, self.workspace)
        if not res.ok and is_mut and self._undo:
            # 失败的变更回滚, 不污染状态
            self.workspace.restore(self._undo.pop())
        if record:
            self.trajectory.append({
                "step": len(self.trajectory) + 1,
                "tool": tool, "args": args or {},
                "ok": res.ok,
                "result": res.to_dict(),
                "t": round(time.time() - self._t0, 3),
            })
        return res

    # —— perceive: 看懂一个对象 ——
    def perceive(self, name: str, **kw: Any) -> ToolResult:
        return self.act(self._perceive_tool, {"name": name, **kw}, record=False)

    # —— undo ——
    def undo(self) -> bool:
        if not self._undo:
            return False
        self.workspace.restore(self._undo.pop())
        self.trajectory.append({"step": len(self.trajectory) + 1, "tool": "undo", "ok": True})
        return True

    # —— verify: 对当前工作区跑断言 ——
    def verify(self, checks: List[Check]) -> VerifyReport:
        import numpy as np
        rep = VerifyReport()
        for c in checks:
            mark, detail = FAIL, ""
            try:
                if c.kind == "exists":
                    ok = self.workspace.has(c.obj)
                    mark = PASS if ok else FAIL
                elif c.kind == "not_exists":
                    ok = not self.workspace.has(c.obj)
                    mark = PASS if ok else FAIL
                elif c.kind == "count":
                    n = len(self.workspace)
                    mark = PASS if n == c.value else FAIL
                    detail = f"实得 {n}"
                elif c.kind in ("watertight", "volume", "extent"):
                    m = self.act(self._measure_tool, {"name": c.obj}, record=False)
                    if not m.ok:
                        detail = m.error or "measure 失败"
                    elif c.kind == "watertight":
                        mark = PASS if m.data.get("watertight") else FAIL
                    elif c.kind == "volume":
                        v = m.data.get("volume")
                        if v is None:
                            mark, detail = WARN, "非水密, 无体积"
                        else:
                            mark = PASS if _in(v, c.lo, c.hi) else FAIL
                            detail = f"体积 {v}"
                    elif c.kind == "extent":
                        e = m.data["extents"][c.axis]
                        mark = PASS if _in(e, c.lo, c.hi) else FAIL
                        detail = f"尺寸 {e}"
                elif c.kind == "min_distance":
                    m = self.act(self._measure_tool, {"name": c.obj, "to": c.other}, record=False)
                    d = m.data.get("min_distance_to", {}).get("distance") if m.ok else None
                    if d is None:
                        detail = "无法求间距"
                    else:
                        mark = PASS if _in(d, c.lo, c.hi) else FAIL
                        detail = f"间距 {d}"
                elif c.kind == "custom" and c.fn is not None:
                    ok, detail = c.fn(self.workspace)
                    mark = PASS if ok else FAIL
                else:
                    detail = f"未知 check 类型 '{c.kind}'"
            except Exception as e:  # noqa: BLE001
                mark, detail = FAIL, f"{type(e).__name__}: {e}"
            rep.results.append({"mark": mark, "check": c.describe(), "detail": detail})
        return rep

    # —— run: 顺序执行计划 + 可选 verify ——
    def run(self, plan: List[Dict[str, Any]],
            checks: Optional[List[Check]] = None,
            stop_on_error: bool = True) -> Dict[str, Any]:
        """plan: [{"tool":..., "args":{...}}, ...]. 返回执行摘要 + 可选验证报告."""
        outcomes: List[Dict[str, Any]] = []
        for step in plan:
            res = self.act(step["tool"], step.get("args", {}))
            outcomes.append({"tool": step["tool"], "ok": res.ok,
                             "error": res.error, "data": res.data if res.ok else None})
            if not res.ok and stop_on_error:
                break
        summary = {
            "steps": len(outcomes),
            "ok": all(o["ok"] for o in outcomes),
            "outcomes": outcomes,
            "objects": self.workspace.names(),
        }
        if checks is not None:
            vr = self.verify(checks)
            summary["verify"] = {"ok": vr.ok, "passed": vr.passed,
                                 "failed": vr.failed,
                                 "results": vr.results,
                                 "render": vr.render()}
        return summary

    # —— 状态 ——
    def state(self) -> Dict[str, Any]:
        sl = self.act("scene.list", {}, record=False)
        return {
            "session": self.name,
            "n_objects": len(self.workspace),
            "objects": sl.data.get("objects", []) if sl.ok else [],
            "steps_taken": len([t for t in self.trajectory if t.get("tool") != "undo"]),
            "undo_depth": len(self._undo),
        }


def _in(x: float, lo: Optional[float], hi: Optional[float]) -> bool:
    if lo is not None and x < lo:
        return False
    if hi is not None and x > hi:
        return False
    return True
