#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cad_agent — AI + CAD 通用智体本源
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 无为而无不为.

把 "AI 全程参与三维建模" 拆成与 AI 编程同构的三层:
    perception   三维感知   (看见 + 读懂几何)        ← AI 的 "眼"
    tools        工具协议   (引擎无关的标准动作)      ← AI 的 "手" (MCP-for-CAD)
    session      智体会话   (perceive→act→verify 闭环) ← AI 的 "神"

后端 (backends/) 以同一份工具契约接入任意 CAD 引擎; mesh 后端为零外依赖参考实现.
mcp_server 以 stdio JSON-RPC 把工具集暴露给外部驱动器 (Cursor-like).
"""
from __future__ import annotations

__version__ = "0.1.0"


def build_default_registry():
    """构造默认工具登记处 (当前装载 mesh 后端)."""
    from .tools import ToolRegistry
    from .backends import register_mesh_tools
    reg = ToolRegistry()
    register_mesh_tools(reg)
    return reg


def build_freecad_registry(kernel=None):
    """构造装载 FreeCAD 后端的工具登记处 (真实 BREP 实体, 经 freecadcmd 子进程内核).

    需系统已安装 FreeCAD (freecadcmd 可见, 或设环境变量 FREECADCMD).
    工具命名空间为 solid.*; 与 mesh 后端同义, 故上层会话/感知/MCP 完全复用.
    """
    from .tools import ToolRegistry
    from .backends.freecad_backend import register_freecad_tools
    reg = ToolRegistry()
    register_freecad_tools(reg, kernel=kernel)
    return reg


def build_freecad_live_registry(bridge=None):
    """构造装载 FreeCAD *在世* 后端的工具登记处 (就地操作 App.ActiveDocument 活文档).

    须于 FreeCAD 自身 python 内运行 (GUI 或 freecadcmd); 工具命名空间同为 solid.*,
    与子进程后端同义 —— 故上层会话/感知/MCP 完全复用, 仅 "手" 落在活文档而非子进程.
    """
    from .tools import ToolRegistry
    from .backends.freecad_live import register_freecad_live_tools
    reg = ToolRegistry()
    register_freecad_live_tools(reg, bridge=bridge)
    return reg


def new_session(name: str = "session", engine: str = "mesh"):
    """便捷工厂: 建一个智体会话. engine ∈ {"mesh", "freecad", "freecad-live"}.

    无论何种引擎, 返回的 AgentSession 操作面 (perceive/act/verify/undo/run) 一字不变 ——
    此即 "万法归一": 同一套 看→动→验 闭环, 仅 "手" (后端引擎) 可换.
    """
    from .session import AgentSession
    if engine == "freecad":
        return AgentSession(name=name, registry=build_freecad_registry())
    elif engine == "freecad-live":
        return AgentSession(name=name, registry=build_freecad_live_registry())
    elif engine == "mesh":
        return AgentSession(name=name, registry=build_default_registry())
    raise ValueError("engine 须为 'mesh' / 'freecad' / 'freecad-live', 实得: %r" % engine)


__all__ = ["__version__", "build_default_registry", "build_freecad_registry",
           "build_freecad_live_registry", "new_session"]
