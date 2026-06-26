#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cad_agent.backends — 各 CAD 引擎后端 (mesh / FreeCAD / SolidWorks…) 同契约接入.

惰性导出: 各后端各有所依 (mesh→trimesh; freecad_live→FreeCAD python), 故按需加载,
避免在缺某依赖的环境 (如 freecadcmd 内无 trimesh) 引入即崩.
"""
__all__ = ["register_mesh_tools", "register_freecad_tools", "register_freecad_live_tools"]


def __getattr__(name):
    if name == "register_mesh_tools":
        from .mesh_backend import register_mesh_tools
        return register_mesh_tools
    if name == "register_freecad_tools":
        from .freecad_backend import register_freecad_tools
        return register_freecad_tools
    if name == "register_freecad_live_tools":
        from .freecad_live import register_freecad_live_tools
        return register_freecad_live_tools
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
