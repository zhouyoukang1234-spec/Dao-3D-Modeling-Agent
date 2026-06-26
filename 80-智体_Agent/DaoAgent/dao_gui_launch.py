#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dao_gui_launch.py — 仅在 GUI 内打开 DaoAgent 对话停靠面板, 然后交还事件循环.
以 `freecad.exe dao_gui_launch.py` 启动: 面板即时可交互 (鼠键/录屏真实操作),
不在启动脚本里跑任何几何, 避免无事件循环时的同步阻塞.
"""
import os
import sys

_ROOT = os.environ.get(
    "DAO_AGENT_ROOT",
    r"C:\Users\Administrator\Dao-3D-Modeling-Agent\80-智体_Agent",
)
_MOD = os.path.join(os.environ.get("APPDATA", ""), "FreeCAD", "Mod", "DaoAgent")
for _p in (_MOD, os.path.join(_ROOT, "DaoAgent"), _ROOT):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import FreeCAD as App  # noqa: E402
import FreeCADGui as Gui  # noqa: E402

try:
    import dao_panel
    dao_panel.DaoDock.show_dock()
    App.Console.PrintMessage("[DAO] 面板已打开, 等待交互.\n")
except Exception:
    import traceback
    App.Console.PrintError("[DAO] 打开面板失败:\n" + traceback.format_exc())
