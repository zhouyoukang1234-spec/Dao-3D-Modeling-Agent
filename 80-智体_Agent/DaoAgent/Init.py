# -*- coding: utf-8 -*-
"""
Init.py — DaoAgent 应用级初始化 (GUI 与 freecadcmd 皆加载).
仅把智体层路径并入 sys.path, 不做任何 GUI 动作.
注意: FreeCAD 以 exec() 加载本文件, 不注入 __file__, 故不依赖 __file__.
"""
import os
import sys

import FreeCAD as App

_MOD_DIR = os.path.join(App.getUserAppDataDir(), "Mod", "DaoAgent")
_REPO_AGENT = os.environ.get(
    "DAO_AGENT_ROOT",
    r"C:\Users\Administrator\Dao-3D-Modeling-Agent\80-智体_Agent",
)
for _p in (_MOD_DIR, _REPO_AGENT):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
