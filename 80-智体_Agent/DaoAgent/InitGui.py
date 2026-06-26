# -*- coding: utf-8 -*-
"""
InitGui.py — DaoAgent 工作台入口 (FreeCAD GUI 加载, 等价于 VS Code 的"插件激活").
═══════════════════════════════════════════════════════════════════════════════
道法自然: 不改 FreeCAD 本体, 仅在其成熟的 Workbench 框架上"挂"一层 AI 对话面板,
如 Cursor 之于 VS Code. 操作从"改代码"变为"改三维模型", 范式不变.

注意: FreeCAD 以 exec(code, globals, locals) 加载本文件 (两字典*不同*), 故
  · 不注入 __file__;
  · 在本文件顶层定义的类体/方法看不到顶层名字.
因此这里只做最简: 配好 sys.path, 再 import 正常模块 dao_workbench 并注册.
"""
import os
import sys

import FreeCAD as App
import FreeCADGui as Gui  # noqa: F401  (供 dao_workbench 顶层基类引用)

# —— 让智体层可被导入 ——
_MOD_DIR = os.path.join(App.getUserAppDataDir(), "Mod", "DaoAgent")
_REPO_AGENT = os.environ.get(
    "DAO_AGENT_ROOT",
    r"C:\Users\Administrator\Dao-3D-Modeling-Agent\80-智体_Agent",
)
for _p in (_MOD_DIR, _REPO_AGENT):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import dao_workbench
    dao_workbench.register()
except Exception:
    import traceback
    App.Console.PrintError("[DaoAgent] 工作台注册失败:\n" + traceback.format_exc())
