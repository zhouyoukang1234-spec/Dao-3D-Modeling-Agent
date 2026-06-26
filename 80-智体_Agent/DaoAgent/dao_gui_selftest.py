#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dao_gui_selftest.py — DaoAgent 面板在真实 FreeCAD GUI 内的端到端自检
═══════════════════════════════════════════════════════════════════════════════
以 `freecad.exe dao_gui_selftest.py` 启动: 在 GUI 进程内构造对话停靠面板, 跑一遍
"法兰示例"(底板 ∪ 立筋 − 2 孔 → 感知), 然后:
  · 抓取面板日志 (dock.log) → 文本
  · 列出活文档对象 (模型树即时所见)
  · 令三维视图等轴测+适配并存图 (验证软件 OpenGL 渲染不崩)
全部结果写入 result 文件, 供无界面侧 (shell) 读取核验 —— 不依赖鼠标键盘模拟.

道法自然: 面板只是"框", 底层仍是 perceive→act→verify 同一闭环驱动活文档.
"""
import os
import sys
import json
import traceback

_ROOT = os.environ.get(
    "DAO_AGENT_ROOT",
    r"C:\Users\Administrator\Dao-3D-Modeling-Agent\80-智体_Agent",
)
_MOD = os.path.join(os.environ.get("APPDATA", ""), "FreeCAD", "Mod", "DaoAgent")
for _p in (_MOD, os.path.join(_ROOT, "DaoAgent"), _ROOT):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import FreeCAD as App
import FreeCADGui as Gui

_RESULT = os.path.join(os.path.expanduser("~"), "dao_selftest_result.json")
_VIEWPNG = os.path.join(os.path.expanduser("~"), "dao_selftest_view.png")

out = {"ok": False}
try:
    import dao_panel

    dock = dao_panel.DaoDock.show_dock()
    Gui.updateGui()

    # 跑完整法兰示例 (内部对每行 _dispatch + processEvents)
    dock._run_demo()
    Gui.updateGui()

    out["log"] = dock.log.toPlainText()

    doc = App.ActiveDocument
    out["doc"] = doc.Name if doc else None
    out["objects"] = [{"name": o.Name, "label": o.Label,
                       "type": o.TypeId} for o in (doc.Objects if doc else [])]

    # 智体之眼 (软件光栅器) 预览是否生成
    px = dock.preview.pixmap()
    out["preview_has_pixmap"] = bool(px and not px.isNull())

    # 令真实三维视图渲染并存图 (验证软件 OpenGL)
    try:
        v = Gui.ActiveDocument.ActiveView
        v.viewAxonometric()
        v.fitAll()
        Gui.updateGui()
        v.saveImage(_VIEWPNG, 1024, 768, "White")
        out["view_png"] = _VIEWPNG
        out["view_png_exists"] = os.path.exists(_VIEWPNG)
    except Exception as e:
        out["view_err"] = repr(e)

    out["ok"] = True
except Exception:
    out["error"] = traceback.format_exc()

with open(_RESULT, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)

App.Console.PrintMessage("[DAO SELFTEST] 完成 -> %s\n" % _RESULT)
