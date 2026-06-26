# -*- coding: utf-8 -*-
"""
DaoCommands.py — DaoAgent 命令 (FreeCAD Command 框架, 等价于 IDE 的命令面板项).
"""
import FreeCAD as App
import FreeCADGui as Gui


class _OpenPanel:
    def GetResources(self):
        return {
            "MenuText": "打开对话面板",
            "ToolTip": "打开 DaoAgent 对话停靠面板 (CAD 界的 Cursor 聊天框)",
        }

    def IsActive(self):
        return True

    def Activated(self):
        import dao_panel
        dao_panel.DaoDock.show_dock()


class _RunDemo:
    def GetResources(self):
        return {
            "MenuText": "法兰示例",
            "ToolTip": "一键演示: 底板 ∪ 立筋 − 2 孔 → 感知",
        }

    def IsActive(self):
        return App.ActiveDocument is not None or True

    def Activated(self):
        import dao_panel
        dock = dao_panel.DaoDock.show_dock()
        dock._run_demo()


Gui.addCommand("Dao_OpenPanel", _OpenPanel())
Gui.addCommand("Dao_Demo", _RunDemo())
