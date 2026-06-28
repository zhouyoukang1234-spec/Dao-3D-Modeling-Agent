"""Gui command(s) for the DAO workbench."""
import os

import FreeCADGui as Gui

_HERE = os.path.dirname(os.path.realpath(__file__))


class ShowPanel:
    def GetResources(self):
        return {"Pixmap": os.path.join(_HERE, "resources", "dao.svg"),
                "MenuText": "DAO AI 工作台",
                "ToolTip": "显示/聚焦 DAO AI 对话工作台"}

    def Activated(self):
        import dao_panel
        dao_panel.ensure_panel()

    def IsActive(self):
        return True


Gui.addCommand("DAO_ShowPanel", ShowPanel())
