# -*- coding: utf-8 -*-
"""
dao_workbench.py — DaoAgent 工作台定义 (作为*正常模块*被 InitGui 导入).
═══════════════════════════════════════════════════════════════════════════════
为何独立成模块: FreeCAD 以 exec(code, g, l) 加载 InitGui.py, 其 globals≠locals,
导致在 InitGui 顶层定义的类体/方法看不到模块级名字 (__file__/_ICON_XPM/App 皆失踪).
把工作台定义放进*被 import 的正常模块*即恢复单一命名空间, 一切名字解析正常.
"""
import FreeCAD as App
import FreeCADGui as Gui

_ICON_XPM = [
    "16 16 3 1",
    "  c None",
    ". c #1b5e20",
    "+ c #66bb6a",
    "                ",
    "      ....      ",
    "    ..++++..    ",
    "   .++....++.   ",
    "  .++.    .++.  ",
    "  .+.      .+.  ",
    " .++.      .++. ",
    " .+.        .+. ",
    " .+.        .+. ",
    " .++.      .++. ",
    "  .+.      .+.  ",
    "  .++.    .++.  ",
    "   .++....++.   ",
    "    ..++++..    ",
    "      ....      ",
    "                ",
]


class DaoAgentWorkbench(Gui.Workbench):
    MenuText = "道 DaoAgent"
    ToolTip = "道法自然 · AI 对话驱动的三维建模 (CAD 界的 Cursor)"
    Icon = _ICON_XPM

    def Initialize(self):
        import DaoCommands  # noqa: F401  注册命令
        cmds = ["Dao_OpenPanel", "Dao_Demo"]
        self.appendToolbar("DaoAgent", cmds)
        self.appendMenu("道 DaoAgent", cmds)
        App.Console.PrintMessage("[DaoAgent] 工作台已初始化.\n")

    def Activated(self):
        try:
            import dao_panel
            dao_panel.DaoDock.show_dock()
        except Exception:
            import traceback
            App.Console.PrintError("[DaoAgent] 打开面板失败:\n" + traceback.format_exc())

    def Deactivated(self):
        pass

    def GetClassName(self):
        return "Gui::PythonWorkbench"


def register():
    """注册工作台 (幂等)."""
    Gui.addWorkbench(DaoAgentWorkbench())
