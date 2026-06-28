"""DAO workbench registration (GUI side).

Loaded automatically by FreeCAD when this addon directory is on the Mod path.
Registers a lightweight workbench whose only job is to surface the dockable AI
panel, and also auto-shows the dock as soon as the GUI is ready so the human/AI
workspace is present regardless of which workbench is active — true fusion, not a
mode you have to switch into.
"""
import os

import FreeCAD as App
import FreeCADGui as Gui

# FreeCAD execs InitGui.py without a module ``__file__``; derive the addon dir
# from the user Mod path instead so imports of our sibling modules resolve.
_HERE = os.path.join(App.getUserAppDataDir(), "Mod", "DAO")
if _HERE not in os.sys.path:
    os.sys.path.insert(0, _HERE)


class DAOWorkbench(Gui.Workbench):
    # NOTE: FreeCAD execs InitGui.py inside a function, so module-level names are
    # function locals invisible to a class body. Keep this body free of external
    # name references (Icon is attached after the class is defined).
    MenuText = "DAO AI"
    ToolTip = "道法自然 · AI-driven modelling fused into FreeCAD"

    def Initialize(self):
        import dao_commands  # noqa: F401  (registers the Gui command)
        self.appendToolbar("DAO", ["DAO_ShowPanel"])
        self.appendMenu("DAO", ["DAO_ShowPanel"])

    def Activated(self):
        import dao_panel
        dao_panel.ensure_panel()

    def GetClassName(self):
        return "Gui::PythonWorkbench"


DAOWorkbench.Icon = os.path.join(_HERE, "resources", "dao.svg")
Gui.addWorkbench(DAOWorkbench())


def _autoshow():
    """Show the dock once the main window exists (deferred from import time)."""
    try:
        import dao_panel
        if dao_panel.ensure_panel() is not None:
            return
    except Exception as exc:
        App.Console.PrintWarning("DAO autoshow retry: %r\n" % (exc,))
    from PySide import QtCore
    QtCore.QTimer.singleShot(800, _autoshow)


try:
    from PySide import QtCore
    QtCore.QTimer.singleShot(1200, _autoshow)
except Exception as exc:
    App.Console.PrintWarning("DAO: could not schedule autoshow: %r\n" % (exc,))
