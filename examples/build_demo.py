"""Build a demonstration model through the agent and save it as .FCStd.

Produces a parametric, spreadsheet-driven mounting bracket plus a small
two-part assembly, then writes ``demo.FCStd`` so it can be opened in the
FreeCAD GUI to inspect the live feature tree. Run with system Python (it spawns
the FreeCAD kernel itself):

    FREECADCMD=".../freecadcmd.exe" python examples/build_demo.py [out.FCStd]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session


def build(out_path: str) -> None:
    s = new_session("demo")
    a = s.act

    # --- parametric mounting bracket, dimensions driven by a spreadsheet ---
    a("param.body", {"name": "Bracket"})
    a("param.pad", {"body": "Bracket", "feature": "Plate",
                    "profile": {"rect": [80, 60]}, "length": 8})
    a("param.pocket", {"body": "Bracket", "feature": "BoreA",
                       "profile": {"circle": 6}, "through": True})
    a("param.fillet", {"body": "Bracket", "feature": "Round", "edges": None, "radius": 3})

    a("ss.create", {"cells": {"thickness": 8, "bore": 6, "width": 80, "depth": 60}})
    a("ss.bind", {"param": "Plate.length", "alias": "thickness"})
    a("ss.bind", {"param": "Plate.width", "alias": "width"})
    a("ss.bind", {"param": "Plate.height", "alias": "depth"})
    a("ss.bind", {"param": "BoreA.radius", "alias": "bore"})
    # drive it: thicken the plate and enlarge the bore from the table
    a("ss.set", {"alias": "thickness", "value": 12})
    a("ss.set", {"alias": "bore", "value": 7})

    print("bracket:", a("param.measure", {"body": "Bracket"}).data)
    print("diagnose:", a("param.diagnose", {}).data)

    # --- a second part + assembly stacked on the bracket ---
    a("param.body", {"name": "Boss"})
    a("param.pad", {"body": "Boss", "feature": "BossPad",
                    "profile": {"circle": 14}, "length": 18})
    a("asm.create", {"name": "Demo"})
    a("asm.add", {"name": "bracket", "body": "Bracket", "fixed": True})
    a("asm.add", {"name": "boss", "body": "Boss"})
    a("asm.stack", {"base": "bracket", "top": "boss"})
    print("interference:", a("asm.interference", {}).data)
    print("bom:", a("asm.bom", {"density": 0.00785}).data)

    r = a("doc.save", {"path": out_path})
    print("saved:", r.data)
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "demo.FCStd")
    build(out)
