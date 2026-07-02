"""Live GUI command layer (L2).

Exercises ``gui.*``: the full ``freecad`` GUI application driven headlessly under
the Qt offscreen platform (no X server / Xvfb). Proves that every menu/toolbar
``Command`` a human could click is invokable over the same session API as the
curated ``solid.*`` operators -- workbench activation, the ~450-command registry,
``Gui.runCommand`` primitive creation *one request at a time* (the case that used
to wedge FreeCAD's command queue), ``Gui.Selection`` management, live command
introspection, and a real ``.FCStd`` save -- plus argument guards.

The GUI process is heavy, so it is booted lazily by the registry on the first
``gui.*`` call and shut down at the end of this suite.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("smoke_gui")
    for op in ("gui.workbenches", "gui.activate", "gui.commands", "gui.run",
               "gui.select", "gui.selection", "gui.clear_selection",
               "gui.objects", "gui.command_info", "gui.recompute", "gui.save"):
        assert op in s.registry.names(), "missing %s" % op
    print("gui ops registered")

    # ---- workbench + command registry (what a human sees in the UI) ------ #
    r = s.act("gui.workbenches", {})
    assert r.ok, r.error
    wbs = r.data["workbenches"]
    assert "PartWorkbench" in wbs and len(wbs) >= 10, (len(wbs),)
    print("workbenches:", len(wbs))

    r = s.act("gui.activate", {"name": "PartWorkbench"})
    assert r.ok and r.data["active"] == "PartWorkbench", r.data

    r = s.act("gui.commands", {})
    assert r.ok and r.data["count"] > 300, r.data
    all_cmds = r.data["count"]
    r = s.act("gui.commands", {"contains": "box"})
    assert r.ok and "Part_Box" in r.data["commands"], r.data
    print("gui commands: %d total, filtered box -> %d" % (all_cmds, r.data["count"]))

    # ---- run menu/toolbar commands ONE AT A TIME (the wedge case) -------- #
    expect = {"Part_Box": 1000.0, "Part_Cylinder": 125.6637,
              "Part_Sphere": 523.5988, "Part_Cone": 293.2153}
    for name, vol in expect.items():
        r = s.act("gui.run", {"name": name})
        assert r.ok, (name, r.error)
        assert r.data["created_count"] == 1, r.data
        made = r.data["created"][0]
        assert made["shape_type"] == "Solid", made
        assert abs(made["volume"] - vol) < 1e-2, (name, made["volume"], vol)
    print("ran 4 GUI primitive commands one-at-a-time; volumes verified")

    r = s.act("gui.objects", {})
    assert r.ok and r.data["count"] == 4, r.data
    names = [o["name"] for o in r.data["objects"]]
    assert names == ["Box", "Cylinder", "Sphere", "Cone"], names

    # ---- Selection layer ------------------------------------------------- #
    r = s.act("gui.select", {"names": ["Box", "Sphere"]})
    assert r.ok and r.data["selected"] == ["Box", "Sphere"], r.data
    r = s.act("gui.selection", {})
    assert r.ok and set(r.data["selected"]) == {"Box", "Sphere"}, r.data
    r = s.act("gui.clear_selection", {})
    assert r.ok and r.data["selected"] == [], r.data
    print("selection add/read/clear ok")

    # ---- live command introspection ------------------------------------- #
    r = s.act("gui.command_info", {"name": "Part_Box"})
    assert r.ok, r.error
    assert (r.data["info"].get("menuText") or "").strip(), r.data
    print("command_info Part_Box menuText:", r.data["info"].get("menuText"))

    # ---- real .FCStd save ------------------------------------------------ #
    path = "/tmp/dao_smoke_gui.FCStd"
    if os.path.exists(path):
        os.remove(path)
    r = s.act("gui.save", {"path": path})
    assert r.ok and r.data["objects"] == 4, r.data
    assert os.path.exists(path) and os.path.getsize(path) > 1000, r.data
    # independently confirm the GUI-authored file is a real, readable document
    from cad_agent.docformat import summarize
    objs = summarize(path)
    assert len(objs) == 4, objs
    print("saved GUI-authored .FCStd (%d bytes); docformat re-read %d objects"
          % (r.data["bytes"], len(objs)))

    # ---- guards ---------------------------------------------------------- #
    r = s.act("gui.run", {"name": "No_Such_Command"})
    assert not r.ok and "no such GUI command" in (r.error or ""), r
    r = s.act("gui.run", {})
    assert not r.ok and "must be a GUI command name" in (r.error or ""), r
    r = s.act("gui.activate", {"name": 123})
    assert not r.ok and "workbench name" in (r.error or ""), r
    r = s.act("gui.select", {"names": ["GhostObject"]})
    assert not r.ok and "no document object" in (r.error or ""), r
    r = s.act("gui.command_info", {"name": "Nope_Nope"})
    assert not r.ok and "no such GUI command" in (r.error or ""), r
    r = s.act("gui.save", {})
    assert not r.ok and "file path" in (r.error or ""), r
    print("guards ok: bad/absent command, bad workbench, ghost select, bad save")

    print("SMOKE OK gui", s.summary())
    s.registry.kernel.shutdown()
    if getattr(s.registry, "gui_kernel", None) is not None:
        s.registry.gui_kernel.shutdown()


if __name__ == "__main__":
    main()
