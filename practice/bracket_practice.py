"""Practice round 6: machined bracket -- draft/fem/path/doc versioning alive.

A rib-stiffened mounting bracket with a polar bolt circle (draft.polar_array),
FEM static check, CAM pocket + G-code, saved revisions compared by doc.diff,
all validated structurally with percept.*.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session

DEFECTS = []


def defect(msg):
    DEFECTS.append(msg)
    print("DEFECT:", msg)


def act(s, tool, args, expect_ok=True):
    r = s.act(tool, args)
    if r.ok != expect_ok:
        defect("%s %s -> %s" % (tool, args, r.error or r.data))
    return r


def main():
    s = new_session("bracket_practice")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)

    # ---- bracket body: base plate + boss, polar bolt circle ---------------
    act(s, "solid.box", {"name": "bracket", "length": 100, "width": 100,
                         "height": 12})
    act(s, "solid.cylinder", {"name": "boss", "radius": 22, "height": 30,
                              "pos": [50, 50, 12]})
    act(s, "solid.union", {"a": "bracket", "b": "boss", "out": "bracket"})
    act(s, "solid.cylinder", {"name": "hole0", "radius": 4, "height": 12,
                              "pos": [85, 50, 0]})
    r = act(s, "draft.polar_array", {"name": "hole0", "count": 4,
                                     "center": [50, 50, 0], "out": "holes"})
    print("draft.polar_array:", r.ok, str(r.data if r.ok else r.error)[:200])
    if r.ok:
        act(s, "solid.cut", {"a": "bracket", "b": "holes", "out": "bracket"})

    # percept: the bolt circle must read back as a circular pattern
    r = act(s, "percept.features", {"object": "bracket"})
    if r.ok:
        pats = r.data.get("patterns", [])
        print("patterns:", pats)
        if not any(p.get("count") == 4 and p.get("of") == "through_hole"
                   for p in pats):
            defect("bolt circle should be perceived as a circular pattern of "
                   "4 through holes: %s" % pats)

    # ---- save rev A, modify, diff ------------------------------------------
    rev_a = os.path.join(out_dir, "bracket_a.FCStd")
    act(s, "doc.save", {"path": rev_a})
    act(s, "solid.fillet", {"name": "bracket", "radius": 2})
    rev_b = os.path.join(out_dir, "bracket_b.FCStd")
    act(s, "doc.save", {"path": rev_b})
    r = act(s, "doc.diff", {"a": rev_a, "b": rev_b})
    print("doc.diff:", str(r.data if r.ok else r.error)[:250])

    # ---- FEM static check ----------------------------------------------------
    r = act(s, "fem.setup", {"target": "bracket", "material": "aluminum"})
    print("fem.setup:", r.ok, str(r.data if r.ok else r.error)[:150])
    if r.ok:
        act(s, "fem.fix", {"select": {"axis": "z", "side": "min"}})
        act(s, "fem.load", {"select": {"axis": "z", "side": "max"},
                            "kind": "force", "value": 500,
                            "direction": [0, 0, -1]})
        r = act(s, "fem.solve", {"allowable_mpa": 90})
        print("fem.solve:", str(r.data if r.ok else r.error)[:250])

    # ---- CAM: pocket the boss top, emit G-code -------------------------------
    r = act(s, "path.job", {"target": "bracket", "tool_diameter": 6})
    print("path.job:", r.ok, str(r.data if r.ok else r.error)[:150])
    if r.ok:
        r = act(s, "path.drill", {"select": {"axis": "z", "side": "min"},
                                  "diameter": 8})
        print("path.drill:", r.ok, str(r.data if r.ok else r.error)[:200])
        r = act(s, "path.gcode", {"path": os.path.join(out_dir,
                                                       "bracket.nc")})
        print("path.gcode:", r.ok, str(r.data if r.ok else r.error)[:200])

    # ---- final structural audit ----------------------------------------------
    r = act(s, "percept.describe", {"object": "bracket"})
    print("bracket:", str(r.data.get("description", ""))[:300])

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 6 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
