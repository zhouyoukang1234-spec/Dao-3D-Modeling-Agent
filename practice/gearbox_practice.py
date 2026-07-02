"""Practice round 2: cross-module campaign -- parametric bodies, gear train,
CAM drilling, mesh pipeline -- each stage read back through percept.*.
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
    s = new_session("gearbox_practice")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)

    # ---- parametric route: sketch -> pad -> pocket (PartDesign) ----------
    r = s.act("param.body", {"name": "housing"})
    print("param.body:", r.ok, r.data if r.ok else r.error)
    if r.ok:
        r = act(s, "param.pad", {"body": "housing", "plane": "XY",
                                 "profile": {"rect": [80, 50]},
                                 "length": 20})
        print("param.pad:", r.ok, str(r.data if r.ok else r.error)[:150])
        r = act(s, "param.pocket", {"body": "housing", "plane": "XY",
                                    "profile": {"circle": 10},
                                    "length": 12, "offset": 20})
        print("param.pocket:", r.ok, str(r.data if r.ok else r.error)[:150])
        r = s.act("percept.describe", {"object": "housing"})
        print("housing describe:", r.ok,
              str(r.data if r.ok else r.error)[:250])
    # perceive whatever the pad produced (find it in the scene)
    r = act(s, "percept.scene", {"relations": False})
    print("scene after param:", [o["name"] for o in r.data["objects"]])

    # ---- gear train synthesis + perception -------------------------------
    r = s.act("solid.geartrain", {"meshes": [{"driver": 17, "driven": 51}]})
    print("geartrain:", r.ok, str(r.data if r.ok else r.error)[:200])
    if not r.ok:
        defect("solid.geartrain failed: %s" % r.error)

    # ---- CAM: drill cycle on a plate --------------------------------------
    act(s, "solid.box", {"name": "camplate", "length": 60, "width": 40,
                         "height": 8})
    for i, (x, y) in enumerate([(15, 20), (30, 20), (45, 20)]):
        act(s, "solid.cylinder", {"name": "cd%d" % i, "radius": 3,
                                  "height": 8, "pos": [x, y, 0]})
        act(s, "solid.cut", {"a": "camplate", "b": "cd%d" % i,
                             "out": "camplate"})
    r = act(s, "percept.features", {"object": "camplate"})
    lin = [p for p in r.data["patterns"] if p["type"] == "linear_pattern"]
    print("camplate patterns:", r.data["patterns"])
    if not (lin and lin[0]["count"] == 3 and abs(lin[0]["spacing"] - 15) < 1e-3):
        defect("expected linear pattern 3 x r=3 spacing 15, got %s"
               % r.data["patterns"])
    r = s.act("path.job", {"target": "camplate"})
    print("path.job:", r.ok, str(r.data if r.ok else r.error)[:150])
    if r.ok:
        r = s.act("path.drill", {})
        print("path.drill:", r.ok, str(r.data if r.ok else r.error)[:200])
        r = s.act("path.gcode", {"path": os.path.join(out_dir, "drill.ngc")})
        print("path.gcode:", r.ok, str(r.data if r.ok else r.error)[:150])

    # ---- mesh pipeline: solid -> mesh -> analyze -> export ----------------
    r = act(s, "mesh.from_shape", {"name": "camplate", "out": "cammesh"})
    r = act(s, "mesh.analyze", {"name": "camplate"})
    print("mesh.analyze:", str(r.data)[:200])
    r = act(s, "mesh.export", {"name": "camplate",
                               "path": os.path.join(out_dir, "camplate.stl")})
    print("mesh.export:", r.data if r.ok else r.error)

    # ---- percept.diff across a design edit --------------------------------
    act(s, "solid.box", {"name": "rev_a", "length": 60, "width": 40,
                         "height": 8})
    act(s, "solid.box", {"name": "rev_b", "length": 60, "width": 40,
                         "height": 8})
    act(s, "solid.cylinder", {"name": "newhole", "radius": 4, "height": 8,
                              "pos": [50, 30, 0]})
    act(s, "solid.cut", {"a": "rev_b", "b": "newhole", "out": "rev_b"})
    r = act(s, "percept.diff", {"a": "rev_a", "b": "rev_b"})
    gained = [f["type"] for f in r.data["features_gained"]]
    print("rev diff:", r.data["volume_delta"], gained)
    if gained != ["through_hole"]:
        defect("rev diff should read exactly one gained through_hole: %s"
               % gained)

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 2 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
