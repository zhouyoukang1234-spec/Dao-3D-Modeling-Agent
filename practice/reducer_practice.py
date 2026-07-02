"""Practice round 3: single-stage reducer gearbox -- the whole system alive.

Housing (shelled box with two coaxial bearing bores), stepped shafts seated
by real coaxial mates, kinematics (train value + mobility), measure.* reads,
doc revision diff, a supplementary render, and resource search. Every stage
is perceived structurally before moving on.
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
    s = new_session("reducer_practice")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)

    # ---- housing: box, hollowed, with two coaxial bearing bores -----------
    act(s, "solid.box", {"name": "housing", "length": 120, "width": 80,
                         "height": 70})
    # cavity by shelling (open the top face)
    r = s.act("solid.shell", {"name": "housing", "thickness": 6,
                              "faces": "zmax"})
    print("solid.shell:", r.ok, str(r.data if r.ok else r.error)[:150])
    if not r.ok:
        defect("solid.shell failed: %s" % r.error)
    # input bore (front wall) and output bore (back wall), coaxial along y
    # the shell offsets walls outward: bores must span the *shelled* extent,
    # not the original box, or the shafts hit uncut outer-wall material.
    act(s, "solid.cylinder", {"name": "bore_in", "radius": 12, "height": 100,
                              "pos": [35, -10, 35], "dir": [0, 1, 0]})
    act(s, "solid.cut", {"a": "housing", "b": "bore_in", "out": "housing"})
    act(s, "solid.cylinder", {"name": "bore_out", "radius": 16, "height": 100,
                              "pos": [85, -10, 35], "dir": [0, 1, 0]})
    act(s, "solid.cut", {"a": "housing", "b": "bore_out", "out": "housing"})

    r = act(s, "percept.features", {"object": "housing"})
    fh = [f for f in r.data["features"] if "hole" in f["type"]]
    print("housing holes:", [(f["type"], f["radius"]) for f in fh])
    r = act(s, "percept.describe", {"object": "housing"})
    print("housing:", r.data["description"][:400])

    # section through the bore axis height: see cavity + both bores
    r = act(s, "percept.section", {"object": "housing", "normal": [0, 0, 1],
                                   "offset": 35})
    print("housing z=35 section loops:", r.data["loop_count"])

    # ---- stepped shafts ----------------------------------------------------
    act(s, "solid.cylinder", {"name": "shaft_in", "radius": 11.8,
                              "height": 100, "pos": [35, -10, 35],
                              "dir": [0, 1, 0]})
    act(s, "solid.cylinder", {"name": "step_in", "radius": 18, "height": 30,
                              "pos": [35, 25, 35], "dir": [0, 1, 0]})
    act(s, "solid.union", {"a": "shaft_in", "b": "step_in", "out": "shaft_in"})
    act(s, "solid.cylinder", {"name": "shaft_out", "radius": 15.8,
                              "height": 100, "pos": [85, -10, 35],
                              "dir": [0, 1, 0]})

    # perceive the stepped shaft: step shoulder edge must read concave
    r = act(s, "percept.topology", {"object": "shaft_in"})
    convs = [e.get("convexity") for e in r.data["edges"] if e.get("convexity")]
    print("shaft_in convexity:", {c: convs.count(c) for c in set(convs)})
    if "concave" not in convs:
        defect("stepped shaft shoulder should show a concave edge: %s" % convs)

    # ---- relations & clearances -------------------------------------------
    parts = ["housing", "shaft_in", "shaft_out"]
    r = act(s, "percept.relations", {"objects": parts})
    for rel in r.data["relations"]:
        print("rel:", rel)
    r = act(s, "solid.interference", {"names": parts})
    print("interference:", r.data)
    if r.data["interfering"]:
        defect("reducer parts must not interfere: %s" % r.data["pairs"])
    r = act(s, "solid.clearance", {"a": "housing", "b": "shaft_in"})
    print("clearance housing/shaft_in:", r.data if r.ok else r.error)

    # ---- kinematics: ratio + mobility --------------------------------------
    r = act(s, "solid.geartrain", {"meshes": [{"driver": 20, "driven": 60}]})
    print("geartrain 20:60:", r.data)
    if abs(r.data["reduction"] - 3.0) > 1e-9:
        defect("20:60 mesh must reduce 3:1, got %s" % r.data)
    r = s.act("solid.spatial_mobility", {
        "joints": [{"type": "revolute"}, {"type": "revolute"}],
        "links": 3})
    print("spatial_mobility:", r.ok, str(r.data if r.ok else r.error)[:200])

    # ---- measure.* reads ----------------------------------------------------
    r = act(s, "measure.volume", {"object": "housing"})
    print("measure.volume:", r.data)
    r = act(s, "measure.com", {"object": "shaft_in"})
    print("measure.com:", r.data)
    r = act(s, "measure.plane_distance", {"a": "shaft_in", "b": "shaft_out"})
    print("plane_distance:", r.data if r.ok else r.error)

    # ---- doc revision diff --------------------------------------------------
    rev1 = os.path.join(out_dir, "reducer_rev1.FCStd")
    act(s, "doc.save", {"path": rev1})
    # design change: enlarge output bore
    act(s, "solid.cylinder", {"name": "rebore", "radius": 18, "height": 100,
                              "pos": [85, -10, 35], "dir": [0, 1, 0]})
    act(s, "solid.cut", {"a": "housing", "b": "rebore", "out": "housing"})
    rev2 = os.path.join(out_dir, "reducer_rev2.FCStd")
    act(s, "doc.save", {"path": rev2})
    r = s.act("doc.diff", {"a": rev1, "b": rev2})
    print("doc.diff:", r.ok, str(r.data if r.ok else r.error)[:250])

    # ---- supplementary optics (view.*) --------------------------------------
    r = s.act("view.render", {"path": os.path.join(out_dir, "reducer.png")})
    print("view.render:", r.ok, str(r.data if r.ok else r.error)[:150])

    # ---- resource discovery (network; tolerate offline) ---------------------
    r = s.act("resource.search", {"query": "gear"})
    print("resource.search:", r.ok, str(r.data if r.ok else r.error)[:200])

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 3 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
