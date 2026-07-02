"""Practice round 4: pipe-skid plant module -- wire/bop/bim/measure alive.

A skid base plate with a filleted pipe run (wire.* path -> profile checks),
sliced-plate nesting (bop.*), a BIM shed around it (bim.*), dimensional
audit (measure.*), all perceived structurally along the way.
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
    s = new_session("pipeskid_practice")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)

    # ---- pipe routing centerline: L-run with a fillet elbow ---------------
    act(s, "wire.make", {"name": "run", "points": [[0, 0, 100], [400, 0, 100],
                                                   [400, 300, 100]]})
    r = act(s, "wire.fillet", {"wire": "run", "radius": 60})
    print("wire.fillet:", r.ok, str(r.data if r.ok else r.error)[:150])
    r = act(s, "wire.info", {"wire": "run"})
    print("run info:", str(r.data)[:250])

    # pipe cross-section, extrude a flange ring pad from it
    act(s, "wire.circle", {"name": "sect", "radius": 25})
    r = act(s, "wire.extrude", {"wire": "sect", "dir": [0, 0, 8],
                                "out": "ring"})
    print("wire.extrude:", r.ok, str(r.data if r.ok else r.error)[:150])
    r = act(s, "percept.features", {"object": "ring"})
    print("ring features:", [(f["type"], f["radius"])
                             for f in r.data["features"]])

    # ---- skid plate, sliced into nested cut pieces (bop.*) ----------------
    act(s, "solid.box", {"name": "plate", "length": 500, "width": 400,
                         "height": 12})
    act(s, "solid.box", {"name": "knife1", "length": 2, "width": 400,
                         "height": 12, "pos": [249, 0, 0]})
    r = s.act("bop.slice", {"base": "plate", "tools": ["knife1"]})
    print("bop.slice:", r.ok, str(r.data if r.ok else r.error)[:200])
    if not r.ok:
        defect("bop.slice failed: %s" % r.error)
    act(s, "solid.box", {"name": "blockA", "length": 60, "width": 60,
                         "height": 60, "pos": [0, 0, 40]})
    act(s, "solid.box", {"name": "blockB", "length": 60, "width": 60,
                         "height": 60, "pos": [30, 30, 70]})
    r = s.act("bop.xor", {"shapes": ["blockA", "blockB"], "out": "xored"})
    print("bop.xor:", r.ok, str(r.data if r.ok else r.error)[:200])

    # ---- BIM shed around the skid (bim.*) ---------------------------------
    r = s.act("bim.wall", {"name": "w_north", "length": 600, "width": 20,
                           "height": 300})
    print("bim.wall:", r.ok, str(r.data if r.ok else r.error)[:200])
    if r.ok:
        r = act(s, "bim.floor", {"name": "shed_floor",
                                 "members": ["w_north"]})
        print("bim.floor:", r.ok, str(r.data if r.ok else r.error)[:150])
        r = act(s, "bim.building", {"name": "shed",
                                    "members": ["shed_floor"]})
        print("bim.building:", r.ok, str(r.data if r.ok else r.error)[:150])

    # ---- dimensional audit (measure.*) -------------------------------------
    r = act(s, "measure.volume", {"object": "ring"})
    print("ring volume:", r.data)
    exp = 3.141592653589793 * 25 * 25 * 8
    if abs(r.data["volume"] - exp) > exp * 0.01:
        defect("ring volume %s != pi*r^2*h %s" % (r.data["volume"], exp))
    r = act(s, "measure.area", {"object": "plate"})
    print("plate area:", r.data)
    r = act(s, "measure.delta", {"a": "blockA", "b": "blockB"})
    print("measure.delta:", r.data if r.ok else r.error)

    # ---- perceive the whole scene ------------------------------------------
    r = act(s, "percept.scene", {})
    print("scene objects:", len(r.data.get("objects", [])))
    r = act(s, "percept.relations", {"objects": ["blockA", "blockB"]})
    print("blockA/B relation:", r.data["relations"])

    # export the skid plate for fabrication
    r = act(s, "solid.export", {"names": ["plate", "ring"],
                                "path": os.path.join(out_dir, "skid.step")})
    print("solid.export:", r.ok, str(r.data if r.ok else r.error)[:150])

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 4 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
