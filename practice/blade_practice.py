"""Practice round 5: turbine blade freeform -- surface/mesh/points/ss alive.

A ruled blade surface lofted between two profile wires, thickened to a solid,
meshed, decimated, reversed from a point cloud, with a spreadsheet driving a
hub dimension. Everything is perceived structurally at each stage.
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
    s = new_session("blade_practice")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)

    # ---- blade: ruled surface between root and tip profiles ---------------
    r = act(s, "surface.ruled", {
        "edge1": [[0, 0, 0], [40, 6, 0], [80, 2, 0]],
        "edge2": [[5, 2, 120], [42, 12, 120], [78, 6, 120]],
        "out": "bladesurf"})
    print("surface.ruled:", r.ok, str(r.data if r.ok else r.error)[:150])
    r = act(s, "surface.extrude", {
        "points": [[0, 0, 0], [40, 6, 0], [80, 2, 0]],
        "direction": [0, 4, 120], "out": "bladewall"})
    print("surface.extrude:", r.ok, str(r.data if r.ok else r.error)[:150])

    # ---- spreadsheet drives the hub ----------------------------------------
    r = act(s, "ss.create", {"name": "params", "cells": {"hub_r": 30}})
    print("ss.create:", r.ok, str(r.data if r.ok else r.error)[:150])
    r = act(s, "ss.set", {"alias": "hub_r", "value": 32})
    print("ss.set:", r.ok, str(r.data if r.ok else r.error)[:150])
    act(s, "solid.cylinder", {"name": "hub", "radius": 32, "height": 25})
    r = act(s, "ss.table", {})
    print("ss.table:", str(r.data)[:200])
    if r.ok and float(r.data["table"]["hub_r"]) != 32:
        defect("spreadsheet should read back hub_r=32: %s" % r.data)

    # ---- mesh pipeline -------------------------------------------------------
    r = act(s, "mesh.from_shape", {"source": "hub", "out": "hubmesh",
                                   "linear_deflection": 0.5})
    print("mesh.from_shape:", r.ok, str(r.data if r.ok else r.error)[:200])
    if r.ok:
        r = act(s, "mesh.analyze", {"name": "hubmesh"})
        print("mesh.analyze:", str(r.data)[:250])
        if not r.data.get("watertight", r.data.get("is_solid", True)):
            defect("hub mesh should be watertight: %s" % r.data)
        r = act(s, "mesh.decimate", {"name": "hubmesh", "reduction": 0.5})
        print("mesh.decimate:", r.ok, str(r.data if r.ok else r.error)[:200])
        r = act(s, "mesh.export", {"name": "hubmesh",
                                   "path": os.path.join(out_dir, "hub.stl")})
        print("mesh.export:", r.ok, str(r.data if r.ok else r.error)[:150])

    # ---- point-cloud reverse engineering ------------------------------------
    r = act(s, "points.from_shape", {"source": "hub", "out": "hubcloud"})
    print("points.from_shape:", r.ok, str(r.data if r.ok else r.error)[:200])
    if r.ok:
        r = act(s, "points.reverse", {"cloud": "hubcloud", "out": "refit"})
        print("points.reverse:", r.ok, str(r.data if r.ok else r.error)[:300])

    # ---- reflection: the system reads its own capabilities ------------------
    r = act(s, "reflect.roots", {})
    print("reflect.roots:", str(r.data)[:200])

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 5 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
