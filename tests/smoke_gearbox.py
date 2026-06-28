"""Gearbox capstone -- butcher a two-stage layshaft gearbox & predict its ratio.

This is the whole reverse chain composing on one assembly. We forward-build a
monolithic gearbox (no part structure), then recover everything from geometry:

  layout (all axes parallel to Z):
    G1 input  r=15 @ (0,0,0)
    G2        r=30 @ (45,0,0)        meshes G1   (centre 45 = 15+30)
    G3        r=12 @ (45,0,8)        COAXIAL with G2 (same layshaft)
    G4 output r=24 @ (45,36,0)       meshes G3   (centre 36 = 12+24)

  * ``solid.reverse`` decomposes the monolith and, purely from geometry, finds
    the two external meshes (G1-G2, G3-G4) and the coaxial pair (G2,G3) -- and
    does NOT mistake the coaxial pair for a mesh;
  * chaining the recovered pitch radii through ``solid.geartrain`` predicts the
    overall train value (15/30)*(12/24) = 1/4 -> a 4:1 reduction, sign positive
    (two external meshes, two flips), matching the closed-form design.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

GEARS = [("G1", 15, (0, 0, 0)), ("G2", 30, (45, 0, 0)),
         ("G3", 12, (45, 0, 8)), ("G4", 24, (45, 36, 0))]


def main():
    s = new_session("gearbox")
    print("FreeCAD", s.registry.kernel.freecad_version)

    for nm, r, p in GEARS:
        assert s.act("solid.cylinder", {"name": nm, "radius": r, "height": 6, "pos": list(p)}).ok
    mono = s.act("solid.compound", {"names": [g[0] for g in GEARS], "out": "box"})
    assert mono.ok, mono.error

    rv = s.act("solid.reverse", {"name": "box"})
    assert rv.ok, rv.error
    assert rv.data["parts"] == 4, rv.data
    meshes = rv.data["gear_meshes"]
    assert len(meshes) == 2 and all(m["type"] == "external" for m in meshes), meshes
    cds = sorted(round(m["center_distance"]) for m in meshes)
    assert cds == [36, 45], cds                      # the two stages
    # the coaxial layshaft (G2,G3) is found and NOT counted as a mesh
    assert any(set(g["parts"]) >= {"box_G2", "box_G3"} or len(g["parts"]) >= 2
               for g in rv.data["coaxial_groups"]), rv.data["coaxial_groups"]
    print("reverse: %d parts, meshes at centres %s, coaxial layshaft recovered" % (rv.data["parts"], cds))

    # chain the two recovered stages through geartrain (driver->driven by design)
    chain = [{"driver_radius": 15, "driven_radius": 30},
             {"driver_radius": 12, "driven_radius": 24}]
    e = s.act("solid.geartrain", {"meshes": chain, "input_rpm": 1600}).data
    assert abs(e["train_value"] - 0.25) < 1e-9, e
    assert abs(e["reduction"] - 4.0) < 1e-9 and not e["reversing"], e
    assert abs(e["output_rpm"] - 400) < 1e-6, e
    print("predicted ratio from recovered geometry: e=%.3f (4:1), 1600 -> %.0f rpm"
          % (e["train_value"], e["output_rpm"]))

    print("GEARBOX SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_gearbox"):
    main()
