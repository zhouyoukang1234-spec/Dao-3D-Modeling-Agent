"""Epicyclic (planetary) gear-stage smoke — polar multi-mesh assembly.

A sun gear (z=24) with three planet gears (z=18) equally spaced at 120 deg on the
sun-planet centre circle a = m*(z_s + z_p)/2, held by a carrier disc and three
pins. Every planet meshes the sun simultaneously and the three meshes must be
identical by symmetry; the planets must never collide each other. This exercises
polar placement and several simultaneous gear meshes at once -- the rotational-
symmetry counterpart to the parallel-shaft reducer in smoke_engineering.py.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
Z_SUN, Z_PLANET = 24, 18
A = M * (Z_SUN + Z_PLANET) / 2.0          # 42 mm sun-planet centre distance
GW = 10.0
PLANET_TIP = M * Z_PLANET / 2.0 + M
PIN_R = 5.0


def main():
    s = new_session("epicyclic")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # sun + 3 planet gears
    for nm, z in [("Sun", Z_SUN)] + [("Planet%d" % i, Z_PLANET) for i in range(3)]:
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"gear": {"module": M, "teeth": z}},
                                   "length": GW}).ok
    # carrier disc + 3 planet pins
    assert s.act("solid.cylinder", {"name": "Carrier",
                                    "radius": A + PLANET_TIP + 4, "height": 4}).ok
    for i in range(3):
        assert s.act("solid.cylinder", {"name": "Pin%d" % i,
                                        "radius": PIN_R, "height": 30}).ok

    # assemble
    assert s.act("asm.create", {"name": "Epi"}).ok
    assert s.act("asm.add", {"assembly": "Epi", "body": "Sun",
                             "name": "sun", "fixed": True}).ok
    for i in range(3):
        assert s.act("asm.add", {"assembly": "Epi", "body": "Planet%d" % i,
                                 "name": "planet%d" % i}).ok
        assert s.act("asm.add", {"assembly": "Epi", "body": "Pin%d" % i,
                                 "name": "pin%d" % i}).ok
    assert s.act("asm.add", {"assembly": "Epi", "body": "Carrier", "name": "carrier"}).ok

    # place planets + pins at 120 deg around the sun; PHASE each planet to mesh
    centres = []
    for i in range(3):
        beta = 120.0 * i
        th = math.radians(beta)
        x, y = A * math.cos(th), A * math.sin(th)
        centres.append((x, y))
        assert s.act("asm.place", {"name": "planet%d" % i, "pos": [x, y, 0]}).ok
        assert s.act("asm.place", {"name": "pin%d" % i, "pos": [x, y, -10]}).ok
        phi = meshing_phase_deg(beta, Z_SUN, Z_PLANET)
        assert s.act("asm.rotate", {"name": "planet%d" % i, "axis": [0, 0, 1],
                                    "angle": phi, "at": [x, y, 0]}).ok
    assert s.act("asm.place", {"name": "carrier", "pos": [0, 0, GW]}).ok

    out = s.act("asm.interference", {"assembly": "Epi"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}

    # phased -> every planet truly meshes (interleaves) the sun: ~0 interference
    meshes = [cm.get(tuple(sorted(("sun", "planet%d" % i))), 0.0) for i in range(3)]
    for i, mv in enumerate(meshes):
        assert mv < 8.0, ("planet %d jams sun (not phased)" % i, cm)
    print("3x sun-planet phased mesh = %.1f mm^3 each (true interleave)" % max(meshes))

    # engagement proof: mis-phase planet0 half a tooth -> it jams the sun
    x0, y0 = centres[0]
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": 180.0 / Z_PLANET, "at": [x0, y0, 0]}).ok
    jam = s.act("asm.interference", {"assembly": "Epi"})
    jcm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in jam.data["clashes"]}
    assert jcm.get(("planet0", "sun"), 0.0) > 80.0, ("mis-phase should jam", jcm)
    print("mis-phased planet0 jams sun = %.0f mm^3 (engagement is real)"
          % jcm.get(("planet0", "sun"), 0.0))
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": -180.0 / Z_PLANET, "at": [x0, y0, 0]}).ok

    # planets must never touch each other
    for i in range(3):
        for j in range(i + 1, 3):
            assert tuple(sorted(("planet%d" % i, "planet%d" % j))) not in cm, \
                ("planets collide", i, j, cm)
    print("no planet-planet collision (3 planets clear at 120 deg)")

    bom = s.act("asm.bom", {"assembly": "Epi", "density": 0.00785})
    assert bom.data["component_count"] == 8, bom.data
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))

    if "view.render" in s.tools():
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "_out", "smoke_epicyclic.png")
        # render the gear train by component name (assembled placements) so the
        # sun + 3 planets meshing at 120 deg is visible, not hidden by the carrier
        rr = s.act("view.render", {"names": ["sun", "planet0", "planet1", "planet2"],
                                   "view": "top", "path": o})
        assert rr.ok and rr.data["bytes"] > 5000, rr.data
        print("render -> %s (%d bytes)" % (o, rr.data["bytes"]))

    print("EPICYCLIC SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_epicyclic"):
    main()
