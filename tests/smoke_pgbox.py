"""Complete planetary GEARBOX smoke -- the full integrated mechanism.

Ties together everything the earlier suites built up to: an internal ring gear
(fixed), a sun gear on an input shaft, three planet gears at 120 deg, three planet
pins, and a carrier disc (the output member). Ten components in one assembly.

It asserts the *entire* clash map is exactly the intended set of gear meshes and
press fits and nothing else -- i.e. every functional contact is present and there
are zero stray collisions anywhere in a 10-body mechanism:

  * sun  <-> planet_i   (external mesh, x3, symmetric)
  * ring <-> planet_i   (internal mesh, x3, symmetric)
  * pin_i <-> planet_i  (planet bearing press fit, x3)
  * carrier <-> pin_i   (pin seated in carrier, x3)
  * inshaft <-> sun     (sun keyed to input shaft, x1)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
Z_S, Z_P = 24, 18
Z_R = Z_S + 2 * Z_P                 # 60
A = M * (Z_S + Z_P) / 2.0          # 42
GW = 10.0
PIN_R = 4.0
SUN_BORE_R = 5.0


def main():
    s = new_session("pgbox")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # gears: internal ring + sun + 3 planets
    assert s.act("param.body", {"name": "Ring"}).ok
    assert s.act("param.pad", {"body": "Ring", "feature": "Rf",
                               "profile": {"gear": {"module": M, "teeth": Z_R, "internal": True}},
                               "length": GW}).ok
    for nm, z in [("Sun", Z_S)] + [("Planet%d" % i, Z_P) for i in range(3)]:
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"gear": {"module": M, "teeth": z}},
                                   "length": GW}).ok
    # carrier disc + 3 planet pins + input shaft
    assert s.act("solid.cylinder", {"name": "Carrier", "radius": A + 8, "height": 4}).ok
    for i in range(3):
        assert s.act("solid.cylinder", {"name": "Pin%d" % i, "radius": PIN_R, "height": 26}).ok
    assert s.act("solid.cylinder", {"name": "InShaft", "radius": SUN_BORE_R, "height": 30}).ok

    # assemble
    assert s.act("asm.create", {"name": "PG"}).ok
    assert s.act("asm.add", {"assembly": "PG", "body": "Ring", "name": "ring", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "PG", "body": "Sun", "name": "sun"}).ok
    assert s.act("asm.add", {"assembly": "PG", "body": "InShaft", "name": "inshaft"}).ok
    assert s.act("asm.place", {"name": "inshaft", "pos": [0, 0, -20]}).ok
    for i in range(3):
        assert s.act("asm.add", {"assembly": "PG", "body": "Planet%d" % i,
                                 "name": "planet%d" % i}).ok
        assert s.act("asm.add", {"assembly": "PG", "body": "Pin%d" % i,
                                 "name": "pin%d" % i}).ok
        beta = 120.0 * i
        th = math.radians(beta)
        x, y = A * math.cos(th), A * math.sin(th)
        assert s.act("asm.place", {"name": "planet%d" % i, "pos": [x, y, 0]}).ok
        assert s.act("asm.place", {"name": "pin%d" % i, "pos": [x, y, -8]}).ok
        # PHASE the planet so it truly meshes the sun and ring (interleaves)
        phi = meshing_phase_deg(beta, Z_S, Z_P)
        assert s.act("asm.rotate", {"name": "planet%d" % i, "axis": [0, 0, 1],
                                    "angle": phi, "at": [x, y, 0]}).ok
    assert s.act("asm.add", {"assembly": "PG", "body": "Carrier", "name": "carrier"}).ok
    assert s.act("asm.place", {"name": "carrier", "pos": [0, 0, GW + 1]}).ok

    out = s.act("asm.interference", {"assembly": "PG"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}

    # phased gears truly mesh -> ~0 interference, so they do NOT show as clashes;
    # what remains must be exactly the press fits and nothing else.
    sun_m = [cm.get(tuple(sorted(("sun", "planet%d" % i))), 0.0) for i in range(3)]
    ring_m = [cm.get(tuple(sorted(("ring", "planet%d" % i))), 0.0) for i in range(3)]
    for i in range(3):
        assert sun_m[i] < 8.0, ("sun-planet%d jams (not phased)" % i, cm)
        assert ring_m[i] < 8.0, ("ring-planet%d jams (not phased)" % i, cm)

    # press fits present
    for i in range(3):
        assert cm.get(tuple(sorted(("pin%d" % i, "planet%d" % i)), ), 0.0) > 100.0, cm
        assert cm.get(tuple(sorted(("carrier", "pin%d" % i))), 0.0) > 50.0, cm
    assert cm.get(("inshaft", "sun"), 0.0) > 100.0, cm

    # the WHOLE clash map must be exactly the press fits -- no stray fouling and
    # (because the gears are correctly phased) no gear-on-gear interpenetration.
    allowed = {("inshaft", "sun")}
    for i in range(3):
        allowed.add(tuple(sorted(("pin%d" % i, "planet%d" % i))))
        allowed.add(tuple(sorted(("carrier", "pin%d" % i))))
    stray = set(cm) - allowed
    assert not stray, ("unexpected collisions in 10-body gearbox", stray)
    print("7 press fits + 6 phased gear meshes (~0), 0 stray collisions across 10 bodies")

    # engagement proof: mis-phase planet0 half a tooth -> it jams sun and ring
    th0 = math.radians(0.0)
    x0, y0 = A * math.cos(th0), A * math.sin(th0)
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": 180.0 / Z_P, "at": [x0, y0, 0]}).ok
    jam = s.act("asm.interference", {"assembly": "PG"})
    jcm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in jam.data["clashes"]}
    assert jcm.get(("planet0", "sun"), 0.0) > 80.0 and jcm.get(("planet0", "ring"), 0.0) > 80.0, jcm
    print("mis-phased planet0 jams sun=%.0f ring=%.0f (engagement is real)"
          % (jcm.get(("planet0", "sun"), 0.0), jcm.get(("planet0", "ring"), 0.0)))
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": -180.0 / Z_P, "at": [x0, y0, 0]}).ok

    bom = s.act("asm.bom", {"assembly": "PG", "density": 0.00785})
    assert bom.data["component_count"] == 10, bom.data
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))

    # reduction ratio of a planetary with fixed ring, sun input, carrier output:
    ratio = 1.0 + float(Z_R) / float(Z_S)
    print("reduction ratio (fixed ring, sun in, carrier out) = 1 + Zr/Zs = %.3f" % ratio)

    if "view.render" in s.tools():
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "_out", "smoke_pgbox.png")
        rv = s.act("view.render", {"assembly": "PG", "view": "iso", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("PGBOX SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_pgbox"):
    main()
