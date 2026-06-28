"""Complete planetary gear set smoke -- with an internal RING gear, correctly PHASED.

Two boundaries this run pushed into:

1. The gear generator only made external spur gears, so a true planetary was
   impossible. ``internal: true`` now builds a ring gear (each external involute
   flank reflected radially about the pitch circle, padded as an annulus).

2. A gear pair at the right centre distance can still be assembled *jammed* (tooth
   tip on tooth tip) instead of *meshed* (teeth interleaving). The static overlap
   is large when jammed and ~0 when truly meshed, so the assembly must PHASE each
   gear (``_gearmath.meshing_phase_deg``) to interleave it. This test asserts the
   phased planets mesh BOTH the sun and the ring to ~0 interference, and -- to
   prove the engagement is real and not mere clearance -- that mis-phasing a
   planet by half a tooth makes it JAM against both.

Standard planetary tooth count z_ring = z_sun + 2*z_planet keeps every gear at the
same module; planets sit on radius a = m*(z_sun+z_planet)/2 = rp_ring - rp_planet.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
Z_S, Z_P = 24, 18
Z_R = Z_S + 2 * Z_P              # 60
A = M * (Z_S + Z_P) / 2.0       # 42
GW = 10.0
HALF_TOOTH = 180.0 / Z_P        # half a planet tooth pitch (deg)


def _overlaps(s, a, b):
    out = s.act("asm.interference", {"assembly": "Plan"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}
    return cm.get(tuple(sorted((a, b))), 0.0)


def main():
    s = new_session("planetary")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # internal ring gear (annulus with a toothed bore)
    assert s.act("param.body", {"name": "Ring"}).ok
    rr = s.act("param.pad", {"body": "Ring", "feature": "Ringf",
                             "profile": {"gear": {"module": M, "teeth": Z_R, "internal": True}},
                             "length": GW})
    assert rr.ok, rr.error
    assert rr.data["volume"] > 5000.0, rr.data
    print("ring gear: internal z=%d, annulus volume = %.0f mm^3" % (Z_R, rr.data["volume"]))

    # sun + 3 planets (external)
    for nm, z in [("Sun", Z_S)] + [("Planet%d" % i, Z_P) for i in range(3)]:
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"gear": {"module": M, "teeth": z}},
                                   "length": GW}).ok

    # assemble: ring fixed, sun at centre (phase 0), planets at 120 deg, each
    # PHASED to mesh the sun and ring.
    assert s.act("asm.create", {"name": "Plan"}).ok
    assert s.act("asm.add", {"assembly": "Plan", "body": "Ring",
                             "name": "ring", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "Plan", "body": "Sun", "name": "sun", "fixed": True}).ok
    centres = []
    for i in range(3):
        assert s.act("asm.add", {"assembly": "Plan", "body": "Planet%d" % i,
                                 "name": "planet%d" % i}).ok
        beta = 120.0 * i
        th = math.radians(beta)
        x, y = A * math.cos(th), A * math.sin(th)
        centres.append((x, y))
        assert s.act("asm.place", {"name": "planet%d" % i, "pos": [x, y, 0]}).ok
        phi = meshing_phase_deg(beta, Z_S, Z_P)
        assert s.act("asm.rotate", {"name": "planet%d" % i, "axis": [0, 0, 1],
                                    "angle": phi, "at": [x, y, 0]}).ok

    # phased -> every planet truly meshes (interleaves) sun AND ring: ~0 interference
    for i in range(3):
        assert _overlaps(s, "sun", "planet%d" % i) < 8.0, ("planet %d jams sun" % i)
        assert _overlaps(s, "ring", "planet%d" % i) < 8.0, ("planet %d jams ring" % i)
    print("all 3 planets phased -> mesh sun AND ring at ~0 interference (true mesh)")

    # engagement proof: mis-phase planet0 by half a tooth -> it JAMS both
    x0, y0 = centres[0]
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": HALF_TOOTH, "at": [x0, y0, 0]}).ok
    jam_sun = _overlaps(s, "sun", "planet0")
    jam_ring = _overlaps(s, "ring", "planet0")
    assert jam_sun > 80.0 and jam_ring > 80.0, ("mis-phase should jam", jam_sun, jam_ring)
    print("mis-phased planet0 jams: sun=%.0f ring=%.0f mm^3 (engagement is real)"
          % (jam_sun, jam_ring))
    # restore the mesh
    assert s.act("asm.rotate", {"name": "planet0", "axis": [0, 0, 1],
                                "angle": -HALF_TOOTH, "at": [x0, y0, 0]}).ok
    assert _overlaps(s, "sun", "planet0") < 8.0

    # sun must sit clear inside the ring
    assert _overlaps(s, "ring", "sun") < 1.0, "sun fouls the ring"
    print("sun clear of ring")

    bom = s.act("asm.bom", {"assembly": "Plan", "density": 0.00785})
    assert bom.data["component_count"] == 5, bom.data
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))

    if "view.render" in s.tools():
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "_out", "smoke_planetary.png")
        rv = s.act("view.render", {"assembly": "Plan", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("PLANETARY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_planetary"):
    main()
