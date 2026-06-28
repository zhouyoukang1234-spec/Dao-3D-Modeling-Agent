"""Helical involute gear smoke -- a non-planar tooth feature.

Spur gears so far were a planar profile padded straight up. A helical gear twists
that involute profile along the axis: ``param.helical`` lofts involute
cross-sections rotated by an increasing angle, total twist = W*tan(beta)/rp at the
pitch radius. This exercises a genuinely 3D (non-prismatic) feature.

Checks:
  * the helical body is a valid solid, full face width tall;
  * its volume equals the equivalent spur gear's (Cavalieri: twisting a constant
    cross-section preserves volume) -- proof the twist didn't corrupt the solid;
  * the reported twist matches W*tan(beta)/rp;
  * a meshing pair of OPPOSITE hand, phased by the same closed-form rule, meshes
    to ~0 interference and jams when mis-phased -- the meshing model carries over
    from spur to helical.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
ZA, ZB = 20, 30
W = 14.0
BETA = 20.0                       # pitch helix angle (deg)
A = M * (ZA + ZB) / 2.0           # 50 mm centre distance


def _mesh(s, a, b):
    out = s.act("asm.interference", {"assembly": "Hel"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}
    return cm.get(tuple(sorted((a, b))), 0.0)


def main():
    s = new_session("helical")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # equivalent spur gear, for the Cavalieri volume comparison
    assert s.act("param.body", {"name": "Spur"}).ok
    sp = s.act("param.pad", {"body": "Spur", "feature": "Sf",
                             "profile": {"gear": {"module": M, "teeth": ZA}}, "length": W})
    assert sp.ok, sp.error

    # right-hand helical pinion
    assert s.act("param.body", {"name": "HelA"}).ok
    ha = s.act("param.helical", {"body": "HelA", "feature": "HA", "module": M, "teeth": ZA,
                                 "length": W, "helix_angle": BETA, "hand": "right"})
    assert ha.ok, ha.error
    assert ha.data["valid"], ha.data
    # full face width tall
    assert abs(ha.data["bbox_size"][2] - W) < 1e-3, ha.data["bbox_size"]
    # Cavalieri: helical volume == spur volume (twist preserves cross-section area)
    assert abs(ha.data["volume"] - sp.data["volume"]) / sp.data["volume"] < 0.01, \
        ("twist changed volume", ha.data["volume"], sp.data["volume"])
    # reported twist matches the helix-angle relation
    expect_twist = math.degrees(W * math.tan(math.radians(BETA)) / (M * ZA / 2.0))
    assert abs(ha.data["twist_deg"] - expect_twist) < 1e-6, (ha.data["twist_deg"], expect_twist)
    print("helical pinion: valid solid, h=%.1f, vol=%.1f (spur %.1f), twist=%.2f deg"
          % (ha.data["bbox_size"][2], ha.data["volume"], sp.data["volume"], ha.data["twist_deg"]))

    # left-hand helical gear (opposite hand meshes on parallel axes)
    assert s.act("param.body", {"name": "HelB"}).ok
    assert s.act("param.helical", {"body": "HelB", "feature": "HB", "module": M, "teeth": ZB,
                                   "length": W, "helix_angle": BETA, "hand": "left"}).ok

    # assemble + phase, then verify mesh vs jam
    assert s.act("asm.create", {"name": "Hel"}).ok
    assert s.act("asm.add", {"assembly": "Hel", "body": "HelA", "name": "ha", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "Hel", "body": "HelB", "name": "hb"}).ok
    assert s.act("asm.place", {"name": "hb", "pos": [A, 0, 0]}).ok
    base = meshing_phase_deg(0.0, ZA, ZB)
    assert s.act("asm.rotate", {"name": "hb", "axis": [0, 0, 1],
                                "angle": base, "at": [A, 0, 0]}).ok
    meshed = _mesh(s, "ha", "hb")
    assert meshed < 8.0, ("helical pair jams when phased", meshed)
    print("phased helical pair (opposite hand) meshes: overlap = %.1f mm^3" % meshed)

    # mis-phase half a tooth -> jam
    assert s.act("asm.rotate", {"name": "hb", "axis": [0, 0, 1],
                                "angle": 180.0 / ZB, "at": [A, 0, 0]}).ok
    jam = _mesh(s, "ha", "hb")
    assert jam > 80.0, ("mis-phase should jam", jam)
    print("mis-phased helical pair jams: overlap = %.0f mm^3 (engagement is real)" % jam)
    assert s.act("asm.rotate", {"name": "hb", "axis": [0, 0, 1],
                                "angle": -180.0 / ZB, "at": [A, 0, 0]}).ok

    if "view.render" in s.tools():
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_helical.png")
        rv = s.act("view.render", {"names": ["HelA"], "view": "iso", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("HELICAL SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_helical"):
    main()
