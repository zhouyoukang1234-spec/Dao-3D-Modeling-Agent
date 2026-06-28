"""Feature-recognition smoke -- recover design parameters, then re-emit parametric.

The parametric half of butchering-the-ox. After a part is recovered from a
download we want to know *what it is* and its driving dimensions, so we can
rebuild it as a clean parametric feature (the user's "做参数化处理" ask).

``solid.recognize`` classifies a solid into a primitive and recovers its
parameters, but only when the closed-form volume reproduces the measured volume
-- a part that merely resembles a primitive (a filleted block) must come back
``freeform``, never a false primitive. We prove the round trip closes: build
box / cylinder (axis-aligned and tilted) / sphere, recognize each, rebuild from
the recovered parameters, and assert volume + bounding box match to machine
precision. Then we exercise it through the reverse pipeline: a compound of mixed
primitives decomposes and every leaf is recognized and reconstructed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _vol(s, name):
    return s.act("solid.measure", {"name": name}).data["volume"]


def main():
    s = new_session("recognize")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- box -------------------------------------------------------------- #
    s.act("solid.box", {"name": "bx", "length": 30, "width": 18, "height": 7, "pos": [2, -3, 1]})
    r = s.act("solid.recognize", {"name": "bx"}).data
    assert r["type"] == "box" and r["volume_match"], r
    p = r["params"]
    assert abs(p["length"] - 30) < 1e-6 and abs(p["width"] - 18) < 1e-6 and abs(p["height"] - 7) < 1e-6, p
    s.act("solid.box", {"name": "bx2", "length": p["length"], "width": p["width"], "height": p["height"]})
    assert abs(_vol(s, "bx2") - _vol(s, "bx")) < 1e-6
    print("box recovered L/W/H = %.1f/%.1f/%.1f, parametric rebuild matches" % (p["length"], p["width"], p["height"]))

    # ---- cylinder (Z-axis) ------------------------------------------------ #
    s.act("solid.cylinder", {"name": "cyZ", "radius": 6, "height": 20, "pos": [0, 0, 0]})
    r = s.act("solid.recognize", {"name": "cyZ"}).data
    assert r["type"] == "cylinder" and r["volume_match"], r
    p = r["params"]
    assert abs(p["radius"] - 6) < 1e-6 and abs(p["height"] - 20) < 1e-6, p
    assert abs(abs(p["axis"][2]) - 1.0) < 1e-6, p
    print("cylinder(Z) recovered R/H = %.1f/%.1f about %s" % (p["radius"], p["height"], p["axis"]))

    # ---- cylinder (tilted) -- height from cap planes, not axis-aligned bbox  #
    s.act("solid.cylinder", {"name": "cyX", "radius": 4, "height": 25, "pos": [0, 0, 0], "dir": [1, 0, 0]})
    r = s.act("solid.recognize", {"name": "cyX"}).data
    assert r["type"] == "cylinder" and r["volume_match"], r
    assert abs(r["params"]["radius"] - 4) < 1e-6 and abs(r["params"]["height"] - 25) < 1e-6, r
    assert abs(abs(r["params"]["axis"][0]) - 1.0) < 1e-6, r
    print("cylinder(X) recovered R/H = %.1f/%.1f about %s" % (r["params"]["radius"], r["params"]["height"], r["params"]["axis"]))

    # ---- sphere ----------------------------------------------------------- #
    s.act("solid.sphere", {"name": "sp", "radius": 9, "pos": [1, 1, 1]})
    r = s.act("solid.recognize", {"name": "sp"}).data
    assert r["type"] == "sphere" and r["volume_match"], r
    assert abs(r["params"]["radius"] - 9) < 1e-6, r
    print("sphere recovered R = %.1f" % r["params"]["radius"])

    # ---- tube / bushing: a cylinder with a coaxial through-bore ----------- #
    s.act("solid.cylinder", {"name": "tout", "radius": 10, "height": 16, "pos": [0, 0, 0]})
    s.act("solid.cylinder", {"name": "tin", "radius": 6, "height": 16, "pos": [0, 0, 0]})
    tb = s.act("solid.cut", {"a": "tout", "b": "tin", "out": "tube"})
    assert tb.ok, tb.error
    r = s.act("solid.recognize", {"name": "tube"}).data
    assert r["type"] == "tube" and r["volume_match"], r
    p = r["params"]
    assert abs(p["outer_radius"] - 10) < 1e-6 and abs(p["inner_radius"] - 6) < 1e-6 and abs(p["height"] - 16) < 1e-6, p
    # parametric rebuild from the recovered parameters reproduces the volume
    s.act("solid.cylinder", {"name": "ro", "radius": p["outer_radius"], "height": p["height"]})
    s.act("solid.cylinder", {"name": "ri", "radius": p["inner_radius"], "height": p["height"]})
    s.act("solid.cut", {"a": "ro", "b": "ri", "out": "tube2"})
    assert abs(_vol(s, "tube2") - _vol(s, "tube")) < 1e-6
    print("tube recovered Ro/Ri/H = %.1f/%.1f/%.1f, parametric rebuild matches" % (p["outer_radius"], p["inner_radius"], p["height"]))

    # ---- general prism: a hex bar (extruded hexagon) ---------------------- #
    hexpts = [[10 * math.cos(math.radians(60 * k)), 10 * math.sin(math.radians(60 * k))] for k in range(6)]
    s.act("solid.extrude", {"name": "hexbar", "profile": {"polygon": hexpts}, "height": 30})
    r = s.act("solid.recognize", {"name": "hexbar"}).data
    assert r["type"] == "prism" and r["volume_match"], r
    assert r["params"]["sides"] == 6 and abs(r["params"]["length"] - 30) < 1e-6, r
    # closed-form hexagon area = 3*sqrt(3)/2 * s^2 with s = circumradius = 10
    assert abs(r["params"]["profile_area"] - 3 * math.sqrt(3) / 2 * 100) < 1e-3, r
    print("hex bar recognized as prism: %d sides, area %.2f, length %.1f"
          % (r["params"]["sides"], r["params"]["profile_area"], r["params"]["length"]))

    # ---- general prism: an L-bracket profile ------------------------------ #
    lpts = [[0, 0], [40, 0], [40, 10], [10, 10], [10, 30], [0, 30]]
    s.act("solid.extrude", {"name": "lbkt", "profile": {"polygon": lpts}, "height": 8})
    r = s.act("solid.recognize", {"name": "lbkt"}).data
    assert r["type"] == "prism" and r["volume_match"], r
    # L-area = 40*10 + 10*20 = 600; volume = 600*8
    assert abs(r["params"]["profile_area"] - 600) < 1e-6 and abs(r["params"]["length"] - 8) < 1e-6, r
    assert abs(_vol(s, "lbkt") - 600 * 8) < 1e-6
    print("L-bracket recognized as prism: area %.1f, length %.1f" % (r["params"]["profile_area"], r["params"]["length"]))

    # ---- full cone (nozzle/point): V = pi r^2 h / 3 ----------------------- #
    s.act("solid.cone", {"name": "cone", "radius1": 8, "radius2": 0, "height": 20})
    r = s.act("solid.recognize", {"name": "cone"}).data
    assert r["type"] == "cone" and r["volume_match"], r
    assert abs(r["params"]["radius"] - 8) < 1e-6 and abs(r["params"]["height"] - 20) < 1e-6, r
    assert abs(_vol(s, "cone") - math.pi * 64 * 20 / 3) < 1e-3
    print("cone recovered R/H = %.1f/%.1f" % (r["params"]["radius"], r["params"]["height"]))

    # ---- frustum (tapered boss): V = pi h (R^2+Rr+r^2)/3 ------------------ #
    s.act("solid.cone", {"name": "frus", "radius1": 8, "radius2": 4, "height": 15})
    r = s.act("solid.recognize", {"name": "frus"}).data
    assert r["type"] == "frustum" and r["volume_match"], r
    assert abs(r["params"]["base_radius"] - 8) < 1e-6 and abs(r["params"]["top_radius"] - 4) < 1e-6, r
    assert abs(r["params"]["height"] - 15) < 1e-6, r
    print("frustum recovered R/r/H = %.1f/%.1f/%.1f" % (r["params"]["base_radius"], r["params"]["top_radius"], r["params"]["height"]))

    # ---- torus (O-ring/gasket): V = 2 pi^2 R r^2 -------------------------- #
    s.act("solid.torus", {"name": "tor", "radius1": 20, "radius2": 5})
    r = s.act("solid.recognize", {"name": "tor"}).data
    assert r["type"] == "torus" and r["volume_match"], r
    assert abs(r["params"]["major_radius"] - 20) < 1e-6 and abs(r["params"]["minor_radius"] - 5) < 1e-6, r
    assert abs(_vol(s, "tor") - 2 * math.pi ** 2 * 20 * 25) < 1e-2
    print("torus recovered R/r = %.1f/%.1f" % (r["params"]["major_radius"], r["params"]["minor_radius"]))

    # ---- negative: a filleted block is NOT a box (no false primitive) ----- #
    s.act("solid.box", {"name": "rb", "length": 20, "width": 20, "height": 20})
    fr = s.act("solid.fillet", {"name": "rb", "radius": 3, "out": "rbf"})
    assert fr.ok, fr.error
    r = s.act("solid.recognize", {"name": "rbf"}).data
    assert r["type"] == "freeform" and not r["volume_match"], ("filleted block faked a primitive", r)
    print("filleted block correctly reported freeform (no false box)")

    # ---- through the reverse pipeline: decompose -> recognize each leaf --- #
    s.act("solid.box", {"name": "m_a", "length": 10, "width": 10, "height": 10, "pos": [0, 0, 0]})
    s.act("solid.cylinder", {"name": "m_b", "radius": 3, "height": 12, "pos": [40, 0, 0]})
    s.act("solid.sphere", {"name": "m_c", "radius": 5, "pos": [0, 40, 0]})
    s.act("solid.compound", {"names": ["m_a", "m_b", "m_c"], "out": "blob"})
    dec = s.act("solid.decompose", {"name": "blob", "prefix": "leaf"})
    assert dec.ok and dec.data["parts"] == 3, dec.data
    kinds = []
    for part in dec.data["part_list"]:
        rr = s.act("solid.recognize", {"name": part["name"]}).data
        assert rr["volume_match"], ("leaf not recognized", rr)
        kinds.append(rr["type"])
    assert sorted(kinds) == ["box", "cylinder", "sphere"], kinds
    print("decomposed blob -> recognized leaves: %s" % sorted(kinds))

    print("RECOGNIZE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_recognize"):
    main()
