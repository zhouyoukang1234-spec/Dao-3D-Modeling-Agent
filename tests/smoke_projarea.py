"""Projected-area smoke — silhouette / shadow footprint along a direction.

``solid.projected_area`` evaluates (1/2) * integral |n . d| dA over the solid
boundary by tessellation. Validated against textbook closed forms:

  * box LxWxH projected along Z -> L*W, along X -> W*H (planar => exact);
  * cylinder r,h along its axis -> pi*r^2, across the axis -> 2*r*h;
  * sphere r along any direction -> pi*r^2 (curved => mesh-converged);
  * an L-bracket (still front/back single-covered along Z) -> its plan area.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-3):
    return abs(got - exp) <= rel * abs(exp) + 1e-6


def main():
    s = new_session("projarea")
    print("FreeCAD", s.registry.kernel.freecad_version)

    s.act("solid.box", {"name": "b", "length": 60, "width": 20, "height": 10})
    z = s.act("solid.projected_area", {"name": "b", "dir": [0, 0, 1]}).data
    x = s.act("solid.projected_area", {"name": "b", "dir": [1, 0, 0]}).data
    print("box  :", z["projected_area"], x["projected_area"], "exact=", z["exact"])
    assert z["exact"] and x["exact"], (z, x)
    assert _close(z["projected_area"], 60 * 20), z
    assert _close(x["projected_area"], 20 * 10), x

    s.act("solid.cylinder", {"name": "c", "radius": 12, "height": 30})
    ca = s.act("solid.projected_area", {"name": "c", "dir": [0, 0, 1]}).data
    cs = s.act("solid.projected_area", {"name": "c", "dir": [1, 0, 0],
                                        "deflection": 0.01}).data
    print("cyl  :", ca["projected_area"], cs["projected_area"])
    assert _close(ca["projected_area"], math.pi * 12**2, rel=2e-3), ca
    assert _close(cs["projected_area"], 2 * 12 * 30, rel=2e-3), cs

    s.act("solid.sphere", {"name": "sp", "radius": 15})
    spa = s.act("solid.projected_area", {"name": "sp", "dir": [0, 0, 1],
                                         "deflection": 0.01}).data
    print("sph  :", spa["projected_area"])
    assert _close(spa["projected_area"], math.pi * 15**2, rel=1e-2), spa

    # L-bracket: union of two blocks; plan (Z) footprint = sum of the two
    # rectangles they cover (front/back single-covered along Z).
    s.act("solid.box", {"name": "h", "length": 40, "width": 30, "height": 8})
    s.act("solid.box", {"name": "v", "length": 8, "width": 30, "height": 40})
    assert s.act("solid.union", {"a": "h", "b": "v", "out": "L"}).ok
    la = s.act("solid.projected_area", {"name": "L", "dir": [0, 0, 1]}).data
    print("Lbrk :", la["projected_area"])
    assert _close(la["projected_area"], 40 * 30), la  # vertical leg sits on h's footprint

    # a zero projection direction used to silently report a 0 mm^2 footprint;
    # it must now fail loud with guidance.
    zd = s.act("solid.projected_area", {"name": "L", "dir": [0, 0, 0]})
    assert not zd.ok and "non-zero" in (zd.error or ""), zd.error
    print("zero dir refused:", zd.error)

    print("PROJAREA SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_projarea"):
    main()
