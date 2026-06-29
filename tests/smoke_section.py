"""Section-properties smoke — beam/structural cross-section moments of area.

``solid.section`` cuts a solid with a plane and reports the engineering section
properties (area, centroid, second moments Ix/Iy, polar J). Validated against
textbook closed forms:

  * rectangle b x h:  A = b*h,  Ix = b*h^3/12,  Iy = h*b^3/12,  J = Ix + Iy;
  * solid circle r:   A = pi*r^2,  Ix = Iy = pi*r^4/4,  J = 2*Ix;
  * hollow tube R/r:  Ix = pi/4*(R^4 - r^4)  (the section face carries the hole);
  * pulling the cut along a different axis re-maps which diagonal term is the
    polar (perpendicular-axis theorem: J = Ix + Iy holds about the normal);
  * a plane that misses the solid -> hit == False.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-3):
    return abs(got - exp) <= rel * abs(exp) + 1e-6


def main():
    s = new_session("section")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # rectangle 60 (x) by 20 (y), cut perpendicular to Z.
    s.act("solid.box", {"name": "r", "length": 60, "width": 20, "height": 10})
    r = s.act("solid.section", {"name": "r", "normal": [0, 0, 1], "d": 5}).data
    print("rect :", r["area"], r["Ix"], r["Iy"], r["J"])
    assert _close(r["area"], 1200), r
    assert _close(r["Ix"], 60 * 20**3 / 12) and _close(r["Iy"], 20 * 60**3 / 12), r
    assert _close(r["J"], r["Ix"] + r["Iy"]), r
    assert _close(r["centroid"][0], 30) and _close(r["centroid"][1], 10), r

    # solid circle r=12.
    s.act("solid.cylinder", {"name": "c", "radius": 12, "height": 30})
    cc = s.act("solid.section", {"name": "c", "normal": [0, 0, 1], "d": 15}).data
    print("circ :", cc["area"], cc["Ix"], cc["J"])
    assert _close(cc["area"], math.pi * 12**2), cc
    assert _close(cc["Ix"], math.pi * 12**4 / 4) and _close(cc["Iy"], cc["Ix"]), cc
    assert _close(cc["J"], 2 * cc["Ix"]), cc

    # hollow tube R=12 r=8 -> the section face has a hole (2 loops).
    s.act("solid.cylinder", {"name": "to", "radius": 12, "height": 30})
    s.act("solid.cylinder", {"name": "ti", "radius": 8, "height": 30})
    assert s.act("solid.cut", {"a": "to", "b": "ti", "out": "tube"}).ok
    tb = s.act("solid.section", {"name": "tube", "normal": [0, 0, 1], "d": 15}).data
    print("tube :", tb["area"], tb["Ix"], "loops=", tb["loops"])
    assert tb["loops"] == 2, tb
    assert _close(tb["area"], math.pi * (12**2 - 8**2)), tb
    assert _close(tb["Ix"], math.pi / 4 * (12**4 - 8**4)), tb

    # same rectangle cut along X: perpendicular-axis theorem still holds, and
    # the polar term re-maps to the X-normal diagonal entry.
    rx = s.act("solid.section", {"name": "r", "normal": [1, 0, 0], "d": 30}).data
    print("rectX:", rx["area"], rx["Ix"], rx["Iy"], rx["J"])
    assert _close(rx["area"], 200), rx
    assert _close(rx["J"], rx["Ix"] + rx["Iy"]), rx

    # a plane beyond the solid misses it entirely.
    miss = s.act("solid.section", {"name": "c", "normal": [0, 0, 1], "d": 999}).data
    assert not miss["hit"] and miss["area"] == 0.0, miss

    # a zero cutting-plane normal used to leak a bare OCCError "gp_Dir() ...
    # zero norm"; it must now fail loud with guidance.
    zn = s.act("solid.section", {"name": "c", "normal": [0, 0, 0]})
    assert not zn.ok and "non-zero" in (zn.error or "") and "OCCError" not in (zn.error or ""), zn.error
    print("zero normal refused:", zn.error)

    print("SECTION SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_section"):
    main()
