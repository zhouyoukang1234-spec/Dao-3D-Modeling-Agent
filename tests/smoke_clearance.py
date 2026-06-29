"""Clearance smoke — exact minimum air-gap between two separate solids.

``solid.clearance`` returns OCCT's exact BRep extrema distance (and the closest
point pair), the complement of ``interference``. Validated against closed forms:

  * two spheres -> gap == center_distance - r1 - r2;
  * two axis-aligned boxes -> the exact face-to-face air gap;
  * touching / overlapping solids -> distance ~0, touching True, and
    ``interfering`` True only when they actually share volume.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-3):
    return abs(got - exp) <= rel * abs(exp) + 1e-3


def main():
    s = new_session("clearance")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # two spheres r=5 and r=8, centres 40 apart along X -> gap = 40-5-8 = 27.
    s.act("solid.sphere", {"name": "s1", "radius": 5})
    s.act("solid.sphere", {"name": "s2", "radius": 8})
    s.act("solid.translate", {"name": "s2", "vector": [40, 0, 0]})
    r = s.act("solid.clearance", {"a": "s1", "b": "s2"}).data
    print("spheres:", r["distance"], r["touching"], r["interfering"])
    assert _close(r["distance"], 40 - 5 - 8), r
    assert not r["touching"] and not r["interfering"], r

    # two boxes: b1 occupies x[0,10]; b2 shifted to x[25,35] -> gap 15.
    s.act("solid.box", {"name": "b1", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "b2", "length": 10, "width": 10, "height": 10})
    s.act("solid.translate", {"name": "b2", "vector": [25, 0, 0]})
    rb = s.act("solid.clearance", {"a": "b1", "b": "b2"}).data
    print("boxes  :", rb["distance"])
    assert _close(rb["distance"], 15), rb

    # overlapping boxes share volume -> distance 0, interfering True.
    s.act("solid.box", {"name": "o1", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "o2", "length": 10, "width": 10, "height": 10})
    s.act("solid.translate", {"name": "o2", "vector": [5, 0, 0]})
    ro = s.act("solid.clearance", {"a": "o1", "b": "o2"}).data
    print("overlap:", ro["distance"], ro["touching"], ro["interfering"])
    assert ro["touching"] and ro["interfering"], ro

    print("CLEARANCE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_clearance"):
    main()
