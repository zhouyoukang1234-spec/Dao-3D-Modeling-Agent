"""Spatial mobility smoke -- general 3-D Kutzbach-Grubler against textbook cases.

M = 6(n-1) - sum (6 - f_i). We check the canonical spatial mechanisms:

  * one free body relative to ground (n=2, no joints) -> M = 6 ;
  * RSSR spatial four-bar (R,S,S,R): gross M = 2, minus 1 idle (the S-S coupler
    spins freely about its own axis) -> effective mobility 1 ;
  * spatial 4R (four revolutes, generic axes): M = -2, the classic Kutzbach
    *paradox* -- it is flagged overconstrained even though the planar/spherical
    special case actually moves (geometry beats the generic count) ;
  * a 6R serial robot arm (open chain, base + 6 links, 6 revolutes) -> M = 6 .
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("spatial_mobility")
    print("FreeCAD", s.registry.kernel.freecad_version)

    free = s.act("solid.spatial_mobility", {"links": 2, "joints": []}).data
    assert free["mobility"] == 6, free
    print("free body: M=%d" % free["mobility"])

    # RSSR: 4 links, joints R,S,S,R ; gross 2, one idle S-S spin -> effective 1
    rssr = s.act("solid.spatial_mobility", {"links": 4,
                 "joints": ["revolute", "spherical", "spherical", "revolute"],
                 "idle_dof": 1}).data
    assert rssr["mobility"] == 2 and rssr["effective_mobility"] == 1, rssr
    print("RSSR spatial 4-bar: gross M=%d, idle=%d, effective=%d"
          % (rssr["mobility"], rssr["idle_dof"], rssr["effective_mobility"]))

    # spatial 4R: Kutzbach paradox -> M = -2, flagged overconstrained
    r4 = s.act("solid.spatial_mobility", {"links": 4, "joints": [{"type": "revolute", "count": 4}]}).data
    assert r4["mobility"] == -2 and r4["overconstrained"], r4
    print("spatial 4R: M=%d (Kutzbach paradox, flagged overconstrained)" % r4["mobility"])

    # 6R serial robot arm: base + 6 links = 7, six revolutes -> M = 6
    arm = s.act("solid.spatial_mobility", {"links": 7, "joints": [{"type": "revolute", "count": 6}]}).data
    assert arm["mobility"] == 6 and not arm["overconstrained"], arm
    print("6R serial arm: M=%d (a 6-DOF manipulator)" % arm["mobility"])

    # unknown joint type rejected
    assert not s.act("solid.spatial_mobility", {"links": 3, "joints": ["wobble"]}).ok
    print("SPATIAL_MOBILITY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_spatial_mobility"):
    main()
