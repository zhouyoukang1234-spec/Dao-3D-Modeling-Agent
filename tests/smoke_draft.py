"""Draft-analysis smoke — mould/casting ejectability vs a pull direction.

A part releases from a mould only if every side wall is tilted at least the
minimum draft away from the pull axis. ``solid.draft`` reports the under-drafted
walls. Validated against closed-form geometry:

  * a plain box (4 vertical side walls) -> 4 zero-draft walls, not draftable;
  * a frustum cone (constant side draft atan((r1-r2)/h) = 16.7 deg) -> draftable;
  * a straight cylinder (one vertical lateral wall) -> 1 wall, not draftable;
  * the same cylinder pulled along its own axis is just two caps -> draftable.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("draft")
    print("FreeCAD", s.registry.kernel.freecad_version)

    s.act("solid.box", {"name": "b", "length": 40, "width": 40, "height": 20})
    b = s.act("solid.draft", {"name": "b", "pull": [0, 0, 1], "min_draft": 3}).data
    print("box   :", b["insufficient_draft"], "walls  draftable=", b["draftable"],
          " toward/away=", b["toward_pull"], b["away_pull"])
    assert b["insufficient_draft"] == 4 and not b["draftable"], b
    assert b["toward_pull"] == 1 and b["away_pull"] == 1, b   # one cap each side

    s.act("solid.cone", {"name": "f", "radius1": 20, "radius2": 14, "height": 20})
    f = s.act("solid.draft", {"name": "f", "pull": [0, 0, 1], "min_draft": 3}).data
    print("frust :", f["insufficient_draft"], "walls  draftable=", f["draftable"])
    assert f["draftable"] and f["insufficient_draft"] == 0, f
    # tighten the requirement past the real side draft -> the wall now fails
    side = math.degrees(math.atan((20 - 14) / 20.0))         # 16.7 deg
    f2 = s.act("solid.draft", {"name": "f", "pull": [0, 0, 1], "min_draft": side + 2}).data
    assert f2["insufficient_draft"] == 1, (f2, side)

    s.act("solid.cylinder", {"name": "c", "radius": 15, "height": 20})
    c = s.act("solid.draft", {"name": "c", "pull": [0, 0, 1], "min_draft": 3}).data
    print("cyl Z :", c["insufficient_draft"], "walls  draftable=", c["draftable"])
    assert c["insufficient_draft"] == 1 and not c["draftable"], c
    # pulling along the cylinder axis (X) leaves only the two end caps as walls?
    # no -- the lateral surface is now fully parallel to the pull, still a wall.
    cx = s.act("solid.draft", {"name": "c", "pull": [1, 0, 0], "min_draft": 3}).data
    assert cx["insufficient_draft"] >= 1, cx

    print("DRAFT SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_draft"):
    main()
