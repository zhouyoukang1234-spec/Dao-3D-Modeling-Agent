"""Undercut smoke — two-plate mould side-action detection vs a pull direction.

Completing the mould DFM trio (draft + thickness + undercut): a face is an
undercut when a ray fired from it outward along the pull axis re-enters the
solid (it is shadowed by material), so it needs a side core / lifter rather than
a simple open/close mould. Validated against closed-form geometry:

  * plain box pulled Z -> no undercut, moldable;
  * a box with a cross hole (axis perpendicular to pull) -> the bore is shadowed
    along Z -> 1 undercut, not moldable;
  * the SAME part pulled along the hole axis -> the bore becomes a core-pin
    direction -> no undercut, moldable (changing pull resolves it);
  * a box with a hole parallel to the pull -> a core pin, never an undercut.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("undercut")
    print("FreeCAD", s.registry.kernel.freecad_version)

    s.act("solid.box", {"name": "b", "length": 40, "width": 40, "height": 20})
    b = s.act("solid.undercut", {"name": "b", "pull": [0, 0, 1]}).data
    print("plain box   -> undercuts=%d moldable=%s" % (b["undercuts"], b["moldable"]))
    assert b["undercuts"] == 0 and b["moldable"], b

    # cross hole along X (perpendicular to a Z pull) -> the bore is an undercut.
    s.act("solid.box", {"name": "x", "length": 40, "width": 40, "height": 20})
    s.act("solid.cylinder", {"name": "px", "radius": 6, "height": 60})
    s.act("solid.rotate", {"name": "px", "axis": [0, 1, 0], "angle": 90, "center": [0, 0, 0]})
    s.act("solid.translate", {"name": "px", "vector": [-10, 20, 10]})
    assert s.act("solid.cut", {"a": "x", "b": "px", "out": "x"}).ok
    xz = s.act("solid.undercut", {"name": "x", "pull": [0, 0, 1], "samples": 3}).data
    print("cross-hole Z-> undercuts=%d moldable=%s" % (xz["undercuts"], xz["moldable"]))
    assert xz["undercuts"] >= 1 and not xz["moldable"], xz
    # pulling along the hole axis turns the bore into a core pin -> moldable.
    xx = s.act("solid.undercut", {"name": "x", "pull": [1, 0, 0], "samples": 3}).data
    print("cross-hole X-> undercuts=%d moldable=%s" % (xx["undercuts"], xx["moldable"]))
    assert xx["undercuts"] == 0 and xx["moldable"], xx

    # hole parallel to the pull is just a core pin, never an undercut.
    s.act("solid.box", {"name": "z", "length": 40, "width": 40, "height": 20})
    s.act("solid.cylinder", {"name": "pz", "radius": 6, "height": 60})
    s.act("solid.translate", {"name": "pz", "vector": [20, 20, -10]})
    assert s.act("solid.cut", {"a": "z", "b": "pz", "out": "z"}).ok
    zz = s.act("solid.undercut", {"name": "z", "pull": [0, 0, 1], "samples": 3}).data
    print("axial hole  -> undercuts=%d moldable=%s" % (zz["undercuts"], zz["moldable"]))
    assert zz["undercuts"] == 0 and zz["moldable"], zz

    # a zero pull vector used to silently call every face a side wall and report
    # the part vacuously moldable; it must now fail loud with guidance.
    zp = s.act("solid.undercut", {"name": "z", "pull": [0, 0, 0]})
    assert not zp.ok and "non-zero" in (zp.error or ""), zp.error
    print("zero pull refused:", zp.error)

    print("UNDERCUT SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_undercut"):
    main()
