"""DFM closed-loop smoke — a molded housing analysed by the full mould trio.

This is the integration of the three mould-DFM pillars (draft + thickness +
undercut) on one realistic part: an open-top injection-molded tub built by
``loft`` + boolean ``cut``. The boolean returns a ``Part.Compound`` (not a lone
``Solid``), which is exactly the case that used to break ``solid.draft`` — it
read ``shape.CenterOfMass``, an attribute a compound does not carry. So this
suite also pins the compound-tolerant centroid fix.

The closed loop it exercises is the real DFM lesson of a molded tub:

  * WRONG taper — the cavity *narrows* toward the opening, so the core that
    forms it is wider at its base than at the mouth and cannot withdraw upward
    -> ``solid.undercut`` flags the four cavity walls, ``moldable`` is False;
  * RIGHT taper — both the outer skin and the cavity *widen* toward the opening,
    so the core releases up and the cavity block releases down -> zero
    undercuts, ``moldable`` is True, with an acceptable ~2 mm wall throughout.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _build(s, name, outer, cavity):
    """Loft an outer skin and a cavity, then cut to make an open-top tub.

    ``outer``/``cavity`` are ((w,h,offset), (w,h,offset)) bottom/top sections.
    """
    (ow0, oh0, oz0), (ow1, oh1, oz1) = outer
    (cw0, ch0, cz0), (cw1, ch1, cz1) = cavity
    s.act("solid.loft", {"name": name + "_o", "sections": [
        {"profile": {"rect": [ow0, oh0]}, "offset": oz0},
        {"profile": {"rect": [ow1, oh1]}, "offset": oz1}]})
    s.act("solid.loft", {"name": name + "_c", "sections": [
        {"profile": {"rect": [cw0, ch0]}, "offset": cz0},
        {"profile": {"rect": [cw1, ch1]}, "offset": cz1}]})
    return s.act("solid.cut", {"a": name + "_o", "b": name + "_c", "out": name})


def main():
    s = new_session("dfm_housing")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # WRONG taper: outer narrows up (44->40), cavity narrows up (40->36) -> the
    # cavity core is trapped by the narrower mouth.
    r = _build(s, "bad", ((44, 44, 0), (40, 40, 20)), ((40, 40, 2), (36, 36, 22)))
    assert r.ok, r.error
    # the boolean cut yields a compound -> draft must not blow up on it.
    db = s.act("solid.draft", {"name": "bad", "pull": [0, 0, 1], "min_draft": 3})
    assert db.ok, db.error
    ub = s.act("solid.undercut", {"name": "bad", "pull": [0, 0, 1], "samples": 3}).data
    print("bad  -> draftable=%s undercuts=%d moldable=%s"
          % (db.data["draftable"], ub["undercuts"], ub["moldable"]))
    assert ub["undercuts"] >= 1 and not ub["moldable"], ub

    # RIGHT taper: both surfaces widen toward the opening -> core releases up,
    # cavity block releases down, no side action needed.
    r = _build(s, "good", ((40, 40, 0), (44, 44, 20)), ((36, 36, 2), (41, 41, 24)))
    assert r.ok, r.error
    dg = s.act("solid.draft", {"name": "good", "pull": [0, 0, 1], "min_draft": 3}).data
    tg = s.act("solid.thickness", {"name": "good", "min_wall": 1.0, "samples": 3}).data
    ug = s.act("solid.undercut", {"name": "good", "pull": [0, 0, 1], "samples": 3}).data
    print("good -> draftable=%s undercuts=%d moldable=%s min_wall=%s"
          % (dg["draftable"], ug["undercuts"], ug["moldable"], tg["min_thickness"]))
    assert dg["draftable"], dg
    assert ug["undercuts"] == 0 and ug["moldable"], ug
    assert tg["min_thickness"] is not None and tg["min_thickness"] >= 1.0, tg

    print("DFM HOUSING SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_dfm_housing"):
    main()
