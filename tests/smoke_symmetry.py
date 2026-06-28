"""Symmetry smoke -- recovered symmetry group cross-checked against known shapes.

Symmetry is proven geometrically (reflect/rotate the real BREP, demand the
symmetric difference vanish), so the answers are exact and verifiable:

  * a box a x b x c (a != b != c): exactly 3 mirror planes (the principal
    planes), 2-fold rotational symmetry about each of the 3 axes, and it is
    centrally symmetric ;
  * a cylinder: 3 mirror planes too, but its axis carries the highest tested
    rotational order -- flagged ``continuous`` (a surface of revolution) ;
  * a sphere: every principal axis is continuous and it is centro-symmetric ;
  * an L-bracket: at most one mirror plane, no rotational symmetry, and it is
    NOT centrally symmetric -- the honest asymmetry of a real bracket ;
  * a missing solid is refused loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("symmetry")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- box a!=b!=c : 3 mirror planes, 2-fold x3, centro-symmetric ------ #
    s.act("solid.box", {"name": "blk", "length": 20, "width": 30, "height": 40})
    r = s.act("solid.symmetry", {"name": "blk"}).data
    assert r["mirror_plane_count"] == 3, r
    assert r["max_rotational_order"] == 2, r
    assert len(r["rotational_axes"]) == 3 and all(x["order"] == 2 for x in r["rotational_axes"]), r
    assert r["point_symmetric"] is True, r
    print("box: %d mirror planes, 2-fold about %d axes, centro-symmetric=%s"
          % (r["mirror_plane_count"], len(r["rotational_axes"]), r["point_symmetric"]))

    # ---- cylinder: 3 planes + one continuous axis ----------------------- #
    s.act("solid.cylinder", {"name": "cyl", "radius": 8, "height": 40})
    rc = s.act("solid.symmetry", {"name": "cyl"}).data
    assert rc["mirror_plane_count"] == 3, rc
    cont = [x for x in rc["rotational_axes"] if x["continuous"]]
    assert len(cont) == 1, rc
    assert rc["point_symmetric"] is True, rc
    print("cylinder: %d mirror planes, continuous axis order=%d, centro-symmetric=%s"
          % (rc["mirror_plane_count"], cont[0]["order"], rc["point_symmetric"]))

    # ---- sphere: every axis continuous, centro-symmetric ---------------- #
    s.act("solid.sphere", {"name": "sph", "radius": 12})
    rs = s.act("solid.symmetry", {"name": "sph"}).data
    assert rs["mirror_plane_count"] == 3, rs
    assert all(x["continuous"] for x in rs["rotational_axes"]) and len(rs["rotational_axes"]) == 3, rs
    assert rs["point_symmetric"] is True, rs
    print("sphere: 3 continuous axes (order %d), centro-symmetric=%s"
          % (rs["rotational_axes"][0]["order"], rs["point_symmetric"]))

    # ---- L-bracket: <=1 plane, no rotation, NOT centro-symmetric -------- #
    s.act("solid.box", {"name": "lh", "length": 40, "width": 20, "height": 10})
    s.act("solid.box", {"name": "lv", "length": 10, "width": 20, "height": 30})
    s.act("solid.union", {"a": "lh", "b": "lv", "out": "lbrk"})
    rl = s.act("solid.symmetry", {"name": "lbrk"}).data
    assert rl["mirror_plane_count"] <= 1, rl
    assert rl["max_rotational_order"] == 1, rl
    assert rl["point_symmetric"] is False, rl
    print("L-bracket: %d mirror plane(s), max rot order %d, centro-symmetric=%s"
          % (rl["mirror_plane_count"], rl["max_rotational_order"], rl["point_symmetric"]))

    # ---- fast 'invariant' method must AGREE with the exact boolean proof -- #
    # The face-centroid invariant test is what lets the high-face real parts
    # (a 299-face pulley the boolean proof must refuse) still report symmetry.
    # Its trustworthiness rests on matching the exact method on shapes we know.
    for nm in ("blk", "cyl", "lbrk"):
        ex = s.act("solid.symmetry", {"name": nm}).data
        iv = s.act("solid.symmetry", {"name": nm, "method": "invariant"}).data
        assert iv["proven"] is False and iv["method"] == "face-invariant", iv
        assert iv["mirror_plane_count"] == ex["mirror_plane_count"], (nm, iv, ex)
        assert iv["max_rotational_order"] == ex["max_rotational_order"], (nm, iv, ex)
        assert iv["point_symmetric"] == ex["point_symmetric"], (nm, iv, ex)
        assert iv["max_face_deviation"] < 1e-6, (nm, iv)
    print("invariant method agrees with exact boolean proof on box/cyl/L-bracket")

    # ---- invariant bypasses the O(faces) boolean budget by design -------- #
    # exact refuses past the budget; invariant has no booleans, so the same
    # tiny budget does not stop it -- this is the whole point for real parts.
    refused = s.act("solid.symmetry", {"name": "blk", "max_faces": 2})
    assert not refused.ok and "max_faces" in (refused.error or ""), refused
    okfast = s.act("solid.symmetry", {"name": "blk", "method": "invariant", "max_faces": 2})
    assert okfast.ok and okfast.data["mirror_plane_count"] == 3, okfast
    print("invariant path runs under a budget that refuses the exact proof")

    # ---- an unknown method is refused loudly ----------------------------- #
    badm = s.act("solid.symmetry", {"name": "blk", "method": "guess"})
    assert not badm.ok and "method must be" in (badm.error or ""), badm

    # ---- a missing solid is refused loudly ------------------------------ #
    bad = s.act("solid.symmetry", {"name": "nope"})
    assert not bad.ok and "no such solid" in (bad.error or "").lower()
    print("missing solid refused: %s" % bad.error)

    print("SYMMETRY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_symmetry"):
    main()
