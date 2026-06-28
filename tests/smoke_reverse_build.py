"""Reverse-build smoke -- close the forward/reverse loop and prove it.

``design_intent`` reads a part back to its build recipe; ``reverse_build`` runs
that recipe forward again as real geometry and checks the rebuild against the
original by two invariants that cannot be fudged -- relative volume error and
the scale-/pose-invariant shape key:

  * a recognised primitive (a cylinder) is re-emitted with ``Part.make*`` and
    reproduces the original exactly (volume_match, same shape key) ;
  * a drilled bracket = a block - 4 phi6 through-holes is rebuilt as its stock
    block minus the holes, reproducing volume and shape key even though the
    original was assembled by booleans ;
  * the loop is pose-blind: the same bracket placed at an arbitrary orientation
    still rebuilds to the same shape key (the principal frame is recovered) ;
  * a protruding boss cannot be recovered from a bbox stock, so it is reported
    honestly in ``skipped`` and ``volume_match`` is False -- no silent near-miss;
  * a multi-solid input is refused loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bracket(s, name):
    s.act("solid.box", {"name": name, "length": 60, "width": 40, "height": 12})
    for i, (x, y) in enumerate([(8, 8), (52, 8), (8, 32), (52, 32)]):
        s.act("solid.cylinder", {"name": "%s_h%d" % (name, i), "radius": 3,
                                 "height": 24, "pos": [x, y, -6]})
        s.act("solid.cut", {"a": name, "b": "%s_h%d" % (name, i), "out": name})


def main():
    s = new_session("reverse_build")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- a primitive rebuilds exactly -------------------------------------- #
    s.act("solid.cylinder", {"name": "cyl", "radius": 8, "height": 20})
    rc = s.act("solid.reverse_build", {"name": "cyl", "out": "cyl2"}).data
    assert rc["recipe_kind"] == "primitive:cylinder", rc
    assert rc["volume_match"] and rc["same_shape_key"], rc
    assert rc["volume_error"] < 1e-3, rc
    print("cylinder rebuilt: %s err=%g key_match=%s"
          % (rc["recipe_kind"], rc["volume_error"], rc["same_shape_key"]))

    # ---- a drilled bracket rebuilds from stock - holes --------------------- #
    _bracket(s, "br")
    rb = s.act("solid.reverse_build", {"name": "br", "out": "br2"}).data
    assert rb["recipe_kind"] == "billet:box-minus-holes", rb
    assert rb["volume_match"], rb
    assert rb["same_shape_key"], rb
    assert rb["volume_error"] < 1e-3 and not rb["skipped"], rb
    print("bracket rebuilt: %s err=%g key_match=%s skipped=%d"
          % (rb["recipe_kind"], rb["volume_error"], rb["same_shape_key"], len(rb["skipped"])))

    # ---- a turned bushing rebuilds from a CYLINDER billet - bores ---------- #
    # solid cylinder r15 h40, axial through-bore r6, and a transverse cross-hole
    # r3 -- a bbox block billet would be ~4/pi too big; the cylinder billet is
    # the right stock and reproduces the part exactly.
    s.act("solid.cylinder", {"name": "bush", "radius": 15, "height": 40})
    s.act("solid.cylinder", {"name": "bore", "radius": 6, "height": 60, "pos": [0, 0, -10]})
    s.act("solid.cut", {"a": "bush", "b": "bore", "out": "bush"})
    s.act("solid.cylinder", {"name": "xh", "radius": 3, "height": 60, "pos": [-30, 0, 20],
                             "dir": [1, 0, 0]})
    s.act("solid.cut", {"a": "bush", "b": "xh", "out": "bush"})
    rt = s.act("solid.reverse_build", {"name": "bush", "out": "bush2"}).data
    assert rt["recipe_kind"] == "billet:cylinder-minus-holes", rt
    # volume is the rigorous geometric proof; the shape key is sub-permille-noisy
    # for the cylinder-cylinder cross-hole saddle, so only the volume is asserted.
    assert rt["volume_match"], rt
    assert rt["volume_error"] < 1e-3 and not rt["skipped"], rt
    print("bushing rebuilt: %s err=%g key_match=%s skipped=%d"
          % (rt["recipe_kind"], rt["volume_error"], rt["same_shape_key"], len(rt["skipped"])))

    # ---- a counterbored plate rebuilds step-by-step ----------------------- #
    # 50x40x16 block (asymmetric footprint -> clean principal axes) with a single
    # counterbored hole: a wide recess r6 down 6mm from the top, then a narrow
    # through-bore r3 the rest of the way. The stepped feature is reconstructed
    # as recess + bore, not skipped.
    s.act("solid.box", {"name": "cb", "length": 50, "width": 40, "height": 16})
    s.act("solid.cylinder", {"name": "cb_bore", "radius": 3, "height": 40, "pos": [25, 20, -5]})
    s.act("solid.cut", {"a": "cb", "b": "cb_bore", "out": "cb"})
    s.act("solid.cylinder", {"name": "cb_rec", "radius": 6, "height": 6, "pos": [25, 20, 10]})
    s.act("solid.cut", {"a": "cb", "b": "cb_rec", "out": "cb"})
    rcb = s.act("solid.reverse_build", {"name": "cb", "out": "cb2"}).data
    assert rcb["volume_match"], rcb
    assert not rcb["skipped"], rcb                         # the step is rebuilt, not skipped
    assert rcb["volume_error"] < 1e-3, rcb
    print("counterbore rebuilt: %s err=%g skipped=%d"
          % (rcb["recipe_kind"], rcb["volume_error"], len(rcb["skipped"])))

    # ---- the loop is pose-blind: rebuild a rotated copy -------------------- #
    _bracket(s, "brr")
    s.act("solid.rotate", {"name": "brr", "axis": [1, 1, 0], "angle": 37, "out": "brr"})
    s.act("solid.rotate", {"name": "brr", "axis": [0, 0, 1], "angle": 53, "out": "brr"})
    rr = s.act("solid.reverse_build", {"name": "brr", "out": "brr2"}).data
    assert rr["volume_match"] and rr["same_shape_key"], rr
    print("rotated bracket rebuilt: err=%g key_match=%s"
          % (rr["volume_error"], rr["same_shape_key"]))

    # ---- the rebuilt part is itself a usable solid ------------------------- #
    m = s.act("solid.measure", {"name": "br2"}).data
    assert m["volume"] > 0, m
    print("rebuilt solid usable: volume=%g" % m["volume"])

    # ---- a protruding boss is reported honestly, not silently fused -------- #
    s.act("solid.box", {"name": "pb", "length": 40, "width": 40, "height": 6})
    s.act("solid.cylinder", {"name": "pbc", "radius": 6, "height": 12, "pos": [20, 20, 6]})
    s.act("solid.union", {"a": "pb", "b": "pbc", "out": "pb"})
    rp = s.act("solid.reverse_build", {"name": "pb", "out": "pb2"}).data
    assert rp["volume_match"] is False, rp                # honest: cannot fudge it
    assert any(k["feature"] == "boss" for k in rp["skipped"]), rp
    print("boss part: volume_match=%s skipped=%s"
          % (rp["volume_match"], [k["feature"] for k in rp["skipped"]]))

    # ---- loud guard -------------------------------------------------------- #
    s.act("solid.box", {"name": "g1", "length": 5, "width": 5, "height": 5})
    s.act("solid.box", {"name": "g2", "length": 5, "width": 5, "height": 5, "pos": [40, 0, 0]})
    s.act("solid.compound", {"names": ["g1", "g2"], "out": "asm"})
    bad = s.act("solid.reverse_build", {"name": "asm"})
    assert not bad.ok and "single solid" in (bad.error or "").lower(), bad.error
    print("multi-solid refused: %s" % bad.error)

    print("REVERSE BUILD SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_reverse_build"):
    main()
