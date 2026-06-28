"""Design-intent smoke -- walk a real part back to "the first thought".

``design_intent`` composes the reverse-engineering reads (obb stock, recognize
primitive, symmetry, holes, fillets) into one card plus an ordered build recipe
-- the forward program a clean CAD model would run to reproduce the part. The
digest is cross-checked against parts we build with a known recipe:

  * a plain primitive (a cylinder) is recognised as that primitive, with no
    holes / bosses / blends ;
  * a bracket = a 60x40x12 block - 4 phi6 corner through-holes + a phi12 boss
    comes back with stock ~[12,40,60], 4 through-holes, 1 boss, and a recipe
    that names the block, the drilling and the boss ;
  * a filleted block reports its broken edges in the recipe ;
  * a non-solid / multi-solid input is refused loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("design_intent")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- a plain cylinder is recovered as that primitive -------------------- #
    s.act("solid.cylinder", {"name": "cyl", "radius": 8, "height": 20})
    d = s.act("solid.design_intent", {"name": "cyl"}).data
    assert d["primitive"]["type"] == "cylinder" and d["primitive"]["volume_match"], d
    assert d["holes"]["count"] == 0 and d["holes"]["bosses"] == 0, d
    assert d["blends"]["rounds"] == 0 and d["blends"]["fillets"] == 0, d
    assert any("cylinder" in step for step in d["recipe"]), d["recipe"]
    print("cylinder digest: primitive=%s, recipe=%s" % (d["primitive"]["type"], d["recipe"]))

    # ---- a bracket: block - 4 through-holes + a boss ------------------------ #
    s.act("solid.box", {"name": "br", "length": 60, "width": 40, "height": 12})
    for i, (x, y) in enumerate([(8, 8), (52, 8), (8, 32), (52, 32)]):
        s.act("solid.cylinder", {"name": "hh%d" % i, "radius": 3, "height": 24, "pos": [x, y, -6]})
        s.act("solid.cut", {"a": "br", "b": "hh%d" % i, "out": "br"})
    s.act("solid.cylinder", {"name": "bs", "radius": 6, "height": 8, "pos": [30, 20, 12]})
    s.act("solid.union", {"a": "br", "b": "bs", "out": "br"})

    db = s.act("solid.design_intent", {"name": "br"}).data
    assert db["primitive"]["type"] == "freeform", db        # a holed bracket is no primitive
    assert sorted(db["stock"]["size"]) == db["stock"]["size"], db
    assert abs(db["stock"]["size"][1] - 40) < 1e-3 and abs(db["stock"]["size"][2] - 60) < 1e-3, db
    assert db["holes"]["count"] == 4 and db["holes"]["through"] == 4, db
    assert db["holes"]["bosses"] == 1, db
    assert any("block" in s0 for s0 in db["recipe"]), db["recipe"]
    assert sum("drill" in s0 for s0 in db["recipe"]) == 4, db["recipe"]
    assert any("boss" in s0 for s0 in db["recipe"]), db["recipe"]
    print("bracket digest recipe:")
    for step in db["recipe"]:
        print("   -", step)

    # ---- a filleted block lists its broken edges in the recipe -------------- #
    s.act("solid.box", {"name": "fb", "length": 30, "width": 20, "height": 10})
    s.act("solid.fillet", {"name": "fb", "radius": 2, "out": "fbf"})
    df = s.act("solid.design_intent", {"name": "fbf"}).data
    assert df["blends"]["rounds"] > 0 and 2.0 in df["blends"]["radii"], df
    assert any("break edges" in s0 for s0 in df["recipe"]), df["recipe"]
    print("filleted block: %s" % [s0 for s0 in df["recipe"] if "break" in s0])

    # ---- loud guard --------------------------------------------------------- #
    s.act("solid.box", {"name": "g1", "length": 5, "width": 5, "height": 5})
    s.act("solid.box", {"name": "g2", "length": 5, "width": 5, "height": 5, "pos": [40, 0, 0]})
    s.act("solid.compound", {"names": ["g1", "g2"], "out": "asm"})
    bad = s.act("solid.design_intent", {"name": "asm"})
    assert not bad.ok and "single solid" in (bad.error or "").lower(), bad.error
    print("multi-solid refused: %s" % bad.error)

    print("DESIGN INTENT SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_design_intent"):
    main()
