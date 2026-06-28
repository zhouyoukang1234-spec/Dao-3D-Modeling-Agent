"""Holes smoke -- recover cylindrical holes and bosses, the mounting-feature
channel of reverse engineering.

``recognize`` names a whole simple solid; a real bracket is a block *minus holes
plus bosses*. ``solid.holes`` recovers those round features from raw geometry and
the answers are cross-checked against a part we build with known features:

  * a plate with four corner through-holes (r3) -> four hole features, each
    radius 3, axis +/-Z, through=True ;
  * a blind hole (r5, depth 4) -> a hole feature, through=False, depth ~4 ;
  * a boss (r6) added on top -> a boss feature (normal points away from axis) ;
  * a counterbored hole (r3 through + r6 spotface) -> ONE feature carrying both
    radii (counterbored=True), since the two cylinders are coaxial ;
  * a non-solid / multi-solid input is refused loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _zaxis(ax):
    return abs(abs(ax[2]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6


def main():
    s = new_session("holes")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- a plate with 4 through-holes, 1 blind hole, 1 boss ------------- #
    s.act("solid.box", {"name": "p", "length": 60, "width": 40, "height": 10})
    for i, (x, y) in enumerate([(10, 10), (50, 10), (10, 30), (50, 30)]):
        s.act("solid.cylinder", {"name": "h%d" % i, "radius": 3, "height": 20, "pos": [x, y, -5]})
        s.act("solid.cut", {"a": "p", "b": "h%d" % i, "out": "p"})
    s.act("solid.cylinder", {"name": "blind", "radius": 5, "height": 4, "pos": [30, 20, 6]})
    s.act("solid.cut", {"a": "p", "b": "blind", "out": "p"})
    s.act("solid.cylinder", {"name": "boss", "radius": 6, "height": 8, "pos": [30, 20, 10]})
    s.act("solid.union", {"a": "p", "b": "boss", "out": "p"})

    r = s.act("solid.holes", {"name": "p"}).data
    assert r["hole_count"] == 5 and r["boss_count"] == 1, r
    assert r["through_holes"] == 4, r
    holes = [f for f in r["features"] if f["kind"] == "hole"]
    bosses = [f for f in r["features"] if f["kind"] == "boss"]

    through3 = [h for h in holes if h["through"] and abs(h["radius"] - 3) < 1e-6]
    assert len(through3) == 4, holes
    assert all(_zaxis(h["axis"]) and abs(h["depth"] - 10) < 1e-3 for h in through3), through3

    blind = [h for h in holes if not h["through"]]
    assert len(blind) == 1 and abs(blind[0]["radius"] - 5) < 1e-6, blind
    assert abs(blind[0]["depth"] - 4) < 1e-3, blind[0]

    assert len(bosses) == 1 and abs(bosses[0]["radius"] - 6) < 1e-6, bosses
    print("plate: 4 through-holes r3 (depth10), 1 blind hole r5 (depth4), 1 boss r6")

    # ---- a counterbored hole comes back as ONE feature with both radii -- #
    s.act("solid.box", {"name": "cb", "length": 30, "width": 30, "height": 12})
    s.act("solid.cylinder", {"name": "bore", "radius": 3, "height": 20, "pos": [15, 15, -4]})
    s.act("solid.cut", {"a": "cb", "b": "bore", "out": "cb"})
    s.act("solid.cylinder", {"name": "spot", "radius": 6, "height": 4, "pos": [15, 15, 8]})
    s.act("solid.cut", {"a": "cb", "b": "spot", "out": "cb"})
    rc = s.act("solid.holes", {"name": "cb"}).data
    cbf = [f for f in rc["features"] if f["kind"] == "hole"]
    assert len(cbf) == 1, rc          # the two coaxial cylinders are one feature
    assert cbf[0]["counterbored"] is True and cbf[0]["radii"] == [3.0, 6.0], cbf[0]
    assert cbf[0]["through"] is True, cbf[0]
    print("counterbore: one hole feature, radii=%s, counterbored, through" % cbf[0]["radii"])

    # ---- a plain block has no round features ---------------------------- #
    s.act("solid.box", {"name": "blk", "length": 10, "width": 10, "height": 10})
    rb = s.act("solid.holes", {"name": "blk"}).data
    assert rb["hole_count"] == 0 and rb["boss_count"] == 0, rb
    print("plain block: no holes, no bosses")

    # ---- loud guards ---------------------------------------------------- #
    s.act("solid.box", {"name": "g1", "length": 5, "width": 5, "height": 5})
    s.act("solid.box", {"name": "g2", "length": 5, "width": 5, "height": 5, "pos": [40, 0, 0]})
    s.act("solid.compound", {"names": ["g1", "g2"], "out": "asm"})
    bad = s.act("solid.holes", {"name": "asm"})
    assert not bad.ok and "single solid" in (bad.error or "").lower(), bad.error
    print("multi-solid refused: %s" % bad.error)

    print("HOLES SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_holes"):
    main()
