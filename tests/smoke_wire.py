"""Wire / 2-D geometry utility smoke test.

Exercises:  wire.make, wire.offset, wire.fillet, wire.normal, wire.intersect,
            wire.mirror, wire.info, wire.extrude, wire.circle, wire.arc
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session


def main():
    s = new_session("smoke_wire")

    # --- wire ops registered ---
    wire_ops = [n for n in s.registry.names() if n.startswith("wire.")]
    assert len(wire_ops) >= 10, "expected >= 10 wire ops, got %d: %s" % (len(wire_ops), wire_ops)
    print("wire ops:", sorted(wire_ops))

    # --- make a square wire ---
    r = s.act("wire.make", {
        "points": [[0, 0], [100, 0], [100, 100], [0, 100]],
        "close": True, "name": "square"})
    assert r.ok, "wire.make failed: %s" % r
    assert r.data["closed"], "square wire should be closed"
    assert r.data["edges"] == 4, "expected 4 edges"
    assert abs(r.data["length"] - 400) < 0.1, "expected length 400"
    sq_name = r.data["name"]
    print("square wire:", sq_name, "edges:", r.data["edges"], "len:", r.data["length"])

    # --- info ---
    r = s.act("wire.info", {"wire": sq_name})
    assert r.ok, "wire.info failed: %s" % r
    assert r.data["planar"], "square wire should be planar"
    assert r.data["normal"] == [0, 0, 1], "expected Z normal"
    print("info:", "planar:", r.data["planar"], "normal:", r.data["normal"])

    # --- offset ---
    r = s.act("wire.offset", {"wire": sq_name, "distance": 10})
    assert r.ok, "wire.offset failed: %s" % r
    off_name = r.data["name"]
    print("offset:", off_name, "len:", r.data["length"])

    # --- fillet ---
    r = s.act("wire.fillet", {"wire": sq_name, "radius": 5})
    assert r.ok, "wire.fillet failed: %s" % r
    assert r.data["edges"] == 8, "filleted square should have 8 edges (4 lines + 4 arcs)"
    print("fillet:", r.data["name"], "edges:", r.data["edges"])

    # --- normal ---
    r = s.act("wire.normal", {"wire": sq_name})
    assert r.ok, "wire.normal failed"
    assert r.data["planar"]
    print("normal:", r.data["normal"])

    # --- make two crossing lines and find intersection ---
    r1 = s.act("wire.make", {"points": [[0, 0], [100, 100]], "name": "diag1"})
    r2 = s.act("wire.make", {"points": [[100, 0], [0, 100]], "name": "diag2"})
    assert r1.ok and r2.ok
    r = s.act("wire.intersect", {"a": r1.data["name"], "b": r2.data["name"]})
    assert r.ok, "wire.intersect failed: %s" % r
    assert r.data["count"] == 1, "expected 1 intersection"
    pt = r.data["points"][0]
    assert abs(pt[0] - 50) < 0.1 and abs(pt[1] - 50) < 0.1, "expected (50,50)"
    print("intersect:", r.data["points"])

    # --- mirror ---
    r = s.act("wire.mirror", {"wire": sq_name, "point": [0, 0, 0], "axis": [0, 1, 0]})
    assert r.ok, "wire.mirror failed: %s" % r
    print("mirror:", r.data["name"], "len:", r.data["length"])

    # --- extrude closed wire into solid ---
    r = s.act("wire.extrude", {"wire": sq_name, "direction": [0, 0, 50]})
    assert r.ok, "wire.extrude failed: %s" % r
    assert r.data["shape_type"] == "Solid", "expected Solid"
    expected_vol = 100 * 100 * 50
    assert abs(r.data["volume"] - expected_vol) < 1, "expected volume %s, got %s" % (
        expected_vol, r.data["volume"])
    print("extrude:", r.data["name"], "vol:", r.data["volume"])

    # --- extrude with out/dir aliases; solid.* ops adopt the result ---
    r = s.act("wire.extrude", {"wire": sq_name, "dir": [0, 0, 20],
                               "out": "wblock"})
    assert r.ok and r.data["name"] == "wblock", \
        "wire.extrude must honour out/dir aliases: %s" % r
    r = s.act("solid.measure", {"name": "wblock"})
    assert r.ok, "solid.* should adopt a wire-produced solid: %s" % r
    assert abs(r.data["volume"] - 100 * 100 * 20) < 1
    print("cross-module adopt ok:", r.data["volume"])

    # --- circle wire ---
    r = s.act("wire.circle", {"radius": 25, "name": "circ"})
    assert r.ok, "wire.circle failed: %s" % r
    expected_circ = 2 * math.pi * 25
    assert abs(r.data["length"] - expected_circ) < 0.1
    print("circle:", r.data["name"], "len:", r.data["length"])

    # --- arc wire ---
    r = s.act("wire.arc", {"radius": 20, "start": 0, "end": 90, "name": "quarter"})
    assert r.ok, "wire.arc failed: %s" % r
    expected_arc = 2 * math.pi * 20 / 4  # quarter circle
    assert abs(r.data["length"] - expected_arc) < 0.1
    print("arc:", r.data["name"], "len:", r.data["length"])

    # --- guards ---
    r = s.act("wire.make", {"points": [[0, 0]]})
    assert not r.ok, "wire.make should reject single point"
    r = s.act("wire.info", {"wire": "nonexistent"})
    assert not r.ok, "wire.info should reject nonexistent"
    r = s.act("wire.circle", {"radius": -5})
    assert not r.ok, "wire.circle should reject negative radius"
    r = s.act("wire.extrude", {"wire": r1.data["name"], "direction": [0, 0, 10]})
    assert not r.ok, "wire.extrude should reject open wire"
    print("guards ok: bad inputs rejected")

    print("WIRE SMOKE OK", s.summary())


if __name__ == "__main__":
    main()
