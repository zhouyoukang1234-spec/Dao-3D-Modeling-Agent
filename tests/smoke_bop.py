"""BOPTools advanced boolean operations smoke test.

Exercises:  bop.slice, bop.fragments, bop.xor, bop.connect
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session


def main():
    s = new_session("smoke_bop")

    # --- bop ops registered ---
    bop_ops = [n for n in s.registry.names() if n.startswith("bop.")]
    assert len(bop_ops) >= 4, "expected >= 4 bop ops, got %d: %s" % (len(bop_ops), bop_ops)
    print("bop ops:", sorted(bop_ops))

    # --- create two overlapping boxes ---
    r1 = s.act("solid.box", {"name": "A", "length": 20, "width": 20, "height": 20})
    assert r1.ok, "solid.box A failed"
    r2 = s.act("solid.box", {"name": "B", "length": 20, "width": 20, "height": 20,
                             "pos": [10, 10, 0]})
    assert r2.ok, "solid.box B failed"

    # --- bop.slice: slice A with B → produces 2 pieces ---
    r = s.act("bop.slice", {"base": "A", "tools": ["B"], "name": "sliced"})
    assert r.ok, "bop.slice failed: %s" % r
    assert r.data["solids"] == 2, "slice should produce 2 solids, got %d" % r.data["solids"]
    print("slice: solids:", r.data["solids"], "vol:", r.data["volume"])

    # --- bop.fragments: decompose A+B → 3 disjoint volume cells ---
    r = s.act("bop.fragments", {"shapes": ["A", "B"], "name": "frags"})
    assert r.ok, "bop.fragments failed: %s" % r
    assert r.data["solids"] == 3, "fragments should produce 3 solids, got %d" % r.data["solids"]
    print("fragments: solids:", r.data["solids"], "vol:", r.data["volume"])

    # --- bop.xor: symmetric difference → 2 non-overlapping pieces ---
    r = s.act("bop.xor", {"shapes": ["A", "B"], "name": "xored"})
    assert r.ok, "bop.xor failed: %s" % r
    # XOR removes the overlap, leaving 2 unique parts
    assert r.data["solids"] == 2, "xor should produce 2 solids, got %d" % r.data["solids"]
    print("xor: solids:", r.data["solids"], "vol:", r.data["volume"])

    # --- bop.connect: fuse keeping internal boundaries ---
    r = s.act("bop.connect", {"shapes": ["A", "B"], "name": "connected"})
    assert r.ok, "bop.connect failed: %s" % r
    assert r.data["solids"] >= 1, "connect should produce >= 1 solid"
    print("connect: solids:", r.data["solids"], "vol:", r.data["volume"])

    # --- guards ---
    r = s.act("bop.slice", {"base": "nonexistent", "tools": ["B"]})
    assert not r.ok, "bop.slice should reject nonexistent base"
    r = s.act("bop.fragments", {"shapes": ["A"]})
    assert not r.ok, "bop.fragments should reject < 2 shapes"
    r = s.act("bop.xor", {"shapes": []})
    assert not r.ok, "bop.xor should reject empty shapes"
    print("guards ok: bad inputs rejected")

    print("BOP SMOKE OK", s.summary())


if __name__ == "__main__":
    main()
