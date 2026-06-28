import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("smoke")
    print("FreeCAD", s.registry.kernel.freecad_version)
    print("ops:", len(s.registry.kernel.ops), "tools:", len(s.tools()))

    # flange = plate with a centered hole, via live booleans
    r = s.act("solid.box", {"name": "plate", "length": 60, "width": 40, "height": 8})
    assert r.ok, r.error
    r = s.act("solid.cylinder", {"name": "hole", "radius": 8, "height": 8, "pos": [30, 20, 0]})
    assert r.ok, r.error
    r = s.act("solid.cut", {"a": "plate", "b": "hole", "out": "flange"})
    assert r.ok, r.error
    print("flange volume", r.data["volume"])

    # verify volume = box - cylinder
    import math
    expect_v = 60 * 40 * 8 - math.pi * 8 * 8 * 8
    v = s.verify("flange", {"volume": expect_v, "valid": True}, tol=1e-2)
    print("verify flange:", v.ok, v.error or v.data.get("measured", {}).get("volume"))
    assert v.ok, v.data

    # fillet all edges, then mass-properties inspect
    r = s.act("solid.fillet", {"name": "flange", "radius": 1.5, "out": "flange_f"})
    print("fillet ok", r.ok, "faces", r.data.get("faces"))
    r = s.act("solid.inspect", {"name": "flange_f", "density": 0.00785})  # steel g/mm^3
    print("inspect mass(g)", r.data.get("mass"), "com", r.data.get("center_of_mass"))
    # inspect must report the structural counts the reverse pipeline keys on:
    # is this one body or an assembly? (a downloaded STEP that came in as a
    # single-solid compound vs a real multi-solid assembly).
    assert r.data["solids"] == 1 and r.data["shells"] == 1, r.data
    s.act("solid.box", {"name": "tb1", "length": 5, "width": 5, "height": 5})
    s.act("solid.box", {"name": "tb2", "length": 5, "width": 5, "height": 5, "pos": [20, 0, 0]})
    s.act("solid.compound", {"names": ["tb1", "tb2"], "out": "asm2"})
    ai = s.act("solid.inspect", {"name": "asm2"})
    assert ai.data["solids"] == 2 and ai.data["shells"] == 2, ai.data
    print("inspect structure: single body solids=1; 2-box compound solids=2")

    # selecting an out-of-range edge must fail with a clear message (which index,
    # what range), not leak a bare IndexError the caller cannot act on.
    s.act("solid.box", {"name": "cube", "length": 10, "width": 10, "height": 10})
    fr = s.act("solid.fillet", {"name": "cube", "radius": 1, "edges": [999], "out": "cf"})
    assert not fr.ok and "out of range" in (fr.error or ""), fr
    cr = s.act("solid.chamfer", {"name": "cube", "size": 1, "edges": [999], "out": "cc"})
    assert not cr.ok and "out of range" in (cr.error or ""), cr
    ok_edge = s.act("solid.chamfer", {"name": "cube", "size": 1, "edges": [0], "out": "cc"})
    assert ok_edge.ok, ok_edge.error
    print("edge-index guard ok: out-of-range rejected, valid index chamfers")

    # interference: overlap two boxes
    s.act("solid.box", {"name": "A", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "B", "length": 10, "width": 10, "height": 10, "pos": [5, 0, 0]})
    r = s.act("solid.interference", {"a": "A", "b": "B"})
    print("interference", r.data)
    assert r.data["interfering"] and abs(r.data["overlap_volume"] - 500) < 1e-2

    # export STEP + STL
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)
    r = s.act("solid.export", {"names": ["flange_f"], "path": os.path.join(out, "flange.step")})
    print("export step bytes", r.data.get("bytes"))
    r = s.act("solid.export", {"names": ["flange_f"], "path": os.path.join(out, "flange.stl")})
    print("export stl bytes", r.data.get("bytes"))

    print("SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
