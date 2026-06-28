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
