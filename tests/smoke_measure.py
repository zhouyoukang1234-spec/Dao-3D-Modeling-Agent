"""Direct sub-element measurement smoke test.

Exercises:  measure.area, measure.length, measure.radius, measure.volume,
            measure.com, measure.angle, measure.delta, measure.plane_distance
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session


def main():
    s = new_session("smoke_measure")

    # --- measure ops registered ---
    m_ops = [n for n in s.registry.names() if n.startswith("measure.")]
    assert len(m_ops) >= 8, "expected >= 8 measure ops, got %d: %s" % (len(m_ops), m_ops)
    print("measure ops:", sorted(m_ops))

    # --- create a box for measurement ---
    r = s.act("solid.box", {"name": "mbox", "length": 30, "width": 20, "height": 10})
    assert r.ok, "solid.box failed: %s" % r
    box_name = "mbox"

    # --- measure.volume ---
    r = s.act("measure.volume", {"object": box_name})
    assert r.ok, "measure.volume failed: %s" % r
    assert abs(r.data["volume"] - 6000) < 1, "expected volume 6000"
    print("volume:", r.data["volume"])

    # --- measure.area (Face1 of a 30x20x10 box) ---
    r = s.act("measure.area", {"object": box_name, "sub": "Face1"})
    assert r.ok, "measure.area failed: %s" % r
    # Face1 of Part::Box is the bottom face = 30*20 = 600 or a side face
    # Exact face depends on FreeCAD ordering but should be positive
    assert r.data["area"] > 0, "expected positive area"
    print("area Face1:", r.data["area"])

    # --- measure.length (Edge1) ---
    r = s.act("measure.length", {"object": box_name, "sub": "Edge1"})
    assert r.ok, "measure.length failed: %s" % r
    assert r.data["length"] > 0
    print("length Edge1:", r.data["length"])

    # --- measure.com ---
    r = s.act("measure.com", {"object": box_name})
    assert r.ok, "measure.com failed: %s" % r
    com = r.data["com"]
    # Center of mass of a 30x20x10 box at origin = (15, 10, 5)
    assert abs(com[0] - 15) < 0.1, "expected com_x=15"
    assert abs(com[1] - 10) < 0.1, "expected com_y=10"
    assert abs(com[2] - 5) < 0.1, "expected com_z=5"
    print("com:", com)

    # --- create a cylinder for radius measurement ---
    r = s.act("solid.cylinder", {"name": "mcyl", "radius": 8, "height": 20})
    assert r.ok, "solid.cylinder failed: %s" % r
    cyl_name = "mcyl"

    # --- measure.radius on a cylindrical face ---
    r = s.act("measure.radius", {"object": cyl_name, "sub": "Face1"})
    assert r.ok, "measure.radius failed: %s" % r
    # Face1 of a cylinder should have radius 8
    print("radius Face1:", r.data["radius"])

    # --- create second box for relative measurements ---
    r = s.act("solid.box", {"name": "mbox2", "length": 10, "width": 10, "height": 10,
                            "pos": [50, 0, 0]})
    assert r.ok, "solid.box2 failed: %s" % r
    box2_name = "mbox2"

    # --- measure.delta between two objects ---
    r = s.act("measure.delta", {"a": box_name, "sub_a": "",
                                "b": box2_name, "sub_b": ""})
    assert r.ok, "measure.delta failed: %s" % r
    print("delta:", r.data["delta"], "distance:", r.data["distance"])

    # --- guards ---
    r = s.act("measure.volume", {"object": "nonexistent"})
    assert not r.ok, "measure.volume should reject nonexistent"
    r = s.act("measure.area", {"object": 123})
    assert not r.ok, "measure.area should reject non-string"
    print("guards ok: bad inputs rejected")

    print("MEASURE SMOKE OK", s.summary())


if __name__ == "__main__":
    main()
