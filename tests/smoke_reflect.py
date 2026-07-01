"""Reflective universal dispatch (L1) + raw OCCT geometry access (L3).

Exercises ``reflect.*``: that *any* FreeCAD Python callable is reachable over the
same pipe as the curated operators, that the marshalling layer round-trips values
and keeps non-JSON objects as chainable handles, that a reflective call shares the
one live document with ``solid.*``, and that raw OCCT geometry (Part.Geom* curves,
Wire -> Face -> Solid construction) can be built and inspected at the kernel level.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("smoke_reflect")
    ops = s.registry.kernel.ops
    for op in ("reflect.roots", "reflect.call", "reflect.get", "reflect.set",
               "reflect.methods", "reflect.help", "reflect.free"):
        assert op in ops, "missing %s" % op
    print("reflect ops present; kernel ops:", len(ops))

    # roots: the whitelisted namespaces, and which are modules.
    r = s.act("reflect.roots", {})
    assert r.ok, r.error
    assert {"App", "FreeCAD", "Part", "doc"} <= set(r.data["roots"]), r.data
    assert "Part" in r.data["modules"] and "doc" not in r.data["modules"], r.data
    print("roots:", len(r.data["roots"]))

    # ---- L1: call any callable, chain via handles ------------------------ #
    r = s.act("reflect.call", {"target": "Part.makeBox", "args": [10, 20, 5]})
    assert r.ok, r.error
    box = r.data["result"]
    assert box["$ref"] and box["class"] == "Solid", box
    assert abs(box["volume"] - 1000) < 1e-6, box
    assert box["faces"] == 6 and box["edges"] == 12 and box["solids"] == 1, box
    ref_box = box["$ref"]

    r = s.act("reflect.call", {"target": "Part.makeBox", "args": [6, 6, 6]})
    ref_b2 = r.data["result"]["$ref"]

    # chain: call a *method* on a returned handle, feeding another handle
    r = s.act("reflect.call", {"on": {"$ref": ref_box}, "method": "common",
                               "args": [{"$ref": ref_b2}]})
    assert r.ok, r.error
    assert abs(r.data["result"]["volume"] - 6 * 6 * 5) < 1e-6, r.data
    print("chained common volume:", round(r.data["result"]["volume"], 3))

    # App.Vector round-trips through the $vec tag both ways
    r = s.act("reflect.call", {"target": "App.Vector", "args": [1, 2, 3]})
    assert r.ok and r.data["result"]["$vec"] == [1, 2, 3], r.data
    # feed a $vec back in: box at an offset point, bbox origin shifts
    r = s.act("reflect.call", {"target": "Part.makeBox",
                               "args": [2, 2, 2, {"$vec": [10, 0, 0]}]})
    assert r.ok, r.error
    # BoundBox of that shape via chained method call returns a handle
    r2 = s.act("reflect.call", {"on": {"$ref": r.data["result"]["$ref"]},
                                "method": "BoundBox"})
    # BoundBox is an attribute, not a method -> reflect.get instead
    r2 = s.act("reflect.get", {"on": {"$ref": r.data["result"]["$ref"]},
                               "attr": "BoundBox"})
    assert r2.ok, r2.error
    print("offset box bbox handle:", r2.data["value"].get("class"))

    # help + methods: live introspection of the same surface capability_map froze
    r = s.act("reflect.help", {"target": "Part.makeBox"})
    assert r.ok and r.data["callable"] and "box" in (r.data["doc"] or "").lower(), r.data
    r = s.act("reflect.methods", {"on": {"$ref": ref_box}})
    assert r.ok and "common" in r.data["callables"] and len(r.data["callables"]) > 50, \
        (len(r.data["callables"]),)
    print("shape method surface:", len(r.data["callables"]))

    # ---- shared document: reflect + solid.* operate on one model --------- #
    s.act("solid.box", {"name": "plate", "length": 8, "width": 8, "height": 8})
    r = s.act("reflect.call", {"target": "doc.getObject", "args": ["plate"]})
    assert r.ok, r.error
    assert r.data["result"]["$obj"] == "plate", r.data
    assert r.data["result"]["type"] == "Part::Feature", r.data
    # read a live property off that doc object by $obj reference
    r = s.act("reflect.get", {"on": {"$obj": "plate"}, "attr": "Label"})
    assert r.ok and r.data["value"] == "plate", r.data
    print("reflect sees curated solid.* object 'plate'")

    # ---- L3: raw OCCT geometry -- curves -> wire -> face -> solid -------- #
    # four corner points of a 30 x 20 rectangle
    pts = [[0, 0, 0], [30, 0, 0], [30, 20, 0], [0, 20, 0]]
    edge_refs = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        seg = s.act("reflect.call", {"target": "Part.LineSegment",
                                     "args": [{"$vec": a}, {"$vec": b}]})
        assert seg.ok, seg.error
        edge = s.act("reflect.call", {"on": {"$ref": seg.data["result"]["$ref"]},
                                      "method": "toShape"})
        assert edge.ok, edge.error
        edge_refs.append({"$ref": edge.data["result"]["$ref"]})
    wire = s.act("reflect.call", {"target": "Part.Wire", "args": [edge_refs]})
    assert wire.ok, wire.error
    assert wire.data["result"]["edges"] == 4, wire.data
    face = s.act("reflect.call", {"target": "Part.Face",
                                  "args": [{"$ref": wire.data["result"]["$ref"]}]})
    assert face.ok, face.error
    assert abs(face.data["result"]["area"] - 600) < 1e-6, face.data
    print("raw OCCT face area:", round(face.data["result"]["area"], 3))
    # extrude the face into a solid via the shape's own .extrude(vector)
    solid = s.act("reflect.call", {"on": {"$ref": face.data["result"]["$ref"]},
                                   "method": "extrude", "args": [{"$vec": [0, 0, 12]}]})
    assert solid.ok, solid.error
    assert abs(solid.data["result"]["volume"] - 600 * 12) < 1e-6, solid.data
    print("raw OCCT extruded solid volume:", round(solid.data["result"]["volume"], 3))

    # a raw OCCT circle curve, kernel-level radius/length inspection
    circ = s.act("reflect.call", {"target": "Part.Circle",
                                  "args": [{"$vec": [0, 0, 0]}, {"$vec": [0, 0, 1]}, 5.0]})
    assert circ.ok, circ.error
    r = s.act("reflect.get", {"on": {"$ref": circ.data["result"]["$ref"]},
                              "attr": "Radius"})
    assert r.ok and abs(r.data["value"] - 5.0) < 1e-9, r.data
    # mutate the curve radius through reflect.set, then re-read
    r = s.act("reflect.set", {"on": {"$ref": circ.data["result"]["$ref"]},
                              "attr": "Radius", "value": 7.5})
    assert r.ok and abs(r.data["value"] - 7.5) < 1e-9, r.data
    cedge = s.act("reflect.call", {"on": {"$ref": circ.data["result"]["$ref"]},
                                   "method": "toShape"})
    r = s.act("reflect.get", {"on": {"$ref": cedge.data["result"]["$ref"]},
                              "attr": "Length"})
    assert r.ok and abs(r.data["value"] - 2 * math.pi * 7.5) < 1e-6, r.data
    print("raw OCCT circle: radius dialed 5->7.5, circumference",
          round(r.data["value"], 3))

    # ---- guards --------------------------------------------------------- #
    r = s.act("reflect.call", {"target": "Nope.foo"})
    assert not r.ok and "unknown root" in (r.error or ""), r
    r = s.act("reflect.call", {"target": "Part.thereIsNoSuchThing"})
    assert not r.ok and "no attribute" in (r.error or ""), r
    r = s.act("reflect.call", {"target": "App.Version", "args": "notalist"})
    assert not r.ok and "must be a list" in (r.error or ""), r
    r = s.act("reflect.call", {})
    assert not r.ok and "target" in (r.error or ""), r
    r = s.act("reflect.get", {"on": {"$obj": "does_not_exist"}, "attr": "Label"})
    assert not r.ok and "no document object" in (r.error or ""), r
    r = s.act("reflect.call", {"on": {"$ref": 999999}, "method": "x"})
    assert not r.ok and "unknown handle" in (r.error or ""), r
    r = s.act("reflect.set", {"on": {"$obj": "plate"}, "attr": "Nope", "value": 1})
    assert not r.ok and "no attribute" in (r.error or ""), r
    print("guards ok: unknown root/attr, bad args, missing target/obj/handle")

    # free everything
    r = s.act("reflect.free", {"all": True})
    assert r.ok and r.data["freed"] > 0, r.data
    print("freed handles:", r.data["freed"])

    print("SMOKE OK reflect", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
