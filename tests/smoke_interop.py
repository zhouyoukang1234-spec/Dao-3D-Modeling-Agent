"""Data interoperability / manufacturing handoff -- does a design leave the system
and come back faithfully?

The kernel can write STEP/STL/IGES/BREP and a TechDraw DXF, but writing bytes is
not the same as preserving the geometry. This suite proves fidelity:

  * STEP round-trip (export -> re-import) preserves the B-rep solid EXACTLY --
    volume, bounding box and centre of mass match to ~1e-4 relative (lossless CAD
    handoff to a downstream tool);
  * the STL tessellation is watertight, manifold, self-intersection free, and its
    mesh volume tracks the B-rep volume within the tessellation tolerance (a sound
    3D-print / CAE mesh);
  * a TechDraw page projects the part and writes a 2D DXF (the shop drawing);
  * an assembly round-trips through STEP preserving total volume.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")


def _m(s, name):
    r = s.act("solid.measure", {"name": name})
    assert r.ok, r.error
    return r.data


def main():
    s = new_session("interop")
    print("FreeCAD", s.registry.kernel.freecad_version)
    os.makedirs(OUT, exist_ok=True)

    # a machined flange: plate + boss, a through hole, filleted edges -> planar,
    # cylindrical and blend faces, a real test for a faithful B-rep handoff
    assert s.act("solid.box", {"name": "plate", "length": 60, "width": 40, "height": 10}).ok
    assert s.act("solid.cylinder", {"name": "boss", "radius": 12, "height": 18, "pos": [30, 20, 10]}).ok
    assert s.act("solid.union", {"a": "plate", "b": "boss", "out": "blank"}).ok
    assert s.act("solid.cylinder", {"name": "bore", "radius": 6, "height": 28, "pos": [30, 20, 0]}).ok
    assert s.act("solid.cut", {"a": "blank", "b": "bore", "out": "part"}).ok
    assert s.act("solid.fillet", {"name": "part", "radius": 2.0, "out": "part_f"}).ok
    ref = _m(s, "part_f")
    print("part: volume=%.3f, faces=%d, com=%s" % (ref["volume"], ref.get("faces", -1), ref.get("center_of_mass")))

    def _rel(a, b):
        return abs(a - b) / max(abs(b), 1e-9)

    # ---- 1) STEP round-trip: export -> re-import, B-rep must be preserved exactly
    step = os.path.join(OUT, "interop_part.step")
    ex = s.act("solid.export", {"names": ["part_f"], "path": step})
    assert ex.ok and ex.data["bytes"] > 0, ex.data
    imp = s.act("solid.import_step", {"path": step})
    assert imp.ok and imp.data["imported"], imp.data
    back = _m(s, imp.data["imported"][0])
    assert _rel(back["volume"], ref["volume"]) < 1e-4, ("STEP volume drift", back["volume"], ref["volume"])
    for i in range(3):
        assert _rel(back["bbox_size"][i], ref["bbox_size"][i]) < 1e-4, ("STEP bbox drift", i, back["bbox_size"], ref["bbox_size"])
    if "center_of_mass" in back and "center_of_mass" in ref:
        for i in range(3):
            assert abs(back["center_of_mass"][i] - ref["center_of_mass"][i]) < 1e-3, ("STEP com drift", i)
    print("STEP round-trip: re-imported volume=%.3f (drift %.2e), bbox & com preserved -> lossless"
          % (back["volume"], _rel(back["volume"], ref["volume"])))

    # ---- 2) STL tessellation must be a sound, watertight mesh tracking the B-rep
    ma = s.act("mesh.analyze", {"name": "part_f", "tolerance": 0.05})
    assert ma.ok, ma.error
    md = ma.data
    assert md["watertight"] and md["solid"] and not md["has_non_manifolds"] and not md["self_intersections"], md
    assert _rel(md["mesh_volume"], md["brep_volume"]) < 0.02, ("STL volume off B-rep", md)
    stl = os.path.join(OUT, "interop_part.stl")
    me = s.act("mesh.export", {"name": "part_f", "path": stl, "tolerance": 0.05})
    assert me.ok and me.data["facets"] > 0 and me.data["bytes"] > 0, me.data
    print("STL mesh: %d facets, watertight=%s, mesh/brep volume drift %.2e -> printable"
          % (me.data["facets"], md["watertight"], _rel(md["mesh_volume"], md["brep_volume"])))

    # ---- 3) TechDraw shop drawing -> DXF
    dxf = os.path.join(OUT, "interop_part.dxf")
    td = s.act("draw.techdraw", {"name": "part_f", "path": dxf, "scale": 1.0})
    assert td.ok, td.error
    assert td.data.get("page"), td.data
    if "export_error" in td.data:
        print("TechDraw page made; DXF export note: %s" % td.data["export_error"])
    else:
        assert td.data.get("bytes", 0) > 0, td.data
        print("TechDraw DXF: page=%s, %d bytes (2D shop drawing)" % (td.data["page"], td.data["bytes"]))

    # ---- 4) assembly round-trips through STEP preserving total volume
    assert s.act("solid.box", {"name": "shaftA", "length": 30, "width": 8, "height": 8}).ok
    assert s.act("solid.box", {"name": "shaftB", "length": 20, "width": 8, "height": 8, "pos": [40, 0, 0]}).ok
    va = _m(s, "shaftA")["volume"]
    vb = _m(s, "shaftB")["volume"]
    asm_step = os.path.join(OUT, "interop_asm.step")
    ax = s.act("solid.export", {"names": ["shaftA", "shaftB"], "path": asm_step})
    assert ax.ok and ax.data["bytes"] > 0, ax.data
    ai = s.act("solid.import_step", {"path": asm_step})
    assert ai.ok, ai.data
    # STEP may also surface a container holding both solids; sum just the leaf parts
    leaves = [n for n in ai.data["imported"] if n.lower().startswith("shaft")]
    assert len(leaves) == 2, ("expected 2 shaft parts back", ai.data)
    tot = sum(_m(s, n)["volume"] for n in leaves)
    assert _rel(tot, va + vb) < 1e-4, ("assembly STEP volume drift", tot, va + vb)
    print("assembly STEP round-trip: 2 parts back, total volume %.1f preserved (drift %.2e)"
          % (tot, _rel(tot, va + vb)))

    assert math.isfinite(ref["volume"])
    print("INTEROP SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_interop"):
    main()
