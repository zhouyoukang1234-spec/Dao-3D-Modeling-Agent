import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")


def main():
    s = new_session("adv")
    k = s.registry.kernel
    print("adv ops:", [o for o in k.ops if o.split(".")[0] in ("ss", "analyze", "mesh", "draw", "view")])
    os.makedirs(OUT, exist_ok=True)

    # build a parametric bracket
    assert s.act("param.body", {"name": "Brk"}).ok
    assert s.act("param.pad", {"body": "Brk", "feature": "Plate",
                              "profile": {"rect": [40, 30]}, "length": 5}).ok
    assert s.act("param.pocket", {"body": "Brk", "feature": "Hole",
                                 "profile": {"circle": 5}, "through": True}).ok

    # --- spreadsheet-driven parameters ---
    assert s.act("ss.create", {"cells": {"thickness": 5, "hole": 5}}).ok
    assert s.act("ss.bind", {"param": "Plate.length", "alias": "thickness"}).ok
    assert s.act("ss.bind", {"param": "Hole.radius", "alias": "hole"}).ok
    v0 = s.act("param.measure", {"body": "Brk"}).data["volume"]
    # drive thickness 5 -> 9 via the table
    assert s.act("ss.set", {"alias": "thickness", "value": 9}).ok
    v1 = s.act("param.measure", {"body": "Brk"}).data["volume"]
    print("spreadsheet drove volume %.1f -> %.1f" % (v0, v1))
    assert v1 > v0, "spreadsheet thickness change should grow volume"
    # expected: 40*30*9 - pi*25*9
    assert abs(v1 - (40 * 30 * 9 - math.pi * 25 * 9)) < 1.0, v1
    print("ss.table", s.act("ss.table", {}).data["table"])

    # --- spreadsheet drives an integer pattern count (Occurrences) ---
    assert s.act("param.body", {"name": "Disc"}).ok
    assert s.act("param.pad", {"body": "Disc", "feature": "D",
                               "profile": {"circle": 40}, "length": 6}).ok
    db = s.act("param.measure", {"body": "Disc"}).data["volume"]
    assert s.act("param.pocket", {"body": "Disc", "feature": "H",
                                  "profile": {"circle": 3, "at": [30, 0]}, "through": True}).ok
    hv = db - s.act("param.measure", {"body": "Disc"}).data["volume"]
    assert s.act("param.pattern_polar", {"body": "Disc", "feature": "Ring",
                                         "originals": ["H"], "count": 6, "angle": 360}).ok
    assert s.act("ss.create", {"cells": {"nbolts": 6}}).ok  # add alias to the existing sheet
    assert s.act("ss.bind", {"param": "Ring.occurrences", "alias": "nbolts"}).ok
    assert s.act("ss.set", {"alias": "nbolts", "value": 12}).ok
    removed = db - s.act("param.measure", {"body": "Disc"}).data["volume"]
    print("table drove Occurrences -> holes:", round(removed / hv, 2))
    assert abs(removed - 12 * hv) < 1.0, "spreadsheet did not drive pattern Occurrences"

    # --- section analysis ---
    r = s.act("analyze.section", {"name": "Brk", "plane": "XY", "offset": 2})
    print("section @z=2:", r.data)
    assert r.data["wires"] >= 1

    # --- min distance between two solids ---
    s.act("solid.box", {"name": "b1", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "b2", "length": 10, "width": 10, "height": 10, "pos": [25, 0, 0]})
    r = s.act("analyze.distance", {"a": "b1", "b": "b2"})
    print("distance b1<->b2:", r.data["min_distance"])
    assert abs(r.data["min_distance"] - 15) < 1e-6

    # --- mesh watertightness ---
    r = s.act("mesh.analyze", {"name": "Brk", "tolerance": 0.2})
    print("mesh:", {k2: r.data[k2] for k2 in ("facets", "watertight", "mesh_volume", "brep_volume")})
    assert r.data["watertight"], r.data

    # --- malformed-input guards (no_raw_leak): non-numeric tolerance/scale and
    #     a non-string export path used to leak raw 'could not convert string to
    #     float' / TypeError; they must be guided. -----------------------------
    def _guided(r, token):
        err = r.error or ""
        assert not r.ok, "expected failure, got %r" % (r.data,)
        for raw in ("TypeError", "could not convert", "AttributeError"):
            assert raw not in err, "leaked raw %s: %r" % (raw, err)
        assert token in err, "error %r lacks %r" % (err, token)

    _guided(s.act("mesh.analyze", {"name": "Brk", "tolerance": "x"}), "must be a number")
    _guided(s.act("mesh.export", {"name": "Brk", "path": 123}), "path")
    _guided(s.act("mesh.export", {"name": "Brk", "path": os.path.join(OUT, "b.stl"),
                                  "tolerance": "x"}), "must be a number")
    _guided(s.act("draw.techdraw", {"name": "Brk", "scale": "x"}), "must be a number")
    # analyze.section coerced offset with a bare float() and indexed an unchecked
    # plane dict: a non-numeric offset / bad plane must guide, not leak.
    _guided(s.act("analyze.section", {"name": "Brk", "offset": "x"}),
            "must be a number")
    _guided(s.act("analyze.section", {"name": "Brk", "plane": "QQ"}),
            "'XY'/'XZ'/'YZ'")
    _guided(s.act("analyze.section", {"name": "Brk", "plane": 123}),
            "'XY'/'XZ'/'YZ'")
    print("mesh/draw malformed-input guards refused cleanly")

    # --- perception renders ---
    r = s.act("view.render", {"names": ["Brk"], "view": "iso",
                             "path": os.path.join(OUT, "brk_iso.png")})
    print("render iso bytes:", r.data.get("bytes"))
    assert r.data.get("bytes", 0) > 0
    r = s.act("view.views", {"names": ["Brk"], "path": os.path.join(OUT, "brk_views.png")})
    print("contact sheet bytes:", r.data.get("bytes"))
    assert r.data.get("bytes", 0) > 0

    # --- TechDraw 2D drawing ---
    r = s.act("draw.techdraw", {"name": "Brk", "path": os.path.join(OUT, "brk.dxf")})
    print("techdraw:", {k2: r.data.get(k2) for k2 in ("page", "views", "bytes", "export_error")})
    assert r.data.get("page")

    # --- STEP roundtrip: export a PARAMETRIC body, reimport, compare ---
    step = os.path.join(OUT, "brk.step")
    ex = s.act("solid.export", {"names": ["Brk"], "path": step})  # param body via solid.export
    assert ex.ok and ex.data["bytes"] > 0, ex.error
    before = s.act("solid.list", {}).data
    im = s.act("solid.import_step", {"path": step})
    assert im.ok, im.error
    # exactly one real solid is registered (not the datum planes / sketches / features)
    assert len(im.data["imported"]) == 1, im.data["imported"]
    v_param = s.act("param.measure", {"body": "Brk"}).data["volume"]
    rt = s.act("solid.inspect", {"name": im.data["imported"][0]}).data
    assert abs(rt["volume"] - v_param) < v_param * 1e-3, (rt["volume"], v_param)
    print("step roundtrip:", before, "->", s.act("solid.list", {}).data,
          "vol", round(rt["volume"], 1))

    # --- mixed parametric + BREP boolean ---
    # cut an imported (BREP) solid out of a parametric body; the boolean must
    # consume the body's tip shape via _get's body fallback.
    gname = im.data["imported"][0]
    assert s.act("param.body", {"name": "Slab40"}).ok
    assert s.act("param.pad", {"body": "Slab40", "feature": "S",
                               "profile": {"rect": [60, 60]}, "length": 20}).ok
    slabv = s.act("param.measure", {"body": "Slab40"}).data["volume"]
    # default out would shadow the body name -> must be rejected
    bad = s.act("solid.cut", {"a": "Slab40", "b": gname})
    assert not bad.ok and "parametric body" in (bad.error or ""), bad.error
    # intersection of body + imported BREP solid, then cut: volume must balance
    common = s.act("solid.common", {"a": "Slab40", "b": gname, "out": "Overlap"})
    assert common.ok, common.error
    cut = s.act("solid.cut", {"a": "Slab40", "b": gname, "out": "Slab_cut"})
    assert cut.ok, cut.error
    assert abs(cut.data["volume"] - (slabv - common.data["volume"])) < slabv * 1e-3, \
        (cut.data["volume"], slabv, common.data["volume"])
    assert s.act("param.measure", {"body": "Slab40"}).data["volume"] == slabv  # body intact
    print("mixed cut:", round(slabv, 1), "- overlap", round(common.data["volume"], 1),
          "=", round(cut.data["volume"], 1))

    print("ADV SMOKE OK", s.summary())
    k.shutdown()


if __name__ == "__main__":
    main()
