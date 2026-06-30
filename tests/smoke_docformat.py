"""Fuse the file layer with the API layer.

Reverse-engineering FreeCAD to its root means owning the ``.FCStd`` persistence
format, not just the scripting surface. This suite builds a document on the live
kernel, saves it, then reads it back *without the kernel* via
:mod:`cad_agent.docformat` and proves the two views agree:

* the object graph parsed from ``Document.xml`` == the kernel's live object list
  (same names, same ``TypeId``s) -- the file layer and the API layer are the
  same truth;
* the dependency DAG and per-shape BREP files are recovered from the file alone;
* the saved file re-opens in the kernel with the very names the file-level
  parser reported -- a real round-trip; and
* malformed inputs are guided, never leaking a raw zip/XML error.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import docformat, new_session  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")


def main():
    os.makedirs(OUT, exist_ok=True)
    s = new_session("doc")

    # A parametric body builds a real dependency DAG (Body -> Origin / Sketch /
    # Pad), plus a couple of plain solids -- a representative document.
    assert s.act("param.body", {"name": "Brk"}).ok
    assert s.act("param.pad", {"body": "Brk", "feature": "Plate",
                               "profile": {"rect": [40, 30]}, "length": 5}).ok
    assert s.act("solid.box", {"name": "Blk", "length": 10, "width": 10,
                               "height": 10}).ok
    assert s.act("solid.cylinder", {"name": "Pin", "radius": 3,
                                    "height": 12}).ok

    # The model lives in the kernel subprocess; doc.info reports the live object
    # list and doc.save persists the .FCStd we then parse without the kernel.
    live = {o["name"]: o["type"]
            for o in s.act("doc.info", {}).data["objects"]}
    path = os.path.join(OUT, "fusion.FCStd")
    assert s.act("doc.save", {"path": path}).ok

    # ---- file layer == API layer ---------------------------------------- #
    info = docformat.inspect_document(path)
    assert info["schema_version"], info
    assert "1.0" in (info["program_version"] or ""), info
    file_objs = {o["name"]: o["type"] for o in info["objects"]}
    assert file_objs == live, ("file vs kernel object graph diverged",
                               file_objs, live)
    assert info["object_count"] == len(live), info
    # the parametric body must have produced real dependency edges...
    assert info["dependency_edges"] > 0, info["dependencies"]
    # ...and every dep target must itself be a real object (no dangling edges).
    for src, dsts in info["dependencies"].items():
        assert src in file_objs, (src, file_objs)
        for dst in dsts:
            assert dst in file_objs, (dst, file_objs)
    # shapes persisted as BREP files; total geometry is non-trivial (some
    # internal placeholder shapes are legitimately empty, so check the sum).
    assert info["brep_files"], info
    assert info["brep_bytes"] > 0, info
    for b in info["brep_files"]:
        assert b["file"].lower().endswith((".brp", ".brep")), b
    print("docformat: %d objects, %d dep-edges, %d BREP files (%d bytes) -- "
          "file graph == kernel graph"
          % (info["object_count"], info["dependency_edges"],
             len(info["brep_files"]), info["brep_bytes"]))

    # ---- the fingerprint is stable across a re-save (structure, not bytes) - #
    fp1 = docformat.fingerprint(path)
    path2 = os.path.join(OUT, "fusion2.FCStd")
    assert s.act("doc.save", {"path": path2}).ok
    assert docformat.fingerprint(path2) == fp1, (fp1, docformat.fingerprint(path2))

    # ---- real round-trip: the saved file re-opens in the kernel with the
    #      names the file-level parser reported (no kernel used to parse). ---- #
    import FreeCAD as App
    reopened = App.openDocument(path)
    try:
        reloaded = {o.Name for o in reopened.Objects}
    finally:
        App.closeDocument(reopened.Name)
    assert reloaded == set(file_objs), (reloaded, set(file_objs))
    print("docformat: re-opened %s -> %d objects match the file-level parse"
          % (os.path.basename(path), len(reloaded)))

    # ---- diff: the verify half -- what an edit changed, read from files --- #
    # identical documents diff to nothing.
    same = docformat.diff(path, path2)
    assert same["identical"], same
    # now make real edits on the kernel and prove the file-level diff reports
    # exactly them -- across all three layers a change can hide in:
    #   * a new object           (Document.xml object graph)
    #   * a resized plain solid  (the BREP file -- not Document.xml)
    #   * a spreadsheet cell      (a complex container property)
    assert s.act("solid.cylinder", {"name": "Pin2", "radius": 2,
                                     "height": 8}).ok        # new object
    assert s.act("solid.box", {"name": "Blk", "length": 25, "width": 10,
                               "height": 10}).ok             # resize -> new BREP
    assert s.act("ss.create", {"cells": {"k": 1}}).ok        # sheet (also new)
    assert s.act("ss.set", {"alias": "k", "value": 42}).ok   # edit a cell
    path3 = os.path.join(OUT, "fusion3.FCStd")
    assert s.act("doc.save", {"path": path3}).ok
    d = docformat.diff(path, path3)
    assert not d["identical"], d
    assert "Pin2" in d["objects_added"], d
    assert d["objects_removed"] == [], d
    # the resized box surfaces only in its BREP hash (geometry, not Document.xml).
    assert "Blk.Shape.brp" in d["brep_changes"], d
    print("docformat.diff: +%s objects, %d BREP changed -- edit read from files"
          % (d["objects_added"], len(d["brep_changes"])))

    # a complex container property (a spreadsheet cell) edited on a *shared*
    # object is caught via the canonical-XML value, not lost.
    s.act("solid.box", {"name": "Anchor", "length": 1, "width": 1, "height": 1})
    s.act("ss.create", {"cells": {"w": 3}})
    base = os.path.join(OUT, "cellA.FCStd")
    assert s.act("doc.save", {"path": base}).ok
    assert s.act("ss.set", {"alias": "w", "value": 7}).ok
    after = os.path.join(OUT, "cellB.FCStd")
    assert s.act("doc.save", {"path": after}).ok
    cd = docformat.diff(base, after)
    assert "Spreadsheet" in cd["property_changes"], cd
    print("docformat.diff: spreadsheet cell edit caught on shared object")

    # ---- edit_property: the act half -- author the file, kernel honours it - #
    # build a parametric pad (Length is a plain scalar in Document.xml), save,
    # then change its Length purely by file surgery -- no kernel touched.
    e = new_session("edit")
    assert e.act("param.body", {"name": "Body"}).ok
    assert e.act("param.pad", {"body": "Body", "feature": "Pad",
                               "profile": {"rect": [40, 30]}, "length": 5}).ok
    v0 = e.act("param.measure", {"body": "Body"}).data["volume"]
    src = os.path.join(OUT, "edit_src.FCStd")
    assert e.act("doc.save", {"path": src}).ok
    dst = os.path.join(OUT, "edit_dst.FCStd")
    r = docformat.edit_property(src, "Pad", "Length", 15, out=dst)
    assert float(r["old"]) == 5.0 and float(r["new"]) == 15.0, r
    # the file-level view confirms the new value without any kernel.
    assert docformat.inspect_document(dst)["properties"]["Pad"]["Length"][
        "value"] == 15, docformat.inspect_document(dst)["properties"]["Pad"]
    ed = docformat.diff(src, dst)
    assert ed["property_changes"]["Pad"]["Length"] == {"from": 5, "to": 15}, ed
    assert ed["objects_added"] == [] and ed["objects_removed"] == [], ed
    # now prove the *kernel* honours the file edit: reopen, force recompute, and
    # the body's volume reflects the file-authored Length (40*30*15 = 18000).
    import FreeCAD as App
    doc2 = App.openDocument(dst)
    try:
        pad = doc2.getObject("Pad")
        pad.touch()
        doc2.recompute(None, True)
        body = next(o for o in doc2.Objects
                    if o.TypeId.startswith("PartDesign::Body"))
        vol = body.Shape.Volume
    finally:
        App.closeDocument(doc2.Name)
    assert abs(vol - 40 * 30 * 15) < 1.0, (vol, v0)
    print("docformat.edit_property: file-authored Length 5->15 -> kernel volume "
          "%.0f->%.0f (file edit drives geometry)" % (v0, vol))

    # guarded file-level edits.
    _ep = docformat.edit_property
    for call, token in (
        (lambda: _ep(src, "Nope", "Length", 1), "no object"),
        (lambda: _ep(src, "Pad", "Nope", 1), "no property"),
        (lambda: _ep(src, "Pad", "Placement", 1), "not a simple scalar"),
    ):
        try:
            call()
        except ValueError as exc:
            assert token in str(exc), (token, str(exc))
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat.edit_property: malformed edits guided")

    # ---- guided failures (no raw BadZipFile / KeyError leak) ------------- #
    def _guided(call, token):
        try:
            call()
        except ValueError as e:
            assert token in str(e), (token, str(e))
            return
        raise AssertionError("expected ValueError for %r" % token)

    _guided(lambda: docformat.inspect_document(""), "non-empty string")
    _guided(lambda: docformat.inspect_document("/tmp/_nope.FCStd"), "no such file")
    notzip = os.path.join(OUT, "notzip.FCStd")
    with open(notzip, "w") as fh:
        fh.write("this is not a zip")
    _guided(lambda: docformat.inspect_document(notzip), "not a .FCStd")
    print("docformat: malformed inputs guided (no raw zip/XML leak)")

    # ---- the meta-tool as first-class ops: doc.inspect / diff / edit ----- #
    # the persistence layer is now usable through the same s.act(...) loop as
    # every kernel op -- file-level perceive / verify / act, fused in.
    o = new_session("ops")
    assert o.act("param.body", {"name": "Body"}).ok
    assert o.act("param.pad", {"body": "Body", "feature": "Pad",
                               "profile": {"rect": [20, 20]}, "length": 4}).ok
    op_a = os.path.join(OUT, "ops_a.FCStd")
    assert o.act("doc.save", {"path": op_a}).ok
    ins = o.act("doc.inspect", {"path": op_a})
    assert ins.ok and ins.data["properties"]["Pad"]["Length"]["value"] == 4, ins
    op_b = os.path.join(OUT, "ops_b.FCStd")
    ed = o.act("doc.edit", {"path": op_a, "object": "Pad",
                            "property": "Length", "value": 9, "out": op_b})
    assert ed.ok and float(ed.data["new"]) == 9.0, ed
    df = o.act("doc.diff", {"a": op_a, "b": op_b})
    assert df.ok and df.data["property_changes"]["Pad"]["Length"] == {
        "from": 4, "to": 9}, df
    # guided through the op layer too (no raw exception leak).
    assert not o.act("doc.inspect", {"path": ""}).ok
    assert not o.act("doc.edit", {"path": op_a, "object": "Nope",
                                  "property": "Length", "value": 1}).ok
    print("doc.inspect/diff/edit: persistence meta-tool fused into the op loop")

    # ---- expression wiring: the parametric graph read from the file alone -- #
    # bind a pad's Length to a spreadsheet alias via the ExpressionEngine, then
    # recover that wiring from the .FCStd with no kernel: the formula, and the
    # cross-object edge (Pad -> Spreadsheet) that no App::PropertyLink carries.
    x = new_session("expr")
    assert x.act("param.body", {"name": "Body"}).ok
    assert x.act("param.pad", {"body": "Body", "feature": "Pad",
                               "profile": {"rect": [40, 30]}, "length": 5}).ok
    assert x.act("ss.create", {"cells": {"plen": 5, "pwid": 9}}).ok
    assert x.act("ss.bind", {"param": "Pad.length", "alias": "plen"}).ok
    ex_a = os.path.join(OUT, "expr_a.FCStd")
    assert x.act("doc.save", {"path": ex_a}).ok

    ix = docformat.inspect_document(ex_a)
    assert ix["expression_count"] >= 1, ix["expressions"]
    assert "Pad" in ix["expressions"], ix["expressions"]
    bound = {e["path"]: e["formula"] for e in ix["expressions"]["Pad"]}
    assert bound.get("Length") == "Spreadsheet.plen", ix["expressions"]
    # the formula references Spreadsheet, an edge the recompute link DAG lacks:
    assert "Pad->Spreadsheet" in ix["expression_edges"], ix["expression_edges"]
    sheet = next(o["name"] for o in ix["objects"]
                 if (o["type"] or "").startswith("Spreadsheet"))
    assert ("Pad->%s" % sheet) in ix["expression_edges"], ix["expression_edges"]
    # every expression edge target is a real object (no dangling refs).
    file_names = {o["name"] for o in ix["objects"]}
    for edge in ix["expression_edges"]:
        src, dst = edge.split("->")
        assert src in file_names and dst in file_names, (edge, file_names)
    # a document with no expressions reports zero, not noise.
    assert docformat.inspect_document(path)["expression_count"] == 0, path
    print("docformat: %d expression(s), edges=%s -- parametric wiring from file"
          % (ix["expression_count"], ix["expression_edges"]))

    # ---- the second parametric graph: Sketcher constraints (kernel-free) -- #
    # the fusion doc's pad profile is a fully-constrained sketch; its constraint
    # list -- the solver graph the GUI authors click by click -- is read here
    # straight from the file, with its named driving dimensions surfaced as the
    # user-facing knobs (width / height).
    sinfo = docformat.inspect_document(path)
    assert sinfo["sketch_constraint_count"] > 0, sinfo
    assert sinfo["sketch_dimensions"], sinfo["sketches"]
    sk_name, sk = next(iter(sinfo["sketches"].items()))
    tnames = {c["type_name"] for c in sk["constraints"]}
    # the rectangle resolves to the canonical solver mix, every type id mapped.
    assert {"Coincident", "DistanceX", "DistanceY"} <= tnames, tnames
    assert not any(t.startswith("Type") for t in tnames), tnames
    assert sk["dimensions"].get("width") == 40, sk["dimensions"]
    # kernel cross-check: the live sketch holds exactly the constraints the file
    # parser recovered -- the constraint graph is one truth across both layers.
    rp = App.openDocument(path)
    try:
        assert len(rp.getObject(sk_name).Constraints) == sk["count"], sk["count"]
    finally:
        App.closeDocument(rp.Name)
    print("docformat: %d sketch constraint(s), dims=%s -- solver graph from file "
          "== kernel" % (sinfo["sketch_constraint_count"], sinfo["sketch_dimensions"]))

    # a re-dialled named dimension surfaces in diff.dimension_changes -- a
    # parametric edit the collapsed constraint-list blob diff can't otherwise see.
    def _padded(tag, width):
        sess = new_session("dim" + tag)
        assert sess.act("param.body", {"name": "Bd"}).ok
        assert sess.act("param.pad", {"body": "Bd", "feature": "Pl",
                                      "profile": {"rect": [width, 30]},
                                      "length": 5}).ok
        out = os.path.join(OUT, "dim_%s.FCStd" % tag)
        assert sess.act("doc.save", {"path": out}).ok
        return out
    dim_a, dim_b = _padded("a", 40), _padded("b", 55)
    dd = docformat.diff(dim_a, dim_b)
    assert not dd["identical"], dd
    dk = next(k for k in dd["dimension_changes"] if k.endswith(".width"))
    assert dd["dimension_changes"][dk] == {"from": 40, "to": 55}, dd["dimension_changes"]
    assert docformat.diff(dim_a, dim_a)["dimension_changes"] == {}, "self-diff"
    print("docformat.diff: re-dialled sketch dimension named in dimension_changes")

    # ---- set_dimension: the act half for the constraint graph ------------- #
    # re-dial the sketch's named driving 'width' from 40 to 70 purely by file
    # surgery, then prove the kernel re-solves the sketch and rebuilds the pad:
    # a file-authored constraint edit reshapes real geometry (70*30*5 = 10500).
    sk_id = next(iter(docformat.inspect_document(dim_a)["sketches"]))
    dim_set = os.path.join(OUT, "dim_set.FCStd")
    sr = docformat.set_dimension(dim_a, sk_id, "width", 70, out=dim_set)
    assert sr["old"] == 40 and sr["new"] == 70, sr
    assert docformat.inspect_document(dim_set)["sketch_dimensions"][
        "%s.width" % sk_id] == 70, dim_set
    doc5 = App.openDocument(dim_set)
    try:
        for o in doc5.Objects:
            o.touch()
        doc5.recompute(None, True)
        body5 = next(o for o in doc5.Objects
                     if o.TypeId.startswith("PartDesign::Body"))
        vol5 = body5.Shape.Volume
    finally:
        App.closeDocument(doc5.Name)
    assert abs(vol5 - 70 * 30 * 5) < 1.0, vol5
    print("docformat.set_dimension: file-authored width 40->70 -> kernel volume "
          "%.0f (constraint edit reshapes geometry)" % vol5)

    # guarded: a missing sketch, an absent/geometric dimension, a bad value.
    _sd = docformat.set_dimension
    for call, token in (
            (lambda: _sd(dim_a, "Nope", "width", 1), "no object"),
            (lambda: _sd(dim_a, sk_id, "depth", 1), "named driving dimension"),
            (lambda: _sd(dim_a, sk_id, "width", "big"), "must be a number")):
        try:
            call()
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat.set_dimension: malformed edits guided")

    # ---- the geometry root: BREP topology census (kernel-free) ----------- #
    # every shape persists to an OpenCASCADE .brp -- the geometric root the API
    # only wraps. inspect_document counts it straight from the file; the census
    # must equal the kernel's own Shape.Solids / Faces / Edges / Vertexes.
    gb = new_session("brep")
    assert gb.act("solid.box", {"name": "Bx", "length": 10, "width": 8,
                                "height": 6}).ok
    box_p = os.path.join(OUT, "brep_box.FCStd")
    assert gb.act("doc.save", {"path": box_p}).ok
    bi = docformat.inspect_document(box_p)
    solid = next(b for b in bi["brep_files"] if b["topology"]["solids"] == 1)
    assert solid["version"] == "V1", solid
    assert solid["topology"] == {"vertices": 8, "edges": 12, "wires": 6,
                                 "faces": 6, "shells": 1, "solids": 1,
                                 "compsolids": 0, "compounds": 0}, solid["topology"]
    # the geometry tables the shape references are surfaced too (a box: 6 planes).
    assert solid["sections"].get("surfaces") == 6, solid["sections"]
    assert bi["topology_totals"]["solids"] >= 1, bi["topology_totals"]
    # kernel cross-check: the file census == the live shape's own topology.
    bd = App.openDocument(box_p)
    try:
        sh = bd.getObject("Bx").Shape
        kernel_topo = {"vertices": len(sh.Vertexes), "edges": len(sh.Edges),
                       "wires": len(sh.Wires), "faces": len(sh.Faces),
                       "shells": len(sh.Shells), "solids": len(sh.Solids)}
    finally:
        App.closeDocument(bd.Name)
    for k, v in kernel_topo.items():
        assert solid["topology"][k] == v, (k, solid["topology"][k], v)
    print("docformat: BREP topology %s -- the geometry root from file == kernel"
          % {k: v for k, v in solid["topology"].items() if v})

    # ---- synthesize: author a whole model from nothing (no kernel) -------- #
    # the most upstream act -- a model written as a file, the way code is. A box
    # and a placed cylinder are authored straight into Document.xml with no BREP;
    # the kernel generates the geometry on its first forced recompute, proving a
    # hand-written file *is* a model.
    syn_p = os.path.join(OUT, "synth.FCStd")
    sres = docformat.synthesize(syn_p, [
        {"type": "Part::Box", "name": "Blk",
         "properties": {"Length": 12, "Width": 7, "Height": 4}},
        {"type": "Part::Cylinder", "name": "Cyl",
         "properties": {"Radius": 5, "Height": 10},
         "placement": {"position": [30, 0, 0]}},
    ])
    assert sres["object_count"] == 2, sres
    # the file layer reads back exactly what was authored -- no kernel involved.
    syn_ix = docformat.inspect_document(syn_p)
    assert syn_ix["type_counts"] == {"Part::Box": 1, "Part::Cylinder": 1}, syn_ix
    assert syn_ix["brep_files"] == [], "authored file carries no geometry"
    # the kernel realises the geometry from the authored scalars alone.
    sd = App.openDocument(syn_p)
    try:
        for o in sd.Objects:
            o.touch()
        sd.recompute(None, True)
        blk, cyl = sd.getObject("Blk"), sd.getObject("Cyl")
        box_vol, cyl_vol = blk.Shape.Volume, cyl.Shape.Volume
        cyl_x = cyl.Placement.Base.x
    finally:
        App.closeDocument(sd.Name)
    assert abs(box_vol - 12 * 7 * 4) < 1e-6, box_vol
    assert abs(cyl_vol - math.pi * 25 * 10) < 1.0, cyl_vol
    assert abs(cyl_x - 30) < 1e-6, cyl_x
    print("docformat.synthesize: hand-authored file -> kernel builds box %.0f + "
          "cylinder %.0f (a written file is a model)" % (box_vol, cyl_vol))

    # ---- synthesize parametric: an expression-driven model from nothing -- #
    # a box whose Height is bound to Cyl.Radius*2, authored straight to file;
    # the cross-object reference becomes a dependency edge, and the kernel
    # evaluates the formula on recompute -- file-first *parametric* modelling.
    par_p = os.path.join(OUT, "synth_param.FCStd")
    docformat.synthesize(par_p, [
        {"type": "Part::Cylinder", "name": "Cyl",
         "properties": {"Radius": 5, "Height": 10}},
        {"type": "Part::Box", "name": "Bx2",
         "properties": {"Length": 3, "Width": 3, "Height": 1},
         "expressions": {"Height": "Cyl.Radius * 2"}},
    ])
    par_ix = docformat.inspect_document(par_p)
    assert par_ix["expression_edges"] == ["Bx2->Cyl"], par_ix["expression_edges"]
    assert par_ix["expressions"]["Bx2"][0]["formula"] == "Cyl.Radius * 2", par_ix
    pd = App.openDocument(par_p)
    try:
        for o in pd.Objects:
            o.touch()
        pd.recompute(None, True)
        bx2 = pd.getObject("Bx2")
        bx2_h, bx2_vol = float(bx2.Height), bx2.Shape.Volume
    finally:
        App.closeDocument(pd.Name)
    assert abs(bx2_h - 10) < 1e-6, bx2_h            # 5 * 2
    assert abs(bx2_vol - 3 * 3 * 10) < 1e-6, bx2_vol
    print("docformat.synthesize: authored parametric edge Bx2.Height=Cyl.Radius*2"
          " -> kernel evaluates to %.0f (file-first parametric)" % bx2_h)

    # ---- synthesize boolean: a CSG tree authored from nothing ------------ #
    # a box minus a centred cylinder, authored straight to file as a Part::Cut
    # whose base/tool link two primitives; the object-link DAG is recovered from
    # the file, and the kernel performs the boolean on recompute.
    csg_p = os.path.join(OUT, "synth_csg.FCStd")
    docformat.synthesize(csg_p, [
        {"type": "Part::Box", "name": "Base",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        {"type": "Part::Cylinder", "name": "Tool",
         "properties": {"Radius": 3, "Height": 20},
         "placement": {"position": [5, 5, -5]}},
        {"type": "Part::Cut", "name": "Cut", "base": "Base", "tool": "Tool"},
    ])
    csg_ix = docformat.inspect_document(csg_p)
    assert csg_ix["type_counts"].get("Part::Cut") == 1, csg_ix["type_counts"]
    assert csg_ix["dependencies"]["Cut"] == ["Base", "Tool"], csg_ix["dependencies"]
    cd = App.openDocument(csg_p)
    try:
        for o in cd.Objects:
            o.touch()
        cd.recompute(None, True)
        cut_vol = cd.getObject("Cut").Shape.Volume
    finally:
        App.closeDocument(cd.Name)
    assert abs(cut_vol - (1000 - math.pi * 9 * 10)) < 1e-3, cut_vol
    print("docformat.synthesize: authored CSG Cut(Base-Tool) -> kernel carves "
          "vol %.1f (file-first constructive solid geometry)" % cut_vol)

    # ---- synthesize rotation: an oriented primitive authored from nothing - #
    # a 10x2x2 bar rotated 90 deg about Z; FreeCAD persists a rotation twice
    # (quaternion + axis-angle) and honours the axis-angle, so synthesize must
    # author both -- the kernel then orients the bar, swapping its X/Y extents.
    rot_p = os.path.join(OUT, "synth_rot.FCStd")
    docformat.synthesize(rot_p, [
        {"type": "Part::Box", "name": "Bar",
         "properties": {"Length": 10, "Width": 2, "Height": 2},
         "placement": {"position": [0, 0, 0], "axis": [0, 0, 1], "angle": 90}},
    ])
    rd = App.openDocument(rot_p)
    try:
        for o in rd.Objects:
            o.touch()
        rd.recompute(None, True)
        bb = rd.getObject("Bar").Shape.BoundBox
        ext = (round(bb.XLength, 3), round(bb.YLength, 3), round(bb.ZLength, 3))
    finally:
        App.closeDocument(rd.Name)
    assert ext == (2.0, 10.0, 2.0), ext       # X/Y swapped by the 90 deg turn
    print("docformat.synthesize: authored 90deg rotation -> kernel orients bar "
          "to bbox %s (file-first orientation)" % (ext,))

    # ---- synthesize spreadsheet: a parametric control table from nothing -- #
    # author a Spreadsheet of aliased cells (one literal, one formula off it),
    # then bind two box dimensions to those aliases; the kernel resolves the
    # table on recompute -- the master-model surface, written straight to file.
    sht_p = os.path.join(OUT, "synth_sheet.FCStd")
    docformat.synthesize(sht_p, [
        {"type": "Spreadsheet::Sheet", "name": "Params",
         "cells": {"width": 7, "height": "=width + 3"}},
        {"type": "Part::Box", "name": "Panel",
         "properties": {"Length": 1, "Width": 1, "Height": 1},
         "expressions": {"Width": "Params.width", "Height": "Params.height"}},
    ])
    sht_ix = docformat.inspect_document(sht_p)
    assert sht_ix["type_counts"].get("Spreadsheet::Sheet") == 1, sht_ix
    # binding to the table is a dependency edge Panel -> Params.
    assert "Params" in sht_ix["dependencies"]["Panel"], sht_ix["dependencies"]
    # the file layer reads the control table back (the author->read dual): the
    # aliases and their cell contents recovered with no kernel.
    assert sht_ix["spreadsheet_cell_count"] == 2, sht_ix["spreadsheet_cell_count"]
    f_aliases = sht_ix["spreadsheets"]["Params"]["aliases"]
    assert f_aliases == {"width": "7", "height": "=width + 3"}, f_aliases
    shd = App.openDocument(sht_p)
    try:
        for o in shd.Objects:
            o.touch()
        shd.recompute(None, True)
        panel = shd.getObject("Panel")
        pw, ph, pvol = float(panel.Width), float(panel.Height), panel.Shape.Volume
        sheet = shd.getObject("Params")
        # two layers, one truth: every alias the file parser found resolves in
        # the running kernel to the value its cell content implies.
        k_width, k_height = float(sheet.width), float(sheet.height)
    finally:
        App.closeDocument(shd.Name)
    assert abs(pw - 7) < 1e-6 and abs(ph - 10) < 1e-6, (pw, ph)   # w=7, h=w+3
    assert abs(pvol - 1 * 7 * 10) < 1e-6, pvol
    assert abs(k_width - 7) < 1e-6 and abs(k_height - 10) < 1e-6, (k_width,
                                                                   k_height)
    print("docformat: authored control table Params(width=7,height=width+3) -> "
          "file reads aliases %s, kernel resolves to %g/%g, drives Panel %gx%g "
          "(author==read, two layers one truth)"
          % (sorted(f_aliases), k_width, k_height, pw, ph))

    # ---- synthesize N-ary boolean: one fold over many operands from file --- #
    # three 10-cubes, offset so they overlap pairwise; a Part::MultiFuse folds
    # the union of all three in a single recompute -- what a human assembles as
    # repeated pairwise fuses, authored at once. Union volume = 3*1000 minus the
    # two 5x5x5 pairwise overlaps (A&B, A&C) = 3000 - 2*125 = 2750.
    mf_p = os.path.join(OUT, "synth_multifuse.FCStd")
    cube = lambda nm, pos: {       # noqa: E731 - terse local box spec
        "type": "Part::Box", "name": nm,
        "properties": {"Length": 10, "Width": 10, "Height": 10},
        "placement": {"position": pos}}
    docformat.synthesize(mf_p, [
        cube("A", [0, 0, 0]), cube("B", [5, 5, 5]), cube("C", [-5, -5, 5]),
        {"type": "Part::MultiFuse", "name": "Union",
         "shapes": ["A", "B", "C"]},
    ])
    mf_ix = docformat.inspect_document(mf_p)
    # the link-list authors three dependency edges Union -> {A, B, C}.
    assert set(mf_ix["dependencies"]["Union"]) == {"A", "B", "C"}, mf_ix[
        "dependencies"]["Union"]
    mfd = App.openDocument(mf_p)
    try:
        for o in mfd.Objects:
            o.touch()
        mfd.recompute(None, True)
        uvol = mfd.getObject("Union").Shape.Volume
    finally:
        App.closeDocument(mfd.Name)
    assert abs(uvol - 2750) < 1e-3, uvol
    print("docformat.synthesize: authored Part::MultiFuse over 3 cubes -> kernel "
          "folds union vol %g in one recompute (N-ary CSG from file)" % uvol)

    # ---- synthesize Part::Compound: group disjoint shapes, no CSG ---------- #
    # two non-overlapping 10-cubes grouped into one Compound; no union/carve --
    # the shapes coexist, so the compound volume is the plain sum 2000. Authored
    # via the same link-list path as the N-ary booleans, under 'links'.
    cp_p = os.path.join(OUT, "synth_compound.FCStd")
    docformat.synthesize(cp_p, [
        {"type": "Part::Box", "name": "L",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        {"type": "Part::Box", "name": "R",
         "properties": {"Length": 10, "Width": 10, "Height": 10},
         "placement": {"position": [20, 0, 0]}},
        {"type": "Part::Compound", "name": "Grp", "links": ["L", "R"]},
    ])
    cp_ix = docformat.inspect_document(cp_p)
    assert set(cp_ix["dependencies"]["Grp"]) == {"L", "R"}, cp_ix[
        "dependencies"]["Grp"]
    # summarize round-trips the compound back to a 'links' spec.
    cp_spec = next(s for s in docformat.summarize(cp_p) if s["name"] == "Grp")
    assert cp_spec["links"] == ["L", "R"], cp_spec
    cpd = App.openDocument(cp_p)
    try:
        for o in cpd.Objects:
            o.touch()
        cpd.recompute(None, True)
        gvol = cpd.getObject("Grp").Shape.Volume
    finally:
        App.closeDocument(cpd.Name)
    assert abs(gvol - 2000) < 1e-3, gvol
    print("docformat.synthesize: authored Part::Compound of 2 disjoint cubes -> "
          "kernel groups vol %g (sum, no CSG; links round-trip)" % gvol)

    # ---- linear_pattern: author an N-copy array from one spec ------------- #
    # the file layer's leverage: one parametric description -> a whole array a
    # human would stamp out by repeating a GUI place-copy. Five disjoint cubes
    # spaced 20 apart, grouped into a Compound; kernel volume = 5 * 1000.
    pat_specs = docformat.linear_pattern(
        {"type": "Part::Box", "name": "Cell",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        count=5, offset=[20, 0, 0], group="Part::Compound")
    assert [s["name"] for s in pat_specs] == [
        "Cell_0", "Cell_1", "Cell_2", "Cell_3", "Cell_4", "Cell_all"], pat_specs
    # each copy is translated i*offset from the base position.
    assert pat_specs[3]["placement"]["position"] == [60, 0, 0], pat_specs[3]
    pat_p = os.path.join(OUT, "synth_pattern.FCStd")
    docformat.synthesize(pat_p, pat_specs)
    pat_ix = docformat.inspect_document(pat_p)
    assert pat_ix["type_counts"].get("Part::Box") == 5, pat_ix["type_counts"]
    assert len(pat_ix["dependencies"]["Cell_all"]) == 5, pat_ix["dependencies"]
    patd = App.openDocument(pat_p)
    try:
        for o in patd.Objects:
            o.touch()
        patd.recompute(None, True)
        avol = patd.getObject("Cell_all").Shape.Volume
    finally:
        App.closeDocument(patd.Name)
    assert abs(avol - 5 * 1000) < 1e-3, avol
    # group=None yields just the copies (no grouping object).
    assert len(docformat.linear_pattern(
        {"type": "Part::Box", "name": "C",
         "properties": {"Length": 1, "Width": 1, "Height": 1}},
        count=3, offset=[2, 0, 0])) == 3
    print("docformat.linear_pattern: one spec -> 5-cube array grouped to vol %g "
          "(superhuman authoring: a whole pattern from one parametric line)"
          % avol)

    # ---- polar_pattern: revolve copies about an axis from one spec -------- #
    # four cubes at radius 30 revolved about Z, full 360 (step 90), grouped to a
    # Compound. The file layer does the revolve arithmetic (Rodrigues) itself:
    # copies land at the four cardinal radii, disjoint, so vol = 4 * 1000.
    pol_specs = docformat.polar_pattern(
        {"type": "Part::Box", "name": "Tooth",
         "properties": {"Length": 10, "Width": 10, "Height": 10},
         "placement": {"position": [30, 0, 0]}},
        count=4, axis=[0, 0, 1], total_angle=360, group="Part::Compound")
    assert [s["name"] for s in pol_specs] == [
        "Tooth_0", "Tooth_1", "Tooth_2", "Tooth_3", "Tooth_all"], pol_specs
    # copy 1 revolved 90 deg about Z: [30,0,0] -> [0,30,0].
    p1 = pol_specs[1]["placement"]["position"]
    assert abs(p1[0]) < 1e-6 and abs(p1[1] - 30) < 1e-6, p1
    assert abs(pol_specs[1]["placement"]["angle"] - 90) < 1e-9, pol_specs[1]
    pol_p = os.path.join(OUT, "synth_polar.FCStd")
    docformat.synthesize(pol_p, pol_specs)
    pold = App.openDocument(pol_p)
    try:
        for o in pold.Objects:
            o.touch()
        pold.recompute(None, True)
        rvol = pold.getObject("Tooth_all").Shape.Volume
    finally:
        App.closeDocument(pold.Name)
    assert abs(rvol - 4 * 1000) < 1e-3, rvol
    print("docformat.polar_pattern: one spec -> 4-tooth ring revolved about Z, "
          "grouped to vol %g (file layer does the revolve math itself)" % rvol)

    # ---- Part::Mirroring: reflect a shape across a plane, kernel-free ----- #
    # author a box at x in [5,15] and a mirror across the x=0 plane (normal
    # [1,0,0]); the reflection is rigid so volume is preserved (1000) and the
    # copy's centroid lands at x=-10 (mirror of +10). The file builds a mirror
    # feature with no kernel; the kernel only reflects geometry on recompute.
    mir_p = os.path.join(OUT, "synth_mirror.FCStd")
    docformat.synthesize(mir_p, [
        {"type": "Part::Box", "name": "A",
         "properties": {"Length": 10, "Width": 10, "Height": 10},
         "placement": {"position": [5, 0, 0]}},
        {"type": "Part::Mirroring", "name": "Mir", "source": "A",
         "normal": [1, 0, 0]},
    ])
    # summarize recovers source + non-default normal (default base omitted).
    mir_spec = next(s for s in docformat.summarize(mir_p) if s["name"] == "Mir")
    assert mir_spec["source"] == "A", mir_spec
    assert mir_spec["normal"] == [1.0, 0.0, 0.0], mir_spec
    assert "base" not in mir_spec, mir_spec
    mir_rt = os.path.join(OUT, "synth_mirror_rt.FCStd")
    docformat.synthesize(mir_rt, docformat.summarize(mir_p))
    assert docformat.fingerprint(mir_p) == docformat.fingerprint(mir_rt)
    mird = App.openDocument(mir_p)
    try:
        for o in mird.Objects:
            o.touch()
        mird.recompute(None, True)
        mo = mird.getObject("Mir")
        mvol, mcx = mo.Shape.Volume, mo.Shape.CenterOfMass.x
    finally:
        App.closeDocument(mird.Name)
    assert abs(mvol - 1000) < 1e-3, mvol
    assert abs(mcx - (-10)) < 1e-6, mcx
    # malformed mirror specs are guided, not leaked as TypeErrors.
    for bad, token in (
            ([{"type": "Part::Mirroring", "name": "M"}], "needs a 'source'"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 1, "Width": 1, "Height": 1}},
              {"type": "Part::Mirroring", "name": "M", "source": "B",
               "normal": [0, 0, 0]}], "non-zero"),
            ([{"type": "Part::Mirroring", "name": "M", "source": "Ghost"}],
             "not a defined object")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_mir.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Part::Mirroring: box reflected across x=0 -> vol %g, "
          "centroid x=%g, round-trips identically (file builds the mirror)"
          % (mvol, mcx))

    # ---- summarize: decompile a file back to a synthesize spec (round-trip) - #
    # author a document spanning every type the authoring layer writes -- a
    # parametric primitive, a placed/rotated primitive, a 2-way boolean, an
    # N-ary boolean, and a spreadsheet -- then read it straight back out as a
    # spec and re-author it; the two files must fingerprint identically. The
    # author->read loop closing on every type: 反者道之动.
    rt_a = os.path.join(OUT, "roundtrip_a.FCStd")
    docformat.synthesize(rt_a, [
        {"type": "Spreadsheet::Sheet", "name": "P",
         "cells": {"side": 8, "twice": "=side * 2"}},
        {"type": "Part::Box", "name": "Base",
         "properties": {"Length": 1, "Width": 4, "Height": 4},
         "expressions": {"Length": "P.twice"}},
        {"type": "Part::Cylinder", "name": "Rod",
         "properties": {"Radius": 2, "Height": 20},
         "placement": {"position": [2, 2, 0], "axis": [0, 1, 0], "angle": 30}},
        {"type": "Part::Cut", "name": "Carved", "base": "Base", "tool": "Rod"},
        {"type": "Part::Box", "name": "X",
         "properties": {"Length": 5, "Width": 5, "Height": 5}},
        {"type": "Part::MultiFuse", "name": "All", "shapes": ["Carved", "X"]},
    ])
    spec_back = docformat.summarize(rt_a)
    # the decompiled spec recovers every object, and the spreadsheet formula
    # survives as a formula (not flattened to its computed number).
    assert [s["name"] for s in spec_back] == [
        "P", "Base", "Rod", "Carved", "X", "All"], spec_back
    p_cells = next(s for s in spec_back if s["name"] == "P")["cells"]
    assert p_cells == {"side": 8, "twice": "=side * 2"}, p_cells
    rod = next(s for s in spec_back if s["name"] == "Rod")
    # angle recovered from the persisted radians, which the parser rounds to 6
    # decimals -- so a coarse tolerance, not exact equality.
    assert "placement" in rod and abs(rod["placement"]["angle"] - 30) < 1e-3, rod
    base = next(s for s in spec_back if s["name"] == "Base")
    assert base["expressions"] == {"Length": "P.twice"}, base
    rt_b = os.path.join(OUT, "roundtrip_b.FCStd")
    docformat.synthesize(rt_b, spec_back)
    assert docformat.fingerprint(rt_a) == docformat.fingerprint(rt_b), (
        docformat.fingerprint(rt_a), docformat.fingerprint(rt_b))
    print("docformat.summarize: decompiled %d-object file -> re-synthesized to "
          "identical fingerprint (author<->read round-trip closes)"
          % len(spec_back))

    # guarded: empty spec, unknown primitive, duplicate name, undefined property,
    # a boolean whose operand does not resolve, a degenerate rotation axis, a
    # spreadsheet with no cells, and an N-ary boolean with too few operands.
    _sy = docformat.synthesize
    bad = os.path.join(OUT, "synth_bad.FCStd")
    for spec, token in (
            ([], "non-empty list"),
            ([{"type": "Part::Widget", "name": "X"}], "unknown type"),
            ([{"type": "Part::Box", "name": "D"},
              {"type": "Part::Box", "name": "D"}], "duplicate"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Radius": 3}}], "no propert"),
            ([{"type": "Part::Cut", "name": "C", "base": "P", "tool": "Q"}],
             "not a defined object"),
            ([{"type": "Part::Box", "name": "R",
               "properties": {"Length": 1, "Width": 1, "Height": 1},
               "placement": {"axis": [0, 0, 0], "angle": 45}}], "axis"),
            ([{"type": "Spreadsheet::Sheet", "name": "S", "cells": {}}],
             "non-empty 'cells'"),
            ([{"type": "Part::Box", "name": "A",
               "properties": {"Length": 1, "Width": 1, "Height": 1}},
              {"type": "Part::MultiFuse", "name": "F", "shapes": ["A"]}],
             "list of >=2 object names")):
        try:
            _sy(bad, spec)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat.synthesize: malformed specs guided")

    # ---- doc.synthesize / doc.realize: file-first authoring as agent ops -- #
    # the agent authors a BREP-less file via the kernel-free op, then *realises*
    # it: the kernel builds the geometry from the authored scalars and writes it
    # back. The realised file now carries BREP the file layer can read -- the
    # full create->bake loop, write-like-code then let FreeCAD solve.
    op_p = os.path.join(OUT, "op_synth.FCStd")
    sr = s.act("doc.synthesize", {"path": op_p, "objects": [
        {"type": "Part::Box", "name": "A",
         "properties": {"Length": 6, "Width": 6, "Height": 6}},
        {"type": "Part::Cylinder", "name": "B",
         "properties": {"Radius": 2, "Height": 8},
         "placement": {"position": [3, 3, -1]}},
        {"type": "Part::Cut", "name": "AB", "base": "A", "tool": "B"},
    ]})
    assert sr.ok, sr
    assert sr.data["object_count"] == 3, sr.data
    # before realise: authored file is geometry-free.
    assert docformat.inspect_document(op_p)["brep_files"] == [], "no BREP yet"
    op_out = os.path.join(OUT, "op_realized.FCStd")
    rr = s.act("doc.realize", {"path": op_p, "out": op_out})
    assert rr.ok, rr
    vols = {o["name"]: o["volume"] for o in rr.data["objects"]}
    assert abs(vols["AB"] - (6 * 6 * 6 - math.pi * 4 * 6)) < 1e-3, vols
    # after realise: the kernel-baked file carries BREP the file layer reads.
    assert docformat.inspect_document(op_out)["brep_files"], "realised -> BREP"
    # doc.summarize: the inverse op decompiles the authored file back to a spec,
    # which re-authors to an identical fingerprint -- author<->read as agent ops.
    sm = s.act("doc.summarize", {"path": op_p})
    assert sm.ok and sm.data["object_count"] == 3, sm
    assert [o["name"] for o in sm.data["objects"]] == ["A", "B", "AB"], sm.data
    op_rt = os.path.join(OUT, "op_roundtrip.FCStd")
    assert s.act("doc.synthesize",
                 {"path": op_rt, "objects": sm.data["objects"]}).ok
    assert docformat.fingerprint(op_p) == docformat.fingerprint(op_rt)
    # doc.synthesize guards a missing object list rather than leaking a TypeError;
    # doc.summarize guards a missing path the same way.
    assert not s.act("doc.synthesize", {"path": op_p}).ok
    assert not s.act("doc.summarize", {}).ok
    print("doc.synthesize+realize+summarize: authored CSG -> kernel baked vol "
          "%.1f, decompiled+re-authored to identical fingerprint (file-first "
          "authoring as agent ops, author<->read closes)" % vols["AB"])

    # ---- doc.pattern: array generation as an agent op -------------------- #
    # the agent expands one base spec into a whole array via the op and
    # synthesizes it straight to a file -- linear (3-cube row) and polar (a
    # 6-tooth ring), both grouped into a Compound the kernel groups by volume.
    op_lin = os.path.join(OUT, "op_lin_pattern.FCStd")
    pl = s.act("doc.pattern", {
        "mode": "linear", "path": op_lin,
        "base": {"type": "Part::Box", "name": "U",
                 "properties": {"Length": 10, "Width": 10, "Height": 10}},
        "count": 3, "offset": [20, 0, 0], "group": "Part::Compound"})
    assert pl.ok and pl.data["object_count"] == 4, pl
    assert pl.data["out"] == op_lin, pl.data
    op_pol = os.path.join(OUT, "op_pol_pattern.FCStd")
    pp = s.act("doc.pattern", {
        "mode": "polar", "path": op_pol,
        "base": {"type": "Part::Box", "name": "T",
                 "properties": {"Length": 5, "Width": 5, "Height": 5},
                 "placement": {"position": [40, 0, 0]}},
        "count": 6, "axis": [0, 0, 1], "total_angle": 360,
        "group": "Part::Compound"})
    assert pp.ok and pp.data["object_count"] == 7, pp
    for pth, n, unit in ((op_lin, 3, 1000), (op_pol, 6, 125)):
        pd = App.openDocument(pth)
        try:
            for o in pd.Objects:
                o.touch()
            pd.recompute(None, True)
            gv = pd.getObject(pd.Objects[-1].Name).Shape.Volume
        finally:
            App.closeDocument(pd.Name)
        assert abs(gv - n * unit) < 1e-3, (pth, gv)
    # doc.pattern guards an unknown mode rather than leaking a TypeError.
    assert not s.act("doc.pattern", {"mode": "spiral", "base": {}, "count": 2}).ok
    print("doc.pattern: linear 3-cube row + polar 6-tooth ring authored from one "
          "base spec each (array generation as an agent op, file-layer leverage)")

    # ---- two-layer fusion: the live kernel agrees with the file ---------- #
    # ss.bindings reads the same ExpressionEngine wiring from the *running*
    # document; it must match what the file-level parser recovered -- the two
    # views of the parametric graph are one truth.
    kb = x.act("ss.bindings", {})
    assert kb.ok, kb
    assert kb.data["count"] == ix["expression_count"], (kb.data, ix["expression_count"])
    assert sorted(kb.data["edges"]) == sorted(ix["expression_edges"]), (
        kb.data["edges"], ix["expression_edges"])
    k_pad = {e["path"]: e["formula"] for e in kb.data["bindings"]["Pad"]}
    assert k_pad == bound, (k_pad, bound)
    print("ss.bindings: kernel expression graph == file-level parse (two layers, "
          "one truth)")

    # re-point the binding to a different alias: a parametric-intent change that
    # the structured diff names explicitly (not just an opaque blob value flip).
    assert x.act("ss.bind", {"param": "Pad.length", "alias": "pwid"}).ok
    ex_b = os.path.join(OUT, "expr_b.FCStd")
    assert x.act("doc.save", {"path": ex_b}).ok
    xd = docformat.diff(ex_a, ex_b)
    assert not xd["identical"], xd
    assert xd["expression_changes"].get("Pad.Length") == {
        "from": "Spreadsheet.plen", "to": "Spreadsheet.pwid"}, xd["expression_changes"]
    # and a document diffs to no expression change against itself.
    assert docformat.diff(ex_a, ex_a)["expression_changes"] == {}, "self-diff"
    print("docformat.diff: expression re-binding named in expression_changes")

    # ---- set_expression: the act half for wiring -- author the binding ---- #
    # ex_a binds Pad.Length -> Spreadsheet.plen (=5). Re-point it to pwid (=9)
    # purely by file surgery (no kernel), then prove the kernel honours the
    # rewired graph on reopen: the body's volume follows the new alias.
    ex_set = os.path.join(OUT, "expr_set.FCStd")
    r = docformat.set_expression(ex_a, "Pad", "Length", "Spreadsheet.pwid",
                                 out=ex_set)
    assert r["old"] == "Spreadsheet.plen" and r["new"] == "Spreadsheet.pwid", r
    # file-level view confirms the rewire with no kernel.
    si = docformat.inspect_document(ex_set)
    sb = {e["path"]: e["formula"] for e in si["expressions"]["Pad"]}
    assert sb.get("Length") == "Spreadsheet.pwid", si["expressions"]
    assert docformat.diff(ex_a, ex_set)["expression_changes"].get("Pad.Length") == {
        "from": "Spreadsheet.plen", "to": "Spreadsheet.pwid"}, ex_set
    # the kernel honours the file-authored binding: pwid=9 -> 40*30*9 = 10800.
    doc3 = App.openDocument(ex_set)
    try:
        doc3.getObject("Pad").touch()
        doc3.recompute(None, True)
        body3 = next(o for o in doc3.Objects
                     if o.TypeId.startswith("PartDesign::Body"))
        vol3 = body3.Shape.Volume
    finally:
        App.closeDocument(doc3.Name)
    assert abs(vol3 - 40 * 30 * 9) < 1.0, vol3
    print("docformat.set_expression: file-authored re-point plen->pwid -> kernel "
          "volume %.0f (file edit rewires the parametric graph)" % vol3)

    # removing the binding (formula=None) leaves the property unbound.
    ex_rm = os.path.join(OUT, "expr_rm.FCStd")
    rr = docformat.set_expression(ex_set, "Pad", "Length", None, out=ex_rm)
    assert rr["old"] == "Spreadsheet.pwid" and rr["new"] is None, rr
    assert docformat.inspect_document(ex_rm)["expression_count"] == 0, ex_rm

    # author a binding where none existed: the now-unbound Pad gets re-wired to
    # plen (=5) purely from the file -- and the kernel builds 40*30*5 = 6000.
    ex_add = os.path.join(OUT, "expr_add.FCStd")
    ra = docformat.set_expression(ex_rm, "Pad", "Length", "Spreadsheet.plen",
                                  out=ex_add)
    assert ra["old"] is None and ra["new"] == "Spreadsheet.plen", ra
    assert docformat.inspect_document(ex_add)["expression_count"] == 1, ex_add
    doc4 = App.openDocument(ex_add)
    try:
        doc4.getObject("Pad").touch()
        doc4.recompute(None, True)
        body4 = next(o for o in doc4.Objects
                     if o.TypeId.startswith("PartDesign::Body"))
        vol4 = body4.Shape.Volume
    finally:
        App.closeDocument(doc4.Name)
    assert abs(vol4 - 40 * 30 * 5) < 1.0, vol4
    print("docformat.set_expression: authored a binding from the file -> kernel "
          "volume %.0f (no kernel used to wire it)" % vol4)

    # guarded: a missing object, and removing an absent binding -- both refuse
    # before writing anything.
    _se = docformat.set_expression
    for call, token in (
            (lambda: _se(ex_add, "Nope", "Length", "Spreadsheet.plen"), "no object"),
            (lambda: _se(ex_rm, "Pad", "Length", None), "to remove")):
        try:
            call()
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat.set_expression: malformed edits guided")

    print("DOCFORMAT SMOKE OK")


if __name__ == "__main__":
    main()
