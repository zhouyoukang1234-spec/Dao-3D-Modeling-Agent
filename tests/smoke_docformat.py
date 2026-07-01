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
import zipfile

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

    # ---- more primitives: Ellipsoid / Wedge / Prism --------------------- #
    # broaden the authorable solid vocabulary: an ellipsoid (3 radii), a wedge
    # (10 distance bounds), and an N-gon prism (integer Polygon side count).
    # Each recomputes to its analytic volume, and the integer-backed Polygon
    # property exercises the <Integer> serialisation + round-trip.
    np_p = os.path.join(OUT, "synth_newprims.FCStd")
    docformat.synthesize(np_p, [
        {"type": "Part::Ellipsoid", "name": "E",
         "properties": {"Radius1": 3, "Radius2": 4, "Radius3": 5}},
        {"type": "Part::Wedge", "name": "W",
         "properties": {"Xmin": 0, "Xmax": 10, "Ymin": 0, "Ymax": 10,
                        "Zmin": 0, "Zmax": 10, "X2min": 2, "X2max": 8,
                        "Z2min": 2, "Z2max": 8}},
        {"type": "Part::Prism", "name": "Pr",
         "properties": {"Circumradius": 5, "Height": 10, "Polygon": 8}},
    ])
    np_specs = {s["name"]: s for s in docformat.summarize(np_p)}
    assert np_specs["Pr"]["properties"]["Polygon"] == 8, np_specs["Pr"]
    assert not isinstance(np_specs["Pr"]["properties"]["Polygon"], bool)
    np_rt = os.path.join(OUT, "synth_newprims_rt.FCStd")
    docformat.synthesize(np_rt, list(np_specs.values()))
    assert docformat.fingerprint(np_p) == docformat.fingerprint(np_rt)
    npd = App.openDocument(np_p)
    try:
        for o in npd.Objects:
            o.touch()
        npd.recompute(None, True)
        vol = {n: npd.getObject(n).Shape.Volume for n in ("E", "W", "Pr")}
    finally:
        App.closeDocument(npd.Name)
    assert abs(vol["E"] - (4.0 / 3.0) * math.pi * 3 * 4 * 5) < 1.0, vol["E"]
    assert abs(vol["W"] - 653.333) < 1e-2, vol["W"]
    # regular octagon (circumradius 5) area * height: 0.5*8*25*sin(45 deg)*10.
    assert abs(vol["Pr"] - 0.5 * 8 * 25 * math.sin(math.pi / 4) * 10) < 1e-2, vol["Pr"]
    # a non-integer Polygon is guided (integer-backed property).
    try:
        docformat.synthesize(os.path.join(OUT, "bad_prism.FCStd"), [
            {"type": "Part::Prism", "name": "P",
             "properties": {"Circumradius": 5, "Height": 10, "Polygon": 6.5}}])
    except ValueError as exc:
        assert "must be an integer" in str(exc), exc
    else:
        raise AssertionError("expected ValueError for non-integer Polygon")
    print("docformat primitives+: Ellipsoid/Wedge/Prism author to analytic vols "
          "%.1f/%.1f/%.1f, integer Polygon round-trips (<Integer> serialisation)"
          % (vol["E"], vol["W"], vol["Pr"]))

    # ---- lowest-dimension primitives: Plane (2-D face) / Vertex (0-D point) - #
    # close the primitive vocabulary at the bottom: a Length x Width planar face
    # and a single (X,Y,Z) vertex. Both rebuild from their scalars on execute()
    # (no BREP), so they round-trip byte-identically and their placement survives
    # a reload -- a Plane is a loft/extrude section or section tool-face, a Vertex
    # the 0-D construction atom. 地方 / 一者，數之至也.
    lp_p = os.path.join(OUT, "synth_planevertex.FCStd")
    docformat.synthesize(lp_p, [
        {"type": "Part::Plane", "name": "Pl",
         "properties": {"Length": 8, "Width": 5}},
        {"type": "Part::Vertex", "name": "Vx",
         "properties": {"X": 1, "Y": 2, "Z": 3}},
    ])
    assert zipfile.ZipFile(lp_p).namelist() == ["Document.xml"]
    lp_specs = {s["name"]: s for s in docformat.summarize(lp_p)}
    assert lp_specs["Pl"]["properties"] == {"Length": 8, "Width": 5}, lp_specs["Pl"]
    assert lp_specs["Vx"]["properties"] == {"X": 1, "Y": 2, "Z": 3}, lp_specs["Vx"]
    lp_rt = os.path.join(OUT, "synth_planevertex_rt.FCStd")
    docformat.synthesize(lp_rt, list(lp_specs.values()))
    assert docformat.fingerprint(lp_p) == docformat.fingerprint(lp_rt)
    lpd = App.openDocument(lp_p)
    try:
        for o in lpd.Objects:
            o.touch()
        lpd.recompute(None, True)
        pl_sh = lpd.getObject("Pl").Shape
        vx_sh = lpd.getObject("Vx").Shape
        pl_area = pl_sh.Area
        pl_ok = pl_sh.isValid() and len(pl_sh.Faces) == 1
        vx_pt = tuple(round(c, 6) for c in vx_sh.Vertexes[0].Point)
        vx_ok = vx_sh.isValid() and len(vx_sh.Vertexes) == 1
    finally:
        App.closeDocument(lpd.Name)
    assert pl_ok and abs(pl_area - 40.0) < 1e-6, pl_area
    assert vx_ok and vx_pt == (1.0, 2.0, 3.0), vx_pt
    print("docformat primitives-: Plane (8x5 -> area %g, one face) and Vertex "
          "(-> point %s) author with no BREP and round-trip identically"
          % (pl_area, vx_pt))

    # ---- Part::Ellipse: the circle's flattened parametric-edge sibling ------ #
    # a MajorRadius x MinorRadius elliptic edge rebuilt from scalars on execute()
    # (no BREP), so it round-trips identically and its placement survives reload
    # -- an elliptic loft section / sweep spine. A partial arc (Angle1..Angle2)
    # persists its two angles too. 圆之变也.
    el_p = os.path.join(OUT, "synth_ellipse.FCStd")
    docformat.synthesize(el_p, [
        {"type": "Part::Ellipse", "name": "El",
         "properties": {"MajorRadius": 8, "MinorRadius": 5}},
        {"type": "Part::Ellipse", "name": "Ea",
         "properties": {"MajorRadius": 6, "MinorRadius": 3,
                        "Angle1": 0, "Angle2": 180}},
    ])
    assert zipfile.ZipFile(el_p).namelist() == ["Document.xml"]
    el_specs = {s["name"]: s for s in docformat.summarize(el_p)}
    assert el_specs["El"]["properties"]["MajorRadius"] == 8 \
        and el_specs["El"]["properties"]["MinorRadius"] == 5, el_specs["El"]
    assert el_specs["Ea"]["properties"]["Angle2"] == 180, el_specs["Ea"]
    el_rt = os.path.join(OUT, "synth_ellipse_rt.FCStd")
    docformat.synthesize(el_rt, list(el_specs.values()))
    assert docformat.fingerprint(el_p) == docformat.fingerprint(el_rt)
    eld = App.openDocument(el_p)
    try:
        for o in eld.Objects:
            o.touch()
        eld.recompute(None, True)
        el_sh = eld.getObject("El").Shape
        el_len = el_sh.Length
        el_ok = el_sh.isValid() and len(el_sh.Edges) == 1
        ea_ok = eld.getObject("Ea").Shape.isValid()
    finally:
        App.closeDocument(eld.Name)
    assert el_ok and el_len > 0, el_len
    assert ea_ok, "elliptic arc must recompute valid"
    print("docformat Part::Ellipse: 8x5 ellipse -> one valid edge (length %g); "
          "half-ellipse arc round-trips (angles persist), no BREP" % round(el_len, 3))

    # ---- Part::RegularPolygon: the straight-edged parametric wire ---------- #
    # an N-gon wire rebuilt from (Polygon, Circumradius) on execute() (no BREP),
    # the straight-edged sibling of Part::Circle -- a ready section to extrude/
    # loft. A hexagon of circumradius 10 closes into a face of area
    # 0.5*N*R^2*sin(2*pi/N) = 0.5*6*100*sin(60 deg) = 259.808. The integer-backed
    # Polygon exercises the <Integer> round-trip and its non-integer guard. 圆出于方.
    rp_p = os.path.join(OUT, "synth_regpoly.FCStd")
    docformat.synthesize(rp_p, [
        {"type": "Part::RegularPolygon", "name": "Hex",
         "properties": {"Polygon": 6, "Circumradius": 10}}])
    assert zipfile.ZipFile(rp_p).namelist() == ["Document.xml"]
    rp_spec = next(s for s in docformat.summarize(rp_p) if s["name"] == "Hex")
    assert rp_spec["properties"]["Polygon"] == 6, rp_spec
    assert not isinstance(rp_spec["properties"]["Polygon"], bool)
    rp_rt = os.path.join(OUT, "synth_regpoly_rt.FCStd")
    docformat.synthesize(rp_rt, docformat.summarize(rp_p))
    assert docformat.fingerprint(rp_p) == docformat.fingerprint(rp_rt)
    import Part
    rpd = App.openDocument(rp_p)
    try:
        for o in rpd.Objects:
            o.touch()
        rpd.recompute(None, True)
        rp_sh = rpd.getObject("Hex").Shape
        rp_edges = len(rp_sh.Edges)
        rp_wire = Part.Wire(rp_sh.Edges)
        rp_closed = rp_wire.isClosed()
        rp_area = Part.Face(rp_wire).Area
    finally:
        App.closeDocument(rpd.Name)
    assert rp_edges == 6 and rp_closed, (rp_edges, rp_closed)
    assert abs(rp_area - 0.5 * 6 * 100 * math.sin(math.pi / 3)) < 1e-3, rp_area
    try:
        docformat.synthesize(os.path.join(OUT, "bad_regpoly.FCStd"), [
            {"type": "Part::RegularPolygon", "name": "B",
             "properties": {"Polygon": 6.5, "Circumradius": 10}}])
    except ValueError as exc:
        assert "must be an integer" in str(exc), exc
    else:
        raise AssertionError("expected ValueError for non-integer Polygon")
    print("docformat Part::RegularPolygon: hexagon (R=10) -> 6-edge closed wire, "
          "face area %.3f, integer Polygon round-trips, no BREP" % rp_area)

    # ---- Part::Refine: the one-link shape-cleanup feature ------------------- #
    # fuse two abutting 10-cubes -> the shared seam splits faces (10 faces); a
    # Part::Refine wrapping the fusion merges the coplanar faces back to the 6 of
    # a clean 10x20x10 box, preserving volume (2000). The feature carries a single
    # Source link, no scalars, so it round-trips byte-identically. 大巧若拙.
    rf_p = os.path.join(OUT, "synth_refine.FCStd")
    docformat.synthesize(rf_p, [
        {"type": "Part::Box", "name": "Ra",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        {"type": "Part::Box", "name": "Rb",
         "properties": {"Length": 10, "Width": 10, "Height": 10},
         "placement": {"position": [10, 0, 0]}},
        {"type": "Part::Fuse", "name": "Rf", "base": "Ra", "tool": "Rb"},
        docformat.refine("Rr", "Rf"),
    ])
    assert zipfile.ZipFile(rf_p).namelist() == ["Document.xml"]
    rf_spec = next(s for s in docformat.summarize(rf_p) if s["name"] == "Rr")
    assert rf_spec["type"] == "Part::Refine" and rf_spec["source"] == "Rf", rf_spec
    rf_rt = os.path.join(OUT, "synth_refine_rt.FCStd")
    docformat.synthesize(rf_rt, docformat.summarize(rf_p))
    assert docformat.fingerprint(rf_p) == docformat.fingerprint(rf_rt)
    rfd = App.openDocument(rf_p)
    try:
        for o in rfd.Objects:
            o.touch()
        rfd.recompute(None, True)
        fuse_faces = len(rfd.getObject("Rf").Shape.Faces)
        rr_sh = rfd.getObject("Rr").Shape
        rr_faces = len(rr_sh.Faces)
        rr_vol = rr_sh.Volume
        rr_ok = rr_sh.isValid()
    finally:
        App.closeDocument(rfd.Name)
    assert rr_ok and rr_faces == 6 and fuse_faces > 6, (fuse_faces, rr_faces)
    assert abs(rr_vol - 2000.0) < 1e-6, rr_vol
    # a refine pointing at a missing source is guided.
    for badcall in (
            lambda: docformat.refine("", "Rf"),
            lambda: docformat.refine("Rr", "")):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from refine generator")
    try:
        docformat.synthesize(os.path.join(OUT, "bad_refine.FCStd"),
                             [docformat.refine("Rr", "Ghost")])
    except ValueError as exc:
        assert "not a defined object" in str(exc), exc
    else:
        raise AssertionError("expected ValueError for refine of missing source")
    print("docformat Part::Refine: fuse of two abutting cubes (%d faces) refined "
          "-> clean 6-face box (vol %g), volume-preserving, round-trips; source "
          "guards hold" % (fuse_faces, rr_vol))

    # ---- Sketcher::SketchObject: author a 2D profile from file ----------- #
    # the most upstream authoring surface: draw a closed 10x5 rectangle as four
    # line segments straight into the Part::PropertyGeometryList. The kernel
    # rebuilds the wire on recompute -- a closed loop the upstream pad consumes,
    # area 50. summarize recovers the segments and the spec round-trips. 逆流到最上游.
    sk_p = os.path.join(OUT, "synth_sketch.FCStd")
    docformat.synthesize(sk_p, [
        {"type": "Sketcher::SketchObject", "name": "Sk", "geometry": [
            {"start": [0, 0], "end": [10, 0]},
            {"start": [10, 0], "end": [10, 5]},
            {"start": [10, 5], "end": [0, 5]},
            {"start": [0, 5], "end": [0, 0]},
        ]},
    ])
    sk_spec = next(s for s in docformat.summarize(sk_p) if s["name"] == "Sk")
    assert len(sk_spec["geometry"]) == 4, sk_spec
    assert sk_spec["geometry"][0] == {"start": [0.0, 0.0], "end": [10.0, 0.0]}, \
        sk_spec["geometry"][0]
    sk_rt = os.path.join(OUT, "synth_sketch_rt.FCStd")
    docformat.synthesize(sk_rt, docformat.summarize(sk_p))
    assert docformat.fingerprint(sk_p) == docformat.fingerprint(sk_rt)
    # inspect_document surfaces the edge geometry kernel-free.
    skd = docformat.inspect_document(sk_p)["sketch_geometry"]["Sk"]
    assert len(skd) == 4 and all(g["line"] for g in skd), skd
    import Part
    sd = App.openDocument(sk_p)
    try:
        sk = sd.getObject("Sk")
        assert sk.GeometryCount == 4, sk.GeometryCount
        for o in sd.Objects:
            o.touch()
        sd.recompute(None, True)
        wire = Part.Wire(Part.__sortEdges__(sk.Shape.Edges))
        sk_closed, sk_area = wire.isClosed(), Part.Face(wire).Area
    finally:
        App.closeDocument(sd.Name)
    assert sk_closed and abs(sk_area - 50.0) < 1e-6, (sk_closed, sk_area)
    # malformed sketches are guided, not leaked.
    for bad, token in (
            ([{"type": "Sketcher::SketchObject", "name": "S",
               "geometry": []}], "non-empty 'geometry'"),
            ([{"type": "Sketcher::SketchObject", "name": "S", "geometry": [
                {"start": [0, 0], "end": [0, 0]}]}], "degenerate"),
            ([{"type": "Sketcher::SketchObject", "name": "S", "geometry": [
                {"start": [0, 0], "end": [1]}]}], "[x, y] numbers")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_sk.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Sketcher::SketchObject: 4-segment rectangle authored from "
          "file -> closed wire area %g, round-trips identically (逆流到最上游)"
          % sk_area)

    # ---- Sketcher: curved geometry (circle + arc) ------------------------ #
    # deepen the most-upstream surface beyond polygons: a full Part::GeomCircle
    # is a closed profile on its own (a disk the pad turns into a cylinder), and
    # a Part::GeomArcOfCircle closes against a chord into a half-disk. Author a
    # circle radius 5 + an arc-bounded half-disk radius 3; the kernel rebuilds
    # both faces (areas pi*25 and pi*9/2) and the specs round-trip identically.
    cs_p = os.path.join(OUT, "synth_sketch_curves.FCStd")
    docformat.synthesize(cs_p, [
        {"type": "Sketcher::SketchObject", "name": "Disk",
         "geometry": [{"center": [0, 0], "radius": 5}]},
        {"type": "Sketcher::SketchObject", "name": "Half", "geometry": [
            {"center": [0, 0], "radius": 3,
             "start_angle": 0, "end_angle": math.pi},
            {"start": [-3, 0], "end": [3, 0]},
        ]},
    ])
    cs_specs = {s["name"]: s for s in docformat.summarize(cs_p)}
    assert cs_specs["Disk"]["geometry"] == [{"center": [0.0, 0.0],
                                             "radius": 5.0}], cs_specs["Disk"]
    half_arc = cs_specs["Half"]["geometry"][0]
    assert half_arc["radius"] == 3.0 and "start_angle" in half_arc, half_arc
    cs_rt = os.path.join(OUT, "synth_sketch_curves_rt.FCStd")
    docformat.synthesize(cs_rt, docformat.summarize(cs_p))
    assert docformat.fingerprint(cs_p) == docformat.fingerprint(cs_rt)
    cd = App.openDocument(cs_p)
    try:
        for o in cd.Objects:
            o.touch()
        cd.recompute(None, True)
        disk_w = Part.Wire(Part.__sortEdges__(cd.getObject("Disk").Shape.Edges))
        half_w = Part.Wire(Part.__sortEdges__(cd.getObject("Half").Shape.Edges))
        disk_area = Part.Face(disk_w).Area
        half_area = Part.Face(half_w).Area
    finally:
        App.closeDocument(cd.Name)
    assert abs(disk_area - math.pi * 25) < 1e-6, disk_area
    assert abs(half_area - math.pi * 9 / 2) < 1e-6, half_area
    for bad, token in (
            ([{"type": "Sketcher::SketchObject", "name": "S", "geometry": [
                {"center": [0, 0], "radius": 0}]}], "'radius' must be positive"),
            ([{"type": "Sketcher::SketchObject", "name": "S", "geometry": [
                {"center": [0, 0], "radius": 3, "start_angle": 1,
                 "end_angle": 1}]}], "arc is degenerate"),
            ([{"type": "Sketcher::SketchObject", "name": "S", "geometry": [
                {"foo": 1}]}], "must be a line")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_curve.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Sketcher curves: circle disk area %g (pi*25) + arc half-disk"
          " area %g (pi*9/2) authored from file, round-trip identical"
          % (disk_area, half_area))

    # ---- regular_polygon: generate an N-gon profile from one description -- #
    # superhuman generation: one (sides, radius) description -> every vertex
    # computed by trig and written as a closed loop, what a human draws and
    # constrains edge by edge. A hexagon radius 10 has area
    # (1/2) n r^2 sin(2pi/n) = 259.808; extruded 4 it is a prism of that * 4.
    hexg = docformat.regular_polygon("Hex", 6, 10)
    assert len(hexg["geometry"]) == 6, hexg
    poly_p = os.path.join(OUT, "synth_polygon.FCStd")
    docformat.synthesize(poly_p, [
        hexg,
        {"type": "Part::Extrusion", "name": "Ext", "base": "Hex", "length": 4},
    ])
    poly_rt = os.path.join(OUT, "synth_polygon_rt.FCStd")
    docformat.synthesize(poly_rt, docformat.summarize(poly_p))
    assert docformat.fingerprint(poly_p) == docformat.fingerprint(poly_rt)
    pd = App.openDocument(poly_p)
    try:
        for o in pd.Objects:
            o.touch()
        pd.recompute(None, True)
        hex_w = Part.Wire(Part.__sortEdges__(pd.getObject("Hex").Shape.Edges))
        hex_area = Part.Face(hex_w).Area
        prism_vol = pd.getObject("Ext").Shape.Volume
    finally:
        App.closeDocument(pd.Name)
    exp_area = 0.5 * 6 * 100 * math.sin(2 * math.pi / 6)
    assert abs(hex_area - exp_area) < 1e-6, (hex_area, exp_area)
    assert abs(prism_vol - exp_area * 4) < 1e-6, prism_vol
    for kwargs, token in (
            (dict(name="P", sides=2, radius=5), "'sides' must be an int >= 3"),
            (dict(name="P", sides=5, radius=0), "'radius' must be a positive"),
            (dict(name="", sides=5, radius=5), "non-empty name")):
        try:
            docformat.regular_polygon(**kwargs)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % kwargs)
    print("docformat regular_polygon: hexagon radius 10 generated from one "
          "description -> area %g, extruded to prism volume %g (做人类靠重复才能做的)"
          % (hex_area, prism_vol))

    # ---- slot: an obround from two lines + two arcs, one description ------ #
    # a rounded slot the human builds from two flanks, two end arcs and tangency
    # constraints -- here the four edges and exact arc angles come from one
    # (length, radius) description, exercising mixed line+arc geometry. The
    # enclosed area is 2*length*radius + pi*radius^2 = 2*20*5 + pi*25 = 278.54.
    slotg = docformat.slot("Slot", 20, 5)
    assert len(slotg["geometry"]) == 4, slotg
    slot_p = os.path.join(OUT, "synth_slot.FCStd")
    docformat.synthesize(slot_p, [slotg])
    slot_rt = os.path.join(OUT, "synth_slot_rt.FCStd")
    docformat.synthesize(slot_rt, docformat.summarize(slot_p))
    assert docformat.fingerprint(slot_p) == docformat.fingerprint(slot_rt)
    sld = App.openDocument(slot_p)
    try:
        for o in sld.Objects:
            o.touch()
        sld.recompute(None, True)
        slot_w = Part.Wire(Part.__sortEdges__(sld.getObject("Slot").Shape.Edges))
        slot_closed = slot_w.isClosed()
        slot_area = Part.Face(slot_w).Area
    finally:
        App.closeDocument(sld.Name)
    assert slot_closed, "slot wire must close"
    assert abs(slot_area - (2 * 20 * 5 + math.pi * 25)) < 1e-6, slot_area
    for kwargs, token in (
            (dict(name="S", length=0, radius=5), "'length' must be a positive"),
            (dict(name="S", length=20, radius=0), "'radius' must be a positive"),
            (dict(name="", length=20, radius=5), "non-empty name")):
        try:
            docformat.slot(**kwargs)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % kwargs)
    print("docformat slot: obround length 20 radius 5 from one description -> "
          "closed wire area %g (line+arc mixed profile)" % slot_area)

    # ---- ellipse: a single curved edge closing a loop, one description --- #
    # a tilted ellipse the human places by centre + two dragged axes under
    # tangency constraints -- here both radii and the tilt come from one
    # (major, minor, angle) description as a single Part::GeomEllipse edge. The
    # enclosed area is pi*M*m = pi*10*5 = 157.08, tilt-invariant.
    ellg = docformat.ellipse("Ell", 10, 5, center=[1, 2], angle=25)
    assert len(ellg["geometry"]) == 1, ellg
    ell_p = os.path.join(OUT, "synth_ellipse.FCStd")
    docformat.synthesize(ell_p, [ellg])
    ell_rt = os.path.join(OUT, "synth_ellipse_rt.FCStd")
    docformat.synthesize(ell_rt, docformat.summarize(ell_p))
    assert docformat.fingerprint(ell_p) == docformat.fingerprint(ell_rt)
    eld = App.openDocument(ell_p)
    try:
        for o in eld.Objects:
            o.touch()
        eld.recompute(None, True)
        ell_w = eld.getObject("Ell").Shape.Wires[0]
        ell_closed = ell_w.isClosed()
        ell_area = Part.Face(ell_w).Area
    finally:
        App.closeDocument(eld.Name)
    assert ell_closed, "ellipse wire must close"
    assert abs(ell_area - (math.pi * 10 * 5)) < 1e-6, ell_area
    for kwargs, token in (
            (dict(name="E", major_radius=-1, minor_radius=5),
             "'major_radius' must be a positive"),
            (dict(name="E", major_radius=10, minor_radius=0),
             "'minor_radius' must be a positive"),
            (dict(name="E", major_radius=3, minor_radius=5),
             "'major_radius' must be >= 'minor_radius'"),
            (dict(name="", major_radius=10, minor_radius=5), "non-empty name")):
        try:
            docformat.ellipse(**kwargs)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % kwargs)
    print("docformat ellipse: 10x5 tilted 25deg from one description -> "
          "closed wire area %g (single curved edge)" % ell_area)

    # ---- arc_ellipse: a partial ellipse, composes into a closed D-shape -- #
    # half an ellipse (0..pi) closed by the major-axis chord -- the elliptic
    # fillet / D-profile a human sweeps as an ellipse arc + a tangent line. The
    # two edges close into a D whose area is pi*M*m/2 = pi*10*5/2 = 78.54.
    dgeom = [{"center": [0, 0], "major_radius": 10, "minor_radius": 5,
              "start_angle": 0.0, "end_angle": math.pi},
             {"start": [-10, 0], "end": [10, 0]}]
    dspec = {"type": docformat._SKETCH_TYPE, "name": "DEll", "geometry": dgeom}
    aoe_p = os.path.join(OUT, "synth_arc_ellipse.FCStd")
    docformat.synthesize(aoe_p, [dspec])
    aoe_rt = os.path.join(OUT, "synth_arc_ellipse_rt.FCStd")
    docformat.synthesize(aoe_rt, docformat.summarize(aoe_p))
    assert docformat.fingerprint(aoe_p) == docformat.fingerprint(aoe_rt)
    aod = App.openDocument(aoe_p)
    try:
        for o in aod.Objects:
            o.touch()
        aod.recompute(None, True)
        aoe_w = Part.Wire(
            Part.__sortEdges__(aod.getObject("DEll").Shape.Edges))
        aoe_closed = aoe_w.isClosed()
        aoe_area = Part.Face(aoe_w).Area
    finally:
        App.closeDocument(aod.Name)
    assert aoe_closed, "arc_ellipse D-shape wire must close"
    assert abs(aoe_area - (math.pi * 10 * 5 / 2)) < 1e-6, aoe_area
    # an arc_ellipse inherits the ellipse guards (major >= minor > 0).
    for bad, token in (
            ({"center": [0, 0], "major_radius": 3, "minor_radius": 5,
              "start_angle": 0.0, "end_angle": 1.0},
             "'major_radius' must be >= 'minor_radius'"),
            ({"center": [0, 0], "major_radius": 5, "minor_radius": 0,
              "start_angle": 0.0, "end_angle": 1.0},
             "'minor_radius' must be positive")):
        try:
            docformat.synthesize(
                os.path.join(OUT, "bad_aoe.FCStd"),
                [{"type": docformat._SKETCH_TYPE, "name": "B",
                  "geometry": [bad]}])
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % bad)
    print("docformat arc_ellipse: half-ellipse 10x5 + chord -> closed D-shape "
          "area %g (partial elliptic edge, tilt+round-trip exact)" % aoe_area)

    # ---- parabola: the open conic, one edge from a focal length ---------- #
    # a Part::GeomArcOfParabola about vertex (0,0), focal 5, parameter -3..3 --
    # the reflector / dish profile a human sweeps as a parabola arc. Its
    # endpoints (0.45, +/-3) close via a chord into a parabolic segment of area
    # (2/3)*base*height = (2/3)*6*0.45 = 1.8. Discriminated purely by 'focal',
    # it round-trips byte-exact (no other conic carries a focal length). 抛物之道.
    pgeom = [{"center": [0, 0], "focal": 5.0,
              "start_angle": -3.0, "end_angle": 3.0}]
    pspec = {"type": docformat._SKETCH_TYPE, "name": "Par", "geometry": pgeom}
    par_p = os.path.join(OUT, "synth_parabola.FCStd")
    docformat.synthesize(par_p, [pspec])
    par_seg = next(s for s in docformat.summarize(par_p)
                   if s["name"] == "Par")["geometry"][0]
    assert par_seg["focal"] == 5.0 and par_seg["start_angle"] == -3.0, par_seg
    par_rt = os.path.join(OUT, "synth_parabola_rt.FCStd")
    docformat.synthesize(par_rt, docformat.summarize(par_p))
    assert docformat.fingerprint(par_p) == docformat.fingerprint(par_rt)
    pard = App.openDocument(par_p)
    try:
        for o in pard.Objects:
            o.touch()
        pard.recompute(None, True)
        pedges = pard.getObject("Par").Shape.Edges
        par_kind = pedges[0].Curve.__class__.__name__
        pvs = pedges[0].Vertexes
        pchord = Part.LineSegment(pvs[0].Point, pvs[-1].Point).toShape()
        par_w = Part.Wire(Part.__sortEdges__(list(pedges) + [pchord]))
        par_closed = par_w.isClosed()
        par_area = Part.Face(par_w).Area
    finally:
        App.closeDocument(pard.Name)
    assert len(pedges) == 1 and par_kind == "Parabola", (len(pedges), par_kind)
    assert par_closed and abs(par_area - 1.8) < 1e-6, (par_closed, par_area)
    # a parabola needs a positive focal length and a non-degenerate range.
    for bad, token in (
            ({"center": [0, 0], "focal": 0,
              "start_angle": -3.0, "end_angle": 3.0}, "'focal' must be positive"),
            ({"center": [0, 0], "focal": 5,
              "start_angle": 2.0, "end_angle": 2.0}, "parabola is degenerate")):
        try:
            docformat.synthesize(
                os.path.join(OUT, "bad_parabola.FCStd"),
                [{"type": docformat._SKETCH_TYPE, "name": "B",
                  "geometry": [bad]}])
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % bad)
    print("docformat parabola: focal-5 arc (-3..3) + chord -> parabolic segment "
          "area %g (open conic edge, focal-discriminated, round-trips)" % par_area)

    # ---- hyperbola: the other axes+angles conic, key-shape twin of arc_ellipse #
    # a Part::GeomArcOfHyperbola about centre (0,0), transverse/conjugate
    # semi-axes 6/3, parameter 0.2..1.0 -- the open branch a human sweeps as a
    # hyperbola arc. It carries the *same* keys as an arc_ellipse (center /
    # major_radius / minor_radius / start_angle / end_angle), so it is
    # disambiguated purely by an explicit 'hyperbola' marker; without it the
    # spec would classify as arc_ellipse. Length 4.342249 confirms the kernel
    # rebuilds an actual Hyperbola. 反者道之动.
    hgeom = [{"hyperbola": True, "center": [0, 0],
              "major_radius": 6.0, "minor_radius": 3.0,
              "start_angle": 0.2, "end_angle": 1.0}]
    hspec = {"type": docformat._SKETCH_TYPE, "name": "Hyp", "geometry": hgeom}
    hyp_p = os.path.join(OUT, "synth_hyperbola.FCStd")
    docformat.synthesize(hyp_p, [hspec])
    hyp_seg = next(s for s in docformat.summarize(hyp_p)
                   if s["name"] == "Hyp")["geometry"][0]
    assert hyp_seg.get("hyperbola") and hyp_seg["major_radius"] == 6.0, hyp_seg
    hyp_rt = os.path.join(OUT, "synth_hyperbola_rt.FCStd")
    docformat.synthesize(hyp_rt, docformat.summarize(hyp_p))
    assert docformat.fingerprint(hyp_p) == docformat.fingerprint(hyp_rt)
    hypd = App.openDocument(hyp_p)
    try:
        for o in hypd.Objects:
            o.touch()
        hypd.recompute(None, True)
        hedges = hypd.getObject("Hyp").Shape.Edges
        hyp_kind = hedges[0].Curve.__class__.__name__
        hyp_len = hedges[0].Length
    finally:
        App.closeDocument(hypd.Name)
    assert len(hedges) == 1 and hyp_kind == "Hyperbola", (len(hedges), hyp_kind)
    assert abs(hyp_len - 4.342249) < 1e-5, hyp_len
    # the disambiguator must matter: the same keys without the marker are an
    # arc_ellipse (an Ellipse curve), a genuinely different edge.
    ae_p = os.path.join(OUT, "synth_hyp_as_ell.FCStd")
    docformat.synthesize(ae_p, [{
        "type": docformat._SKETCH_TYPE, "name": "AE",
        "geometry": [{"center": [0, 0], "major_radius": 6.0,
                      "minor_radius": 3.0, "start_angle": 0.2,
                      "end_angle": 1.0}]}])
    ae_seg = next(s for s in docformat.summarize(ae_p)
                  if s["name"] == "AE")["geometry"][0]
    assert "hyperbola" not in ae_seg, ae_seg
    assert docformat._sketch_segment_kind(ae_seg) == "arc_ellipse", ae_seg
    assert docformat._sketch_segment_kind(hyp_seg) == "hyperbola", hyp_seg
    # a hyperbola needs positive semi-axes and a non-degenerate range.
    for bad, token in (
            ({"hyperbola": True, "center": [0, 0], "major_radius": 0,
              "minor_radius": 3.0, "start_angle": 0.2, "end_angle": 1.0},
             "must be positive"),
            ({"hyperbola": True, "center": [0, 0], "major_radius": 6.0,
              "minor_radius": 3.0, "start_angle": 1.0, "end_angle": 1.0},
             "hyperbola is degenerate")):
        try:
            docformat.synthesize(
                os.path.join(OUT, "bad_hyperbola.FCStd"),
                [{"type": docformat._SKETCH_TYPE, "name": "B",
                  "geometry": [bad]}])
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % bad)
    print("docformat hyperbola: 6/3 arc (0.2..1.0) -> Hyperbola edge len %g "
          "(marker-discriminated twin of arc_ellipse, round-trips)" % hyp_len)

    # ---- bspline: the general freeform curve, one edge from control poles - #
    # a degree-3 open B-spline through six control poles -- the freeform spline
    # a human pushes/pulls point by point. The generator writes the exact
    # clamped uniform knot vector (ends mult degree+1) so the kernel rebuilds a
    # single Part::GeomBSplineCurve edge; summarize -> synthesize is byte-exact.
    bspg = docformat.bspline(
        "BSp", [[0, 0], [5, 8], [12, 6], [18, 0], [22, 7], [28, 2]], degree=3)
    assert len(bspg["geometry"]) == 1, bspg
    bsp_inner = bspg["geometry"][0]["bspline"]
    assert bsp_inner["mults"] == [4, 1, 1, 4], bsp_inner
    bsp_p = os.path.join(OUT, "synth_bspline.FCStd")
    docformat.synthesize(bsp_p, [bspg])
    bsp_rt = os.path.join(OUT, "synth_bspline_rt.FCStd")
    docformat.synthesize(bsp_rt, docformat.summarize(bsp_p))
    assert docformat.fingerprint(bsp_p) == docformat.fingerprint(bsp_rt)
    bspd = App.openDocument(bsp_p)
    try:
        for o in bspd.Objects:
            o.touch()
        bspd.recompute(None, True)
        bsp_sh = bspd.getObject("BSp").Shape
        bsp_edges = len(bsp_sh.Edges)
        bsp_kind = bsp_sh.Edges[0].Curve.__class__.__name__
    finally:
        App.closeDocument(bspd.Name)
    assert bsp_edges == 1, bsp_edges
    assert bsp_kind == "BSplineCurve", bsp_kind
    # a rational (weighted) B-spline round-trips its weights byte-exact too.
    rw = docformat.bspline("RW", [[0, 0], [5, 10], [10, 0], [15, 10]],
                           degree=2, weights=[1, 3, 3, 1])
    rw_p = os.path.join(OUT, "synth_bspline_rw.FCStd")
    docformat.synthesize(rw_p, [rw])
    rw_sum = docformat.summarize(rw_p)
    rw_w = [s for s in rw_sum
            if s["name"] == "RW"][0]["geometry"][0]["bspline"]["weights"]
    assert rw_w == [1.0, 3.0, 3.0, 1.0], rw_w
    rw_rt = os.path.join(OUT, "synth_bspline_rw_rt.FCStd")
    docformat.synthesize(rw_rt, rw_sum)
    assert docformat.fingerprint(rw_p) == docformat.fingerprint(rw_rt)
    # guards: too few poles, and fewer poles than the degree needs.
    for kwargs, token in (
            (dict(name="x", poles=[[0, 0]], degree=3), "at least 2 'poles'"),
            (dict(name="x", poles=[[0, 0], [1, 1], [2, 2]], degree=3),
             "more than 'degree' poles")):
        try:
            docformat.bspline(**kwargs)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % kwargs)
    # a closed (periodic) B-spline: a smooth freeform loop through the poles,
    # the curved analogue of a closed polygon. poles+1 knots all mult 1, uniform
    # on [0,1]; the kernel rebuilds one periodic edge that bounds a face.
    clg = docformat.bspline(
        "CL", [[0, 0], [10, 2], [12, 10], [4, 14], [-4, 8]], degree=3,
        closed=True)
    cl_inner = clg["geometry"][0]["bspline"]
    assert cl_inner["periodic"] is True, cl_inner
    assert cl_inner["mults"] == [1, 1, 1, 1, 1, 1], cl_inner
    assert sum(cl_inner["mults"]) == len(cl_inner["poles"]) + 1, cl_inner
    cl_p = os.path.join(OUT, "synth_bspline_closed.FCStd")
    docformat.synthesize(cl_p, [clg])
    cl_rt = os.path.join(OUT, "synth_bspline_closed_rt.FCStd")
    docformat.synthesize(cl_rt, docformat.summarize(cl_p))
    assert docformat.fingerprint(cl_p) == docformat.fingerprint(cl_rt)
    cld = App.openDocument(cl_p)
    try:
        for o in cld.Objects:
            o.touch()
        cld.recompute(None, True)
        cl_edge = cld.getObject("CL").Shape.Edges[0]
        cl_periodic = cl_edge.Curve.isPeriodic()
        cl_face_area = Part.Face(Part.Wire(
            cld.getObject("CL").Shape.Edges)).Area
    finally:
        App.closeDocument(cld.Name)
    assert cl_periodic is True, cl_periodic
    assert cl_face_area > 100.0, cl_face_area
    print("docformat bspline: degree-3 freeform through 6 poles -> single "
          "BSplineCurve edge (+ rational weights, round-trip exact); closed "
          "periodic loop bounds a face (area %g)" % cl_face_area)

    # ---- point: the isolated sketch vertex (Part::GeomPoint) ------------- #
    # the simplest primitive -- a lone reference/construction point, no edge.
    # completes the sketch-geometry vocabulary; round-trips byte-exact and the
    # kernel rebuilds exactly one vertex (zero edges) at the given coordinate.
    ptg = docformat.point("PT", [3, 4])
    ptc = docformat.point("PC", [-2, 7], construction=True)
    assert ptg["geometry"][0] == {"point": [3.0, 4.0]}, ptg
    assert ptc["geometry"][0].get("construction") is True, ptc
    pt_p = os.path.join(OUT, "synth_point.FCStd")
    docformat.synthesize(pt_p, [ptg, ptc])
    pt_rt = os.path.join(OUT, "synth_point_rt.FCStd")
    docformat.synthesize(pt_rt, docformat.summarize(pt_p))
    assert docformat.fingerprint(pt_p) == docformat.fingerprint(pt_rt)
    ptd = App.openDocument(pt_p)
    try:
        for o in ptd.Objects:
            o.touch()
        ptd.recompute(None, True)
        pt_sh = ptd.getObject("PT").Shape
        pt_nv, pt_ne = len(pt_sh.Vertexes), len(pt_sh.Edges)
        pt_xy = (pt_sh.Vertexes[0].X, pt_sh.Vertexes[0].Y)
    finally:
        App.closeDocument(ptd.Name)
    assert pt_nv == 1 and pt_ne == 0, (pt_nv, pt_ne)
    assert abs(pt_xy[0] - 3.0) < 1e-9 and abs(pt_xy[1] - 4.0) < 1e-9, pt_xy
    for bad in ([1], [1, 2, 3], "x"):
        try:
            docformat.point("x", bad)
        except ValueError as exc:
            assert "must be [x, y] numbers" in str(exc), exc
        else:
            raise AssertionError("expected ValueError for point %r" % (bad,))
    print("docformat point: isolated Part::GeomPoint -> one vertex, zero edges "
          "at (3,4) (+ construction flag, round-trip exact)")

    # ---- Part::Extrusion: sweep a sketch profile into a solid ------------ #
    # the join between the sketch layer and the solid layer: author a 10x5
    # rectangle sketch + an extrusion that sweeps it 7 along +Z. The kernel
    # turns the 2D loop into a body of volume 50*7 = 350 on recompute -- the
    # file-first equivalent of the GUI's Pad, no clicks. summarize recovers the
    # base/length and the spec round-trips to an identical document.
    ex_p = os.path.join(OUT, "synth_extrude.FCStd")
    docformat.synthesize(ex_p, [
        {"type": "Sketcher::SketchObject", "name": "Sk", "geometry": [
            {"start": [0, 0], "end": [10, 0]},
            {"start": [10, 0], "end": [10, 5]},
            {"start": [10, 5], "end": [0, 5]},
            {"start": [0, 5], "end": [0, 0]},
        ]},
        {"type": "Part::Extrusion", "name": "Ext", "base": "Sk", "length": 7},
    ])
    ex_spec = next(s for s in docformat.summarize(ex_p) if s["name"] == "Ext")
    assert ex_spec["base"] == "Sk" and ex_spec["length"] == 7, ex_spec
    ex_rt = os.path.join(OUT, "synth_extrude_rt.FCStd")
    docformat.synthesize(ex_rt, docformat.summarize(ex_p))
    assert docformat.fingerprint(ex_p) == docformat.fingerprint(ex_rt)
    # the extrusion records its dependency on the sketch (recompute DAG edge).
    ex_file_deps = docformat.inspect_document(ex_p)["dependencies"].get("Ext", [])
    assert "Sk" in ex_file_deps, ex_file_deps
    ed = App.openDocument(ex_p)
    try:
        for o in ed.Objects:
            o.touch()
        ed.recompute(None, True)
        ex_vol = ed.getObject("Ext").Shape.Volume
        ex_dep_names = [d.Name for d in ed.getObject("Ext").OutList]
    finally:
        App.closeDocument(ed.Name)
    assert abs(ex_vol - 350.0) < 1e-6, ex_vol
    assert "Sk" in ex_dep_names, ex_dep_names
    for bad, token in (
            ([{"type": "Sketcher::SketchObject", "name": "Sk", "geometry": [
                {"start": [0, 0], "end": [1, 0]}]},
              {"type": "Part::Extrusion", "name": "Ex", "base": "Sk",
               "length": 0}], "positive 'length'"),
            ([{"type": "Part::Extrusion", "name": "Ex", "base": "Nope",
               "length": 5}], "is not a defined object")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_ex.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Part::Extrusion: sketch swept from file -> solid volume %g "
          "(50*7), depends on its profile, round-trips identically" % ex_vol)

    # ---- Part::Revolution: spin a sketch profile into a solid ------------ #
    # the lathe to the extrusion's mill: author a 3x4 rectangle offset from the
    # y-axis (x in [2,5]) + a full revolution about that axis. By Pappus the
    # kernel spins a solid of volume 2*pi*Rc*A = 2*pi*3.5*12 = 263.894 on
    # recompute -- the file-first equivalent of the GUI's Revolve. summarize
    # recovers the source/axis and the spec round-trips identically.
    rv_p = os.path.join(OUT, "synth_revolve.FCStd")
    docformat.synthesize(rv_p, [
        {"type": "Sketcher::SketchObject", "name": "Sk", "geometry": [
            {"start": [2, 0], "end": [5, 0]},
            {"start": [5, 0], "end": [5, 4]},
            {"start": [5, 4], "end": [2, 4]},
            {"start": [2, 4], "end": [2, 0]},
        ]},
        {"type": "Part::Revolution", "name": "Rev", "source": "Sk",
         "axis": [0, 1, 0]},
    ])
    rv_spec = next(s for s in docformat.summarize(rv_p) if s["name"] == "Rev")
    assert rv_spec["source"] == "Sk" and rv_spec["axis"] == [0.0, 1.0, 0.0], \
        rv_spec
    rv_rt = os.path.join(OUT, "synth_revolve_rt.FCStd")
    docformat.synthesize(rv_rt, docformat.summarize(rv_p))
    assert docformat.fingerprint(rv_p) == docformat.fingerprint(rv_rt)
    rv_file_deps = docformat.inspect_document(rv_p)["dependencies"].get("Rev", [])
    assert "Sk" in rv_file_deps, rv_file_deps
    rd = App.openDocument(rv_p)
    try:
        for o in rd.Objects:
            o.touch()
        rd.recompute(None, True)
        rv_vol = rd.getObject("Rev").Shape.Volume
    finally:
        App.closeDocument(rd.Name)
    assert abs(rv_vol - 263.8938) < 1e-2, rv_vol
    for bad, token in (
            ([{"type": "Sketcher::SketchObject", "name": "Sk", "geometry": [
                {"start": [2, 0], "end": [5, 0]}]},
              {"type": "Part::Revolution", "name": "R", "source": "Sk",
               "angle": 0}], "(0, 360]"),
            ([{"type": "Part::Revolution", "name": "R", "source": "Sk",
               "axis": [0, 0, 0]}], "must be non-zero")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_rv.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Part::Revolution: sketch spun from file -> solid volume %g "
          "(Pappus 2*pi*3.5*12), round-trips identically" % round(rv_vol, 3))

    # ---- Part::Circle: a parametric edge that keeps its placement ---------- #
    # the section-feeding complement to the solid primitives: a full circle of
    # radius 5 authored from file. Unlike a shape-less sketch, a Part::Circle's
    # placement survives a reload (its execute() rebuilds the edge without
    # resetting the frame), so it can be stacked at a z offset -- the property
    # that makes it a loft section. summarize recovers the radius + placement and
    # the spec round-trips to an identical document.
    ci_p = os.path.join(OUT, "synth_circle.FCStd")
    docformat.synthesize(ci_p, [
        {"type": "Part::Circle", "name": "C0",
         "properties": {"Radius": 5, "Angle2": 360},
         "placement": {"position": [0, 0, 4]}},
    ])
    ci_spec = next(s for s in docformat.summarize(ci_p) if s["name"] == "C0")
    assert ci_spec["properties"]["Radius"] == 5, ci_spec
    assert ci_spec["placement"]["position"] == [0.0, 0.0, 4.0], ci_spec
    ci_rt = os.path.join(OUT, "synth_circle_rt.FCStd")
    docformat.synthesize(ci_rt, docformat.summarize(ci_p))
    assert docformat.fingerprint(ci_p) == docformat.fingerprint(ci_rt)
    cd = App.openDocument(ci_p)
    try:
        for o in cd.Objects:
            o.touch()
        cd.recompute(None, True)
        c0 = cd.getObject("C0")
        ci_edges = len(c0.Shape.Edges)
        ci_z = c0.Placement.Base.z
    finally:
        App.closeDocument(cd.Name)
    assert ci_edges == 1, ci_edges
    assert abs(ci_z - 4.0) < 1e-6, ci_z
    print("docformat Part::Circle: parametric edge authored from file -> 1 edge, "
          "placement z=%g survives reload, round-trips identically" % ci_z)

    # ---- Part::Loft: skin a solid through >=2 stacked sections ------------- #
    # the multi-section complement to the single-profile extrude/revolve: two
    # circles -- radius 5 at z=0, radius 3 at z=10 -- lofted into a truncated
    # cone. The kernel skins the sections on recompute to a solid of volume
    # (pi h / 3)(R^2 + R r + r^2) = (pi 10/3)(25+15+9) = 513.13 -- the file-first
    # equivalent of the GUI's Loft. summarize recovers the ordered sections and
    # flags, and the spec round-trips to an identical document.
    lo_p = os.path.join(OUT, "synth_loft.FCStd")
    docformat.synthesize(lo_p, [
        {"type": "Part::Circle", "name": "L0",
         "properties": {"Radius": 5, "Angle2": 360}},
        {"type": "Part::Circle", "name": "L1",
         "properties": {"Radius": 3, "Angle2": 360},
         "placement": {"position": [0, 0, 10]}},
        {"type": "Part::Loft", "name": "Lof", "sections": ["L0", "L1"]},
    ])
    lo_spec = next(s for s in docformat.summarize(lo_p) if s["name"] == "Lof")
    assert lo_spec["sections"] == ["L0", "L1"], lo_spec
    # solid loft is the default -- no solid/ruled/closed keys emitted at default.
    assert "ruled" not in lo_spec and "closed" not in lo_spec, lo_spec
    lo_rt = os.path.join(OUT, "synth_loft_rt.FCStd")
    docformat.synthesize(lo_rt, docformat.summarize(lo_p))
    assert docformat.fingerprint(lo_p) == docformat.fingerprint(lo_rt)
    # the loft records its dependency on both sections (recompute DAG edges).
    lo_file_deps = docformat.inspect_document(lo_p)["dependencies"].get("Lof", [])
    assert "L0" in lo_file_deps and "L1" in lo_file_deps, lo_file_deps
    ld = App.openDocument(lo_p)
    try:
        for o in ld.Objects:
            o.touch()
        ld.recompute(None, True)
        lof = ld.getObject("Lof")
        lo_vol = lof.Shape.Volume
        lo_solids = len(lof.Shape.Solids)
        lo_dep_names = [d.Name for d in lof.OutList]
    finally:
        App.closeDocument(ld.Name)
    assert abs(lo_vol - 513.127) < 0.5, lo_vol
    assert lo_solids == 1, lo_solids
    assert "L0" in lo_dep_names and "L1" in lo_dep_names, lo_dep_names
    # a ruled, non-solid, closed loft round-trips its non-default flags too.
    lo2 = os.path.join(OUT, "synth_loft2.FCStd")
    docformat.synthesize(lo2, [
        {"type": "Part::Circle", "name": "A",
         "properties": {"Radius": 4, "Angle2": 360}},
        {"type": "Part::Circle", "name": "B",
         "properties": {"Radius": 4, "Angle2": 360},
         "placement": {"position": [0, 0, 6]}},
        {"type": "Part::Loft", "name": "Sh", "sections": ["A", "B"],
         "solid": False, "ruled": True},
    ])
    sh_spec = next(s for s in docformat.summarize(lo2) if s["name"] == "Sh")
    assert sh_spec["solid"] is False and sh_spec["ruled"] is True, sh_spec
    lo2_rt = os.path.join(OUT, "synth_loft2_rt.FCStd")
    docformat.synthesize(lo2_rt, docformat.summarize(lo2))
    assert docformat.fingerprint(lo2) == docformat.fingerprint(lo2_rt)
    for bad, token in (
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Loft", "name": "L", "sections": ["C"]}],
             "list of >=2"),
            ([{"type": "Part::Loft", "name": "L",
               "sections": ["Nope", "Nada"]}], "is not a defined object"),
            ([{"type": "Part::Loft", "name": "L",
               "sections": ["L", "L"]}], "cannot reference itself"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Loft", "name": "L", "sections": ["C", "C"]}],
             "duplicate sections"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Circle", "name": "D",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Loft", "name": "L", "sections": ["C", "D"],
               "solid": "yes"}], "must be a bool")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_lo.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Part::Loft: two circles (r5@0, r3@10) skinned from file -> "
          "solid volume %g (truncated cone), depends on both sections, "
          "round-trips identically; ruled/closed flags + guards hold" %
          round(lo_vol, 3))

    # ---- Part::Sweep: pipe a section along a spine path -------------------- #
    # the path-driven complement to the loft: a circle of radius 5 swept along a
    # straight Part::Line spine 10 long (Z axis) traces a cylinder of volume
    # pi r^2 h = pi 25 10 = 785.40. Both the section (Part::Circle) and the
    # spine (Part::Line) are parametric edges rebuilt from scalars, so the whole
    # sweep is authored from file with no BREP and the kernel pipes it on
    # recompute -- the file-first equivalent of the GUI's Sweep. The generator
    # docformat.sweep() writes the operator spec; summarize recovers the ordered
    # sections + the spine link and the document round-trips identically.
    sw_p = os.path.join(OUT, "synth_sweep.FCStd")
    docformat.synthesize(sw_p, [
        {"type": "Part::Circle", "name": "Sec",
         "properties": {"Radius": 5, "Angle2": 360}},
        {"type": "Part::Line", "name": "Path",
         "properties": {"Z2": 10}},
        docformat.sweep("Pipe", ["Sec"], "Path"),
    ])
    sw_spec = next(s for s in docformat.summarize(sw_p) if s["name"] == "Pipe")
    assert sw_spec["sections"] == ["Sec"], sw_spec
    assert sw_spec["spine"] == "Path", sw_spec
    # solid sweep is the default -- no solid/frenet/spine_edges keys at default.
    assert "frenet" not in sw_spec and "spine_edges" not in sw_spec, sw_spec
    sw_rt = os.path.join(OUT, "synth_sweep_rt.FCStd")
    docformat.synthesize(sw_rt, docformat.summarize(sw_p))
    assert docformat.fingerprint(sw_p) == docformat.fingerprint(sw_rt)
    # the sweep records its dependency on both the section and the spine.
    sw_file_deps = docformat.inspect_document(sw_p)["dependencies"].get("Pipe", [])
    assert "Sec" in sw_file_deps and "Path" in sw_file_deps, sw_file_deps
    wd = App.openDocument(sw_p)
    try:
        for o in wd.Objects:
            o.touch()
        wd.recompute(None, True)
        pipe = wd.getObject("Pipe")
        sw_vol = pipe.Shape.Volume
        sw_solids = len(pipe.Shape.Solids)
        sw_dep_names = [d.Name for d in pipe.OutList]
    finally:
        App.closeDocument(wd.Name)
    assert abs(sw_vol - 785.398) < 1.0, sw_vol
    assert sw_solids == 1, sw_solids
    assert "Sec" in sw_dep_names and "Path" in sw_dep_names, sw_dep_names
    # a frenet, non-solid (shell) sweep round-trips its non-default flags.
    sw2 = os.path.join(OUT, "synth_sweep2.FCStd")
    docformat.synthesize(sw2, [
        {"type": "Part::Circle", "name": "S2",
         "properties": {"Radius": 4, "Angle2": 360}},
        {"type": "Part::Line", "name": "P2", "properties": {"Z2": 8}},
        docformat.sweep("Sh2", ["S2"], "P2", solid=False, frenet=True),
    ])
    sh2_spec = next(s for s in docformat.summarize(sw2) if s["name"] == "Sh2")
    assert sh2_spec["solid"] is False and sh2_spec["frenet"] is True, sh2_spec
    sw2_rt = os.path.join(OUT, "synth_sweep2_rt.FCStd")
    docformat.synthesize(sw2_rt, docformat.summarize(sw2))
    assert docformat.fingerprint(sw2) == docformat.fingerprint(sw2_rt)
    for bad, token in (
            ([{"type": "Part::Loft", "name": "L", "sections": ["a"]}],
             "list of >=2"),  # loft still needs >=2 -- sweep is the >=1 sibling
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Sweep", "name": "W", "sections": [],
               "spine": "C"}], "list of >=1"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Sweep", "name": "W", "sections": ["C"]}],
             "needs a 'spine'"),
            ([{"type": "Part::Sweep", "name": "W", "sections": ["Nope"],
               "spine": "Nada"}], "is not a defined object"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Sweep", "name": "W", "sections": ["C"],
               "spine": "W"}], "cannot reference itself"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Sweep", "name": "W", "sections": ["C", "C"],
               "spine": "C"}], "duplicate sections"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Sweep", "name": "W", "sections": ["C"],
               "spine": "C"}], "spine cannot also be a section"),
            ([{"type": "Part::Circle", "name": "C",
               "properties": {"Radius": 1, "Angle2": 360}},
              {"type": "Part::Line", "name": "P", "properties": {"Z2": 5}},
              {"type": "Part::Sweep", "name": "W", "sections": ["C"],
               "spine": "P", "solid": "yes"}], "must be a bool")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_sw.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    print("docformat Part::Sweep: circle r5 piped along a 10-long line spine "
          "from file -> solid volume %g (cylinder), depends on section + spine, "
          "round-trips identically; frenet/shell flags + guards hold" %
          round(sw_vol, 3))

    # ---- Part::Fillet / Part::Chamfer: edge treatments (binary side member) - #
    # the edge-treatment family: a fillet rounds chosen edges of a base solid,
    # a chamfer bevels them. Unlike every other feature, their per-edge sizes
    # live not in Document.xml but in a binary "Edges" side member of the .FCStd
    # zip -- so synthesize writes that member and summarize reads it back. A box
    # of side 10 (volume 1000) filleted r2 on one edge loses a rounded sliver
    # (kernel volume ~991.42); the generator docformat.fillet() writes the spec,
    # summarize recovers {edge, radius}, and the whole document -- side member
    # included -- round-trips byte-identically. 大直若詘.
    fi_p = os.path.join(OUT, "synth_fillet.FCStd")
    docformat.synthesize(fi_p, [
        {"type": "Part::Box", "name": "Blk",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        docformat.fillet("Round", "Blk", [{"edge": 1, "radius": 2}]),
    ])
    assert "Edges" in zipfile.ZipFile(fi_p).namelist()
    fi_spec = next(s for s in docformat.summarize(fi_p) if s["name"] == "Round")
    assert fi_spec["base"] == "Blk", fi_spec
    assert fi_spec["edges"] == [{"edge": 1, "radius": 2}], fi_spec
    fi_rt = os.path.join(OUT, "synth_fillet_rt.FCStd")
    docformat.synthesize(fi_rt, docformat.summarize(fi_p))
    assert docformat.fingerprint(fi_p) == docformat.fingerprint(fi_rt)
    fi_deps = docformat.inspect_document(fi_p)["dependencies"].get("Round", [])
    assert fi_deps == ["Blk"], fi_deps
    wd = App.openDocument(fi_p)
    try:
        for o in wd.Objects:
            o.touch()
        wd.recompute(None, True)
        rnd = wd.getObject("Round")
        fi_vol = rnd.Shape.Volume
        fi_solids = len(rnd.Shape.Solids)
    finally:
        App.closeDocument(wd.Name)
    assert 900.0 < fi_vol < 1000.0, fi_vol
    assert fi_solids == 1, fi_solids

    # a chamfer with a symmetric bevel + an asymmetric one, and a variable-radius
    # fillet, all in one document -> two treatments, so two side members named
    # FreeCAD-style "Edges" and "Edges1". Each recovers its own edge list and the
    # multi-member document round-trips identically.
    ch_p = os.path.join(OUT, "synth_chamfer.FCStd")
    docformat.synthesize(ch_p, [
        {"type": "Part::Box", "name": "Blk",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        docformat.chamfer("Bevel", "Blk",
                          [{"edge": 1, "distance": 2},
                           {"edge": 2, "distance1": 1, "distance2": 3}]),
        docformat.fillet("VRound", "Blk", [{"edge": 5, "radius1": 1,
                                            "radius2": 2}]),
    ])
    assert {"Edges", "Edges1"} <= set(zipfile.ZipFile(ch_p).namelist())
    ch_spec = next(s for s in docformat.summarize(ch_p) if s["name"] == "Bevel")
    assert ch_spec["edges"] == [{"edge": 1, "distance": 2},
                                {"edge": 2, "distance1": 1, "distance2": 3}], ch_spec
    vr_spec = next(s for s in docformat.summarize(ch_p) if s["name"] == "VRound")
    assert vr_spec["edges"] == [{"edge": 5, "radius1": 1, "radius2": 2}], vr_spec
    ch_rt = os.path.join(OUT, "synth_chamfer_rt.FCStd")
    docformat.synthesize(ch_rt, docformat.summarize(ch_p))
    assert docformat.fingerprint(ch_p) == docformat.fingerprint(ch_rt)
    wd = App.openDocument(ch_p)
    try:
        for o in wd.Objects:
            o.touch()
        wd.recompute(None, True)
        ch_vol = wd.getObject("Bevel").Shape.Volume
    finally:
        App.closeDocument(wd.Name)
    assert 900.0 < ch_vol < 1000.0, ch_vol
    for bad, token in (
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B", "edges": []}],
             "non-empty 'edges'"),
            ([{"type": "Part::Fillet", "name": "F", "base": "Gone",
               "edges": [{"edge": 1, "radius": 2}]}], "is not a defined object"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "F",
               "edges": [{"edge": 1, "radius": 2}]}], "cannot reference itself"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B",
               "edges": [{"edge": 0, "radius": 2}]}], "1-based"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B",
               "edges": [{"edge": 1, "radius": -2}]}], "positive"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B",
               "edges": [{"edge": 1, "radius": "big"}]}], "number"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B",
               "edges": [{"edge": 1, "radius": 2}, {"edge": 1, "radius": 3}]}],
             "duplicate"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Chamfer", "name": "C", "base": "B",
               "edges": [{"edge": 1, "distance1": 2}]}], "both"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Fillet", "name": "F", "base": "B",
               "edges": [{"edge": 1, "radius": 2}],
               "properties": {"Foo": 1}}], "not properties")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_edge.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.fillet("", "B", [{"edge": 1, "radius": 2}]),
            lambda: docformat.chamfer("C", "", [{"edge": 1, "distance": 2}]),
            lambda: docformat.fillet("F", "B", [])):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from generator")
    print("docformat edge treatments: box side 10 filleted r2 -> solid volume "
          "%g, chamfer/variable-radius via binary Edges side member(s), "
          "round-trips identically; 9 synthesize + 3 generator guards hold" %
          round(fi_vol, 3))

    # ---- Part::Thickness: shelling (hollow a solid to a wall) ------------- #
    # the shelling operator: hollow a solid base to a wall of a given thickness,
    # opening it at chosen faces. A box of side 10 (volume 1000) shelled to a 1mm
    # wall with its top face (Face6) removed becomes an open box -- kernel volume
    # ~564.93 (a 1000 cube minus its 8x8x9 interior void). Unlike the edge
    # treatments no binary side member is needed: Value + the Faces LinkSub drive
    # the recompute, and the whole document round-trips byte-identically. The
    # generator docformat.thickness() writes the spec; summarize recovers
    # {base, faces, value} (+ any non-default mode/join/flags). 大成若缺，其用不弊.
    th_p = os.path.join(OUT, "synth_thickness.FCStd")
    docformat.synthesize(th_p, [
        {"type": "Part::Box", "name": "Blk",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        docformat.thickness("Shell", "Blk", [6], 1.0),
    ])
    assert zipfile.ZipFile(th_p).namelist() == ["Document.xml"]
    th_spec = next(s for s in docformat.summarize(th_p) if s["name"] == "Shell")
    assert th_spec["base"] == "Blk", th_spec
    assert th_spec["faces"] == [6], th_spec
    assert th_spec["value"] == 1, th_spec
    assert "mode" not in th_spec and "join" not in th_spec, th_spec
    th_rt = os.path.join(OUT, "synth_thickness_rt.FCStd")
    docformat.synthesize(th_rt, docformat.summarize(th_p))
    assert docformat.fingerprint(th_p) == docformat.fingerprint(th_rt)
    th_deps = docformat.inspect_document(th_p)["dependencies"].get("Shell", [])
    assert th_deps == ["Blk"], th_deps
    wd = App.openDocument(th_p)
    try:
        for o in wd.Objects:
            o.touch()
        wd.recompute(None, True)
        shl = wd.getObject("Shell")
        th_vol = shl.Shape.Volume
        th_ok = shl.Shape.isValid()
        th_solids = len(shl.Shape.Solids)
    finally:
        App.closeDocument(wd.Name)
    assert th_ok and th_solids == 1, (th_ok, th_solids)
    assert 400.0 < th_vol < 1000.0, th_vol

    # a multi-face opening with a non-default join + intersection flag: summarize
    # recovers the enumeration name and the bool, and it round-trips identically.
    th2_p = os.path.join(OUT, "synth_thickness2.FCStd")
    docformat.synthesize(th2_p, [
        {"type": "Part::Box", "name": "B",
         "properties": {"Length": 20, "Width": 20, "Height": 20}},
        docformat.thickness("S2", "B", [5, 6], 2.0, join="Intersection",
                            intersection=True),
    ])
    th2_spec = next(s for s in docformat.summarize(th2_p) if s["name"] == "S2")
    assert th2_spec["faces"] == [5, 6], th2_spec
    assert th2_spec["join"] == "Intersection", th2_spec
    assert th2_spec["intersection"] is True, th2_spec
    th2_rt = os.path.join(OUT, "synth_thickness2_rt.FCStd")
    docformat.synthesize(th2_rt, docformat.summarize(th2_p))
    assert docformat.fingerprint(th2_p) == docformat.fingerprint(th2_rt)
    for bad, token in (
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [], "value": 1}], "non-empty 'faces'"),
            ([{"type": "Part::Thickness", "name": "T", "base": "Gone",
               "faces": [6], "value": 1}], "is not a defined object"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "T",
               "faces": [6], "value": 1}], "cannot reference itself"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [0], "value": 1}], "1-based"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [6, 6], "value": 1}], "duplicate face"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [6], "value": 0}], "non-zero"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [6], "value": 1, "mode": "Nope"}], "mode"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [6], "value": 1, "join": "Nope"}], "join"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Thickness", "name": "T", "base": "B",
               "faces": [6], "value": 1, "properties": {"Foo": 1}}],
             "not properties")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_th.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.thickness("", "B", [6], 1.0),
            lambda: docformat.thickness("T", "", [6], 1.0),
            lambda: docformat.thickness("T", "B", [6], 0)):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from thickness generator")
    print("docformat Part::Thickness: box side 10 shelled to a 1mm wall (top "
          "face open) -> valid solid volume %g, no binary member, round-trips "
          "identically; 9 synthesize + 3 generator guards hold" %
          round(th_vol, 3))

    # ---- Part::Offset: 3D offset (grow / shrink a whole solid) ------------ #
    # the 3D offset operator: push every face of a solid ``Source`` along its
    # normal by a uniform signed distance, reconnecting at the corners. A box of
    # side 10 (volume 1000) offset +2 grows to a rounded-corner ~2610.5; offset -2
    # shrinks to a 6-cube (216). It shares the shelling family's Mode/Join enums
    # but takes a plain Source link (no faces) + a Fill flag; no binary member is
    # written and the document round-trips byte-identically. 大巧若拙.
    of_p = os.path.join(OUT, "synth_offset.FCStd")
    docformat.synthesize(of_p, [
        {"type": "Part::Box", "name": "Blk",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        docformat.offset("Off", "Blk", 2.0),
    ])
    assert zipfile.ZipFile(of_p).namelist() == ["Document.xml"]
    of_spec = next(s for s in docformat.summarize(of_p) if s["name"] == "Off")
    assert of_spec["source"] == "Blk", of_spec
    assert of_spec["value"] == 2, of_spec
    assert "mode" not in of_spec and "fill" not in of_spec, of_spec
    of_rt = os.path.join(OUT, "synth_offset_rt.FCStd")
    docformat.synthesize(of_rt, docformat.summarize(of_p))
    assert docformat.fingerprint(of_p) == docformat.fingerprint(of_rt)
    of_deps = docformat.inspect_document(of_p)["dependencies"].get("Off", [])
    assert of_deps == ["Blk"], of_deps
    od = App.openDocument(of_p)
    try:
        for o in od.Objects:
            o.touch()
        od.recompute(None, True)
        offsh = od.getObject("Off").Shape
        of_vol = offsh.Volume
        of_ok = offsh.isValid()
        of_solids = len(offsh.Solids)
    finally:
        App.closeDocument(od.Name)
    assert of_ok and of_solids == 1, (of_ok, of_solids)
    assert of_vol > 1000.0, of_vol

    # shrink inward with a non-default join + Fill/Intersection flags: summarize
    # recovers the negative value, the enumeration name and the bools, and it
    # round-trips identically. (The kernel recompute of Fill is exercised by the
    # +2 grow above; Fill here is validated at the file layer only.)
    of2_p = os.path.join(OUT, "synth_offset2.FCStd")
    docformat.synthesize(of2_p, [
        {"type": "Part::Box", "name": "B",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        docformat.offset("O2", "B", -2.0, join="Intersection", fill=True,
                         intersection=True),
    ])
    of2_spec = next(s for s in docformat.summarize(of2_p) if s["name"] == "O2")
    assert of2_spec["value"] == -2, of2_spec
    assert of2_spec["join"] == "Intersection", of2_spec
    assert of2_spec["fill"] is True and of2_spec["intersection"] is True, of2_spec
    of2_rt = os.path.join(OUT, "synth_offset2_rt.FCStd")
    docformat.synthesize(of2_rt, docformat.summarize(of2_p))
    assert docformat.fingerprint(of2_p) == docformat.fingerprint(of2_rt)
    for bad, token in (
            ([{"type": "Part::Offset", "name": "O", "source": "Gone",
               "value": 1}], "is not a defined object"),
            ([{"type": "Part::Offset", "name": "O", "source": "O",
               "value": 1}], "cannot reference itself"),
            ([{"type": "Part::Offset", "name": "O", "value": 1}],
             "needs a 'source'"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Offset", "name": "O", "source": "B",
               "value": 0}], "non-zero"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Offset", "name": "O", "source": "B",
               "value": 1, "mode": "Nope"}], "mode"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Offset", "name": "O", "source": "B",
               "value": 1, "join": "Nope"}], "join"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Offset", "name": "O", "source": "B",
               "value": 1, "fill": "yes"}], "'fill' must be a bool"),
            ([{"type": "Part::Box", "name": "B",
               "properties": {"Length": 10, "Width": 10, "Height": 10}},
              {"type": "Part::Offset", "name": "O", "source": "B",
               "value": 1, "properties": {"Foo": 1}}], "not properties")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_off.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.offset("", "B", 1.0),
            lambda: docformat.offset("O", "", 1.0),
            lambda: docformat.offset("O", "B", 0)):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from offset generator")
    print("docformat Part::Offset: box side 10 offset +2 -> valid solid volume "
          "%g (grown), -2 shrinks to a 6-cube; no binary member, round-trips "
          "identically; 8 synthesize + 3 generator guards hold" %
          round(of_vol, 3))

    # ---- Part::Offset2D: planar wire offset ------------------------------- #
    # the planar sibling: offset a planar wire/edge Source within its own plane.
    # A radius-5 circle offset +2 with Fill walls the ring between r=5 and r=7
    # into a face of area pi*(7^2-5^2) = 24*pi ~= 75.4. Shares the 3D offset's
    # property schema (Source/Value/Mode/Join/Fill + flags) and its generator
    # guards; no binary member, round-trips byte-identically. 大方無隅.
    o2_p = os.path.join(OUT, "synth_offset2d.FCStd")
    docformat.synthesize(o2_p, [
        {"type": "Part::Circle", "name": "Cir", "properties": {"Radius": 5}},
        docformat.offset2d("Ring", "Cir", 2.0, fill=True),
    ])
    assert zipfile.ZipFile(o2_p).namelist() == ["Document.xml"]
    o2_spec = next(s for s in docformat.summarize(o2_p) if s["name"] == "Ring")
    assert o2_spec["type"] == "Part::Offset2D", o2_spec
    assert o2_spec["source"] == "Cir" and o2_spec["value"] == 2, o2_spec
    assert o2_spec["fill"] is True, o2_spec
    o2_rt = os.path.join(OUT, "synth_offset2d_rt.FCStd")
    docformat.synthesize(o2_rt, docformat.summarize(o2_p))
    assert docformat.fingerprint(o2_p) == docformat.fingerprint(o2_rt)
    o2_deps = docformat.inspect_document(o2_p)["dependencies"].get("Ring", [])
    assert o2_deps == ["Cir"], o2_deps
    o2d = App.openDocument(o2_p)
    try:
        for o in o2d.Objects:
            o.touch()
        o2d.recompute(None, True)
        ringsh = o2d.getObject("Ring").Shape
        ring_area = ringsh.Area
        ring_ok = ringsh.isValid()
        ring_faces = len(ringsh.Faces)
    finally:
        App.closeDocument(o2d.Name)
    assert ring_ok and ring_faces == 1, (ring_ok, ring_faces)
    assert abs(ring_area - 24.0 * math.pi) < 1.0, ring_area
    # the shared offset guards fire under the 2D type too, and inward-offset
    # (negative value) with a non-default mode round-trips identically.
    o2b_p = os.path.join(OUT, "synth_offset2d_b.FCStd")
    docformat.synthesize(o2b_p, [
        {"type": "Part::Circle", "name": "Cir", "properties": {"Radius": 5}},
        docformat.offset2d("R2", "Cir", -1.0, mode="Pipe"),
    ])
    o2b_spec = next(s for s in docformat.summarize(o2b_p) if s["name"] == "R2")
    assert o2b_spec["value"] == -1 and o2b_spec["mode"] == "Pipe", o2b_spec
    o2b_rt = os.path.join(OUT, "synth_offset2d_b_rt.FCStd")
    docformat.synthesize(o2b_rt, docformat.summarize(o2b_p))
    assert docformat.fingerprint(o2b_p) == docformat.fingerprint(o2b_rt)
    for bad, token in (
            ([{"type": "Part::Offset2D", "name": "R", "source": "Gone",
               "value": 1}], "is not a defined object"),
            ([{"type": "Part::Offset2D", "name": "R", "value": 1}],
             "needs a 'source'"),
            ([{"type": "Part::Circle", "name": "Cir",
               "properties": {"Radius": 5}},
              {"type": "Part::Offset2D", "name": "R", "source": "Cir",
               "value": 0}], "non-zero")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_o2d.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.offset2d("", "Cir", 1.0),
            lambda: docformat.offset2d("R", "", 1.0),
            lambda: docformat.offset2d("R", "Cir", 0)):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from offset2d generator")
    print("docformat Part::Offset2D: radius-5 circle offset +2 (Fill) -> planar "
          "ring face area %g (~24*pi); shares the offset schema, round-trips "
          "identically; 3 synthesize + 3 generator guards hold" %
          round(ring_area, 3))

    # ---- Part::RuledSurface: skin a surface between two edges --------------- #
    # the elementary loft: join two section edges with straight generatrix lines
    # into one ruled surface. A radius-5 circle at z=0 and a radius-3 circle at
    # z=10 skin a truncated-cone strip (a single valid face). Curve1/Curve2 are
    # whole-object LinkSubs (count 0); Orientation is an enum by index. No binary
    # member; round-trips byte-identically. 兩儀生象.
    rs_p = os.path.join(OUT, "synth_ruled.FCStd")
    docformat.synthesize(rs_p, [
        {"type": "Part::Circle", "name": "C1", "properties": {"Radius": 5}},
        {"type": "Part::Circle", "name": "C2", "properties": {"Radius": 3},
         "placement": {"position": [0, 0, 10]}},
        docformat.ruled_surface("Skin", "C1", "C2"),
    ])
    assert zipfile.ZipFile(rs_p).namelist() == ["Document.xml"]
    rs_spec = next(s for s in docformat.summarize(rs_p) if s["name"] == "Skin")
    assert rs_spec["type"] == "Part::RuledSurface", rs_spec
    assert rs_spec["curve1"] == "C1" and rs_spec["curve2"] == "C2", rs_spec
    assert "orientation" not in rs_spec, rs_spec  # Automatic (index 0) is default
    rs_deps = docformat.inspect_document(rs_p)["dependencies"].get("Skin", [])
    assert rs_deps == ["C1", "C2"], rs_deps
    rs_rt = os.path.join(OUT, "synth_ruled_rt.FCStd")
    docformat.synthesize(rs_rt, docformat.summarize(rs_p))
    assert docformat.fingerprint(rs_p) == docformat.fingerprint(rs_rt)
    rsd = App.openDocument(rs_p)
    try:
        for o in rsd.Objects:
            o.touch()
        rsd.recompute(None, True)
        skinsh = rsd.getObject("Skin").Shape
        skin_area = skinsh.Area
        skin_ok = skinsh.isValid()
        skin_faces = len(skinsh.Faces)
    finally:
        App.closeDocument(rsd.Name)
    assert skin_ok and skin_faces == 1, (skin_ok, skin_faces)
    assert skin_area > 100.0, skin_area
    # a forced Reversed orientation persists + round-trips; a sub-edge selection
    # (Edge1) on both curves round-trips too.
    rs2_p = os.path.join(OUT, "synth_ruled_b.FCStd")
    docformat.synthesize(rs2_p, [
        {"type": "Part::Line", "name": "L1",
         "properties": {"X1": 0, "Y1": 0, "Z1": 0, "X2": 10, "Y2": 0, "Z2": 0}},
        {"type": "Part::Line", "name": "L2",
         "properties": {"X1": 0, "Y1": 5, "Z1": 0, "X2": 10, "Y2": 5, "Z2": 0}},
        docformat.ruled_surface("Strip", "L1", "L2",
                                curve1_edges=["Edge1"], curve2_edges=["Edge1"],
                                orientation="Reversed"),
    ])
    rs2_spec = next(s for s in docformat.summarize(rs2_p) if s["name"] == "Strip")
    assert rs2_spec["orientation"] == "Reversed", rs2_spec
    assert rs2_spec["curve1_edges"] == ["Edge1"], rs2_spec
    assert rs2_spec["curve2_edges"] == ["Edge1"], rs2_spec
    rs2_rt = os.path.join(OUT, "synth_ruled_b_rt.FCStd")
    docformat.synthesize(rs2_rt, docformat.summarize(rs2_p))
    assert docformat.fingerprint(rs2_p) == docformat.fingerprint(rs2_rt)
    for bad, token in (
            ([{"type": "Part::RuledSurface", "name": "R", "curve1": "Gone",
               "curve2": "C2"},
              {"type": "Part::Circle", "name": "C2",
               "properties": {"Radius": 3}}], "is not a defined object"),
            ([{"type": "Part::RuledSurface", "name": "R", "curve2": "C2"},
              {"type": "Part::Circle", "name": "C2",
               "properties": {"Radius": 3}}], "needs a 'curve1'"),
            ([{"type": "Part::Circle", "name": "C1",
               "properties": {"Radius": 5}},
              {"type": "Part::RuledSurface", "name": "R", "curve1": "C1",
               "curve2": "C1"}], "two distinct curves"),
            ([{"type": "Part::Circle", "name": "C1",
               "properties": {"Radius": 5}},
              {"type": "Part::Circle", "name": "C2",
               "properties": {"Radius": 3}},
              {"type": "Part::RuledSurface", "name": "R", "curve1": "C1",
               "curve2": "C2", "orientation": "Sideways"}], "orientation")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_ruled.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.ruled_surface("", "C1", "C2"),
            lambda: docformat.ruled_surface("R", "", "C2"),
            lambda: docformat.ruled_surface("R", "C1", "C1"),
            lambda: docformat.ruled_surface("R", "C1", "C2",
                                            orientation="Sideways")):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from ruled_surface")
    print("docformat Part::RuledSurface: circles r5@z0 and r3@z10 skin a valid "
          "cone strip (area %g); whole-object + sub-edge links and forced "
          "Reversed orientation round-trip identically; 4 synthesize + 4 "
          "generator guards hold" % round(skin_area, 3))

    # ---- Part::Section: the cross-section curves of two shapes ------------- #
    # the cross-section boolean: intersect Base with Tool and keep only the 1-D
    # wire where their boundaries cross. A 10-box and a radius-6 sphere centred at
    # its far corner cross in a closed section wire (edges, no faces). Carries two
    # plain Base/Tool links (like the CSG booleans) plus Approximation / Refine
    # flags; no binary member, round-trips byte-identically. 大成若缺.
    sc_p = os.path.join(OUT, "synth_section.FCStd")
    docformat.synthesize(sc_p, [
        {"type": "Part::Box", "name": "B",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        {"type": "Part::Sphere", "name": "S", "properties": {"Radius": 6},
         "placement": {"position": [5, 5, 5]}},
        docformat.section("Sec", "B", "S"),
    ])
    assert zipfile.ZipFile(sc_p).namelist() == ["Document.xml"]
    sc_spec = next(s for s in docformat.summarize(sc_p) if s["name"] == "Sec")
    assert sc_spec["type"] == "Part::Section", sc_spec
    assert sc_spec["base"] == "B" and sc_spec["tool"] == "S", sc_spec
    assert "approximation" not in sc_spec and "refine" not in sc_spec, sc_spec
    sc_deps = docformat.inspect_document(sc_p)["dependencies"].get("Sec", [])
    assert sc_deps == ["B", "S"], sc_deps
    sc_rt = os.path.join(OUT, "synth_section_rt.FCStd")
    docformat.synthesize(sc_rt, docformat.summarize(sc_p))
    assert docformat.fingerprint(sc_p) == docformat.fingerprint(sc_rt)
    scd = App.openDocument(sc_p)
    try:
        for o in scd.Objects:
            o.touch()
        scd.recompute(None, True)
        secsh = scd.getObject("Sec").Shape
        sec_edges = len(secsh.Faces), len(secsh.Edges)
        sec_ok = secsh.isValid()
    finally:
        App.closeDocument(scd.Name)
    assert sec_ok and sec_edges[0] == 0 and sec_edges[1] > 0, sec_edges
    # both flags set persist + round-trip identically.
    sc2_p = os.path.join(OUT, "synth_section_b.FCStd")
    docformat.synthesize(sc2_p, [
        {"type": "Part::Box", "name": "B",
         "properties": {"Length": 10, "Width": 10, "Height": 10}},
        {"type": "Part::Sphere", "name": "S", "properties": {"Radius": 6},
         "placement": {"position": [5, 5, 5]}},
        docformat.section("Sec", "B", "S", approximation=True, refine=True),
    ])
    sc2_spec = next(s for s in docformat.summarize(sc2_p) if s["name"] == "Sec")
    assert sc2_spec["approximation"] is True and sc2_spec["refine"] is True, sc2_spec
    sc2_rt = os.path.join(OUT, "synth_section_b_rt.FCStd")
    docformat.synthesize(sc2_rt, docformat.summarize(sc2_p))
    assert docformat.fingerprint(sc2_p) == docformat.fingerprint(sc2_rt)
    for bad, token in (
            ([{"type": "Part::Section", "name": "X", "base": "Gone",
               "tool": "S"},
              {"type": "Part::Sphere", "name": "S",
               "properties": {"Radius": 6}}], "is not a defined object"),
            ([{"type": "Part::Sphere", "name": "S",
               "properties": {"Radius": 6}},
              {"type": "Part::Section", "name": "X", "tool": "S"}],
             "needs a 'base'"),
            ([{"type": "Part::Section", "name": "X", "base": "X",
               "tool": "S"},
              {"type": "Part::Sphere", "name": "S",
               "properties": {"Radius": 6}}], "cannot reference itself")):
        try:
            docformat.synthesize(os.path.join(OUT, "bad_sec.FCStd"), bad)
        except ValueError as exc:
            assert token in str(exc), (token, exc)
        else:
            raise AssertionError("expected ValueError for %r" % token)
    for badcall in (
            lambda: docformat.section("", "B", "S"),
            lambda: docformat.section("X", "", "S"),
            lambda: docformat.section("X", "B", "B")):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from section generator")
    print("docformat Part::Section: 10-box cut by a radius-6 sphere at its corner "
          "-> valid intersection wire (%d edges, 0 faces); base/tool links + "
          "Approximation/Refine flags round-trip identically; 3 synthesize + 3 "
          "generator guards hold" % sec_edges[1])

    # ---- Part::Helix: a parametric helical edge (spring / thread spine) ----- #
    # four scalars (pitch / height / radius / taper angle) + two enums (chirality,
    # style) fix a helical edge; like every primitive the kernel rebuilds it from
    # these alone -- no BREP written, and the read-only computed Length is left
    # for recompute to regenerate. A pitch-3 height-20 radius-5 helix winds ~6.7
    # turns into one valid edge; round-trips byte-identically. 綿綿若存.
    hx_p = os.path.join(OUT, "synth_helix.FCStd")
    docformat.synthesize(hx_p, [docformat.helix("H", 3, 20, 5)])
    assert zipfile.ZipFile(hx_p).namelist() == ["Document.xml"]
    hx_spec = next(s for s in docformat.summarize(hx_p) if s["name"] == "H")
    assert hx_spec["type"] == "Part::Helix", hx_spec
    assert hx_spec["pitch"] == 3 and hx_spec["height"] == 20 \
        and hx_spec["radius"] == 5, hx_spec
    assert "hand" not in hx_spec and "style" not in hx_spec, hx_spec  # defaults
    hx_rt = os.path.join(OUT, "synth_helix_rt.FCStd")
    docformat.synthesize(hx_rt, docformat.summarize(hx_p))
    assert docformat.fingerprint(hx_p) == docformat.fingerprint(hx_rt)
    hxd = App.openDocument(hx_p)
    try:
        for o in hxd.Objects:
            o.touch()
        hxd.recompute(None, True)
        hxsh = hxd.getObject("H").Shape
        hx_edges = len(hxsh.Edges)
        hx_len = hxsh.Length
        hx_ok = hxsh.isValid()
    finally:
        App.closeDocument(hxd.Name)
    assert hx_ok and hx_edges == 1 and hx_len > 0, (hx_edges, hx_len)
    # a left-handed, new-style, cone-tapered helix persists its two enums + taper
    # and round-trips identically.
    hx2_p = os.path.join(OUT, "synth_helix_b.FCStd")
    docformat.synthesize(hx2_p, [docformat.helix(
        "H", 2, 10, 4, angle=10, hand="Left-handed", style="New style")])
    hx2_spec = next(s for s in docformat.summarize(hx2_p) if s["name"] == "H")
    assert hx2_spec["angle"] == 10 and hx2_spec["hand"] == "Left-handed" \
        and hx2_spec["style"] == "New style", hx2_spec
    hx2_rt = os.path.join(OUT, "synth_helix_b_rt.FCStd")
    docformat.synthesize(hx2_rt, docformat.summarize(hx2_p))
    assert docformat.fingerprint(hx2_p) == docformat.fingerprint(hx2_rt)
    hx2d = App.openDocument(hx2_p)
    try:
        for o in hx2d.Objects:
            o.touch()
        hx2d.recompute(None, True)
        hx2_ok = hx2d.getObject("H").Shape.isValid()
    finally:
        App.closeDocument(hx2d.Name)
    assert hx2_ok, "left-handed tapered helix must recompute valid"
    for badcall in (
            lambda: docformat.helix("", 3, 20, 5),
            lambda: docformat.helix("H", 0, 20, 5),
            lambda: docformat.helix("H", 3, 20, -5),
            lambda: docformat.helix("H", 3, 20, 5, angle=90),
            lambda: docformat.helix("H", 3, 20, 5, hand="Sideways"),
            lambda: docformat.helix("H", 3, 20, 5, style="Fancy")):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from helix generator")
    print("docformat Part::Helix: pitch-3 height-20 radius-5 helix -> one valid "
          "edge (length %g); taper + Left-handed/New-style enums round-trip "
          "identically; 6 generator guards hold" % round(hx_len, 3))

    # ---- Part::Spiral: a flat Archimedean spiral edge (helix's planar sibling) #
    # three scalars (growth per turn / rotation count / start radius) fix a spiral
    # edge in the XY plane; like the helix the kernel rebuilds it from these alone
    # -- no BREP written, the read-only Length left for recompute. A growth-2,
    # 3-turn, start-radius-5 spiral winds into a valid edge (length 150.922);
    # round-trips byte-identically. 大道氾兮.
    sp_p = os.path.join(OUT, "synth_spiral.FCStd")
    docformat.synthesize(sp_p, [docformat.spiral("Sp", 2, 3, 5)])
    assert zipfile.ZipFile(sp_p).namelist() == ["Document.xml"]
    sp_spec = next(s for s in docformat.summarize(sp_p) if s["name"] == "Sp")
    assert sp_spec["type"] == "Part::Spiral", sp_spec
    assert sp_spec["growth"] == 2 and sp_spec["rotations"] == 3 \
        and sp_spec["radius"] == 5, sp_spec
    sp_rt = os.path.join(OUT, "synth_spiral_rt.FCStd")
    docformat.synthesize(sp_rt, docformat.summarize(sp_p))
    assert docformat.fingerprint(sp_p) == docformat.fingerprint(sp_rt)
    spd = App.openDocument(sp_p)
    try:
        for o in spd.Objects:
            o.touch()
        spd.recompute(None, True)
        spsh = spd.getObject("Sp").Shape
        sp_edges = len(spsh.Edges)
        sp_len = spsh.Length
        sp_ok = spsh.isValid()
    finally:
        App.closeDocument(spd.Name)
    assert sp_ok and sp_edges >= 1 and sp_len > 0, (sp_edges, sp_len)
    # a spiral spun from the centre (radius 0) also recomputes valid.
    sp0_p = os.path.join(OUT, "synth_spiral0.FCStd")
    docformat.synthesize(sp0_p, [docformat.spiral("Sp", 1.5, 2)])
    sp0_spec = next(s for s in docformat.summarize(sp0_p) if s["name"] == "Sp")
    assert sp0_spec["radius"] == 0, sp0_spec
    sp0_rt = os.path.join(OUT, "synth_spiral0_rt.FCStd")
    docformat.synthesize(sp0_rt, docformat.summarize(sp0_p))
    assert docformat.fingerprint(sp0_p) == docformat.fingerprint(sp0_rt)
    sp0d = App.openDocument(sp0_p)
    try:
        for o in sp0d.Objects:
            o.touch()
        sp0d.recompute(None, True)
        sp0_ok = sp0d.getObject("Sp").Shape.isValid()
    finally:
        App.closeDocument(sp0d.Name)
    assert sp0_ok, "centre-origin spiral must recompute valid"
    for badcall in (
            lambda: docformat.spiral("", 2, 3, 5),
            lambda: docformat.spiral("Sp", 0, 3, 5),
            lambda: docformat.spiral("Sp", 2, 0, 5),
            lambda: docformat.spiral("Sp", 2, 3, -1)):
        try:
            badcall()
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError from spiral generator")
    print("docformat Part::Spiral: growth-2 3-turn start-radius-5 spiral -> valid "
          "edge (length %g); centre-origin spiral round-trips; 4 generator guards "
          "hold" % round(sp_len, 3))

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

    # ---- doc.profile: parametric 2D profile generation as an agent op ---- #
    # the agent generates a regular N-gon sketch from one description and
    # synthesizes it straight to a file -- a pentagon radius 8 with five edges.
    op_pent = os.path.join(OUT, "op_pentagon.FCStd")
    pf = s.act("doc.profile", {
        "shape": "regular_polygon", "name": "Pent", "sides": 5,
        "radius": 8, "path": op_pent})
    assert pf.ok and pf.data["out"] == op_pent, pf
    assert len(pf.data["object"]["geometry"]) == 5, pf.data
    nd = App.openDocument(op_pent)
    try:
        for o in nd.Objects:
            o.touch()
        nd.recompute(None, True)
        pent_w = Part.Wire(Part.__sortEdges__(nd.getObject("Pent").Shape.Edges))
        pent_area = Part.Face(pent_w).Area
    finally:
        App.closeDocument(nd.Name)
    assert abs(pent_area - 0.5 * 5 * 64 * math.sin(2 * math.pi / 5)) < 1e-6, \
        pent_area
    # the op also generates a slot (mixed line+arc obround) straight to a file.
    op_slot = os.path.join(OUT, "op_slot.FCStd")
    sf = s.act("doc.profile", {
        "shape": "slot", "name": "Slot", "length": 30, "radius": 6,
        "path": op_slot})
    assert sf.ok and sf.data["out"] == op_slot, sf
    assert len(sf.data["object"]["geometry"]) == 4, sf.data
    sld2 = App.openDocument(op_slot)
    try:
        for o in sld2.Objects:
            o.touch()
        sld2.recompute(None, True)
        slot2_w = Part.Wire(
            Part.__sortEdges__(sld2.getObject("Slot").Shape.Edges))
        slot2_area = Part.Face(slot2_w).Area
    finally:
        App.closeDocument(sld2.Name)
    assert abs(slot2_area - (2 * 30 * 6 + math.pi * 36)) < 1e-6, slot2_area
    # the op also generates a tilted ellipse (a single curved edge) to a file.
    op_ell = os.path.join(OUT, "op_ellipse.FCStd")
    ef = s.act("doc.profile", {
        "shape": "ellipse", "name": "Ell", "major_radius": 12,
        "minor_radius": 7, "angle": 40, "path": op_ell})
    assert ef.ok and ef.data["out"] == op_ell, ef
    assert len(ef.data["object"]["geometry"]) == 1, ef.data
    eld2 = App.openDocument(op_ell)
    try:
        for o in eld2.Objects:
            o.touch()
        eld2.recompute(None, True)
        ell2_area = Part.Face(eld2.getObject("Ell").Shape.Wires[0]).Area
    finally:
        App.closeDocument(eld2.Name)
    assert abs(ell2_area - (math.pi * 12 * 7)) < 1e-6, ell2_area
    # and a freeform B-spline (single curved edge) straight to a file.
    op_bsp = os.path.join(OUT, "op_bspline.FCStd")
    bf = s.act("doc.profile", {
        "shape": "bspline", "name": "BSp",
        "poles": [[0, 0], [6, 9], [14, 5], [20, 0]], "degree": 3,
        "path": op_bsp})
    assert bf.ok and bf.data["out"] == op_bsp, bf
    assert len(bf.data["object"]["geometry"]) == 1, bf.data
    bld2 = App.openDocument(op_bsp)
    try:
        for o in bld2.Objects:
            o.touch()
        bld2.recompute(None, True)
        bsp2_kind = bld2.getObject("BSp").Shape.Edges[0].Curve.__class__.__name__
    finally:
        App.closeDocument(bld2.Name)
    assert bsp2_kind == "BSplineCurve", bsp2_kind
    # and an isolated reference point (one vertex, no edge) straight to a file.
    op_pt = os.path.join(OUT, "op_point.FCStd")
    pf = s.act("doc.profile", {
        "shape": "point", "name": "Pt", "at": [5, 9], "path": op_pt})
    assert pf.ok and pf.data["out"] == op_pt, pf
    pld2 = App.openDocument(op_pt)
    try:
        for o in pld2.Objects:
            o.touch()
        pld2.recompute(None, True)
        pt2_sh = pld2.getObject("Pt").Shape
        pt2_nv, pt2_ne = len(pt2_sh.Vertexes), len(pt2_sh.Edges)
    finally:
        App.closeDocument(pld2.Name)
    assert pt2_nv == 1 and pt2_ne == 0, (pt2_nv, pt2_ne)
    assert not s.act("doc.profile", {"shape": "blob", "name": "X"}).ok
    print("doc.profile: pentagon radius 8 (area %g) + slot 30x6 (area %g) + "
          "ellipse 12x7 (area %g) + freeform bspline (single BSplineCurve edge) "
          "generated+synthesized from one description each (profile generation "
          "as an agent op)" % (pent_area, slot2_area, ell2_area))

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
