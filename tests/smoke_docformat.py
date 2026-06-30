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

    print("DOCFORMAT SMOKE OK")


if __name__ == "__main__":
    main()
