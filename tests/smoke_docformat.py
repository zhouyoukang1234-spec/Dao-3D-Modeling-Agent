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

    print("DOCFORMAT SMOKE OK")


if __name__ == "__main__":
    main()
