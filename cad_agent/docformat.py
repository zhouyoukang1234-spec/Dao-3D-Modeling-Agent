"""FreeCAD ``.FCStd`` document format -- reverse-engineered to its persistence
root, owned by *our* system.

Below the Python API surface (mapped by :mod:`cad_agent.capability`) lies the
layer where *every* FreeCAD document -- whether authored in the GUI or by a
script -- ultimately lives: the ``.FCStd`` file. It is a plain ``zip``:

* ``Document.xml`` -- the object graph. ``<Objects>`` names every object with
  its kernel ``TypeId`` (e.g. ``Part::Box``), a numeric ``id``, and an
  ``<ObjectDeps>`` adjacency list (the true dependency DAG the recompute engine
  walks). ``<ObjectData>`` then carries each object's typed properties
  (Placement, Length, links to other objects, material, ...).
* ``<Name>.Shape.brp`` -- one OpenCascade BREP file per shape, the raw boundary
  representation; the geometric root the API only ever wraps.
* ``GuiDocument.xml`` -- view-only state (colours, camera); irrelevant headless.

This module reads that file *without the live kernel*: pure ``zipfile`` +
``xml.etree``. So the system can introspect, diff and reason about a document at
the level the GUI actually persists to -- fusing the file layer with the API
layer instead of only ever calling the shallow scripting surface. The companion
:func:`fingerprint` distils a document to a stable, comparable summary used to
prove a scripted build round-trips through the file format unchanged.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
import zipfile
from typing import Any, Dict, List, Optional

DOCUMENT_XML = "Document.xml"
GUI_XML = "GuiDocument.xml"
_BREP_EXT = (".brp", ".brep")


def _prop_value(prop: ET.Element) -> Any:
    """Best-effort compact value for a ``<Property>`` element.

    The persisted forms are heterogeneous; rather than model every property
    type, pull the shape that matters for reasoning about a document: a scalar
    ``value=`` attribute, a link target, a link list, or a placement. Anything
    else collapses to its child tag so the property is still *present* in the
    summary without inventing a value.
    """
    kids = list(prop)
    if not kids:
        return None
    k = kids[0]
    tag = k.tag
    if tag == "Link":
        return {"link": k.get("value")}
    if tag in ("LinkList", "LinkSubList", "LinkSub"):
        return {"link_list": [c.get("value") or c.get("obj")
                              for c in k if (c.get("value") or c.get("obj"))]}
    if tag == "PropertyPlacement":
        return {"placement": {a: _maybe_float(v) for a, v in k.attrib.items()}}
    if "value" in k.attrib:
        return _maybe_float(k.attrib["value"])
    if "file" in k.attrib:
        return {"file": k.attrib["file"]}
    return {"_tag": tag}


def _maybe_float(s: str) -> Any:
    try:
        f = float(s)
    except (TypeError, ValueError):
        if s in ("true", "false"):
            return s == "true"
        return s
    return int(f) if f.is_integer() else round(f, 6)


def _object_deps(objects_el: Optional[ET.Element]) -> Dict[str, List[str]]:
    """The recompute dependency DAG: object name -> the objects it depends on."""
    deps: Dict[str, List[str]] = {}
    if objects_el is None:
        return deps
    for od in objects_el.findall("ObjectDeps"):
        name = od.get("Name")
        if name is None:
            continue
        deps[name] = [d.get("Name") for d in od.findall("Dep")
                      if d.get("Name")]
    return deps


def _object_types(objects_el: Optional[ET.Element]) -> "List[Dict[str, Any]]":
    out: List[Dict[str, Any]] = []
    if objects_el is None:
        return out
    for o in objects_el.findall("Object"):
        out.append({"name": o.get("name"), "type": o.get("type"),
                    "id": o.get("id")})
    return out


def _object_properties(data_el: Optional[ET.Element]) -> "Dict[str, Dict[str, Any]]":
    """name -> {property name -> {type, value}} for every persisted (non-
    transient) property carrying a value."""
    out: Dict[str, Dict[str, Any]] = {}
    if data_el is None:
        return out
    for obj in data_el.findall("Object"):
        name = obj.get("name")
        if name is None:
            continue
        props: Dict[str, Any] = {}
        for props_el in obj.findall("Properties"):
            for prop in props_el.findall("Property"):
                pname = prop.get("name")
                val = _prop_value(prop)
                if pname is not None and val is not None:
                    props[pname] = {"type": prop.get("type"), "value": val}
        out[name] = props
    return out


def inspect_document(path: str) -> Dict[str, Any]:
    """Parse a ``.FCStd`` into a structured, kernel-free view of its contents.

    Returns the document metadata, the object graph (each object's name /
    ``TypeId`` / id), the dependency DAG, the persisted properties, and the BREP
    geometry files with their byte sizes -- everything needed to reason about
    what a document *is* on disk. Raises ``ValueError`` (never a raw
    ``BadZipFile`` / ``KeyError``) for anything that is not a readable FreeCAD
    document.
    """
    if not isinstance(path, str) or not path.strip():
        raise ValueError("inspect_document: path must be a non-empty string")
    if not os.path.exists(path):
        raise ValueError("inspect_document: no such file: %s" % path)
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise ValueError(
            "inspect_document: %s is not a .FCStd (zip) document" % path)
    with z:
        names = z.namelist()
        if DOCUMENT_XML not in names:
            raise ValueError(
                "inspect_document: %s has no %s -- not a FreeCAD document"
                % (path, DOCUMENT_XML))
        try:
            root = ET.fromstring(z.read(DOCUMENT_XML))
        except ET.ParseError as exc:
            raise ValueError(
                "inspect_document: corrupt %s in %s (%s)"
                % (DOCUMENT_XML, path, exc))
        objects_el = root.find("Objects")
        data_el = root.find("ObjectData")
        objects = _object_types(objects_el)
        deps = _object_deps(objects_el)
        props = _object_properties(data_el)
        breps = [{"file": n, "bytes": z.getinfo(n).file_size}
                 for n in names if n.lower().endswith(_BREP_EXT)]
        type_counts: Dict[str, int] = {}
        for o in objects:
            t = o["type"] or "?"
            type_counts[t] = type_counts.get(t, 0) + 1
        return {
            "schema_version": root.get("SchemaVersion"),
            "program_version": root.get("ProgramVersion"),
            "file_version": root.get("FileVersion"),
            "objects": objects,
            "object_count": len(objects),
            "type_counts": dict(sorted(type_counts.items())),
            "dependencies": deps,
            "dependency_edges": sum(len(v) for v in deps.values()),
            "properties": props,
            "brep_files": breps,
            "brep_bytes": sum(b["bytes"] for b in breps),
            "has_gui": GUI_XML in names,
            "entries": sorted(names),
        }


def fingerprint(path: str) -> Dict[str, Any]:
    """A stable, comparable distillation of a document's *structure*.

    Drops volatile metadata (timestamps, ids, byte sizes) and keeps what a
    scripted build is expected to reproduce deterministically: the multiset of
    object ``TypeId``s and the dependency edges (as name pairs). Used to prove a
    document round-trips through the ``.FCStd`` format unchanged -- i.e. that
    the file layer and the API layer agree.
    """
    info = inspect_document(path)
    edges = sorted("%s->%s" % (src, dst)
                   for src, dsts in info["dependencies"].items()
                   for dst in dsts)
    return {
        "object_count": info["object_count"],
        "type_counts": info["type_counts"],
        "dependency_edges": edges,
        "brep_count": len(info["brep_files"]),
    }
