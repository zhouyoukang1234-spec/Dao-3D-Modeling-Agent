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

import hashlib
import os
import re
import xml.etree.ElementTree as ET
import zipfile
from typing import Any, Dict, List, Optional

# An identifier in a FreeCAD expression: a bare object/property name, or a
# label reference wrapped in << >> (which may contain spaces / unicode).
_IDENT_RE = re.compile(r"<<(?P<label>.+?)>>|(?P<name>[A-Za-z_][A-Za-z_0-9]*)")

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
    # complex container property (spreadsheet Cells, ExpressionEngine, ...):
    # collapse the whole subtree to a canonical string so a genuine edit to its
    # contents is still detected by a value comparison, rather than vanishing.
    return {"xml": "".join(_canon(c) for c in kids)}


def _canon(el: ET.Element) -> str:
    """Stable, order-insensitive string for an XML subtree (tag + sorted attrs
    + children), so two serialisations compare equal iff they are structurally
    the same."""
    parts = [el.tag]
    parts += ["%s=%s" % (k, el.attrib[k]) for k in sorted(el.attrib)]
    parts += [_canon(c) for c in el]
    return "(" + " ".join(parts) + ")"


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


def _object_expressions(data_el: Optional[ET.Element]) -> "Dict[str, List[Dict[str, str]]]":
    """name -> bound expressions read from each object's ``ExpressionEngine``.

    ``App::PropertyExpressionEngine`` is where FreeCAD persists the parametric
    *wiring* the GUI's expression editor authors: each ``<Expression>`` binds a
    property ``path`` (e.g. ``Length``, ``Constraints.width``) to a ``formula``
    that may reference other objects (``Spreadsheet.L``, ``<<Base>>.Height``).
    Surfaced structurally here it stops being an opaque XML blob and becomes a
    first-class view of how the document computes itself.
    """
    out: Dict[str, List[Dict[str, str]]] = {}
    if data_el is None:
        return out
    for obj in data_el.findall("Object"):
        name = obj.get("name")
        if name is None:
            continue
        exprs: List[Dict[str, str]] = []
        for props_el in obj.findall("Properties"):
            for prop in props_el.findall("Property"):
                ee = prop.find("ExpressionEngine")
                if ee is None:
                    continue
                for e in ee.findall("Expression"):
                    epath = e.get("path")
                    formula = e.get("expression")
                    if epath is not None and formula is not None:
                        exprs.append({"path": epath, "formula": formula})
        if exprs:
            out[name] = exprs
    return out


def _expression_refs(formula: str, names: set, label_to_name: Dict[str, str]) -> set:
    """The object names a formula references -- ``<<Label>>`` mapped to its name,
    plus any bare identifier that is itself an object name (``Spreadsheet.L`` ->
    ``Spreadsheet``). Best-effort, but only ever yields names that exist in the
    document, so the resulting graph has no dangling targets.
    """
    refs: set = set()
    for m in _IDENT_RE.finditer(formula):
        label, tok = m.group("label"), m.group("name")
        if label is not None and label in label_to_name:
            refs.add(label_to_name[label])
        elif tok is not None and tok in names:
            refs.add(tok)
    return refs


def _expression_edges(
    expressions: Dict[str, List[Dict[str, str]]],
    names: set,
    label_to_name: Dict[str, str],
) -> "List[str]":
    """The parametric dependency graph carried by expressions: ``src->dst`` for
    every object whose formula references another object (self-refs dropped).

    Distinct from ``dependencies`` (the recompute link DAG): two objects can be
    coupled purely through an expression with no ``App::PropertyLink`` between
    them, an edge only this layer sees.
    """
    edges: set = set()
    for src, exprs in expressions.items():
        for e in exprs:
            for dst in _expression_refs(e["formula"], names, label_to_name):
                if dst != src:
                    edges.add("%s->%s" % (src, dst))
    return sorted(edges)


def _label_map(props: Dict[str, Dict[str, Any]]) -> Dict[str, str]:
    """User-facing Label -> object name, for resolving ``<<Label>>`` refs."""
    out: Dict[str, str] = {}
    for name, p in props.items():
        lbl = p.get("Label", {}).get("value")
        if isinstance(lbl, str) and lbl:
            out.setdefault(lbl, name)
    return out


def inspect_document(path: str) -> Dict[str, Any]:
    """Parse a ``.FCStd`` into a structured, kernel-free view of its contents.

    Returns the document metadata, the object graph (each object's name /
    ``TypeId`` / id), the dependency DAG, the persisted properties, and the BREP
    geometry files with their byte size and a content hash -- everything needed
    to reason about what a document *is* on disk. Raises ``ValueError`` (never a raw
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
        expressions = _object_expressions(data_el)
        obj_names = {o["name"] for o in objects if o["name"]}
        expr_edges = _expression_edges(expressions, obj_names, _label_map(props))
        breps = []
        for n in names:
            if not n.lower().endswith(_BREP_EXT):
                continue
            data = z.read(n)
            breps.append({"file": n, "bytes": len(data),
                          "sha1": hashlib.sha1(data).hexdigest()[:16]})
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
            "expressions": expressions,
            "expression_count": sum(len(v) for v in expressions.values()),
            "expression_edges": expr_edges,
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


def _edge_set(deps: Dict[str, List[str]]) -> set:
    return {(src, dst) for src, dsts in deps.items() for dst in dsts}


def diff(path_a: str, path_b: str) -> Dict[str, Any]:
    """Structural diff between two ``.FCStd`` documents (``a`` -> ``b``).

    The *verify* half of working at the persistence layer: given the file before
    and after an edit (scripted or GUI), report exactly what changed without the
    kernel -- which objects were added / removed, whose ``TypeId`` changed, which
    dependency edges appeared or vanished, and, for objects present in both,
    which property values differ. ``identical`` is True iff nothing structural or
    value-level changed.
    """
    a = inspect_document(path_a)
    b = inspect_document(path_b)
    a_types = {o["name"]: o["type"] for o in a["objects"]}
    b_types = {o["name"]: o["type"] for o in b["objects"]}
    added = sorted(set(b_types) - set(a_types))
    removed = sorted(set(a_types) - set(b_types))
    shared = sorted(set(a_types) & set(b_types))
    retyped = {n: {"from": a_types[n], "to": b_types[n]}
               for n in shared if a_types[n] != b_types[n]}

    a_edges, b_edges = _edge_set(a["dependencies"]), _edge_set(b["dependencies"])
    edges_added = sorted("%s->%s" % e for e in (b_edges - a_edges))
    edges_removed = sorted("%s->%s" % e for e in (a_edges - b_edges))

    prop_changes: Dict[str, Dict[str, Any]] = {}
    a_props, b_props = a["properties"], b["properties"]
    for n in shared:
        pa, pb = a_props.get(n, {}), b_props.get(n, {})
        changed: Dict[str, Any] = {}
        for key in sorted(set(pa) | set(pb)):
            va = pa.get(key, {}).get("value")
            vb = pb.get(key, {}).get("value")
            if va != vb:
                changed[key] = {"from": va, "to": vb}
        if changed:
            prop_changes[n] = changed

    # geometry lives in the per-shape BREP files, not Document.xml: a resized
    # plain solid changes only its .brp, often without changing its byte size
    # (same topology, different coordinates). Compare a content hash so any
    # geometry edit is caught.
    a_brep = {b["file"]: b["sha1"] for b in a["brep_files"]}
    b_brep = {b["file"]: b["sha1"] for b in b["brep_files"]}
    brep_changes = sorted(f for f in (set(a_brep) & set(b_brep))
                          if a_brep[f] != b_brep[f])

    # expression wiring: a binding added / removed / re-pointed is real
    # parametric intent that a plain property diff (the collapsed ExpressionEngine
    # blob) only reports as an opaque value change. Surface it per object.path.
    def _expr_map(info: Dict[str, Any]) -> Dict[str, str]:
        return {"%s.%s" % (name, e["path"]): e["formula"]
                for name, exprs in info["expressions"].items() for e in exprs}

    a_expr, b_expr = _expr_map(a), _expr_map(b)
    expr_changes: Dict[str, Dict[str, Any]] = {}
    for key in sorted(set(a_expr) | set(b_expr)):
        fa, fb = a_expr.get(key), b_expr.get(key)
        if fa != fb:
            expr_changes[key] = {"from": fa, "to": fb}

    identical = not (added or removed or retyped or edges_added
                     or edges_removed or prop_changes or brep_changes
                     or expr_changes)
    return {
        "identical": identical,
        "objects_added": added,
        "objects_removed": removed,
        "types_changed": retyped,
        "edges_added": edges_added,
        "edges_removed": edges_removed,
        "property_changes": prop_changes,
        "expression_changes": expr_changes,
        "brep_changes": brep_changes,
    }


def _format_value(value: Any, old: Optional[str]) -> str:
    """Serialise ``value`` to match the style of the ``old`` persisted value
    (float-with-decimals vs int vs bool)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        f = float(value)
        if old is not None and ("." in old or "e" in old.lower()):
            return repr(f)
        return str(int(f)) if f.is_integer() else repr(f)
    return str(value)


def edit_property(path: str, obj: str, prop: str, value: Any,
                  out: Optional[str] = None) -> Dict[str, Any]:
    """Edit a scalar property's value directly in the ``.FCStd`` -- the *act*
    half, kernel-free.

    The persisted document is the root both the GUI and scripts ultimately write
    to. This mutates a value at that layer (the relevant ``<Property>`` in
    ``Document.xml``) and repackages the zip with every other entry byte-for-byte
    intact. The kernel honours the change on reopen + (forced) recompute -- so a
    file-level authoring edit drives the very geometry the GUI would produce.

    Only simple scalar properties (a child carrying a ``value=`` attribute --
    ``App::PropertyLength`` / ``Float`` / ``Integer`` / ``Bool`` / ``String``)
    are editable here; links, placements and container properties are not.
    Writes to ``out`` (default: overwrite ``path``) and returns
    ``{object, property, old, new, out}``. Raises ``ValueError`` for a missing
    file / object / property or a non-scalar property.
    """
    if not isinstance(path, str) or not path.strip():
        raise ValueError("edit_property: path must be a non-empty string")
    if not os.path.exists(path):
        raise ValueError("edit_property: no such file: %s" % path)
    for label, v in (("obj", obj), ("prop", prop)):
        if not isinstance(v, str) or not v.strip():
            raise ValueError("edit_property: %s must be a non-empty string" % label)
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise ValueError(
            "edit_property: %s is not a .FCStd (zip) document" % path)
    with z:
        names = z.namelist()
        if DOCUMENT_XML not in names:
            raise ValueError(
                "edit_property: %s has no %s -- not a FreeCAD document"
                % (path, DOCUMENT_XML))
        entries = {n: z.read(n) for n in names}
    try:
        root = ET.fromstring(entries[DOCUMENT_XML])
    except ET.ParseError as exc:
        raise ValueError("edit_property: corrupt %s (%s)" % (DOCUMENT_XML, exc))
    data_el = root.find("ObjectData")
    objects = ([] if data_el is None
               else [o for o in data_el.findall("Object")])
    target = next((o for o in objects if o.get("name") == obj), None)
    if target is None:
        avail = sorted(o.get("name") for o in objects if o.get("name"))
        raise ValueError("edit_property: no object %r in %s (have: %s)"
                         % (obj, path, ", ".join(avail[:20]) or "none"))
    prop_el = None
    prop_names: List[str] = []
    for props_el in target.findall("Properties"):
        for pr in props_el.findall("Property"):
            prop_names.append(pr.get("name"))
            if pr.get("name") == prop:
                prop_el = pr
    if prop_el is None:
        raise ValueError("edit_property: object %r has no property %r (have: %s)"
                         % (obj, prop, ", ".join(sorted(n for n in prop_names
                                                        if n)[:20]) or "none"))
    child = next((c for c in prop_el if "value" in c.attrib), None)
    if child is None:
        raise ValueError(
            "edit_property: property %r of %r is type %r -- not a simple scalar "
            "(links / placements / container properties can't be edited here)"
            % (prop, obj, prop_el.get("type")))
    old = child.get("value")
    child.set("value", _format_value(value, old))
    entries[DOCUMENT_XML] = (b"<?xml version='1.0' encoding='utf-8'?>\n"
                             + ET.tostring(root, encoding="utf-8"))
    out = out or path
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zo:
        for n in names:
            zo.writestr(n, entries[n])
    return {"object": obj, "property": prop, "old": old,
            "new": child.get("value"), "out": out}
