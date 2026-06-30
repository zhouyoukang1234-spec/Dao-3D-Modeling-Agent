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

import copy
import hashlib
import math
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
    if tag == "PropertyVector":
        return {"vector": {a: _maybe_float(v) for a, v in k.attrib.items()}}
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


# FreeCAD's ``Sketcher::ConstraintType`` enum (Mod/Sketcher/App/Constraint.h),
# stable across the 1.0.x line. The integer is what ``<Constrain Type=.../>``
# persists; the name is what the GUI's constraint toolbar speaks.
_CONSTRAINT_TYPES = {
    0: "None", 1: "Coincident", 2: "Horizontal", 3: "Vertical", 4: "Parallel",
    5: "Tangent", 6: "Distance", 7: "DistanceX", 8: "DistanceY", 9: "Angle",
    10: "Perpendicular", 11: "Radius", 12: "Equal", 13: "PointOnObject",
    14: "Symmetric", 15: "InternalAlignment", 16: "SnellsLaw", 17: "Block",
    18: "Diameter", 19: "Weight",
}
# The subset that carries a driving dimension (a length / angle / radius the
# user dials): a value change here re-shapes geometry, unlike a geometric
# constraint (coincident / horizontal) which only removes a degree of freedom.
_DIMENSIONAL_TYPES = {6, 7, 8, 9, 11, 16, 18, 19}


def _sketch_constraints(data_el: Optional[ET.Element]
                        ) -> "Dict[str, Dict[str, Any]]":
    """sketch name -> its solver constraint list, read kernel-free.

    A Sketcher document's *real* parametric content is its constraint graph: the
    ``Sketcher::PropertyConstraintList`` records each ``<Constrain>`` with a
    ``Type`` (coincident / horizontal / distance / ...), an optional ``Name``,
    its driving ``Value`` and whether it is ``IsDriving`` / ``IsActive``. The
    GUI authors these one toolbar click at a time; surfaced structurally here
    they become a first-class view of how a sketch is pinned down -- the second
    parametric graph (alongside ``ExpressionEngine``) the file actually holds.

    Per sketch: ``constraints`` (each ``{type, type_name, name, value, driving,
    active}``), ``count``, and ``dimensions`` -- the *named driving dimensional*
    constraints as ``{name: value}`` (the user-facing knobs, e.g.
    ``{"width": 40.0, "height": 25.0}``).
    """
    out: Dict[str, Dict[str, Any]] = {}
    if data_el is None:
        return out
    for obj in data_el.findall("Object"):
        name = obj.get("name")
        if name is None:
            continue
        clist = None
        for props_el in obj.findall("Properties"):
            for prop in props_el.findall("Property"):
                if prop.get("type") == "Sketcher::PropertyConstraintList":
                    clist = prop.find("ConstraintList")
                    break
            if clist is not None:
                break
        if clist is None:
            continue
        cons: List[Dict[str, Any]] = []
        dims: Dict[str, float] = {}
        for c in clist.findall("Constrain"):
            ctype = _maybe_float(c.get("Type"))
            ctype = int(ctype) if isinstance(ctype, (int, float)) else None
            cname = c.get("Name") or ""
            value = _maybe_float(c.get("Value"))
            driving = c.get("IsDriving") != "0"
            cons.append({
                "type": ctype,
                "type_name": _CONSTRAINT_TYPES.get(ctype, "Type%s" % ctype),
                "name": cname,
                "value": value,
                "driving": driving,
                "active": c.get("IsActive") != "0",
            })
            if (cname and driving and ctype in _DIMENSIONAL_TYPES
                    and isinstance(value, (int, float))
                    and not isinstance(value, bool)):
                dims[cname] = value
        out[name] = {"constraints": cons, "count": len(cons), "dimensions": dims}
    return out


def _sketch_geometry(data_el: Optional[ET.Element]
                     ) -> "Dict[str, List[Dict[str, Any]]]":
    """sketch name -> its edge geometry list, read kernel-free.

    The ``Part::PropertyGeometryList`` holds the sketch's actual edges. Only the
    ``Part::GeomLineSegment`` form is surfaced here (the shape the authoring
    layer writes): each becomes ``{"line": True, "start": [x, y], "end": [x, y],
    "construction": bool}``. Other geometry kinds are reported by their type so
    the sketch's edge count is still visible without inventing coordinates.
    """
    out: Dict[str, List[Dict[str, Any]]] = {}
    if data_el is None:
        return out
    for obj in data_el.findall("Object"):
        name = obj.get("name")
        if name is None:
            continue
        glist = None
        for props_el in obj.findall("Properties"):
            for prop in props_el.findall("Property"):
                if prop.get("type") == "Part::PropertyGeometryList":
                    glist = prop.find("GeometryList")
                    break
            if glist is not None:
                break
        if glist is None:
            continue
        geoms: List[Dict[str, Any]] = []
        for g in glist.findall("Geometry"):
            gtype = g.get("type")
            entry: Dict[str, Any] = {"type": gtype}
            seg = g.find("LineSegment")
            circ = g.find("Circle")
            arc = g.find("ArcOfCircle")
            ell = g.find("Ellipse")
            aoe = g.find("ArcOfEllipse")
            if gtype == "Part::GeomLineSegment" and seg is not None:
                entry["line"] = True
                entry["start"] = [float(seg.get("StartX", 0)),
                                  float(seg.get("StartY", 0))]
                entry["end"] = [float(seg.get("EndX", 0)),
                                float(seg.get("EndY", 0))]
            elif gtype == "Part::GeomCircle" and circ is not None:
                entry["circle"] = True
                entry["center"] = [float(circ.get("CenterX", 0)),
                                   float(circ.get("CenterY", 0))]
                entry["radius"] = float(circ.get("Radius", 0))
            elif gtype == "Part::GeomArcOfCircle" and arc is not None:
                entry["arc"] = True
                entry["center"] = [float(arc.get("CenterX", 0)),
                                   float(arc.get("CenterY", 0))]
                entry["radius"] = float(arc.get("Radius", 0))
                entry["start_angle"] = float(arc.get("StartAngle", 0))
                entry["end_angle"] = float(arc.get("EndAngle", 0))
            elif gtype == "Part::GeomEllipse" and ell is not None:
                entry["ellipse"] = True
                entry["center"] = [float(ell.get("CenterX", 0)),
                                   float(ell.get("CenterY", 0))]
                entry["major_radius"] = float(ell.get("MajorRadius", 0))
                entry["minor_radius"] = float(ell.get("MinorRadius", 0))
                entry["angle"] = float(ell.get("AngleXU", 0))
            elif gtype == "Part::GeomArcOfEllipse" and aoe is not None:
                entry["arc_ellipse"] = True
                entry["center"] = [float(aoe.get("CenterX", 0)),
                                   float(aoe.get("CenterY", 0))]
                entry["major_radius"] = float(aoe.get("MajorRadius", 0))
                entry["minor_radius"] = float(aoe.get("MinorRadius", 0))
                entry["angle"] = float(aoe.get("AngleXU", 0))
                entry["start_angle"] = float(aoe.get("StartAngle", 0))
                entry["end_angle"] = float(aoe.get("EndAngle", 0))
            cons = g.find("Construction")
            entry["construction"] = (cons is not None
                                      and cons.get("value") == "1")
            geoms.append(entry)
        out[name] = geoms
    return out


def _sketch_dimensions(sketches: "Dict[str, Dict[str, Any]]") -> Dict[str, Any]:
    """Flatten every sketch's named driving dimensions to ``sketch.name ->
    value`` -- the document's user-facing dimensional knobs in one map."""
    out: Dict[str, Any] = {}
    for sk, info in sketches.items():
        for nm, val in info["dimensions"].items():
            out["%s.%s" % (sk, nm)] = val
    return out


def _sheet_cells(data_el: Optional[ET.Element]) -> "Dict[str, Dict[str, Any]]":
    """spreadsheet name -> its aliased cell table, read kernel-free.

    A ``Spreadsheet::Sheet`` stores its content in a
    ``Spreadsheet::PropertySheet`` ``cells`` property: each ``<Cell>`` records an
    ``address`` (e.g. ``A1``), its ``content`` (a literal or an ``=`` formula),
    and an optional ``alias``. The aliased cells are the parametric *control
    table* other objects bind their dimensions to; surfaced here they become the
    file-level read dual of authoring one -- the master model's knobs, recovered
    from the file without the kernel.

    Per sheet: ``cells`` (each ``{address, content, alias}``), ``count``, and
    ``aliases`` -- the alias -> content map (the user-facing knobs).
    """
    out: Dict[str, Dict[str, Any]] = {}
    if data_el is None:
        return out
    for obj in data_el.findall("Object"):
        name = obj.get("name")
        if name is None:
            continue
        cells_el = None
        for props_el in obj.findall("Properties"):
            for prop in props_el.findall("Property"):
                if prop.get("type") == "Spreadsheet::PropertySheet":
                    cells_el = prop.find("Cells")
                    break
            if cells_el is not None:
                break
        if cells_el is None:
            continue
        cells: List[Dict[str, Any]] = []
        aliases: Dict[str, str] = {}
        for c in cells_el.findall("Cell"):
            address = c.get("address") or ""
            content = c.get("content") or ""
            alias = c.get("alias") or ""
            cells.append({"address": address, "content": content,
                          "alias": alias})
            if alias:
                aliases[alias] = content
        out[name] = {"cells": cells, "count": len(cells), "aliases": aliases}
    return out


# OpenCASCADE BREP ASCII shape records open with a two-letter type code on its
# own line, inside the ``TShapes`` section. This is the topology root the API
# only ever wraps -- counted straight from the file it needs no kernel.
_BREP_SHAPE_CODES = {
    "Ve": "vertices", "Ed": "edges", "Wi": "wires", "Fa": "faces",
    "Sh": "shells", "So": "solids", "CS": "compsolids", "Co": "compounds",
}
_BREP_CODE_RE = re.compile(r"^(Ve|Ed|Wi|Fa|Sh|So|CS|Co)\s*$", re.M)
# The geometry tables a shape references (one count each, in the file header).
_BREP_SECTIONS = ("Locations", "Curve2ds", "Curves", "Polygon3D",
                  "PolygonOnTriangulations", "Surfaces", "Triangulations")


def _brep_summary(raw: bytes) -> "Dict[str, Any]":
    """Parse an OpenCASCADE ``.brp`` into a kernel-free topology + geometry
    summary: the shape-type census (how many vertices / edges / wires / faces /
    shells / solids the boundary representation holds) and the counts of the
    geometry tables it references (surfaces, curves, locations, ...).

    The ``.brp`` is the geometric root every FreeCAD shape ultimately persists
    to; the scripting API only ever wraps it. Counting it here lets the system
    reason about *what geometry a document actually contains* -- and the census
    matches the kernel's own ``Shape.Solids`` / ``Faces`` / ``Edges`` /
    ``Vertexes`` exactly, so the file layer and the kernel agree on geometry too.
    """
    text = raw.decode("latin-1", "replace")
    version = None
    m = re.search(r"CASCADE Topology (V\d+)", text)
    if m:
        version = m.group(1)
    sections: Dict[str, int] = {}
    for sec in _BREP_SECTIONS:
        sm = re.search(r"^%s\s+(\d+)" % sec, text, re.M)
        if sm:
            sections[sec.lower()] = int(sm.group(1))
    topo = {name: 0 for name in _BREP_SHAPE_CODES.values()}
    ti = text.find("TShapes")
    if ti != -1:
        for code in _BREP_CODE_RE.findall(text[ti:]):
            topo[_BREP_SHAPE_CODES[code]] += 1
    return {"version": version, "topology": topo, "sections": sections}


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
        sketches = _sketch_constraints(data_el)
        sketch_dims = _sketch_dimensions(sketches)
        sketch_geometry = _sketch_geometry(data_el)
        sheets = _sheet_cells(data_el)
        breps = []
        topo_totals = {name: 0 for name in _BREP_SHAPE_CODES.values()}
        for n in names:
            if not n.lower().endswith(_BREP_EXT):
                continue
            data = z.read(n)
            summary = _brep_summary(data)
            breps.append({"file": n, "bytes": len(data),
                          "sha1": hashlib.sha1(data).hexdigest()[:16],
                          "version": summary["version"],
                          "topology": summary["topology"],
                          "sections": summary["sections"]})
            for k, v in summary["topology"].items():
                topo_totals[k] += v
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
            "sketches": sketches,
            "sketch_constraint_count": sum(s["count"] for s in sketches.values()),
            "sketch_dimensions": sketch_dims,
            "sketch_geometry": sketch_geometry,
            "spreadsheets": sheets,
            "spreadsheet_cell_count": sum(s["count"] for s in sheets.values()),
            "brep_files": breps,
            "brep_bytes": sum(b["bytes"] for b in breps),
            "topology_totals": topo_totals,
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


def _content_to_value(content: str) -> Any:
    """Map a spreadsheet cell's stored ``content`` back to a synthesize value: a
    literal number becomes ``int``/``float``, anything else (a formula, text)
    stays a string -- the inverse of how ``_cells_element`` wrote it."""
    try:
        f = float(content)
    except (TypeError, ValueError):
        return content
    return int(f) if f.is_integer() else f


def summarize(path: str) -> "List[Dict[str, Any]]":
    """Decompile an authored ``.FCStd`` back into a ``synthesize`` spec list.

    The inverse of :func:`synthesize`: reading the file (no kernel), reconstruct
    the per-object spec that would author it again -- primitives with their
    scalar ``properties``/``placement``/``expressions``, booleans with their
    ``base``/``tool``, N-ary booleans with their ``shapes``, and spreadsheets
    with their ``cells``. Feeding the result back to ``synthesize`` reproduces a
    structurally identical document (same fingerprint), so the author->read loop
    closes on *every* type the authoring layer can write -- 反者道之动, the model
    read straight back out of the file it was written into.

    Raises ``ValueError`` if the document holds a type ``synthesize`` cannot
    author, since the round-trip would otherwise silently drop it.
    """
    info = inspect_document(path)
    props_all = info["properties"]
    exprs_all = info["expressions"]
    sheets = info["spreadsheets"]
    geom_all = info["sketch_geometry"]
    specs: List[Dict[str, Any]] = []
    for obj in info["objects"]:
        name, otype = obj["name"], obj["type"]
        if name is None or otype is None:
            continue
        props = props_all.get(name, {})
        spec: Dict[str, Any] = {"type": otype, "name": name}
        if otype in _PRIMITIVES:
            defined = _PRIMITIVES[otype]
            scalars = {p: props[p]["value"] for p in defined
                       if p in props and isinstance(props[p]["value"], (int, float))
                       and not isinstance(props[p]["value"], bool)}
            spec["properties"] = scalars
            placement = _placement_spec(props.get("Placement"))
            if placement:
                spec["placement"] = placement
        elif otype in _BOOLEANS:
            spec["base"] = _link_target(props.get("Base"))
            spec["tool"] = _link_target(props.get("Tool"))
        elif otype in _LINKLIST_TYPES:
            key, prop_name = _LINKLIST_TYPES[otype]
            ll_val = props.get(prop_name, {}).get("value")
            spec[key] = (list(ll_val["link_list"])
                         if isinstance(ll_val, dict)
                         and "link_list" in ll_val else [])
        elif otype == _MIRROR_TYPE:
            spec["source"] = _link_target(props.get("Source"))
            base_vec = _vector_spec(props.get("Base"))
            if base_vec and any(base_vec):
                spec["base"] = base_vec
            normal_vec = _vector_spec(props.get("Normal"))
            if normal_vec and normal_vec != _MIRROR_DEFAULT_NORMAL:
                spec["normal"] = normal_vec
        elif otype == _SHEET_TYPE:
            spec["cells"] = {alias: _content_to_value(content)
                             for alias, content
                             in sheets.get(name, {}).get("aliases", {}).items()}
        elif otype == _SKETCH_TYPE:
            segs: List[Dict[str, Any]] = []
            for g in geom_all.get(name, []):
                if g.get("line"):
                    seg: Dict[str, Any] = {"start": list(g["start"]),
                                           "end": list(g["end"])}
                elif g.get("circle"):
                    seg = {"center": list(g["center"]),
                           "radius": g["radius"]}
                elif g.get("arc"):
                    seg = {"center": list(g["center"]), "radius": g["radius"],
                           "start_angle": g["start_angle"],
                           "end_angle": g["end_angle"]}
                elif g.get("ellipse"):
                    seg = {"center": list(g["center"]),
                           "major_radius": g["major_radius"],
                           "minor_radius": g["minor_radius"]}
                    if g.get("angle"):
                        seg["angle"] = g["angle"]
                elif g.get("arc_ellipse"):
                    seg = {"center": list(g["center"]),
                           "major_radius": g["major_radius"],
                           "minor_radius": g["minor_radius"],
                           "start_angle": g["start_angle"],
                           "end_angle": g["end_angle"]}
                    if g.get("angle"):
                        seg["angle"] = g["angle"]
                else:
                    continue
                if g.get("construction"):
                    seg["construction"] = True
                segs.append(seg)
            spec["geometry"] = segs
        elif otype == _EXTRUDE_TYPE:
            spec["base"] = _link_target(props.get("Base"))
            length = props.get("LengthFwd", {}).get("value")
            if isinstance(length, (int, float)) and not isinstance(length, bool):
                spec["length"] = length
            dir_vec = _vector_spec(props.get("Dir"))
            if dir_vec and dir_vec != _EXTRUDE_DEFAULT_DIR:
                spec["dir"] = dir_vec
            solid = props.get("Solid", {}).get("value")
            if solid is False:
                spec["solid"] = False
        elif otype == _REVOLVE_TYPE:
            spec["source"] = _link_target(props.get("Source"))
            axis_vec = _vector_spec(props.get("Axis"))
            if axis_vec and axis_vec != _REVOLVE_DEFAULT_AXIS:
                spec["axis"] = axis_vec
            base_vec = _vector_spec(props.get("Base"))
            if base_vec and base_vec != _REVOLVE_DEFAULT_BASE:
                spec["base"] = base_vec
            angle = props.get("Angle", {}).get("value")
            if (isinstance(angle, (int, float)) and not isinstance(angle, bool)
                    and angle != _REVOLVE_DEFAULT_ANGLE):
                spec["angle"] = angle
            solid = props.get("Solid", {}).get("value")
            if solid is False:
                spec["solid"] = False
        else:
            raise ValueError(
                "summarize: object %s has type %r that synthesize cannot author"
                % (name, otype))
        bound = exprs_all.get(name)
        if bound:
            spec["expressions"] = {e["path"]: e["formula"] for e in bound}
        specs.append(spec)
    return specs


def _link_target(prop: "Optional[Dict[str, Any]]") -> "Optional[str]":
    """The link target name of an ``App::PropertyLink`` property value."""
    if not isinstance(prop, dict):
        return None
    val = prop.get("value")
    return val["link"] if isinstance(val, dict) and "link" in val else None


def _vector_spec(prop: "Optional[Dict[str, Any]]") -> "Optional[List[float]]":
    """The ``[x, y, z]`` of a persisted ``PropertyVector``-backed property
    (``App::PropertyPosition`` / ``App::PropertyDirection``), or ``None``."""
    if not isinstance(prop, dict):
        return None
    val = prop.get("value")
    if not isinstance(val, dict) or "vector" not in val:
        return None
    v = val["vector"]
    return [float(v.get("valueX", 0)), float(v.get("valueY", 0)),
            float(v.get("valueZ", 0))]


def _placement_spec(prop: "Optional[Dict[str, Any]]") -> "Optional[Dict[str, Any]]":
    """Reconstruct a synthesize ``placement`` from a persisted ``Placement``
    property: position from ``Px/Py/Pz`` and, when the stored axis-angle ``A`` is
    non-zero, the rotation axis ``Ox/Oy/Oz`` and angle in degrees. Returns
    ``None`` for an identity placement (nothing to author)."""
    if not isinstance(prop, dict):
        return None
    val = prop.get("value")
    if not isinstance(val, dict) or "placement" not in val:
        return None
    p = val["placement"]
    px, py, pz = (float(p.get("Px", 0)), float(p.get("Py", 0)),
                  float(p.get("Pz", 0)))
    angle_rad = float(p.get("A", 0))
    out: Dict[str, Any] = {}
    if px or py or pz:
        out["position"] = [px, py, pz]
    if abs(angle_rad) > 1e-12:
        out["axis"] = [float(p.get("Ox", 0)), float(p.get("Oy", 0)),
                       float(p.get("Oz", 1))]
        out["angle"] = math.degrees(angle_rad)
    return out or None


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

    # sketch dimensions: a named driving constraint (width / radius / angle)
    # re-dialled is a parametric edit invisible at the property level (the
    # constraint list collapses to one opaque blob). Surface it per sketch.name.
    a_dim, b_dim = a["sketch_dimensions"], b["sketch_dimensions"]
    dim_changes: Dict[str, Dict[str, Any]] = {}
    for key in sorted(set(a_dim) | set(b_dim)):
        va, vb = a_dim.get(key), b_dim.get(key)
        if va != vb:
            dim_changes[key] = {"from": va, "to": vb}

    identical = not (added or removed or retyped or edges_added
                     or edges_removed or prop_changes or brep_changes
                     or expr_changes or dim_changes)
    return {
        "identical": identical,
        "objects_added": added,
        "objects_removed": removed,
        "types_changed": retyped,
        "edges_added": edges_added,
        "edges_removed": edges_removed,
        "property_changes": prop_changes,
        "expression_changes": expr_changes,
        "dimension_changes": dim_changes,
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


def set_expression(path: str, obj: str, prop_path: str,
                   formula: Optional[str], out: Optional[str] = None
                   ) -> Dict[str, Any]:
    """Author / re-point / remove an ExpressionEngine binding by file surgery --
    the *act* half for parametric wiring (the dual of ``inspect_document``'s read).

    ``edit_property`` rewrites a scalar *value*; this rewrites the *binding* that
    drives it. ``formula`` is the expression to bind on ``prop_path`` (the
    ``path`` an ``<Expression>`` carries, e.g. ``"Length"`` or
    ``"Constraints.width"``); pass ``None`` to remove the binding. The kernel
    honours the change on reopen + (forced) recompute, exactly as it does for
    ``edit_property`` -- so a file-level edit rewires the parametric graph the
    GUI would otherwise wire by hand.

    FreeCAD serialises an ``App::PropertyExpressionEngine`` on every object (an
    empty ``<ExpressionEngine count="0"/>`` when unbound), so authoring, re-
    pointing and removing all work from the file alone -- no kernel needed to
    create the binding. Writes to ``out`` (default: overwrite ``path``) and
    returns ``{object, path, old, new, out}``. Raises ``ValueError`` for a
    missing file / object, an object that (unusually) lacks the property, or a
    remove of an absent binding.
    """
    if not isinstance(path, str) or not path.strip():
        raise ValueError("set_expression: path must be a non-empty string")
    if not os.path.exists(path):
        raise ValueError("set_expression: no such file: %s" % path)
    for label, v in (("obj", obj), ("prop_path", prop_path)):
        if not isinstance(v, str) or not v.strip():
            raise ValueError(
                "set_expression: %s must be a non-empty string" % label)
    if formula is not None and (not isinstance(formula, str) or not formula.strip()):
        raise ValueError(
            "set_expression: formula must be a non-empty string or None "
            "(None removes the binding)")
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise ValueError(
            "set_expression: %s is not a .FCStd (zip) document" % path)
    with z:
        names = z.namelist()
        if DOCUMENT_XML not in names:
            raise ValueError(
                "set_expression: %s has no %s -- not a FreeCAD document"
                % (path, DOCUMENT_XML))
        entries = {n: z.read(n) for n in names}
    try:
        root = ET.fromstring(entries[DOCUMENT_XML])
    except ET.ParseError as exc:
        raise ValueError("set_expression: corrupt %s (%s)" % (DOCUMENT_XML, exc))
    data_el = root.find("ObjectData")
    objects = [] if data_el is None else list(data_el.findall("Object"))
    target = next((o for o in objects if o.get("name") == obj), None)
    if target is None:
        avail = sorted(o.get("name") for o in objects if o.get("name"))
        raise ValueError("set_expression: no object %r in %s (have: %s)"
                         % (obj, path, ", ".join(avail[:20]) or "none"))
    engine = None
    for props_el in target.findall("Properties"):
        for pr in props_el.findall("Property"):
            if pr.get("type") == "App::PropertyExpressionEngine":
                engine = pr.find("ExpressionEngine")
                break
        if engine is not None:
            break
    if engine is None:
        raise ValueError(
            "set_expression: object %r carries no ExpressionEngine -- bind it "
            "once via the kernel first (ss.bind / obj.setExpression); file "
            "surgery can re-point or remove an existing binding, not create the "
            "property from nothing" % obj)
    existing = next((e for e in engine.findall("Expression")
                     if e.get("path") == prop_path), None)
    old = existing.get("expression") if existing is not None else None
    if formula is None:
        if existing is None:
            raise ValueError(
                "set_expression: object %r has no binding on %r to remove "
                "(bound paths: %s)"
                % (obj, prop_path, ", ".join(e.get("path")
                                             for e in engine.findall("Expression"))
                   or "none"))
        engine.remove(existing)
    elif existing is not None:
        existing.set("expression", formula)
    else:
        ET.SubElement(engine, "Expression",
                      {"path": prop_path, "expression": formula})
    engine.set("count", str(len(engine.findall("Expression"))))
    entries[DOCUMENT_XML] = (b"<?xml version='1.0' encoding='utf-8'?>\n"
                             + ET.tostring(root, encoding="utf-8"))
    out = out or path
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zo:
        for n in names:
            zo.writestr(n, entries[n])
    return {"object": obj, "path": prop_path, "old": old, "new": formula,
            "out": out}


def set_dimension(path: str, sketch: str, name: str, value: float,
                  out: Optional[str] = None) -> Dict[str, Any]:
    """Re-dial a named driving dimension of a sketch by file surgery -- the
    *act* half for the constraint graph (the dual of ``inspect_document``'s
    ``sketch_dimensions`` read, and the sibling of ``set_expression`` for the
    other parametric layer).

    A driving dimensional constraint (``DistanceX`` / ``Radius`` / ``Angle`` ...)
    is the user-facing knob a sketch exposes; ``inspect_document`` surfaces the
    named ones as ``sketch_dimensions``. This rewrites such a constraint's
    ``Value`` straight in ``Document.xml`` -- the kernel re-solves the sketch to
    the new dimension on reopen + (forced) recompute, reshaping every feature
    built on it, with no kernel used to author the edit.

    ``value`` is the new dimension (mm / degrees, as the constraint's type
    dictates). Targets the constraint whose ``Name`` matches and that is driving;
    a reference (non-driving) or geometric constraint is refused. Writes to
    ``out`` (default: overwrite ``path``) and returns
    ``{sketch, name, old, new, out}``. Raises ``ValueError`` for a missing file
    / sketch / named driving dimension, or a non-numeric value.
    """
    if not isinstance(path, str) or not path.strip():
        raise ValueError("set_dimension: path must be a non-empty string")
    if not os.path.exists(path):
        raise ValueError("set_dimension: no such file: %s" % path)
    for label, v in (("sketch", sketch), ("name", name)):
        if not isinstance(v, str) or not v.strip():
            raise ValueError("set_dimension: %s must be a non-empty string" % label)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("set_dimension: value must be a number, got %r" % (value,))
    try:
        z = zipfile.ZipFile(path)
    except zipfile.BadZipFile:
        raise ValueError(
            "set_dimension: %s is not a .FCStd (zip) document" % path)
    with z:
        names = z.namelist()
        if DOCUMENT_XML not in names:
            raise ValueError(
                "set_dimension: %s has no %s -- not a FreeCAD document"
                % (path, DOCUMENT_XML))
        entries = {n: z.read(n) for n in names}
    try:
        root = ET.fromstring(entries[DOCUMENT_XML])
    except ET.ParseError as exc:
        raise ValueError("set_dimension: corrupt %s (%s)" % (DOCUMENT_XML, exc))
    data_el = root.find("ObjectData")
    objects = [] if data_el is None else list(data_el.findall("Object"))
    target = next((o for o in objects if o.get("name") == sketch), None)
    if target is None:
        avail = sorted(o.get("name") for o in objects if o.get("name"))
        raise ValueError("set_dimension: no object %r in %s (have: %s)"
                         % (sketch, path, ", ".join(avail[:20]) or "none"))
    clist = None
    for props_el in target.findall("Properties"):
        for prop in props_el.findall("Property"):
            if prop.get("type") == "Sketcher::PropertyConstraintList":
                clist = prop.find("ConstraintList")
                break
        if clist is not None:
            break
    if clist is None:
        raise ValueError(
            "set_dimension: object %r is not a sketch with constraints" % sketch)
    con = next((c for c in clist.findall("Constrain")
                if c.get("Name") == name and c.get("IsDriving") != "0"), None)
    if con is None:
        dims = sorted(c.get("Name") for c in clist.findall("Constrain")
                      if c.get("Name") and c.get("IsDriving") != "0")
        raise ValueError(
            "set_dimension: sketch %r has no named driving dimension %r "
            "(have: %s)" % (sketch, name, ", ".join(dims) or "none"))
    old = _maybe_float(con.get("Value"))
    con.set("Value", "%.16f" % float(value))
    entries[DOCUMENT_XML] = (b"<?xml version='1.0' encoding='utf-8'?>\n"
                             + ET.tostring(root, encoding="utf-8"))
    out = out or path
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zo:
        for n in names:
            zo.writestr(n, entries[n])
    return {"sketch": sketch, "name": name, "old": old,
            "new": _maybe_float(con.get("Value")), "out": out}


# The Part primitives whose ``execute()`` regenerates a Shape purely from scalar
# properties -- so a document holding only these can be authored from nothing
# (no BREP) and the kernel builds the geometry on its first forced recompute.
# Each maps a property name to its FreeCAD type; defaults are FreeCAD's own.
_PRIMITIVES: "Dict[str, Dict[str, str]]" = {
    "Part::Box": {"Length": "App::PropertyLength", "Width": "App::PropertyLength",
                  "Height": "App::PropertyLength"},
    "Part::Cylinder": {"Radius": "App::PropertyLength",
                       "Height": "App::PropertyLength",
                       "Angle": "App::PropertyAngle"},
    "Part::Sphere": {"Radius": "App::PropertyLength",
                     "Angle1": "App::PropertyAngle",
                     "Angle2": "App::PropertyAngle",
                     "Angle3": "App::PropertyAngle"},
    "Part::Cone": {"Radius1": "App::PropertyLength",
                   "Radius2": "App::PropertyLength",
                   "Height": "App::PropertyLength", "Angle": "App::PropertyAngle"},
    "Part::Torus": {"Radius1": "App::PropertyLength",
                    "Radius2": "App::PropertyLength",
                    "Angle1": "App::PropertyAngle", "Angle2": "App::PropertyAngle",
                    "Angle3": "App::PropertyAngle"},
    "Part::Ellipsoid": {"Radius1": "App::PropertyLength",
                        "Radius2": "App::PropertyLength",
                        "Radius3": "App::PropertyLength",
                        "Angle1": "App::PropertyAngle",
                        "Angle2": "App::PropertyAngle",
                        "Angle3": "App::PropertyAngle"},
    "Part::Wedge": {"Xmin": "App::PropertyDistance", "Xmax": "App::PropertyDistance",
                    "Ymin": "App::PropertyDistance", "Ymax": "App::PropertyDistance",
                    "Zmin": "App::PropertyDistance", "Zmax": "App::PropertyDistance",
                    "X2min": "App::PropertyDistance", "X2max": "App::PropertyDistance",
                    "Z2min": "App::PropertyDistance", "Z2max": "App::PropertyDistance"},
    "Part::Prism": {"Circumradius": "App::PropertyLength",
                    "Height": "App::PropertyLength",
                    "Polygon": "App::PropertyIntegerConstraint",
                    "FirstAngle": "App::PropertyAngle",
                    "SecondAngle": "App::PropertyAngle"},
}
# Scalar FreeCAD property types serialise as a single ``<Float>`` child.
_FLOAT_PROP_TYPES = {"App::PropertyLength", "App::PropertyAngle",
                     "App::PropertyFloat", "App::PropertyDistance"}
# Integer-backed property types serialise as a single ``<Integer>`` child.
_INT_PROP_TYPES = {"App::PropertyInteger", "App::PropertyIntegerConstraint"}

# The Part boolean operators -- each is a ``Part::Boolean`` taking a ``Base`` and
# a ``Tool`` link to two other objects, and the kernel performs the CSG on
# recompute. Authoring these from file builds a constructive-solid-geometry tree
# (an object-link DAG), the dual of the primitive leaves above.
_BOOLEANS = {"Part::Cut", "Part::Fuse", "Part::Common"}

# N-ary boolean operators -- each takes a ``Shapes`` link-list of *two or more*
# operands and folds the CSG across all of them in one recompute. A human drives
# these as repeated pairwise GUI operations; authored from file the whole
# multi-operand fold is written at once -- the file layer's leverage over the
# step-by-step tool flow.
_MULTI_BOOLEANS = {"Part::MultiFuse", "Part::MultiCommon"}

# A ``Part::Compound`` *groups* two or more shapes into one object without any
# CSG: the shapes coexist (no union, no carve), the kernel just bundles them.
# Like the N-ary booleans it carries an ``App::PropertyLinkList`` of operands --
# but under the property name ``Links`` rather than ``Shapes``.
_COMPOUND_TYPE = "Part::Compound"

# Types whose operands are an ``App::PropertyLinkList``: type -> (spec key,
# property name). The N-ary booleans fold CSG across the list; the compound
# merely groups it. Treating them uniformly means one code path authors,
# resolves, and decompiles every multi-operand object.
_LINKLIST_TYPES: "Dict[str, tuple]" = {
    "Part::MultiFuse": ("shapes", "Shapes"),
    "Part::MultiCommon": ("shapes", "Shapes"),
    "Part::Compound": ("links", "Links"),
}

# A spreadsheet is a parametric *control table*: named (aliased) cells holding
# numbers or formulae, to which other objects bind their dimensions. Authoring
# one from nothing yields the master-model surface humans drive a design from.
_SHEET_TYPE = "Spreadsheet::Sheet"

# A ``Part::Mirroring`` reflects its ``Source`` shape across a plane defined by a
# point ``Base`` on it and a ``Normal`` direction. The reflection is rigid
# (volume-preserving); authored from file it carries a ``Source`` link plus the
# two vector properties -- the file building a mirror feature with no kernel.
_MIRROR_TYPE = "Part::Mirroring"
_MIRROR_DEFAULT_NORMAL = [0.0, 0.0, 1.0]

# A ``Sketcher::SketchObject`` is the 2D profile upstream of every pad / pocket /
# extrusion -- the most fundamental authoring surface there is. Its edges live in
# a ``Part::PropertyGeometryList``; the authoring layer writes line segments
# (``Part::GeomLineSegment``) so a closed wire can be drawn straight from file
# and the kernel turns it into a face on recompute. 逆流到最上游.
_SKETCH_TYPE = "Sketcher::SketchObject"

# A ``Part::Extrusion`` sweeps a 2D profile (a sketch / wire ``Base``) along a
# ``Dir`` for ``LengthFwd``, optionally capping it into a ``Solid``. It is the
# join between the sketch layer and the solid layer -- author a sketch + an
# extrusion from file and the kernel turns the 2D loop into a 3D body on
# recompute, the file-first equivalent of the GUI's Pad. The ``DirMode`` =
# Custom (enum 0) and the bullseye ``FaceMakerClass`` are fixed so a closed wire
# becomes a face.
_EXTRUDE_TYPE = "Part::Extrusion"
_EXTRUDE_DEFAULT_DIR = [0.0, 0.0, 1.0]
_FACEMAKER = "Part::FaceMakerBullseye"
_EXTRUDE_FACEMAKER = _FACEMAKER  # backwards-compatible alias

# A ``Part::Revolution`` revolves a 2D profile (``Source``) about an axis
# (``Base`` point + ``Axis`` direction) through ``Angle`` degrees, the lathe to
# the extrusion's mill. Author the sketch + the revolution from file and the
# kernel spins the loop into a solid of revolution on recompute (Pappus): the
# file-first equivalent of the GUI's Revolve, no clicks.
_REVOLVE_TYPE = "Part::Revolution"
_REVOLVE_DEFAULT_AXIS = [0.0, 0.0, 1.0]
_REVOLVE_DEFAULT_BASE = [0.0, 0.0, 0.0]
_REVOLVE_DEFAULT_ANGLE = 360.0


def _cells_element(parent: ET.Element, cells: "Dict[str, Any]") -> None:
    """Append a ``Spreadsheet::PropertySheet`` ``cells`` property.

    ``cells`` maps an *alias* to a value: a number (a literal) or a string (a
    cell formula, which FreeCAD recognises by its leading ``=``). Aliases are
    laid out down column A (``A1``, ``A2``, ...) in insertion order; binding an
    object's dimension to ``Sheet.alias`` then reads the cell -- the parametric
    control table other objects are driven from.
    """
    prop = ET.SubElement(parent, "Property",
                         {"name": "cells", "type": "Spreadsheet::PropertySheet",
                          "status": "67108864"})
    cells_el = ET.SubElement(prop, "Cells",
                             {"Count": str(len(cells)), "xlink": "1"})
    ET.SubElement(cells_el, "XLinks", {"count": "0"})
    for row, (alias, value) in enumerate(cells.items(), start=1):
        content = value if isinstance(value, str) else repr(value)
        ET.SubElement(cells_el, "Cell",
                      {"address": "A%d" % row, "content": content,
                       "alias": alias})


def _sketch_segment_kind(seg: "Dict[str, Any]") -> "Optional[str]":
    """Classify a sketch geometry spec into ``line`` / ``circle`` / ``arc``.

    The form is discriminated by which keys are present: a ``line`` has
    ``start``/``end`` endpoints; an ``arc`` adds ``start_angle``/``end_angle`` to
    a ``center``/``radius``; an ``arc_ellipse`` carries ``major_radius`` /
    ``minor_radius`` *and* ``start_angle``/``end_angle``; an ``ellipse`` carries
    ``major_radius`` / ``minor_radius`` only; a ``circle`` is the bare
    ``center``/``radius``.
    Returns ``None`` if no form matches so callers can raise a guided error.
    """
    has_axes = "major_radius" in seg or "minor_radius" in seg
    has_sweep = "start_angle" in seg or "end_angle" in seg
    if "start" in seg or "end" in seg:
        return "line"
    if has_axes and has_sweep:
        return "arc_ellipse"
    if has_sweep:
        return "arc"
    if has_axes:
        return "ellipse"
    if "center" in seg or "radius" in seg:
        return "circle"
    return None


def _geometry_element(parent: ET.Element, segments: "List[Dict[str, Any]]") -> None:
    """Append a ``Part::PropertyGeometryList`` -- a sketch's edges authored from
    file.

    Each entry is one of three forms (all carrying an optional ``construction``
    flag), written as the matching FreeCAD geometry with the
    ``Sketcher::SketchGeometryExtension`` attached to every sketch edge, so the
    kernel rebuilds the wire on recompute and a closed loop becomes a face the
    upstream pad/extrusion/revolution consumes:

    * line    -- ``{"start": [x, y], "end": [x, y]}`` -> ``Part::GeomLineSegment``
    * circle  -- ``{"center": [x, y], "radius": r}`` -> ``Part::GeomCircle``
    * arc     -- ``{"center": [x, y], "radius": r, "start_angle": a,
      "end_angle": b}`` (radians) -> ``Part::GeomArcOfCircle``
    * ellipse -- ``{"center": [x, y], "major_radius": M, "minor_radius": m,
      "angle": t}`` (``angle`` is the major-axis tilt from +X in radians,
      default 0) -> ``Part::GeomEllipse``
    * arc_ellipse -- an ``ellipse`` spec plus ``start_angle``/``end_angle``
      (radians, swept on the ellipse's own parameter) -> ``Part::GeomArcOfEllipse``
    """
    prop = ET.SubElement(parent, "Property",
                         {"name": "Geometry",
                          "type": "Part::PropertyGeometryList",
                          "status": "8192"})
    glist = ET.SubElement(prop, "GeometryList", {"count": str(len(segments))})
    for i, seg in enumerate(segments, start=1):
        kind = _sketch_segment_kind(seg)
        gtype = {"line": "Part::GeomLineSegment",
                 "circle": "Part::GeomCircle",
                 "arc": "Part::GeomArcOfCircle",
                 "ellipse": "Part::GeomEllipse",
                 "arc_ellipse": "Part::GeomArcOfEllipse"}[kind]
        g = ET.SubElement(glist, "Geometry",
                          {"type": gtype, "id": str(i), "migrated": "1"})
        exts = ET.SubElement(g, "GeoExtensions", {"count": "1"})
        ET.SubElement(exts, "GeoExtension",
                      {"type": "Sketcher::SketchGeometryExtension", "id": str(i),
                       "internalGeometryType": "0",
                       "geometryModeFlags": "0" * 32, "geometryLayer": "0"})
        if kind == "line":
            sx, sy = seg["start"]
            ex, ey = seg["end"]
            ET.SubElement(g, "LineSegment",
                          {"StartX": "%.16f" % float(sx),
                           "StartY": "%.16f" % float(sy), "StartZ": "%.16f" % 0.0,
                           "EndX": "%.16f" % float(ex),
                           "EndY": "%.16f" % float(ey), "EndZ": "%.16f" % 0.0})
        elif kind in ("ellipse", "arc_ellipse"):
            cx, cy = seg["center"]
            attrs = {"CenterX": "%.16f" % float(cx),
                     "CenterY": "%.16f" % float(cy), "CenterZ": "%.16f" % 0.0,
                     "NormalX": "%.16f" % 0.0, "NormalY": "%.16f" % 0.0,
                     "NormalZ": "%.16f" % 1.0,
                     "MajorRadius": "%.16f" % float(seg["major_radius"]),
                     "MinorRadius": "%.16f" % float(seg["minor_radius"]),
                     "AngleXU": "%.16f" % float(seg.get("angle", 0.0))}
            if kind == "arc_ellipse":
                attrs["StartAngle"] = "%.16f" % float(seg["start_angle"])
                attrs["EndAngle"] = "%.16f" % float(seg["end_angle"])
                ET.SubElement(g, "ArcOfEllipse", attrs)
            else:
                ET.SubElement(g, "Ellipse", attrs)
        else:
            cx, cy = seg["center"]
            attrs = {"CenterX": "%.16f" % float(cx),
                     "CenterY": "%.16f" % float(cy), "CenterZ": "%.16f" % 0.0,
                     "NormalX": "%.16f" % 0.0, "NormalY": "%.16f" % 0.0,
                     "NormalZ": "%.16f" % 1.0, "AngleXU": "%.16f" % 0.0,
                     "Radius": "%.16f" % float(seg["radius"])}
            if kind == "arc":
                attrs["StartAngle"] = "%.16f" % float(seg["start_angle"])
                attrs["EndAngle"] = "%.16f" % float(seg["end_angle"])
                ET.SubElement(g, "ArcOfCircle", attrs)
            else:
                ET.SubElement(g, "Circle", attrs)
        ET.SubElement(g, "Construction",
                      {"value": "1" if seg.get("construction") else "0"})


def _extrusion_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Extrusion`` properties that turn a 2D ``base`` profile
    into a swept (optionally solid) body.

    Six properties carry the feature: ``Base`` (link to the profile), ``Dir`` +
    ``DirMode`` = Custom (enum 0) for an explicit sweep direction, ``LengthFwd``
    for the distance, ``Solid`` to cap the ends, and a bullseye ``FaceMakerClass``
    so a closed wire becomes a face the sweep can fill. The kernel does the
    sweep on recompute; the file just declares it.
    """
    bp = ET.SubElement(parent, "Property",
                       {"name": "Base", "type": "App::PropertyLink"})
    ET.SubElement(bp, "Link", {"value": spec["base"]})
    dvec = spec.get("dir") or _EXTRUDE_DEFAULT_DIR
    dp = ET.SubElement(parent, "Property",
                       {"name": "Dir", "type": "App::PropertyVector"})
    ET.SubElement(dp, "PropertyVector",
                  {"valueX": "%.16f" % float(dvec[0]),
                   "valueY": "%.16f" % float(dvec[1]),
                   "valueZ": "%.16f" % float(dvec[2])})
    dm = ET.SubElement(parent, "Property",
                       {"name": "DirMode", "type": "App::PropertyEnumeration"})
    ET.SubElement(dm, "Integer", {"value": "0"})
    lp = ET.SubElement(parent, "Property",
                       {"name": "LengthFwd", "type": "App::PropertyDistance"})
    ET.SubElement(lp, "Float", {"value": "%.16f" % float(spec["length"])})
    solid = spec.get("solid", True)
    sp = ET.SubElement(parent, "Property",
                       {"name": "Solid", "type": "App::PropertyBool"})
    ET.SubElement(sp, "Bool", {"value": "true" if solid else "false"})
    fp = ET.SubElement(parent, "Property",
                       {"name": "FaceMakerClass", "type": "App::PropertyString"})
    ET.SubElement(fp, "String", {"value": _EXTRUDE_FACEMAKER})


def _revolution_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Revolution`` properties that spin a 2D ``source``
    profile about an axis into a solid of revolution.

    Six properties carry the feature: ``Source`` (link to the profile),
    ``Axis`` + ``Base`` for the revolution axis (a direction through a point),
    ``Angle`` (degrees) for the sweep, ``Solid`` to cap it, and a bullseye
    ``FaceMakerClass`` so a closed wire becomes a fillable face. The kernel spins
    it on recompute; the file just declares it.
    """
    sp = ET.SubElement(parent, "Property",
                       {"name": "Source", "type": "App::PropertyLink"})
    ET.SubElement(sp, "Link", {"value": spec["source"]})
    axis = spec.get("axis") or _REVOLVE_DEFAULT_AXIS
    ap = ET.SubElement(parent, "Property",
                       {"name": "Axis", "type": "App::PropertyVector"})
    ET.SubElement(ap, "PropertyVector",
                  {"valueX": "%.16f" % float(axis[0]),
                   "valueY": "%.16f" % float(axis[1]),
                   "valueZ": "%.16f" % float(axis[2])})
    base = spec.get("base") or _REVOLVE_DEFAULT_BASE
    bp = ET.SubElement(parent, "Property",
                       {"name": "Base", "type": "App::PropertyVector"})
    ET.SubElement(bp, "PropertyVector",
                  {"valueX": "%.16f" % float(base[0]),
                   "valueY": "%.16f" % float(base[1]),
                   "valueZ": "%.16f" % float(base[2])})
    angle = spec.get("angle", _REVOLVE_DEFAULT_ANGLE)
    gp = ET.SubElement(parent, "Property",
                       {"name": "Angle", "type": "App::PropertyFloatConstraint"})
    ET.SubElement(gp, "Float", {"value": "%.16f" % float(angle)})
    solid = spec.get("solid", True)
    op = ET.SubElement(parent, "Property",
                       {"name": "Solid", "type": "App::PropertyBool"})
    ET.SubElement(op, "Bool", {"value": "true" if solid else "false"})
    fp = ET.SubElement(parent, "Property",
                       {"name": "FaceMakerClass", "type": "App::PropertyString"})
    ET.SubElement(fp, "String", {"value": _FACEMAKER})


def _placement_element(parent: ET.Element, position: "List[float]",
                       axis: "Optional[List[float]]" = None,
                       angle: float = 0.0) -> None:
    """Append a ``Placement`` property: a translation plus an optional rotation
    of ``angle`` degrees about ``axis``.

    FreeCAD persists a placement's rotation **twice** -- as a quaternion
    (``Q0..Q3``) and as axis-angle (``A`` in radians, ``Ox/Oy/Oz``) -- and the
    loader honours the axis-angle pair, so both must be authored consistently or
    the rotation is silently dropped (``A=0`` ignores any quaternion written).
    """
    px, py, pz = (float(c) for c in position)
    ax, ay, az = (float(c) for c in (axis or (0.0, 0.0, 1.0)))
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm <= 1e-12:
        raise ValueError("placement: rotation axis must be non-zero")
    ax, ay, az = ax / norm, ay / norm, az / norm
    theta = math.radians(float(angle))
    half = theta / 2.0
    s = math.sin(half)
    prop = ET.SubElement(parent, "Property",
                         {"name": "Placement", "type": "App::PropertyPlacement"})
    ET.SubElement(prop, "PropertyPlacement", {
        "Px": "%.16f" % px, "Py": "%.16f" % py, "Pz": "%.16f" % pz,
        "Q0": "%.16f" % (ax * s), "Q1": "%.16f" % (ay * s),
        "Q2": "%.16f" % (az * s), "Q3": "%.16f" % math.cos(half),
        "A": "%.16f" % theta, "Ox": "%.16f" % ax,
        "Oy": "%.16f" % ay, "Oz": "%.16f" % az})


def synthesize(path: str, objects: "List[Dict[str, Any]]",
               schema_version: str = "4",
               program_version: str = "1.0.2") -> Dict[str, Any]:
    """Author a complete ``.FCStd`` from a spec, with **no kernel** -- the most
    upstream act of all: a model written as a file, the way code is written.

    ``inspect_document`` / ``set_expression`` / ``set_dimension`` read and edit
    an existing document; this *creates* one from nothing. Each entry in
    ``objects`` is ``{"type": <Part primitive>, "name": str, "properties":
    {prop: value}, "placement": {"position": [x,y,z]?, "axis": [x,y,z]?,
    "angle": degrees?}?, "expressions": {path: formula}?}`` -- ``placement``
    authors a translation and an optional rotation of ``angle`` degrees about
    ``axis``. ``expressions`` author an ``ExpressionEngine`` per object --
    so a *parametric* model can be written from nothing, e.g. one primitive's
    dimension bound to ``"Other.Radius * 2"``; the cross-object references are
    resolved into dependency edges in ``ObjectDeps``. A ``Spreadsheet::Sheet``
    entry instead carries ``"cells": {alias: value}`` -- a parametric control
    table other objects bind their dimensions to via ``Sheet.alias``.

    A spec may instead be a boolean ``{"type": <Part boolean>, "name": str,
    "base": <name>, "tool": <name>, "expressions": ...?}`` (``Part::Cut`` /
    ``Part::Fuse`` / ``Part::Common``), whose ``base``/``tool`` link two other
    objects -- so a whole constructive-solid-geometry tree can be authored from
    nothing, the kernel performing the CSG on recompute. An N-ary boolean
    ``{"type": <Part multi-boolean>, "name": str, "shapes": [<name>, ...]}``
    (``Part::MultiFuse`` / ``Part::MultiCommon``) folds the CSG across *two or
    more* operands in one recompute -- the file authors in a single step what
    the GUI builds as repeated pairwise operations. A ``Part::Compound``
    ``{"type": "Part::Compound", "name": str, "links": [<name>, ...]}`` instead
    *groups* its operands into one object with no CSG -- the shapes coexist.

    Only the Part primitives in ``_PRIMITIVES`` and the booleans in
    ``_BOOLEANS`` are accepted -- their ``execute()`` rebuilds the Shape from
    these scalars/links, so the file needs no BREP: the kernel generates the
    geometry on its first ``recompute(force=True)`` (after a ``touch()``, since
    a freshly loaded object reports up-to-date).

    Writes a single ``Document.xml`` (no geometry files) and returns
    ``{out, objects: [names], object_count}``. Raises ``ValueError`` for an
    empty spec, an unknown primitive type, a duplicate / missing name, or a
    property the primitive does not define.
    """
    if not isinstance(path, str) or not path.strip():
        raise ValueError("synthesize: path must be a non-empty string")
    if not isinstance(objects, list) or not objects:
        raise ValueError("synthesize: objects must be a non-empty list")

    # First pass: validate every spec and learn all names, so cross-object
    # expression references can be resolved into dependency edges.
    seen: set = set()
    for idx, spec in enumerate(objects, start=1):
        if not isinstance(spec, dict):
            raise ValueError("synthesize: object #%d must be a dict" % idx)
        otype = spec.get("type")
        if (otype not in _PRIMITIVES and otype not in _BOOLEANS
                and otype not in _LINKLIST_TYPES and otype != _SHEET_TYPE
                and otype != _MIRROR_TYPE and otype != _SKETCH_TYPE
                and otype != _EXTRUDE_TYPE and otype != _REVOLVE_TYPE):
            raise ValueError(
                "synthesize: object #%d has unknown type %r (supported: %s)"
                % (idx, otype, ", ".join(sorted(
                    set(_PRIMITIVES) | _BOOLEANS | set(_LINKLIST_TYPES)
                    | {_SHEET_TYPE, _MIRROR_TYPE, _SKETCH_TYPE,
                       _EXTRUDE_TYPE, _REVOLVE_TYPE}))))
        name = spec.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("synthesize: object #%d needs a non-empty name" % idx)
        if name in seen:
            raise ValueError("synthesize: duplicate object name %r" % name)
        seen.add(name)
        if otype == _SHEET_TYPE:
            cells = spec.get("cells")
            if not isinstance(cells, dict) or not cells:
                raise ValueError(
                    "synthesize: spreadsheet %s needs a non-empty 'cells' "
                    "{alias: value} map" % name)
            for alias, value in cells.items():
                if not isinstance(alias, str) or not alias.strip():
                    raise ValueError(
                        "synthesize: spreadsheet %s cell alias %r must be a "
                        "non-empty string" % (name, alias))
                if isinstance(value, bool) or not isinstance(
                        value, (int, float, str)):
                    raise ValueError(
                        "synthesize: spreadsheet %s cell %r must be a number or "
                        "formula string (got %r)" % (name, alias, value))
        elif otype in _LINKLIST_TYPES:
            key, _prop = _LINKLIST_TYPES[otype]
            operands = spec.get(key)
            if (not isinstance(operands, list) or len(operands) < 2
                    or not all(isinstance(r, str) and r.strip()
                               for r in operands)):
                raise ValueError(
                    "synthesize: %s (%s) needs '%s': a list of >=2 object "
                    "names" % (name, otype, key))
            if name in operands:
                raise ValueError(
                    "synthesize: %s cannot reference itself" % name)
            if len(set(operands)) != len(operands):
                raise ValueError(
                    "synthesize: %s has duplicate operands in '%s'"
                    % (name, key))
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: %s takes %s, not properties" % (name, key))
        elif otype in _BOOLEANS:
            for role in ("base", "tool"):
                ref = spec.get(role)
                if not isinstance(ref, str) or not ref.strip():
                    raise ValueError(
                        "synthesize: boolean %s (%s) needs a %r object name"
                        % (name, otype, role))
                if ref == name:
                    raise ValueError(
                        "synthesize: boolean %s cannot reference itself" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: boolean %s takes base/tool, not properties" % name)
        elif otype == _MIRROR_TYPE:
            src = spec.get("source")
            if not isinstance(src, str) or not src.strip():
                raise ValueError(
                    "synthesize: mirror %s needs a 'source' object name" % name)
            if src == name:
                raise ValueError(
                    "synthesize: mirror %s cannot reference itself" % name)
            for vkey in ("base", "normal"):
                v = spec.get(vkey)
                if v is not None and (
                        not isinstance(v, (list, tuple)) or len(v) != 3
                        or not all(isinstance(c, (int, float))
                                   and not isinstance(c, bool) for c in v)):
                    raise ValueError(
                        "synthesize: mirror %s '%s' must be [x, y, z] numbers"
                        % (name, vkey))
            if spec.get("normal") is not None and not any(spec["normal"]):
                raise ValueError(
                    "synthesize: mirror %s 'normal' must be non-zero" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: mirror %s takes source/base/normal, not "
                    "properties" % name)
        elif otype == _SKETCH_TYPE:
            segs = spec.get("geometry")
            if not isinstance(segs, list) or not segs:
                raise ValueError(
                    "synthesize: sketch %s needs a non-empty 'geometry' list of "
                    "line/circle/arc entries" % name)

            def _pt2(val: "Any", j: int, key: str) -> None:
                if (not isinstance(val, (list, tuple)) or len(val) != 2
                        or not all(isinstance(c, (int, float))
                                   and not isinstance(c, bool) for c in val)):
                    raise ValueError(
                        "synthesize: sketch %s segment #%d '%s' must be "
                        "[x, y] numbers" % (name, j, key))

            def _num(val: "Any", j: int, key: str) -> None:
                if isinstance(val, bool) or not isinstance(val, (int, float)):
                    raise ValueError(
                        "synthesize: sketch %s segment #%d '%s' must be a "
                        "number" % (name, j, key))

            for j, seg in enumerate(segs):
                if not isinstance(seg, dict):
                    raise ValueError(
                        "synthesize: sketch %s segment #%d must be a dict"
                        % (name, j))
                kind = _sketch_segment_kind(seg)
                if kind is None:
                    raise ValueError(
                        "synthesize: sketch %s segment #%d must be a line "
                        "(start/end), circle (center/radius), arc "
                        "(center/radius/start_angle/end_angle) or ellipse "
                        "(center/major_radius/minor_radius)" % (name, j))
                if kind == "line":
                    _pt2(seg.get("start"), j, "start")
                    _pt2(seg.get("end"), j, "end")
                    if list(seg["start"]) == list(seg["end"]):
                        raise ValueError(
                            "synthesize: sketch %s segment #%d is degenerate "
                            "(start == end)" % (name, j))
                elif kind in ("ellipse", "arc_ellipse"):
                    _pt2(seg.get("center"), j, "center")
                    _num(seg.get("major_radius"), j, "major_radius")
                    _num(seg.get("minor_radius"), j, "minor_radius")
                    if seg["minor_radius"] <= 0:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d 'minor_radius' "
                            "must be positive" % (name, j))
                    if seg["major_radius"] < seg["minor_radius"]:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d 'major_radius' "
                            "must be >= 'minor_radius'" % (name, j))
                    if "angle" in seg:
                        _num(seg.get("angle"), j, "angle")
                    if kind == "arc_ellipse":
                        _num(seg.get("start_angle"), j, "start_angle")
                        _num(seg.get("end_angle"), j, "end_angle")
                else:
                    _pt2(seg.get("center"), j, "center")
                    _num(seg.get("radius"), j, "radius")
                    if seg["radius"] <= 0:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d 'radius' must be "
                            "positive" % (name, j))
                    if kind == "arc":
                        _num(seg.get("start_angle"), j, "start_angle")
                        _num(seg.get("end_angle"), j, "end_angle")
                        if seg["start_angle"] == seg["end_angle"]:
                            raise ValueError(
                                "synthesize: sketch %s segment #%d arc is "
                                "degenerate (start_angle == end_angle)"
                                % (name, j))
                if ("construction" in seg
                        and not isinstance(seg["construction"], bool)):
                    raise ValueError(
                        "synthesize: sketch %s segment #%d 'construction' must "
                        "be a bool" % (name, j))
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: sketch %s takes 'geometry', not properties"
                    % name)
        elif otype == _EXTRUDE_TYPE:
            base = spec.get("base")
            if not isinstance(base, str) or not base.strip():
                raise ValueError(
                    "synthesize: extrusion %s needs a 'base' object name" % name)
            if base == name:
                raise ValueError(
                    "synthesize: extrusion %s cannot reference itself" % name)
            length = spec.get("length")
            if (isinstance(length, bool) or not isinstance(length, (int, float))
                    or length <= 0):
                raise ValueError(
                    "synthesize: extrusion %s needs a positive 'length'" % name)
            d = spec.get("dir")
            if d is not None and (
                    not isinstance(d, (list, tuple)) or len(d) != 3
                    or not all(isinstance(c, (int, float))
                               and not isinstance(c, bool) for c in d)):
                raise ValueError(
                    "synthesize: extrusion %s 'dir' must be [x, y, z] numbers"
                    % name)
            if d is not None and not any(d):
                raise ValueError(
                    "synthesize: extrusion %s 'dir' must be non-zero" % name)
            if "solid" in spec and not isinstance(spec["solid"], bool):
                raise ValueError(
                    "synthesize: extrusion %s 'solid' must be a bool" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: extrusion %s takes base/length/dir/solid, not "
                    "properties" % name)
        elif otype == _REVOLVE_TYPE:
            src = spec.get("source")
            if not isinstance(src, str) or not src.strip():
                raise ValueError(
                    "synthesize: revolution %s needs a 'source' object name"
                    % name)
            if src == name:
                raise ValueError(
                    "synthesize: revolution %s cannot reference itself" % name)
            for vkey in ("axis", "base"):
                v = spec.get(vkey)
                if v is not None and (
                        not isinstance(v, (list, tuple)) or len(v) != 3
                        or not all(isinstance(c, (int, float))
                                   and not isinstance(c, bool) for c in v)):
                    raise ValueError(
                        "synthesize: revolution %s '%s' must be [x, y, z] numbers"
                        % (name, vkey))
            if spec.get("axis") is not None and not any(spec["axis"]):
                raise ValueError(
                    "synthesize: revolution %s 'axis' must be non-zero" % name)
            angle = spec.get("angle", _REVOLVE_DEFAULT_ANGLE)
            if (isinstance(angle, bool) or not isinstance(angle, (int, float))
                    or not 0 < angle <= 360):
                raise ValueError(
                    "synthesize: revolution %s 'angle' must be in (0, 360]"
                    % name)
            if "solid" in spec and not isinstance(spec["solid"], bool):
                raise ValueError(
                    "synthesize: revolution %s 'solid' must be a bool" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: revolution %s takes source/axis/base/angle/"
                    "solid, not properties" % name)
        else:
            props = spec.get("properties") or {}
            if not isinstance(props, dict):
                raise ValueError("synthesize: %r properties must be a dict" % name)
            unknown = sorted(set(props) - set(_PRIMITIVES[otype]))
            if unknown:
                raise ValueError(
                    "synthesize: %s (%s) has no propert%s %s (defines: %s)"
                    % (name, otype, "y" if len(unknown) == 1 else "ies",
                       ", ".join(unknown), ", ".join(sorted(_PRIMITIVES[otype]))))
        exprs = spec.get("expressions") or {}
        if not isinstance(exprs, dict) or not all(
                isinstance(k, str) and isinstance(v, str) for k, v in exprs.items()):
            raise ValueError(
                "synthesize: %r expressions must be a {path: formula} of strings"
                % name)

    all_names = [s["name"] for s in objects]
    # boolean operands must resolve to real objects in the same document.
    for spec in objects:
        if spec["type"] in _BOOLEANS:
            for role in ("base", "tool"):
                if spec[role] not in all_names:
                    raise ValueError(
                        "synthesize: boolean %s %s=%r is not a defined object"
                        % (spec["name"], role, spec[role]))
        elif spec["type"] == _MIRROR_TYPE:
            if spec["source"] not in all_names:
                raise ValueError(
                    "synthesize: mirror %s source=%r is not a defined object"
                    % (spec["name"], spec["source"]))
        elif spec["type"] == _EXTRUDE_TYPE:
            if spec["base"] not in all_names:
                raise ValueError(
                    "synthesize: extrusion %s base=%r is not a defined object"
                    % (spec["name"], spec["base"]))
        elif spec["type"] == _REVOLVE_TYPE:
            if spec["source"] not in all_names:
                raise ValueError(
                    "synthesize: revolution %s source=%r is not a defined object"
                    % (spec["name"], spec["source"]))
        elif spec["type"] in _LINKLIST_TYPES:
            key, _prop = _LINKLIST_TYPES[spec["type"]]
            for ref in spec[key]:
                if ref not in all_names:
                    raise ValueError(
                        "synthesize: %s operand %r is not a defined object"
                        % (spec["name"], ref))

    root = ET.Element("Document", {"SchemaVersion": schema_version,
                                   "ProgramVersion": program_version,
                                   "FileVersion": "1"})
    ET.SubElement(root, "Properties", {"Count": "0"})
    objs_el = ET.SubElement(root, "Objects",
                            {"Count": str(len(objects)),
                             "Dependencies": str(len(objects))})
    data_el = ET.SubElement(root, "ObjectData", {"Count": str(len(objects))})

    for idx, spec in enumerate(objects, start=1):
        name, otype = spec["name"], spec["type"]
        is_bool = otype in _BOOLEANS
        is_linklist = otype in _LINKLIST_TYPES
        ll_key = _LINKLIST_TYPES[otype][0] if is_linklist else None
        is_sheet = otype == _SHEET_TYPE
        is_mirror = otype == _MIRROR_TYPE
        is_sketch = otype == _SKETCH_TYPE
        is_extrude = otype == _EXTRUDE_TYPE
        is_revolve = otype == _REVOLVE_TYPE
        props = ({} if (is_bool or is_linklist or is_sheet or is_mirror
                        or is_sketch or is_extrude or is_revolve)
                 else (spec.get("properties") or {}))
        exprs = spec.get("expressions") or {}
        # links: an explicit DAG (boolean operands) plus every *other* object
        # referenced in a formula -- together the object's dependency edges.
        links = ([spec["base"], spec["tool"]] if is_bool
                 else list(spec[ll_key]) if is_linklist
                 else [spec["source"]] if is_mirror
                 else [spec["base"]] if is_extrude
                 else [spec["source"]] if is_revolve else [])
        dep_set = list(links)
        for other in all_names:
            if other != name and other not in dep_set and any(
                    re.search(r"\b%s\b" % re.escape(other), f)
                    for f in exprs.values()):
                dep_set.append(other)
        dep_el = ET.SubElement(objs_el, "ObjectDeps",
                               {"Name": name, "Count": str(len(dep_set))})
        for dep in dep_set:
            ET.SubElement(dep_el, "Dep", {"Name": dep})
        ET.SubElement(objs_el, "Object",
                      {"type": otype, "name": name, "id": str(idx)})

        od = ET.SubElement(data_el, "Object", {"name": name})
        placement = spec.get("placement") or {}
        if not isinstance(placement, dict):
            raise ValueError("synthesize: %s placement must be a dict" % name)
        position = placement.get("position")
        axis = placement.get("axis")
        angle = placement.get("angle", 0.0)
        has_placement = bool(position or axis or angle)
        prop_items = ([] if (is_bool or is_linklist or is_sheet or is_mirror
                             or is_sketch or is_extrude or is_revolve)
                      else [(p, _PRIMITIVES[otype][p], v) for p, v in props.items()])
        prop_count = (len(prop_items) + (1 if has_placement else 0)
                      + (1 if exprs else 0) + (2 if is_bool else 0)
                      + (1 if is_linklist else 0) + (1 if is_sheet else 0)
                      + (3 if is_mirror else 0) + (1 if is_sketch else 0)
                      + (6 if is_extrude else 0) + (6 if is_revolve else 0))
        props_el = ET.SubElement(
            od, "Properties", {"Count": str(prop_count), "TransientCount": "0"})
        if is_sheet:
            _cells_element(props_el, spec["cells"])
        if is_sketch:
            _geometry_element(props_el, spec["geometry"])
        if is_extrude:
            _extrusion_properties(props_el, spec)
        if is_revolve:
            _revolution_properties(props_el, spec)
        if is_bool:
            for role_name, ref in (("Base", spec["base"]), ("Tool", spec["tool"])):
                lp = ET.SubElement(props_el, "Property",
                                   {"name": role_name, "type": "App::PropertyLink"})
                ET.SubElement(lp, "Link", {"value": ref})
        if is_linklist:
            operands = spec[ll_key]
            lp = ET.SubElement(props_el, "Property",
                               {"name": _LINKLIST_TYPES[otype][1],
                                "type": "App::PropertyLinkList"})
            ll = ET.SubElement(lp, "LinkList", {"count": str(len(operands))})
            for ref in operands:
                ET.SubElement(ll, "Link", {"value": ref})
        if is_mirror:
            sp = ET.SubElement(props_el, "Property",
                               {"name": "Source", "type": "App::PropertyLink"})
            ET.SubElement(sp, "Link", {"value": spec["source"]})
            for pname, ptype, vec in (
                    ("Base", "App::PropertyPosition",
                     spec.get("base") or [0.0, 0.0, 0.0]),
                    ("Normal", "App::PropertyDirection",
                     spec.get("normal") or _MIRROR_DEFAULT_NORMAL)):
                vp = ET.SubElement(props_el, "Property",
                                   {"name": pname, "type": ptype})
                ET.SubElement(vp, "PropertyVector",
                              {"valueX": "%.16f" % float(vec[0]),
                               "valueY": "%.16f" % float(vec[1]),
                               "valueZ": "%.16f" % float(vec[2])})
        if exprs:
            ee = ET.SubElement(props_el, "Property",
                               {"name": "ExpressionEngine",
                                "type": "App::PropertyExpressionEngine",
                                "status": "67108864"})
            elist = ET.SubElement(ee, "ExpressionEngine",
                                  {"count": str(len(exprs))})
            for epath, formula in exprs.items():
                ET.SubElement(elist, "Expression",
                              {"path": epath, "expression": formula})
        for pname, ptype, value in prop_items:
            pe = ET.SubElement(props_el, "Property",
                               {"name": pname, "type": ptype})
            if ptype in _FLOAT_PROP_TYPES:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(
                        "synthesize: %s.%s must be a number, got %r"
                        % (name, pname, value))
                ET.SubElement(pe, "Float", {"value": "%.16f" % float(value)})
            elif ptype in _INT_PROP_TYPES:
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(
                        "synthesize: %s.%s must be an integer, got %r"
                        % (name, pname, value))
                ET.SubElement(pe, "Integer", {"value": str(value)})
            else:
                ET.SubElement(pe, "String", {"value": str(value)})
        if has_placement:
            position = position or [0.0, 0.0, 0.0]
            if len(position) != 3:
                raise ValueError(
                    "synthesize: %s placement position must be [x,y,z]" % name)
            if axis is not None and len(axis) != 3:
                raise ValueError(
                    "synthesize: %s placement axis must be [x,y,z]" % name)
            if not isinstance(angle, (int, float)) or isinstance(angle, bool):
                raise ValueError(
                    "synthesize: %s placement angle must be a number (degrees)"
                    % name)
            _placement_element(props_el, position, axis, angle)

    payload = (b"<?xml version='1.0' encoding='utf-8'?>\n"
               + ET.tostring(root, encoding="utf-8"))
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zo:
        zo.writestr(DOCUMENT_XML, payload)
    return {"out": path, "objects": [s["name"] for s in objects],
            "object_count": len(objects)}


def linear_pattern(
    base: "Dict[str, Any]",
    count: int,
    offset: "List[float]",
    group: "Optional[str]" = None,
) -> "List[Dict[str, Any]]":
    """Expand a base primitive spec into ``count`` translated copies, returning
    a ``synthesize`` spec list (feed it straight to :func:`synthesize`).

    Copy ``i`` (0-based) is the base translated by ``i * offset`` from its own
    position, named ``"<base name>_<i>"``. With ``group`` set to a link-list
    type (``Part::Compound`` to merely bundle them, or ``Part::MultiFuse`` /
    ``Part::MultiCommon`` to fold their CSG) a final object over all copies is
    appended, named ``"<base name>_all"``.

    The file layer's leverage made concrete: a human stamps out an array by
    repeating a GUI place-copy step ``count`` times; here the whole pattern is
    computed and written from one parametric description -- authoring at a scale
    and precision the manual flow cannot match. 道法自然.
    """
    if not isinstance(base, dict) or base.get("type") not in _PRIMITIVES:
        raise ValueError(
            "linear_pattern: 'base' must be a primitive spec (type in %s)"
            % ", ".join(sorted(_PRIMITIVES)))
    bname = base.get("name")
    if not isinstance(bname, str) or not bname.strip():
        raise ValueError("linear_pattern: base needs a non-empty name")
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError("linear_pattern: 'count' must be an int >= 1")
    if (not isinstance(offset, (list, tuple)) or len(offset) != 3
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in offset)):
        raise ValueError("linear_pattern: 'offset' must be [dx, dy, dz] numbers")
    if group is not None and group not in _LINKLIST_TYPES:
        raise ValueError(
            "linear_pattern: 'group' must be one of %s (or None)"
            % ", ".join(sorted(_LINKLIST_TYPES)))
    base_pos = ((base.get("placement") or {}).get("position")) or [0.0, 0.0, 0.0]
    if len(base_pos) != 3:
        raise ValueError("linear_pattern: base placement position must be [x,y,z]")
    specs: List[Dict[str, Any]] = []
    names: List[str] = []
    for i in range(count):
        copy_spec = copy.deepcopy(base)
        cname = "%s_%d" % (bname, i)
        copy_spec["name"] = cname
        placement = dict(copy_spec.get("placement") or {})
        placement["position"] = [base_pos[j] + i * offset[j] for j in range(3)]
        copy_spec["placement"] = placement
        specs.append(copy_spec)
        names.append(cname)
    if group is not None:
        key = _LINKLIST_TYPES[group][0]
        specs.append({"type": group, "name": "%s_all" % bname, key: names})
    return specs


def regular_polygon(
    name: str,
    sides: int,
    radius: float,
    center: "Optional[List[float]]" = None,
    start_angle: float = 0.0,
    construction: bool = False,
) -> "Dict[str, Any]":
    """Generate a closed regular ``sides``-gon as a sketch spec (feed it to
    :func:`synthesize` directly, or use it as the ``base`` of an extrusion /
    revolution).

    The ``sides`` vertices lie on a circle of ``radius`` about ``center``
    (default origin) in the XY plane, the first at ``start_angle`` degrees from
    +X and the rest spaced ``360 / sides`` apart; consecutive vertices are
    joined by line segments and the loop is closed. The whole profile is
    computed from one parametric description -- a human draws and constrains
    each edge by hand, here the trig is done once and written exactly. 道法自然.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("regular_polygon: needs a non-empty name")
    if not isinstance(sides, int) or isinstance(sides, bool) or sides < 3:
        raise ValueError("regular_polygon: 'sides' must be an int >= 3")
    if (isinstance(radius, bool) or not isinstance(radius, (int, float))
            or radius <= 0):
        raise ValueError("regular_polygon: 'radius' must be a positive number")
    if center is None:
        center = [0.0, 0.0]
    if (not isinstance(center, (list, tuple)) or len(center) != 2
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in center)):
        raise ValueError("regular_polygon: 'center' must be [x, y] numbers")
    if (isinstance(start_angle, bool)
            or not isinstance(start_angle, (int, float))):
        raise ValueError("regular_polygon: 'start_angle' must be a number")
    if not isinstance(construction, bool):
        raise ValueError("regular_polygon: 'construction' must be a bool")
    cx, cy = float(center[0]), float(center[1])
    verts = []
    for k in range(sides):
        th = math.radians(start_angle + 360.0 * k / sides)
        verts.append([cx + radius * math.cos(th), cy + radius * math.sin(th)])
    geometry: List[Dict[str, Any]] = []
    for k in range(sides):
        seg: Dict[str, Any] = {"start": verts[k],
                               "end": verts[(k + 1) % sides]}
        if construction:
            seg["construction"] = True
        geometry.append(seg)
    return {"type": _SKETCH_TYPE, "name": name, "geometry": geometry}


def slot(
    name: str,
    length: float,
    radius: float,
    center: "Optional[List[float]]" = None,
    construction: bool = False,
) -> "Dict[str, Any]":
    """Generate a closed slot (obround / stadium) as a sketch spec.

    Two semicircle ends of ``radius`` centred ``length`` apart along X (centres
    at ``center`` +/- ``length/2``) are joined by the two tangent straight
    flanks, giving the rounded slot a human builds from two lines + two arcs +
    tangency constraints. Here the four edges and their exact arc angles are
    computed from one ``(length, radius)`` description -- author it directly or
    sweep/extrude it. The enclosed area is ``2*length*radius + pi*radius**2``.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("slot: needs a non-empty name")
    if (isinstance(length, bool) or not isinstance(length, (int, float))
            or length <= 0):
        raise ValueError("slot: 'length' must be a positive number")
    if (isinstance(radius, bool) or not isinstance(radius, (int, float))
            or radius <= 0):
        raise ValueError("slot: 'radius' must be a positive number")
    if center is None:
        center = [0.0, 0.0]
    if (not isinstance(center, (list, tuple)) or len(center) != 2
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in center)):
        raise ValueError("slot: 'center' must be [x, y] numbers")
    if not isinstance(construction, bool):
        raise ValueError("slot: 'construction' must be a bool")
    cx, cy = float(center[0]), float(center[1])
    r = float(radius)
    hl = float(length) / 2.0
    lx, rx = cx - hl, cx + hl
    geometry: List[Dict[str, Any]] = [
        {"start": [lx, cy + r], "end": [rx, cy + r]},
        {"center": [rx, cy], "radius": r,
         "start_angle": -math.pi / 2, "end_angle": math.pi / 2},
        {"start": [rx, cy - r], "end": [lx, cy - r]},
        {"center": [lx, cy], "radius": r,
         "start_angle": math.pi / 2, "end_angle": 3 * math.pi / 2},
    ]
    if construction:
        for seg in geometry:
            seg["construction"] = True
    return {"type": _SKETCH_TYPE, "name": name, "geometry": geometry}


def ellipse(
    name: str,
    major_radius: float,
    minor_radius: float,
    center: "Optional[List[float]]" = None,
    angle: float = 0.0,
    construction: bool = False,
) -> "Dict[str, Any]":
    """Generate a closed ellipse as a single-edge sketch spec.

    The ellipse is centred at ``center`` (default origin) in the XY plane with
    its major axis of ``major_radius`` tilted ``angle`` degrees from +X and its
    minor axis of ``minor_radius`` perpendicular. One ``Part::GeomEllipse`` edge
    closes the loop, so the profile extrudes / revolves straight from file; the
    enclosed area is ``pi * major_radius * minor_radius``. A human places the
    centre and drags two axes under tangency constraints -- here both radii and
    the tilt come from one description, written exactly. 道法自然.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("ellipse: needs a non-empty name")
    if (isinstance(major_radius, bool)
            or not isinstance(major_radius, (int, float)) or major_radius <= 0):
        raise ValueError("ellipse: 'major_radius' must be a positive number")
    if (isinstance(minor_radius, bool)
            or not isinstance(minor_radius, (int, float)) or minor_radius <= 0):
        raise ValueError("ellipse: 'minor_radius' must be a positive number")
    if major_radius < minor_radius:
        raise ValueError("ellipse: 'major_radius' must be >= 'minor_radius'")
    if center is None:
        center = [0.0, 0.0]
    if (not isinstance(center, (list, tuple)) or len(center) != 2
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in center)):
        raise ValueError("ellipse: 'center' must be [x, y] numbers")
    if isinstance(angle, bool) or not isinstance(angle, (int, float)):
        raise ValueError("ellipse: 'angle' must be a number (degrees)")
    if not isinstance(construction, bool):
        raise ValueError("ellipse: 'construction' must be a bool")
    seg: Dict[str, Any] = {"center": [float(center[0]), float(center[1])],
                           "major_radius": float(major_radius),
                           "minor_radius": float(minor_radius)}
    if angle:
        seg["angle"] = math.radians(float(angle))
    if construction:
        seg["construction"] = True
    return {"type": _SKETCH_TYPE, "name": name, "geometry": [seg]}


def _rodrigues(v: "List[float]", axis: "List[float]", angle_deg: float) -> "List[float]":
    """Rotate vector ``v`` about ``axis`` by ``angle_deg`` degrees (right-hand
    rule), via Rodrigues' formula. ``axis`` need not be unit -- it is normalised
    here. The pure-Python rotation the file layer needs to *place* a polar array
    without the kernel."""
    n = math.sqrt(sum(c * c for c in axis))
    if n == 0:
        raise ValueError("_rodrigues: axis must be non-zero")
    kx, ky, kz = axis[0] / n, axis[1] / n, axis[2] / n
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    dot = kx * v[0] + ky * v[1] + kz * v[2]
    cross = [ky * v[2] - kz * v[1], kz * v[0] - kx * v[2], kx * v[1] - ky * v[0]]
    k = [kx, ky, kz]
    return [v[i] * c + cross[i] * s + k[i] * dot * (1 - c) for i in range(3)]


def polar_pattern(
    base: "Dict[str, Any]",
    count: int,
    axis: "List[float]",
    total_angle: float = 360.0,
    center: "Optional[List[float]]" = None,
    group: "Optional[str]" = None,
) -> "List[Dict[str, Any]]":
    """Expand a base primitive spec into ``count`` copies revolved about an axis,
    returning a ``synthesize`` spec list.

    Copy ``i`` is the base rotated by ``i * step`` degrees about ``axis`` through
    ``center`` (default origin), where ``step`` spans ``total_angle`` over the
    copies -- ``total_angle / count`` for a full 360 (so the ring closes without
    overlap) or ``total_angle / (count - 1)`` otherwise (endpoints inclusive).
    Each copy's placement is computed here (position revolved about the centre
    via :func:`_rodrigues`, plus the same axis-angle rotation), so the kernel
    just rebuilds geometry. ``group`` appends a link-list object over the copies,
    exactly as in :func:`linear_pattern`.

    A radial array a human builds by repeated rotate-copy steps, written from one
    parametric description -- the file layer doing the revolve arithmetic itself.
    """
    if not isinstance(base, dict) or base.get("type") not in _PRIMITIVES:
        raise ValueError(
            "polar_pattern: 'base' must be a primitive spec (type in %s)"
            % ", ".join(sorted(_PRIMITIVES)))
    bname = base.get("name")
    if not isinstance(bname, str) or not bname.strip():
        raise ValueError("polar_pattern: base needs a non-empty name")
    if not isinstance(count, int) or isinstance(count, bool) or count < 2:
        raise ValueError("polar_pattern: 'count' must be an int >= 2")
    if (not isinstance(axis, (list, tuple)) or len(axis) != 3
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in axis)
            or not any(axis)):
        raise ValueError("polar_pattern: 'axis' must be a non-zero [x, y, z]")
    if (not isinstance(total_angle, (int, float))
            or isinstance(total_angle, bool)):
        raise ValueError("polar_pattern: 'total_angle' must be a number (degrees)")
    if center is None:
        center = [0.0, 0.0, 0.0]
    if (not isinstance(center, (list, tuple)) or len(center) != 3
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in center)):
        raise ValueError("polar_pattern: 'center' must be [x, y, z]")
    if group is not None and group not in _LINKLIST_TYPES:
        raise ValueError(
            "polar_pattern: 'group' must be one of %s (or None)"
            % ", ".join(sorted(_LINKLIST_TYPES)))
    base_pos = ((base.get("placement") or {}).get("position")) or [0.0, 0.0, 0.0]
    if len(base_pos) != 3:
        raise ValueError("polar_pattern: base placement position must be [x,y,z]")
    full = abs((total_angle % 360.0)) < 1e-9 and total_angle != 0
    step = total_angle / count if full else total_angle / (count - 1)
    rel = [base_pos[i] - center[i] for i in range(3)]
    axis_l = [float(a) for a in axis]
    specs: List[Dict[str, Any]] = []
    names: List[str] = []
    for i in range(count):
        ang = i * step
        revolved = _rodrigues(rel, axis_l, ang)
        pos = [center[j] + revolved[j] for j in range(3)]
        copy_spec = copy.deepcopy(base)
        cname = "%s_%d" % (bname, i)
        copy_spec["name"] = cname
        copy_spec["placement"] = {"position": pos, "axis": axis_l, "angle": ang}
        specs.append(copy_spec)
        names.append(cname)
    if group is not None:
        key = _LINKLIST_TYPES[group][0]
        specs.append({"type": group, "name": "%s_all" % bname, key: names})
    return specs
