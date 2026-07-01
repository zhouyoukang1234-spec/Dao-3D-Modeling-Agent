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
import struct
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
    if tag == "LinkSub":
        # a link-with-subs: the target object is on the element itself while its
        # children are the sub-elements followed (e.g. ``Edge1``). Keep both so
        # a ``Spine``-style property round-trips its object *and* its edges.
        return {"link": k.get("value"),
                "subs": [c.get("value") for c in k if c.get("value")]}
    if tag in ("LinkList", "LinkSubList"):
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


def _tidy_size(v: float) -> Any:
    """An integral edge-treatment size (``2.0``) read back as ``int`` for a
    clean spec, a fractional one kept as its exact ``float`` -- either way
    ``float()`` of the result re-encodes to the identical double, so a fillet /
    chamfer round-trips byte-for-byte."""
    return int(v) if float(v).is_integer() else v


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
            aop = g.find("ArcOfParabola")
            aoh = g.find("ArcOfHyperbola")
            bsp = g.find("BSplineCurve")
            pt = g.find("GeomPoint")
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
            elif gtype == "Part::GeomArcOfParabola" and aop is not None:
                entry["parabola"] = True
                entry["center"] = [float(aop.get("CenterX", 0)),
                                   float(aop.get("CenterY", 0))]
                entry["focal"] = float(aop.get("Focal", 0))
                entry["angle"] = float(aop.get("AngleXU", 0))
                entry["start_angle"] = float(aop.get("StartAngle", 0))
                entry["end_angle"] = float(aop.get("EndAngle", 0))
            elif gtype == "Part::GeomArcOfHyperbola" and aoh is not None:
                entry["hyperbola"] = True
                entry["center"] = [float(aoh.get("CenterX", 0)),
                                   float(aoh.get("CenterY", 0))]
                entry["major_radius"] = float(aoh.get("MajorRadius", 0))
                entry["minor_radius"] = float(aoh.get("MinorRadius", 0))
                entry["angle"] = float(aoh.get("AngleXU", 0))
                entry["start_angle"] = float(aoh.get("StartAngle", 0))
                entry["end_angle"] = float(aoh.get("EndAngle", 0))
            elif gtype == "Part::GeomBSplineCurve" and bsp is not None:
                poles = [[float(p.get("X", 0)), float(p.get("Y", 0))]
                         for p in bsp.findall("Pole")]
                weights = [float(p.get("Weight", 1)) for p in bsp.findall("Pole")]
                knots = [float(k.get("Value", 0)) for k in bsp.findall("Knot")]
                mults = [int(k.get("Mult", 1)) for k in bsp.findall("Knot")]
                spec: Dict[str, Any] = {"poles": poles, "knots": knots,
                                        "mults": mults,
                                        "degree": int(bsp.get("Degree", 3)),
                                        "periodic": bsp.get("IsPeriodic") == "1"}
                if any(abs(w - 1.0) > 1e-12 for w in weights):
                    spec["weights"] = weights
                entry["bspline"] = spec
            elif gtype == "Part::GeomPoint" and pt is not None:
                entry["point"] = [float(pt.get("X", 0)),
                                  float(pt.get("Y", 0))]
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
        elif otype == _SECTION_TYPE:
            spec["base"] = _link_target(props.get("Base"))
            spec["tool"] = _link_target(props.get("Tool"))
            if props.get("Approximation", {}).get("value") is True:
                spec["approximation"] = True
            if props.get("Refine", {}).get("value") is True:
                spec["refine"] = True
        elif otype == _HELIX_TYPE:
            for key, pname in (("pitch", "Pitch"), ("height", "Height"),
                               ("radius", "Radius"), ("angle", "Angle")):
                v = props.get(pname, {}).get("value")
                if isinstance(v, (int, float)):
                    spec[key] = v
            hand_i = props.get("LocalCoord", {}).get("value")
            if isinstance(hand_i, int) and 0 < hand_i < len(_HELIX_HANDS):
                spec["hand"] = _HELIX_HANDS[hand_i]
            style_i = props.get("Style", {}).get("value")
            if isinstance(style_i, int) and 0 < style_i < len(_HELIX_STYLES):
                spec["style"] = _HELIX_STYLES[style_i]
        elif otype == _SPIRAL_TYPE:
            for key, pname in (("growth", "Growth"), ("rotations", "Rotations"),
                               ("radius", "Radius")):
                v = props.get(pname, {}).get("value")
                if isinstance(v, (int, float)):
                    spec[key] = v
        elif otype == _REFINE_TYPE:
            spec["source"] = _link_target(props.get("Source"))
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
                elif g.get("parabola"):
                    seg = {"center": list(g["center"]), "focal": g["focal"],
                           "start_angle": g["start_angle"],
                           "end_angle": g["end_angle"]}
                    if g.get("angle"):
                        seg["angle"] = g["angle"]
                elif g.get("hyperbola"):
                    seg = {"hyperbola": True, "center": list(g["center"]),
                           "major_radius": g["major_radius"],
                           "minor_radius": g["minor_radius"],
                           "start_angle": g["start_angle"],
                           "end_angle": g["end_angle"]}
                    if g.get("angle"):
                        seg["angle"] = g["angle"]
                elif g.get("bspline"):
                    bs = g["bspline"]
                    inner = {"poles": [list(p) for p in bs["poles"]],
                             "knots": list(bs["knots"]),
                             "mults": list(bs["mults"]),
                             "degree": bs["degree"]}
                    if bs.get("periodic"):
                        inner["periodic"] = True
                    if "weights" in bs:
                        inner["weights"] = list(bs["weights"])
                    seg = {"bspline": inner}
                elif g.get("point") is not None:
                    seg = {"point": list(g["point"])}
                else:
                    continue
                if g.get("construction"):
                    seg["construction"] = True
                segs.append(seg)
            spec["geometry"] = segs
            placement = _placement_spec(props.get("Placement"))
            if placement:
                spec["placement"] = placement
        elif otype == _EXTRUDE_TYPE:
            spec["base"] = _link_target(props.get("Base"))
            length = props.get("LengthFwd", {}).get("value")
            if isinstance(length, (int, float)) and not isinstance(length, bool):
                spec["length"] = length
            dir_vec = _vector_spec(props.get("Dir"))
            if dir_vec and dir_vec != _EXTRUDE_DEFAULT_DIR:
                spec["dir"] = dir_vec
            lrev = props.get("LengthRev", {}).get("value")
            if (isinstance(lrev, (int, float)) and not isinstance(lrev, bool)
                    and lrev):
                spec["length_rev"] = lrev
            taper = props.get("TaperAngle", {}).get("value")
            if (isinstance(taper, (int, float)) and not isinstance(taper, bool)
                    and taper):
                spec["taper"] = taper
            if props.get("Symmetric", {}).get("value") is True:
                spec["symmetric"] = True
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
            if props.get("Symmetric", {}).get("value") is True:
                spec["symmetric"] = True
        elif otype == _LOFT_TYPE:
            ll_val = props.get("Sections", {}).get("value")
            spec["sections"] = (list(ll_val["link_list"])
                                if isinstance(ll_val, dict)
                                and "link_list" in ll_val else [])
            if props.get("Solid", {}).get("value") is False:
                spec["solid"] = False
            if props.get("Ruled", {}).get("value") is True:
                spec["ruled"] = True
            if props.get("Closed", {}).get("value") is True:
                spec["closed"] = True
        elif otype == _SWEEP_TYPE:
            ll_val = props.get("Sections", {}).get("value")
            spec["sections"] = (list(ll_val["link_list"])
                                if isinstance(ll_val, dict)
                                and "link_list" in ll_val else [])
            spine_val = props.get("Spine", {}).get("value")
            if isinstance(spine_val, dict) and spine_val.get("link"):
                spec["spine"] = spine_val["link"]
                subs = spine_val.get("subs") or []
                if subs and list(subs) != ["Edge1"]:
                    spec["spine_edges"] = list(subs)
            if props.get("Solid", {}).get("value") is False:
                spec["solid"] = False
            if props.get("Frenet", {}).get("value") is True:
                spec["frenet"] = True
        elif otype in _EDGE_TREATMENTS:
            spec["base"] = _link_target(props.get("Base"))
            fileref = props.get("Edges", {}).get("value")
            member = fileref.get("file") if isinstance(fileref, dict) else None
            scalar, k1, k2, _noun = _edge_treatment_size_keys(otype)
            edges_spec: List[Dict[str, Any]] = []
            if member:
                with zipfile.ZipFile(path) as _z:
                    triples = _parse_fillet_edges_blob(_z.read(member))
                for eid, s1, s2 in triples:
                    if s1 == s2:
                        edges_spec.append({"edge": int(eid),
                                           scalar: _tidy_size(s1)})
                    else:
                        edges_spec.append({"edge": int(eid),
                                           k1: _tidy_size(s1),
                                           k2: _tidy_size(s2)})
            spec["edges"] = edges_spec
        elif otype == _THICKNESS_TYPE:
            faces_val = props.get("Faces", {}).get("value")
            spec["base"] = (faces_val.get("link")
                            if isinstance(faces_val, dict) else None)
            subs = (faces_val.get("subs") or []
                    if isinstance(faces_val, dict) else [])
            spec["faces"] = [int(s[4:]) for s in subs
                             if isinstance(s, str) and s.startswith("Face")]
            val = props.get("Value", {}).get("value")
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                spec["value"] = _tidy_size(val)
            mode_i = props.get("Mode", {}).get("value")
            if isinstance(mode_i, int) and 0 < mode_i < len(_THICKNESS_MODES):
                spec["mode"] = _THICKNESS_MODES[mode_i]
            join_i = props.get("Join", {}).get("value")
            if isinstance(join_i, int) and 0 < join_i < len(_THICKNESS_JOINS):
                spec["join"] = _THICKNESS_JOINS[join_i]
            if props.get("Intersection", {}).get("value") is True:
                spec["intersection"] = True
            if props.get("SelfIntersection", {}).get("value") is True:
                spec["self_intersection"] = True
        elif otype in _OFFSET_TYPES:
            spec["source"] = _link_target(props.get("Source"))
            val = props.get("Value", {}).get("value")
            if isinstance(val, (int, float)) and not isinstance(val, bool):
                spec["value"] = _tidy_size(val)
            mode_i = props.get("Mode", {}).get("value")
            if isinstance(mode_i, int) and 0 < mode_i < len(_OFFSET_MODES):
                spec["mode"] = _OFFSET_MODES[mode_i]
            join_i = props.get("Join", {}).get("value")
            if isinstance(join_i, int) and 0 < join_i < len(_OFFSET_JOINS):
                spec["join"] = _OFFSET_JOINS[join_i]
            if props.get("Fill", {}).get("value") is True:
                spec["fill"] = True
            if props.get("Intersection", {}).get("value") is True:
                spec["intersection"] = True
            if props.get("SelfIntersection", {}).get("value") is True:
                spec["self_intersection"] = True
        elif otype == _RULED_TYPE:
            for key, pname in (("curve1", "Curve1"), ("curve2", "Curve2")):
                val = props.get(pname, {}).get("value")
                if isinstance(val, dict):
                    spec[key] = val.get("link")
                    subs = val.get("subs") or []
                    if subs:
                        spec[key + "_edges"] = subs
            orient_i = props.get("Orientation", {}).get("value")
            if isinstance(orient_i, int) and 0 < orient_i < len(_RULED_ORIENTS):
                spec["orientation"] = _RULED_ORIENTS[orient_i]
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
    # ``Part::RegularPolygon`` is the parametric N-gon *wire*: a closed loop of
    # ``Polygon`` equal edges inscribed in ``Circumradius``, its execute()
    # rebuilding the wire from the two scalars so (like the circle/line edges)
    # its placement survives a reload. A ready planar section to extrude/loft --
    # the straight-edged sibling of ``Part::Circle``. 圆出于方.
    "Part::RegularPolygon": {"Polygon": "App::PropertyIntegerConstraint",
                             "Circumradius": "App::PropertyLength"},
    # ``Part::Circle`` is the odd one out: a parametric *edge*, not a solid --
    # a circular arc of ``Radius`` from ``Angle1`` to ``Angle2`` degrees (a full
    # circle at 0..360). It carries a placement like any primitive, but unlike a
    # shape-less sketch that placement *survives a reload* (its ``execute()``
    # rebuilds the edge without resetting the frame), which makes it the natural
    # section to feed a ``Part::Loft``: stack circles at different z and the
    # kernel skins a solid between them. 圆者，规之至也.
    "Part::Circle": {"Radius": "App::PropertyLength",
                     "Angle1": "App::PropertyAngle",
                     "Angle2": "App::PropertyAngle"},
    # ``Part::Ellipse`` is the circle's flattened sibling: a parametric elliptic
    # *edge* of ``MajorRadius`` / ``MinorRadius`` swept from ``Angle1`` to
    # ``Angle2`` degrees (a full ellipse at 0..360). Like the circle its
    # execute() rebuilds the edge from these scalars so its placement survives a
    # reload -- an elliptic section to stack in a ``Part::Loft`` or spine a
    # ``Part::Sweep``. 圆之变也.
    "Part::Ellipse": {"MajorRadius": "App::PropertyLength",
                      "MinorRadius": "App::PropertyLength",
                      "Angle1": "App::PropertyAngle",
                      "Angle2": "App::PropertyAngle"},
    # ``Part::Line`` is the other parametric edge: a straight segment from
    # (X1,Y1,Z1) to (X2,Y2,Z2), its geometry rebuilt from these scalars on
    # execute() so it too survives a reload. It is the natural straight *spine*
    # for a ``Part::Sweep`` -- a circle section swept along it traces a
    # cylinder. 直者，繩之至也.
    "Part::Line": {"X1": "App::PropertyDistance", "Y1": "App::PropertyDistance",
                   "Z1": "App::PropertyDistance", "X2": "App::PropertyDistance",
                   "Y2": "App::PropertyDistance", "Z2": "App::PropertyDistance"},
    # ``Part::Plane`` is the parametric planar *face*: a ``Length`` x ``Width``
    # rectangle in its own XY plane, rebuilt from the two scalars on execute() so
    # (like the circle/line edges) its placement survives a reload. The elementary
    # 2-D datum -- a section to loft/extrude, or a tool face to section against.
    # 地方.
    "Part::Plane": {"Length": "App::PropertyLength", "Width": "App::PropertyLength"},
    # ``Part::Vertex`` is the parametric *point*: a single vertex at (X,Y,Z),
    # rebuilt from the three coordinates on execute(). The 0-dimensional atom of
    # the shape hierarchy -- a construction reference or a degenerate section.
    # 一者，數之至也.
    "Part::Vertex": {"X": "App::PropertyDistance", "Y": "App::PropertyDistance",
                     "Z": "App::PropertyDistance"},
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

# Part::Section -- the *cross-section* boolean: intersect a ``Base`` shape with a
# ``Tool`` shape and keep only the intersection *curves* (the wire where their
# boundaries cross), not a solid. Like the CSG booleans it carries two plain
# ``Base`` / ``Tool`` links, but its result is 1-dimensional, so it adds two bool
# flags instead of producing volume: ``Approximation`` (fit a single B-spline
# through the section edges rather than keep the exact analytic curves) and
# ``Refine`` (drop redundant edges/vertices from the result). The kernel rebuilds
# the section on recompute from the two links + two flags alone; no BREP written.
_SECTION_TYPE = "Part::Section"

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

# A ``Part::Refine`` cleans a ``Source`` shape: it merges the coplanar faces and
# collinear edges a boolean leaves behind (a cut across a box splits one face
# into two -- refine fuses them back). Geometry-preserving (same volume), it
# carries a single ``Source`` link and no scalars; authored from file it is the
# leanest one-link feature -- the tidy-up node downstream of a CSG tree. 大巧若拙.
_REFINE_TYPE = "Part::Refine"

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

# A ``Part::Loft`` lofts a solid (or shell) through an ordered list of >=2
# section profiles (``Sections``), morphing one cross-section into the next --
# the multi-section complement to the single-profile extrude/revolve, and the
# feature that most directly cashes in the sketch vocabulary (a circle to a
# closed B-spline to a point apex). ``Solid`` caps it into a body, ``Ruled``
# joins sections with straight (ruled) surfaces rather than a smooth spline,
# and ``Closed`` wraps the last section back to the first into a loop. Author
# the sections + the loft from file and the kernel skins them on recompute:
# the file-first equivalent of the GUI's Loft, no clicks.
_LOFT_TYPE = "Part::Loft"

# A ``Part::Sweep`` sweeps one or more section profiles (``Sections``) along a
# ``Spine`` path -- a linked edge/wire of another object -- extruding the
# cross-section down the curve. It is the path-driven complement to the
# straight ``Part::Extrusion`` and the multi-section ``Part::Loft``: a circle
# swept along a straight line traces a cylinder, along an arc a bent pipe.
# ``Solid`` caps it into a body, ``Frenet`` keeps the moving section aligned to
# the spine's Frenet frame (vs. a corrected frame). Author the sections + the
# spine + the sweep from file and the kernel pipes them on recompute: the
# file-first equivalent of the GUI's Sweep, no clicks.
_SWEEP_TYPE = "Part::Sweep"

# ``Part::Fillet`` and ``Part::Chamfer`` are the edge *treatments*: they take a
# solid ``Base`` and round (fillet) or bevel (chamfer) a chosen set of its edges.
# Unlike every other authored feature their per-edge sizes do not live in
# ``Document.xml`` -- FreeCAD persists the ``Edges`` property
# (``Part::PropertyFilletEdges``) as a *separate binary member* inside the
# ``.FCStd`` zip, referenced by ``<FilletEdges file="Edges"/>``. That blob is a
# little-endian ``<uint32 count>`` then, per edge, ``<uint32 edge-id><double
# size1><double size2>`` -- the 1-based index of the base edge plus its two radii
# (fillet) / setbacks (chamfer), equal for a constant treatment, distinct for a
# variable one. Authoring a treatment therefore means writing that side file as
# well as the XML, the first feature to push the file layer past a lone
# ``Document.xml``. A ``Base``-shape edge index is only meaningful once the base
# is recomputed, so both also carry an ``EdgeLinks`` ``LinkSub`` naming the base
# + its ``Edge<n>`` subs -- the topological-naming handle mirroring the blob.
# 大直若詘: the sharp edge softened is the stronger form.
_FILLET_TYPE = "Part::Fillet"
_CHAMFER_TYPE = "Part::Chamfer"
_EDGE_TREATMENTS = {_FILLET_TYPE, _CHAMFER_TYPE}

# Part::Thickness -- shelling: hollow a solid ``Base`` to a wall of ``Value``
# thick, opening it at a chosen set of its faces (the ``Faces`` LinkSub: the base
# object plus its ``Face<n>`` subs, the removed faces the hollow vents through).
# A positive value walls inward, a negative one outward. ``Mode`` picks the
# offset algorithm and ``Join`` how adjacent offset faces reconnect at a corner;
# both are enumerations persisted by their integer index. Like the edge
# treatments the kernel rebuilds the shell on recompute from these scalars +
# link alone -- no BREP in the file. 大成若缺，其用不弊: the hollowed form still serves.
_THICKNESS_TYPE = "Part::Thickness"
_THICKNESS_MODES = ("Skin", "Pipe", "RectoVerso")
_THICKNESS_JOINS = ("Arc", "Tangent", "Intersection")

# Part::Offset -- 3D offset: grow (positive ``Value``) or shrink (negative) the
# whole ``Source`` solid/shell by a uniform distance, its faces pushed along
# their normals and reconnected at the corners. It shares the shelling family's
# offset vocabulary -- ``Mode`` (Skin/Pipe/RectoVerso) and ``Join`` (Arc/Tangent/
# Intersection), both enumerations persisted by index -- but takes a plain
# ``Source`` link (no face selection) and a ``Fill`` flag that, when set, walls
# the gap between original and offset into a hollow solid. The kernel rebuilds it
# on recompute from these scalars + the link alone; no BREP is written.
_OFFSET_TYPE = "Part::Offset"
# Part::Offset2D -- the planar sibling: offset a *planar* wire/edge ``Source`` in
# its own plane by ``Value`` (outward positive, inward negative), ``Fill`` walling
# the ring between original and offset into a face. Its property schema is
# identical to the 3D offset (Source/Value/Mode/Join/Fill + the two flags), so
# both share ``_norm_offset`` / ``_offset_properties``; only the type id differs.
_OFFSET2D_TYPE = "Part::Offset2D"
_OFFSET_TYPES = frozenset({_OFFSET_TYPE, _OFFSET2D_TYPE})
_OFFSET_MODES = _THICKNESS_MODES
_OFFSET_JOINS = _THICKNESS_JOINS

# Part::RuledSurface -- skin a single ruled surface between *two* edges/wires by
# joining them with straight generatrix lines (the elementary loft: exactly two
# sections, linear interpolation). Each section is an ``App::PropertyLinkSub``
# (``Curve1`` / ``Curve2``) naming an object plus, optionally, the sub-edge of it
# to use -- a bare object link (count 0) uses the object's whole single edge, as
# a ``Part::Circle`` / ``Part::Line`` provides. ``Orientation`` (an enumeration
# by index) picks how the two curves' senses are matched: ``Automatic`` lets the
# kernel choose, ``Forward`` / ``Reversed`` force it (a reversal twists the strip
# into a saddle). The kernel rebuilds the surface on recompute from the two links
# + the enum alone; no BREP is written. 兩儀生象.
_RULED_TYPE = "Part::RuledSurface"
_RULED_ORIENTS = ("Automatic", "Forward", "Reversed")

# Part::Helix -- a parametric helical edge (the archetypal spring / thread spine).
# Four scalars fix the curve: ``Pitch`` (axial rise per turn), ``Height`` (total
# axial length -- the number of turns is Height/Pitch), ``Radius`` and ``Angle``
# (a cone half-angle taper: 0 is a plain cylinder helix, non-zero spirals the
# radius in/out into a conical helix). Two enumerations by index set the chirality
# and the parametrisation: ``LocalCoord`` -- ``Right-handed`` / ``Left-handed`` --
# and ``Style`` -- ``Old style`` / ``New style``. Like every primitive its
# ``execute()`` rebuilds the edge from these alone; no BREP is written, and the
# read-only computed ``Length`` is left for the kernel to regenerate. Fed as a
# Sweep spine it drives screws, springs and threads. 綿綿若存.
_HELIX_TYPE = "Part::Helix"
_HELIX_HANDS = ("Right-handed", "Left-handed")
_HELIX_STYLES = ("Old style", "New style")

# Part::Spiral -- the planar sibling of the helix: a flat Archimedean spiral edge
# lying in the XY plane, where the radius grows linearly with angle instead of the
# curve climbing in z. Three scalars fix it: ``Growth`` (radial increase per full
# turn), ``Rotations`` (how many turns) and ``Radius`` (the starting radius at
# angle 0). Like the helix its ``execute()`` rebuilds the edge from these alone --
# no BREP written, and the read-only computed ``Length`` is left for recompute.
# Fed as a spine it drives volutes, clock springs and scroll profiles. 大道氾兮.
_SPIRAL_TYPE = "Part::Spiral"


def _edge_treatment_size_keys(otype: str) -> "tuple":
    """The (scalar, pair1, pair2, noun) spec keys an edge treatment reads its
    per-edge size from: ``radius`` for a fillet, ``distance`` for a chamfer."""
    scalar = "radius" if otype == _FILLET_TYPE else "distance"
    return scalar, scalar + "1", scalar + "2", scalar


def _norm_edge_treatment(spec: "Dict[str, Any]") -> "List[tuple]":
    """Validate a fillet/chamfer ``edges`` spec and normalise it to an ordered
    list of ``(edge_id, size1, size2)`` tuples -- the exact triples the ``Edges``
    binary blob encodes.

    Each entry is ``{"edge": <1-based int>, "<size>": s}`` for a constant
    treatment or ``{"edge": int, "<size>1": s1, "<size>2": s2}`` for a variable
    one, where ``<size>`` is ``radius`` (fillet) or ``distance`` (chamfer). Raises
    ``ValueError`` (with the object name) for a malformed / empty / duplicate /
    non-positive entry so a bad treatment never reaches the kernel.
    """
    name, otype = spec.get("name"), spec["type"]
    scalar, k1, k2, noun = _edge_treatment_size_keys(otype)
    edges = spec.get("edges")
    if not isinstance(edges, list) or not edges:
        raise ValueError(
            "synthesize: %s %s needs a non-empty 'edges' list" % (otype, name))
    out: List[tuple] = []
    seen: set = set()
    for j, e in enumerate(edges):
        if not isinstance(e, dict):
            raise ValueError(
                "synthesize: %s %s edge #%d must be a dict" % (otype, name, j))
        eid = e.get("edge")
        if isinstance(eid, bool) or not isinstance(eid, int) or eid < 1:
            raise ValueError(
                "synthesize: %s %s edge #%d 'edge' must be a 1-based integer"
                % (otype, name, j))
        if eid in seen:
            raise ValueError(
                "synthesize: %s %s has duplicate edge %d" % (otype, name, eid))
        seen.add(eid)
        if scalar in e:
            if k1 in e or k2 in e:
                raise ValueError(
                    "synthesize: %s %s edge %d takes either '%s' or '%s'/'%s', "
                    "not both" % (otype, name, eid, scalar, k1, k2))
            s1 = s2 = e[scalar]
        elif k1 in e and k2 in e:
            s1, s2 = e[k1], e[k2]
        else:
            raise ValueError(
                "synthesize: %s %s edge %d needs a '%s' (or both '%s' and '%s')"
                % (otype, name, eid, scalar, k1, k2))
        for s in (s1, s2):
            if isinstance(s, bool) or not isinstance(s, (int, float)) or s <= 0:
                raise ValueError(
                    "synthesize: %s %s edge %d %s must be a positive number"
                    % (otype, name, eid, noun))
        out.append((eid, float(s1), float(s2)))
    return out


def _fillet_edges_blob(edges: "List[tuple]") -> bytes:
    """Encode ``[(edge_id, size1, size2)]`` as the ``Part::PropertyFilletEdges``
    side file: little-endian ``<uint32 count>`` then ``<uint32 id><double s1>
    <double s2>`` per edge."""
    out = struct.pack("<I", len(edges))
    for eid, s1, s2 in edges:
        out += struct.pack("<Idd", int(eid), float(s1), float(s2))
    return out


def _parse_fillet_edges_blob(data: bytes) -> "List[tuple]":
    """Decode a ``Part::PropertyFilletEdges`` side file back to
    ``[(edge_id, size1, size2)]`` -- the inverse of :func:`_fillet_edges_blob`."""
    if len(data) < 4:
        raise ValueError("fillet edges blob too short")
    (count,) = struct.unpack_from("<I", data, 0)
    if len(data) != 4 + count * 20:
        raise ValueError("fillet edges blob size %d != 4 + %d*20"
                         % (len(data), count))
    out: List[tuple] = []
    off = 4
    for _ in range(count):
        eid, s1, s2 = struct.unpack_from("<Idd", data, off)
        off += 20
        out.append((eid, s1, s2))
    return out


def _edge_treatment_properties(parent: ET.Element, spec: "Dict[str, Any]",
                               edges: "List[tuple]", edges_file: str) -> None:
    """Append the ``Base`` / ``EdgeLinks`` / ``Edges`` properties of a fillet or
    chamfer. The per-edge sizes themselves live in the ``edges_file`` binary
    member (written separately); here ``Edges`` only *references* it, while
    ``EdgeLinks`` records the base object + its ``Edge<n>`` sub-references."""
    bp = ET.SubElement(parent, "Property",
                       {"name": "Base", "type": "App::PropertyLink"})
    ET.SubElement(bp, "Link", {"value": spec["base"]})
    ep = ET.SubElement(parent, "Property",
                       {"name": "EdgeLinks", "type": "App::PropertyLinkSub"})
    lsub = ET.SubElement(ep, "LinkSub",
                         {"value": spec["base"], "count": str(len(edges))})
    for eid, _s1, _s2 in edges:
        ET.SubElement(lsub, "Sub", {"value": "Edge%d" % eid})
    fp = ET.SubElement(parent, "Property",
                       {"name": "Edges", "type": "Part::PropertyFilletEdges"})
    ET.SubElement(fp, "FilletEdges", {"file": edges_file})


def _norm_thickness(spec: "Dict[str, Any]") -> "Dict[str, Any]":
    """Validate a ``Part::Thickness`` spec and return it normalised: ``faces`` an
    ordered de-duplicated list of 1-based ints, ``value`` a non-zero float,
    ``mode`` / ``join`` valid enumeration names. Raises ``ValueError`` (naming the
    object) so a malformed shell never reaches the kernel."""
    name = spec.get("name")
    faces = spec.get("faces")
    if not isinstance(faces, list) or not faces:
        raise ValueError(
            "synthesize: thickness %s needs a non-empty 'faces' list" % name)
    seen: set = set()
    norm_faces: List[int] = []
    for j, f in enumerate(faces):
        if isinstance(f, bool) or not isinstance(f, int) or f < 1:
            raise ValueError(
                "synthesize: thickness %s face #%d must be a 1-based integer"
                % (name, j))
        if f in seen:
            raise ValueError(
                "synthesize: thickness %s has duplicate face %d" % (name, f))
        seen.add(f)
        norm_faces.append(f)
    value = spec.get("value")
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or value == 0):
        raise ValueError(
            "synthesize: thickness %s 'value' must be a non-zero number" % name)
    mode = spec.get("mode", "Skin")
    if mode not in _THICKNESS_MODES:
        raise ValueError(
            "synthesize: thickness %s mode %r must be one of %s"
            % (name, mode, ", ".join(_THICKNESS_MODES)))
    join = spec.get("join", "Arc")
    if join not in _THICKNESS_JOINS:
        raise ValueError(
            "synthesize: thickness %s join %r must be one of %s"
            % (name, join, ", ".join(_THICKNESS_JOINS)))
    for flag in ("intersection", "self_intersection"):
        if flag in spec and not isinstance(spec[flag], bool):
            raise ValueError(
                "synthesize: thickness %s '%s' must be a bool" % (name, flag))
    return {"faces": norm_faces, "value": float(value), "mode": mode,
            "join": join, "intersection": bool(spec.get("intersection", False)),
            "self_intersection": bool(spec.get("self_intersection", False))}


def _thickness_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Thickness`` properties hollowing ``Base`` to a shell.

    ``Value`` is the wall thickness; ``Faces`` a ``LinkSub`` naming the base plus
    the ``Face<n>`` subs the hollow opens at; ``Mode`` / ``Join`` the offset
    algorithm + corner rule (each an enumeration written as its integer index);
    ``Intersection`` / ``SelfIntersection`` the two self-collision flags. The
    kernel shells it on recompute; the file only declares it."""
    norm = _norm_thickness(spec)
    vp = ET.SubElement(parent, "Property",
                       {"name": "Value", "type": "App::PropertyQuantity"})
    ET.SubElement(vp, "Float", {"value": "%.16f" % norm["value"]})
    fp = ET.SubElement(parent, "Property",
                       {"name": "Faces", "type": "App::PropertyLinkSub"})
    lsub = ET.SubElement(fp, "LinkSub",
                         {"value": spec["base"], "count": str(len(norm["faces"]))})
    for fid in norm["faces"]:
        ET.SubElement(lsub, "Sub", {"value": "Face%d" % fid})
    mp = ET.SubElement(parent, "Property",
                       {"name": "Mode", "type": "App::PropertyEnumeration"})
    ET.SubElement(mp, "Integer",
                  {"value": str(_THICKNESS_MODES.index(norm["mode"]))})
    jp = ET.SubElement(parent, "Property",
                       {"name": "Join", "type": "App::PropertyEnumeration"})
    ET.SubElement(jp, "Integer",
                  {"value": str(_THICKNESS_JOINS.index(norm["join"]))})
    for pname, flag in (("Intersection", norm["intersection"]),
                        ("SelfIntersection", norm["self_intersection"])):
        bp = ET.SubElement(parent, "Property",
                           {"name": pname, "type": "App::PropertyBool"})
        ET.SubElement(bp, "Bool", {"value": "true" if flag else "false"})


def _norm_offset(spec: "Dict[str, Any]") -> "Dict[str, Any]":
    """Validate a ``Part::Offset`` spec and return it normalised: ``value`` a
    non-zero float, ``mode`` / ``join`` valid enumeration names, the three flags
    bools. Raises ``ValueError`` (naming the object) so a malformed offset never
    reaches the kernel."""
    name = spec.get("name")
    value = spec.get("value")
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or value == 0):
        raise ValueError(
            "synthesize: offset %s 'value' must be a non-zero number" % name)
    mode = spec.get("mode", "Skin")
    if mode not in _OFFSET_MODES:
        raise ValueError(
            "synthesize: offset %s mode %r must be one of %s"
            % (name, mode, ", ".join(_OFFSET_MODES)))
    join = spec.get("join", "Arc")
    if join not in _OFFSET_JOINS:
        raise ValueError(
            "synthesize: offset %s join %r must be one of %s"
            % (name, join, ", ".join(_OFFSET_JOINS)))
    for flag in ("fill", "intersection", "self_intersection"):
        if flag in spec and not isinstance(spec[flag], bool):
            raise ValueError(
                "synthesize: offset %s '%s' must be a bool" % (name, flag))
    return {"value": float(value), "mode": mode, "join": join,
            "fill": bool(spec.get("fill", False)),
            "intersection": bool(spec.get("intersection", False)),
            "self_intersection": bool(spec.get("self_intersection", False))}


def _offset_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Offset`` properties growing/shrinking ``Source``.

    ``Value`` is the (signed) offset distance (a plain ``PropertyFloat``, unlike
    the shelling ``Quantity``); ``Mode`` / ``Join`` the offset algorithm + corner
    rule (each an enumeration written as its integer index); ``Fill`` whether to
    wall the gap into a solid; ``Intersection`` / ``SelfIntersection`` the two
    self-collision flags. The ``Source`` link is written by the object loop."""
    norm = _norm_offset(spec)
    vp = ET.SubElement(parent, "Property",
                       {"name": "Value", "type": "App::PropertyFloat"})
    ET.SubElement(vp, "Float", {"value": "%.16f" % norm["value"]})
    mp = ET.SubElement(parent, "Property",
                       {"name": "Mode", "type": "App::PropertyEnumeration"})
    ET.SubElement(mp, "Integer",
                  {"value": str(_OFFSET_MODES.index(norm["mode"]))})
    jp = ET.SubElement(parent, "Property",
                       {"name": "Join", "type": "App::PropertyEnumeration"})
    ET.SubElement(jp, "Integer",
                  {"value": str(_OFFSET_JOINS.index(norm["join"]))})
    for pname, flag in (("Fill", norm["fill"]),
                        ("Intersection", norm["intersection"]),
                        ("SelfIntersection", norm["self_intersection"])):
        bp = ET.SubElement(parent, "Property",
                           {"name": pname, "type": "App::PropertyBool"})
        ET.SubElement(bp, "Bool", {"value": "true" if flag else "false"})


def _norm_ruled(spec: "Dict[str, Any]") -> "Dict[str, Any]":
    """Validate a ``Part::RuledSurface`` spec and return it normalised: two
    section links ``curve1`` / ``curve2`` (each a ``(name, subs)`` pair, ``subs``
    a list of ``Edge<n>`` sub-element names, possibly empty for a whole-object
    link) and a valid ``orientation`` enumeration name. Raises ``ValueError``
    (naming the object) so a malformed surface never reaches the kernel."""
    name = spec.get("name")
    out = {}
    for key in ("curve1", "curve2"):
        link = spec.get(key)
        if not isinstance(link, str) or not link.strip():
            raise ValueError(
                "synthesize: ruled surface %s needs a '%s' object name"
                % (name, key))
        subs = spec.get(key + "_edges", []) or []
        if not isinstance(subs, (list, tuple)) or not all(
                isinstance(s, str) and s.strip() for s in subs):
            raise ValueError(
                "synthesize: ruled surface %s '%s_edges' must be a list of "
                "sub-element names" % (name, key))
        out[key] = str(link)
        out[key + "_edges"] = [str(s) for s in subs]
    if out["curve1"] == out["curve2"] and not (
            out["curve1_edges"] or out["curve2_edges"]):
        raise ValueError(
            "synthesize: ruled surface %s needs two distinct curves "
            "(or sub-edges) to skin between" % name)
    orient = spec.get("orientation", "Automatic")
    if orient not in _RULED_ORIENTS:
        raise ValueError(
            "synthesize: ruled surface %s orientation %r must be one of %s"
            % (name, orient, ", ".join(_RULED_ORIENTS)))
    out["orientation"] = orient
    return out


def _ruled_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::RuledSurface`` properties skinning between two curves.

    ``Curve1`` / ``Curve2`` are each an ``App::PropertyLinkSub`` naming a section
    object plus its optional ``Edge<n>`` subs (a bare link, count 0, uses the
    object's whole edge); ``Orientation`` the sense-matching enumeration written
    as its integer index."""
    norm = _norm_ruled(spec)
    for key, pname in (("curve1", "Curve1"), ("curve2", "Curve2")):
        subs = norm[key + "_edges"]
        lp = ET.SubElement(parent, "Property",
                           {"name": pname, "type": "App::PropertyLinkSub"})
        ls = ET.SubElement(lp, "LinkSub",
                           {"value": norm[key], "count": str(len(subs))})
        for s in subs:
            ET.SubElement(ls, "Sub", {"value": s})
    op = ET.SubElement(parent, "Property",
                       {"name": "Orientation", "type": "App::PropertyEnumeration"})
    ET.SubElement(op, "Integer",
                  {"value": str(_RULED_ORIENTS.index(norm["orientation"]))})


def _norm_helix(spec: "Dict[str, Any]") -> "Dict[str, Any]":
    """Validate a ``Part::Helix`` spec and return it normalised: positive
    ``pitch`` / ``height`` / ``radius``, a taper ``angle`` in (-90, 90) degrees,
    and valid ``hand`` / ``style`` enumeration names. Raises ``ValueError``
    (naming the object) so a degenerate helix never reaches the kernel."""
    name = spec.get("name")
    out = {}
    for key in ("pitch", "height", "radius"):
        val = spec.get(key)
        if not isinstance(val, (int, float)) or isinstance(val, bool) or val <= 0:
            raise ValueError(
                "synthesize: helix %s needs a positive %r (got %r)"
                % (name, key, val))
        out[key] = float(val)
    angle = spec.get("angle", 0.0)
    if not isinstance(angle, (int, float)) or isinstance(angle, bool) \
            or not -90.0 < float(angle) < 90.0:
        raise ValueError(
            "synthesize: helix %s taper angle %r must be within (-90, 90) degrees"
            % (name, angle))
    out["angle"] = float(angle)
    hand = spec.get("hand", "Right-handed")
    if hand not in _HELIX_HANDS:
        raise ValueError(
            "synthesize: helix %s hand %r must be one of %s"
            % (name, hand, ", ".join(_HELIX_HANDS)))
    out["hand"] = hand
    style = spec.get("style", "Old style")
    if style not in _HELIX_STYLES:
        raise ValueError(
            "synthesize: helix %s style %r must be one of %s"
            % (name, style, ", ".join(_HELIX_STYLES)))
    out["style"] = style
    return out


def _helix_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Helix`` properties: the four length/angle scalars
    (``Pitch`` / ``Height`` / ``Radius`` as ``App::PropertyLength``, ``Angle`` as
    ``App::PropertyAngle``) and the two enumerations (``LocalCoord`` chirality,
    ``Style``) written as their integer indices. No BREP; the read-only ``Length``
    is left for the kernel to recompute."""
    norm = _norm_helix(spec)
    for pname, key, ptype in (("Pitch", "pitch", "App::PropertyLength"),
                              ("Height", "height", "App::PropertyLength"),
                              ("Radius", "radius", "App::PropertyLength"),
                              ("Angle", "angle", "App::PropertyAngle")):
        pp = ET.SubElement(parent, "Property", {"name": pname, "type": ptype})
        ET.SubElement(pp, "Float", {"value": "%.16f" % norm[key]})
    lp = ET.SubElement(parent, "Property",
                       {"name": "LocalCoord", "type": "App::PropertyEnumeration"})
    ET.SubElement(lp, "Integer", {"value": str(_HELIX_HANDS.index(norm["hand"]))})
    sp = ET.SubElement(parent, "Property",
                       {"name": "Style", "type": "App::PropertyEnumeration"})
    ET.SubElement(sp, "Integer", {"value": str(_HELIX_STYLES.index(norm["style"]))})


def _norm_spiral(spec: "Dict[str, Any]") -> "Dict[str, Any]":
    """Validate a ``Part::Spiral`` spec and return it normalised: positive
    ``growth`` (radial increase per turn) and ``rotations`` (turn count), a
    non-negative starting ``radius``. Raises ``ValueError`` (naming the object)
    so a degenerate spiral never reaches the kernel."""
    name = spec.get("name")
    out = {}
    for key in ("growth", "rotations"):
        val = spec.get(key)
        if not isinstance(val, (int, float)) or isinstance(val, bool) or val <= 0:
            raise ValueError(
                "synthesize: spiral %s needs a positive %r (got %r)"
                % (name, key, val))
        out[key] = float(val)
    radius = spec.get("radius", 0.0)
    if not isinstance(radius, (int, float)) or isinstance(radius, bool) \
            or radius < 0:
        raise ValueError(
            "synthesize: spiral %s needs a non-negative 'radius' (got %r)"
            % (name, radius))
    out["radius"] = float(radius)
    return out


def _spiral_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Spiral`` properties: ``Growth`` / ``Radius``
    (``App::PropertyLength``) and ``Rotations``
    (``App::PropertyQuantityConstraint``), each a single ``<Float>``. No BREP; the
    read-only ``Length`` is left for the kernel to recompute."""
    norm = _norm_spiral(spec)
    for pname, key, ptype in (
            ("Growth", "growth", "App::PropertyLength"),
            ("Radius", "radius", "App::PropertyLength"),
            ("Rotations", "rotations", "App::PropertyQuantityConstraint")):
        pp = ET.SubElement(parent, "Property", {"name": pname, "type": ptype})
        ET.SubElement(pp, "Float", {"value": "%.16f" % norm[key]})


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
    ``center``/``radius``; a ``bspline`` carries a ``bspline`` sub-dict
    (poles / knots / mults / degree).
    Returns ``None`` if no form matches so callers can raise a guided error.
    """
    has_axes = "major_radius" in seg or "minor_radius" in seg
    has_sweep = "start_angle" in seg or "end_angle" in seg
    if "bspline" in seg:
        return "bspline"
    if "point" in seg:
        return "point"
    if "start" in seg or "end" in seg:
        return "line"
    if "hyperbola" in seg:
        return "hyperbola"
    if "focal" in seg:
        return "parabola"
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
    * parabola -- ``{"center": [x, y], "focal": f, "start_angle": a,
      "end_angle": b, "angle": t}`` (``center`` is the vertex, ``focal`` the
      focal length, ``angle`` the axis tilt from +X in radians default 0, the two
      angles the parameter range) -> ``Part::GeomArcOfParabola``
    * hyperbola -- ``{"hyperbola": True, "center": [x, y], "major_radius": M,
      "minor_radius": m, "start_angle": a, "end_angle": b, "angle": t}``
      (``center`` is the hyperbola centre, ``major_radius``/``minor_radius`` the
      transverse/conjugate semi-axes with no ordering constraint, ``angle`` the
      transverse-axis tilt from +X in radians default 0, the two angles the
      parameter range) -> ``Part::GeomArcOfHyperbola``. The explicit
      ``hyperbola`` marker disambiguates it from ``arc_ellipse``, which shares
      the same key shape.
    * bspline -- ``{"bspline": {"poles": [[x, y], ...], "knots": [...],
      "mults": [...], "degree": d, "periodic": bool, "weights": [...]}}`` ->
      ``Part::GeomBSplineCurve`` (the general freeform curve; ``weights``
      default to 1, ``periodic`` to false)
    * point   -- ``{"point": [x, y]}`` -> ``Part::GeomPoint`` (an isolated
      sketch vertex: a reference / construction point, no edge)
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
                 "arc_ellipse": "Part::GeomArcOfEllipse",
                 "parabola": "Part::GeomArcOfParabola",
                 "hyperbola": "Part::GeomArcOfHyperbola",
                 "bspline": "Part::GeomBSplineCurve",
                 "point": "Part::GeomPoint"}[kind]
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
        elif kind == "parabola":
            cx, cy = seg["center"]
            ET.SubElement(g, "ArcOfParabola",
                          {"CenterX": "%.16f" % float(cx),
                           "CenterY": "%.16f" % float(cy), "CenterZ": "%.16f" % 0.0,
                           "NormalX": "%.16f" % 0.0, "NormalY": "%.16f" % 0.0,
                           "NormalZ": "%.16f" % 1.0,
                           "Focal": "%.16f" % float(seg["focal"]),
                           "AngleXU": "%.16f" % float(seg.get("angle", 0.0)),
                           "StartAngle": "%.16f" % float(seg["start_angle"]),
                           "EndAngle": "%.16f" % float(seg["end_angle"])})
        elif kind == "hyperbola":
            cx, cy = seg["center"]
            ET.SubElement(g, "ArcOfHyperbola",
                          {"CenterX": "%.16f" % float(cx),
                           "CenterY": "%.16f" % float(cy), "CenterZ": "%.16f" % 0.0,
                           "NormalX": "%.16f" % 0.0, "NormalY": "%.16f" % 0.0,
                           "NormalZ": "%.16f" % 1.0,
                           "MajorRadius": "%.16f" % float(seg["major_radius"]),
                           "MinorRadius": "%.16f" % float(seg["minor_radius"]),
                           "AngleXU": "%.16f" % float(seg.get("angle", 0.0)),
                           "StartAngle": "%.16f" % float(seg["start_angle"]),
                           "EndAngle": "%.16f" % float(seg["end_angle"])})
        elif kind == "bspline":
            bs = seg["bspline"]
            poles = bs["poles"]
            weights = bs.get("weights") or [1.0] * len(poles)
            knots, mults = bs["knots"], bs["mults"]
            bel = ET.SubElement(
                g, "BSplineCurve",
                {"PolesCount": str(len(poles)), "KnotsCount": str(len(knots)),
                 "Degree": str(int(bs["degree"])),
                 "IsPeriodic": "1" if bs.get("periodic") else "0"})
            for (px, py), w in zip(poles, weights):
                ET.SubElement(bel, "Pole",
                              {"X": "%.16f" % float(px),
                               "Y": "%.16f" % float(py), "Z": "%.16f" % 0.0,
                               "Weight": "%.16f" % float(w)})
            for kv, km in zip(knots, mults):
                ET.SubElement(bel, "Knot",
                              {"Value": "%.16f" % float(kv),
                               "Mult": str(int(km))})
        elif kind == "point":
            px, py = seg["point"]
            ET.SubElement(g, "GeomPoint",
                          {"X": "%.16f" % float(px),
                           "Y": "%.16f" % float(py), "Z": "%.16f" % 0.0})
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
    so a closed wire becomes a face the sweep can fill. An optional ``taper``
    (draft angle in degrees) authors a ``TaperAngle`` so the swept walls splay
    out (positive) or draw in (negative) along ``Dir`` -- the drafted extrusion
    of a mould/cast; it is written only when non-zero so a plain extrude stays
    byte-identical. An optional ``symmetric`` sweeps half the ``LengthFwd`` to
    each side of the profile plane (``+L/2 .. -L/2``) instead of all one way --
    the balanced pad; it too is written only when true. An optional
    ``length_rev`` authors a ``LengthRev`` so the sweep also grows the other way
    along ``-Dir`` -- the asymmetric two-sided pad (total extent
    ``length + length_rev``); written only when non-zero. The kernel does the
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
    if spec.get("length_rev"):
        rp = ET.SubElement(parent, "Property",
                           {"name": "LengthRev", "type": "App::PropertyDistance"})
        ET.SubElement(rp, "Float", {"value": "%.16f" % float(spec["length_rev"])})
    taper = spec.get("taper")
    if taper:
        tp = ET.SubElement(parent, "Property",
                           {"name": "TaperAngle", "type": "App::PropertyAngle"})
        ET.SubElement(tp, "Float", {"value": "%.16f" % float(taper)})
    if spec.get("symmetric"):
        yp = ET.SubElement(parent, "Property",
                           {"name": "Symmetric", "type": "App::PropertyBool"})
        ET.SubElement(yp, "Bool", {"value": "true"})
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
    ``FaceMakerClass`` so a closed wire becomes a fillable face. An optional
    ``symmetric`` spins the profile half the ``Angle`` to each side of its plane
    (``+Angle/2 .. -Angle/2``) instead of all one way -- the balanced lathe cut;
    it is written only when true so a plain one-sided revolve stays
    byte-identical. The kernel spins it on recompute; the file just declares it.
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
    if spec.get("symmetric"):
        yp = ET.SubElement(parent, "Property",
                           {"name": "Symmetric", "type": "App::PropertyBool"})
        ET.SubElement(yp, "Bool", {"value": "true"})
    fp = ET.SubElement(parent, "Property",
                       {"name": "FaceMakerClass", "type": "App::PropertyString"})
    ET.SubElement(fp, "String", {"value": _FACEMAKER})


def _loft_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Loft`` properties that skin an ordered list of >=2
    ``Sections`` into a lofted body.

    Four properties carry the feature: ``Sections`` (an ``App::PropertyLinkList``
    of the section profiles in loft order), ``Solid`` to cap the ends into a
    solid rather than a shell, ``Ruled`` to join sections with straight (ruled)
    surfaces instead of a smooth interpolating spline, and ``Closed`` to wrap the
    last section back to the first into a loop. The kernel skins them on
    recompute; the file just declares it.
    """
    sections = spec["sections"]
    lp = ET.SubElement(parent, "Property",
                       {"name": "Sections", "type": "App::PropertyLinkList"})
    ll = ET.SubElement(lp, "LinkList", {"count": str(len(sections))})
    for ref in sections:
        ET.SubElement(ll, "Link", {"value": ref})
    for pname, flag in (("Solid", spec.get("solid", True)),
                        ("Ruled", spec.get("ruled", False)),
                        ("Closed", spec.get("closed", False))):
        bp = ET.SubElement(parent, "Property",
                           {"name": pname, "type": "App::PropertyBool"})
        ET.SubElement(bp, "Bool", {"value": "true" if flag else "false"})


def _sweep_properties(parent: ET.Element, spec: "Dict[str, Any]") -> None:
    """Append the ``Part::Sweep`` properties that pipe one or more ``Sections``
    along a ``Spine`` path.

    ``Sections`` is an ``App::PropertyLinkList`` of the profiles to sweep;
    ``Spine`` an ``App::PropertyLinkSub`` naming the path object plus the
    sub-edge(s) of it to follow (defaulting to ``Edge1``, the sole edge of a
    ``Part::Line`` / ``Part::Circle`` spine). ``Solid`` caps the ends into a
    body, ``Frenet`` aligns the moving section to the spine's Frenet frame, and
    ``Transition`` (fixed at 1, the right-corner mode) governs how the swept
    surface turns a corner. The kernel pipes them on recompute; the file just
    declares it.
    """
    sections = spec["sections"]
    lp = ET.SubElement(parent, "Property",
                       {"name": "Sections", "type": "App::PropertyLinkList"})
    ll = ET.SubElement(lp, "LinkList", {"count": str(len(sections))})
    for ref in sections:
        ET.SubElement(ll, "Link", {"value": ref})
    subs = spec.get("spine_edges") or ["Edge1"]
    sp = ET.SubElement(parent, "Property",
                       {"name": "Spine", "type": "App::PropertyLinkSub"})
    lsub = ET.SubElement(sp, "LinkSub",
                         {"value": spec["spine"], "count": str(len(subs))})
    for s in subs:
        ET.SubElement(lsub, "Sub", {"value": s})
    for pname, flag in (("Solid", spec.get("solid", True)),
                        ("Frenet", spec.get("frenet", False))):
        bp = ET.SubElement(parent, "Property",
                           {"name": pname, "type": "App::PropertyBool"})
        ET.SubElement(bp, "Bool", {"value": "true" if flag else "false"})
    tp = ET.SubElement(parent, "Property",
                       {"name": "Transition", "type": "App::PropertyEnumeration"})
    ET.SubElement(tp, "Integer", {"value": "1"})


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
                and otype != _EXTRUDE_TYPE and otype != _REVOLVE_TYPE
                and otype != _LOFT_TYPE and otype != _SWEEP_TYPE
                and otype not in _EDGE_TREATMENTS
                and otype != _THICKNESS_TYPE and otype not in _OFFSET_TYPES
                and otype != _RULED_TYPE and otype != _SECTION_TYPE
                and otype != _HELIX_TYPE and otype != _SPIRAL_TYPE
                and otype != _REFINE_TYPE):
            raise ValueError(
                "synthesize: object #%d has unknown type %r (supported: %s)"
                % (idx, otype, ", ".join(sorted(
                    set(_PRIMITIVES) | _BOOLEANS | set(_LINKLIST_TYPES)
                    | _EDGE_TREATMENTS | _OFFSET_TYPES
                    | {_SHEET_TYPE, _MIRROR_TYPE, _SKETCH_TYPE,
                       _EXTRUDE_TYPE, _REVOLVE_TYPE, _LOFT_TYPE,
                       _SWEEP_TYPE, _THICKNESS_TYPE, _RULED_TYPE,
                       _SECTION_TYPE, _HELIX_TYPE, _SPIRAL_TYPE,
                       _REFINE_TYPE}))))
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
        elif otype == _SECTION_TYPE:
            for role in ("base", "tool"):
                ref = spec.get(role)
                if not isinstance(ref, str) or not ref.strip():
                    raise ValueError(
                        "synthesize: section %s needs a %r object name"
                        % (name, role))
                if ref == name:
                    raise ValueError(
                        "synthesize: section %s cannot reference itself" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: section %s takes base/tool, not properties"
                    % name)
        elif otype == _HELIX_TYPE:
            _norm_helix(spec)  # validates pitch/height/radius/angle/hand/style
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: helix %s takes pitch/height/radius/angle, "
                    "not properties" % name)
        elif otype == _SPIRAL_TYPE:
            _norm_spiral(spec)  # validates growth/rotations/radius
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: spiral %s takes growth/rotations/radius, "
                    "not properties" % name)
        elif otype == _REFINE_TYPE:
            src = spec.get("source")
            if not isinstance(src, str) or not src.strip():
                raise ValueError(
                    "synthesize: refine %s needs a 'source' object name" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: refine %s takes only 'source', not properties"
                    % name)
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
                elif kind == "bspline":
                    bs = seg["bspline"]
                    if not isinstance(bs, dict):
                        raise ValueError(
                            "synthesize: sketch %s segment #%d 'bspline' must "
                            "be a dict" % (name, j))
                    poles = bs.get("poles")
                    if not isinstance(poles, list) or len(poles) < 2:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline needs "
                            "at least 2 'poles'" % (name, j))
                    for p in poles:
                        _pt2(p, j, "pole")
                    deg = bs.get("degree")
                    if isinstance(deg, bool) or not isinstance(deg, int) or deg < 1:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline 'degree' "
                            "must be an integer >= 1" % (name, j))
                    if len(poles) <= deg:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline needs "
                            "more than 'degree' poles" % (name, j))
                    knots, mults = bs.get("knots"), bs.get("mults")
                    if (not isinstance(knots, list) or not isinstance(mults, list)
                            or len(knots) != len(mults) or len(knots) < 2):
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline 'knots' "
                            "and 'mults' must be equal-length lists (>=2)"
                            % (name, j))
                    for k in range(1, len(knots)):
                        if knots[k] <= knots[k - 1]:
                            raise ValueError(
                                "synthesize: sketch %s segment #%d bspline "
                                "'knots' must strictly increase" % (name, j))
                    if any((isinstance(m, bool) or not isinstance(m, int)
                            or m < 1) for m in mults):
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline 'mults' "
                            "must be integers >= 1" % (name, j))
                    total = sum(mults)
                    want = (len(poles) + 1 if bs.get("periodic")
                            else len(poles) + deg + 1)
                    if total != want:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline knot "
                            "multiplicities sum to %d, expected %d (poles=%d "
                            "degree=%d periodic=%s)"
                            % (name, j, total, want, len(poles), deg,
                               bool(bs.get("periodic"))))
                    w = bs.get("weights")
                    if w is not None and (not isinstance(w, list)
                                          or len(w) != len(poles)):
                        raise ValueError(
                            "synthesize: sketch %s segment #%d bspline 'weights' "
                            "must match the pole count" % (name, j))
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
                elif kind == "point":
                    _pt2(seg.get("point"), j, "point")
                elif kind == "parabola":
                    _pt2(seg.get("center"), j, "center")
                    _num(seg.get("focal"), j, "focal")
                    if seg["focal"] <= 0:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d 'focal' must be "
                            "positive" % (name, j))
                    if "angle" in seg:
                        _num(seg.get("angle"), j, "angle")
                    _num(seg.get("start_angle"), j, "start_angle")
                    _num(seg.get("end_angle"), j, "end_angle")
                    if seg["start_angle"] == seg["end_angle"]:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d parabola is "
                            "degenerate (start_angle == end_angle)" % (name, j))
                elif kind == "hyperbola":
                    _pt2(seg.get("center"), j, "center")
                    _num(seg.get("major_radius"), j, "major_radius")
                    _num(seg.get("minor_radius"), j, "minor_radius")
                    if seg["major_radius"] <= 0 or seg["minor_radius"] <= 0:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d hyperbola "
                            "'major_radius'/'minor_radius' must be positive"
                            % (name, j))
                    if "angle" in seg:
                        _num(seg.get("angle"), j, "angle")
                    _num(seg.get("start_angle"), j, "start_angle")
                    _num(seg.get("end_angle"), j, "end_angle")
                    if seg["start_angle"] == seg["end_angle"]:
                        raise ValueError(
                            "synthesize: sketch %s segment #%d hyperbola is "
                            "degenerate (start_angle == end_angle)" % (name, j))
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
            taper = spec.get("taper")
            if taper is not None and (isinstance(taper, bool)
                                      or not isinstance(taper, (int, float))):
                raise ValueError(
                    "synthesize: extrusion %s 'taper' must be a number "
                    "(draft angle in degrees)" % name)
            if isinstance(taper, (int, float)) and abs(taper) >= 90:
                raise ValueError(
                    "synthesize: extrusion %s 'taper' must be within "
                    "(-90, 90) degrees" % name)
            if "symmetric" in spec and not isinstance(spec["symmetric"], bool):
                raise ValueError(
                    "synthesize: extrusion %s 'symmetric' must be a bool" % name)
            lrev = spec.get("length_rev")
            if lrev is not None and (isinstance(lrev, bool)
                                     or not isinstance(lrev, (int, float))):
                raise ValueError(
                    "synthesize: extrusion %s 'length_rev' must be a number"
                    % name)
            if isinstance(lrev, (int, float)) and lrev < 0:
                raise ValueError(
                    "synthesize: extrusion %s 'length_rev' must be >= 0" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: extrusion %s takes base/length/dir/solid/taper/"
                    "symmetric/length_rev, not properties" % name)
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
            if "symmetric" in spec and not isinstance(spec["symmetric"], bool):
                raise ValueError(
                    "synthesize: revolution %s 'symmetric' must be a bool"
                    % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: revolution %s takes source/axis/base/angle/"
                    "solid/symmetric, not properties" % name)
        elif otype == _LOFT_TYPE:
            sections = spec.get("sections")
            if (not isinstance(sections, list) or len(sections) < 2
                    or not all(isinstance(r, str) and r.strip()
                               for r in sections)):
                raise ValueError(
                    "synthesize: loft %s needs 'sections': a list of >=2 "
                    "object names" % name)
            if name in sections:
                raise ValueError(
                    "synthesize: loft %s cannot reference itself" % name)
            if len(set(sections)) != len(sections):
                raise ValueError(
                    "synthesize: loft %s has duplicate sections" % name)
            for bkey in ("solid", "ruled", "closed"):
                if bkey in spec and not isinstance(spec[bkey], bool):
                    raise ValueError(
                        "synthesize: loft %s '%s' must be a bool" % (name, bkey))
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: loft %s takes sections/solid/ruled/closed, "
                    "not properties" % name)
        elif otype == _SWEEP_TYPE:
            sections = spec.get("sections")
            if (not isinstance(sections, list) or len(sections) < 1
                    or not all(isinstance(r, str) and r.strip()
                               for r in sections)):
                raise ValueError(
                    "synthesize: sweep %s needs 'sections': a list of >=1 "
                    "object names" % name)
            spine = spec.get("spine")
            if not isinstance(spine, str) or not spine.strip():
                raise ValueError(
                    "synthesize: sweep %s needs a 'spine' object name" % name)
            if name in sections or name == spine:
                raise ValueError(
                    "synthesize: sweep %s cannot reference itself" % name)
            if len(set(sections)) != len(sections):
                raise ValueError(
                    "synthesize: sweep %s has duplicate sections" % name)
            if spine in sections:
                raise ValueError(
                    "synthesize: sweep %s spine cannot also be a section" % name)
            for bkey in ("solid", "frenet"):
                if bkey in spec and not isinstance(spec[bkey], bool):
                    raise ValueError(
                        "synthesize: sweep %s '%s' must be a bool"
                        % (name, bkey))
            se = spec.get("spine_edges")
            if se is not None and (
                    not isinstance(se, list) or not se
                    or not all(isinstance(x, str) and x.strip() for x in se)):
                raise ValueError(
                    "synthesize: sweep %s 'spine_edges' must be a non-empty "
                    "list of edge names" % name)
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: sweep %s takes sections/spine/solid/frenet/"
                    "spine_edges, not properties" % name)
        elif otype in _EDGE_TREATMENTS:
            base = spec.get("base")
            if not isinstance(base, str) or not base.strip():
                raise ValueError(
                    "synthesize: %s %s needs a 'base' object name" % (otype, name))
            if base == name:
                raise ValueError(
                    "synthesize: %s %s cannot reference itself" % (otype, name))
            _norm_edge_treatment(spec)  # validates 'edges'
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: %s %s takes base/edges, not properties"
                    % (otype, name))
        elif otype == _THICKNESS_TYPE:
            base = spec.get("base")
            if not isinstance(base, str) or not base.strip():
                raise ValueError(
                    "synthesize: thickness %s needs a 'base' object name" % name)
            if base == name:
                raise ValueError(
                    "synthesize: thickness %s cannot reference itself" % name)
            _norm_thickness(spec)  # validates faces / value / mode / join
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: thickness %s takes base/faces/value, "
                    "not properties" % name)
        elif otype in _OFFSET_TYPES:
            source = spec.get("source")
            if not isinstance(source, str) or not source.strip():
                raise ValueError(
                    "synthesize: offset %s needs a 'source' object name" % name)
            if source == name:
                raise ValueError(
                    "synthesize: offset %s cannot reference itself" % name)
            _norm_offset(spec)  # validates value / mode / join / flags
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: offset %s takes source/value, not properties"
                    % name)
        elif otype == _RULED_TYPE:
            for key in ("curve1", "curve2"):
                if spec.get(key) == name:
                    raise ValueError(
                        "synthesize: ruled surface %s cannot reference itself"
                        % name)
            _norm_ruled(spec)  # validates curve1 / curve2 / orientation
            if spec.get("properties"):
                raise ValueError(
                    "synthesize: ruled surface %s takes curve1/curve2, "
                    "not properties" % name)
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
        elif spec["type"] == _SECTION_TYPE:
            for role in ("base", "tool"):
                if spec[role] not in all_names:
                    raise ValueError(
                        "synthesize: section %s %s=%r is not a defined object"
                        % (spec["name"], role, spec[role]))
        elif spec["type"] == _MIRROR_TYPE:
            if spec["source"] not in all_names:
                raise ValueError(
                    "synthesize: mirror %s source=%r is not a defined object"
                    % (spec["name"], spec["source"]))
        elif spec["type"] == _REFINE_TYPE:
            if spec["source"] not in all_names:
                raise ValueError(
                    "synthesize: refine %s source=%r is not a defined object"
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
        elif spec["type"] == _LOFT_TYPE:
            for ref in spec["sections"]:
                if ref not in all_names:
                    raise ValueError(
                        "synthesize: loft %s section %r is not a defined object"
                        % (spec["name"], ref))
        elif spec["type"] == _SWEEP_TYPE:
            for ref in list(spec["sections"]) + [spec["spine"]]:
                if ref not in all_names:
                    raise ValueError(
                        "synthesize: sweep %s reference %r is not a defined "
                        "object" % (spec["name"], ref))
        elif spec["type"] in _EDGE_TREATMENTS:
            if spec["base"] not in all_names:
                raise ValueError(
                    "synthesize: %s %s base=%r is not a defined object"
                    % (spec["type"], spec["name"], spec["base"]))
        elif spec["type"] == _THICKNESS_TYPE:
            if spec["base"] not in all_names:
                raise ValueError(
                    "synthesize: thickness %s base=%r is not a defined object"
                    % (spec["name"], spec["base"]))
        elif spec["type"] in _OFFSET_TYPES:
            if spec["source"] not in all_names:
                raise ValueError(
                    "synthesize: offset %s source=%r is not a defined object"
                    % (spec["name"], spec["source"]))
        elif spec["type"] == _RULED_TYPE:
            for key in ("curve1", "curve2"):
                if spec[key] not in all_names:
                    raise ValueError(
                        "synthesize: ruled surface %s %s=%r is not a defined "
                        "object" % (spec["name"], key, spec[key]))
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

    # Edge treatments (fillet / chamfer) persist their per-edge sizes in binary
    # side members; collect them here (member name -> blob) to write alongside
    # Document.xml. The k-th treatment's file is named FreeCAD-style -- "Edges"
    # for the first, "Edges1", "Edges2", ... after -- deterministically, so a
    # summarize -> synthesize round-trip regenerates the identical member.
    aux_members: "Dict[str, bytes]" = {}

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
        is_loft = otype == _LOFT_TYPE
        is_sweep = otype == _SWEEP_TYPE
        is_edge = otype in _EDGE_TREATMENTS
        is_thick = otype == _THICKNESS_TYPE
        is_offset = otype in _OFFSET_TYPES
        is_ruled = otype == _RULED_TYPE
        is_section = otype == _SECTION_TYPE
        is_helix = otype == _HELIX_TYPE
        is_spiral = otype == _SPIRAL_TYPE
        is_refine = otype == _REFINE_TYPE
        props = ({} if (is_bool or is_linklist or is_sheet or is_mirror
                        or is_sketch or is_extrude or is_revolve or is_loft
                        or is_sweep or is_edge or is_thick or is_offset
                        or is_ruled or is_section or is_helix or is_spiral
                        or is_refine)
                 else (spec.get("properties") or {}))
        exprs = spec.get("expressions") or {}
        # links: an explicit DAG (boolean operands) plus every *other* object
        # referenced in a formula -- together the object's dependency edges.
        links = ([spec["base"], spec["tool"]] if (is_bool or is_section)
                 else list(spec[ll_key]) if is_linklist
                 else [spec["source"]] if is_mirror
                 else [spec["base"]] if is_extrude
                 else [spec["source"]] if is_revolve
                 else list(spec["sections"]) if is_loft
                 else [spec["spine"]] + list(spec["sections"]) if is_sweep
                 else [spec["base"]] if is_edge
                 else [spec["base"]] if is_thick
                 else [spec["source"]] if is_offset
                 else [spec["curve1"], spec["curve2"]] if is_ruled
                 else [spec["source"]] if is_refine
                 else [])
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
                             or is_sketch or is_extrude or is_revolve or is_loft
                             or is_sweep or is_edge or is_thick or is_offset
                             or is_ruled or is_section or is_helix or is_spiral
                             or is_refine)
                      else [(p, _PRIMITIVES[otype][p], v) for p, v in props.items()])
        prop_count = (len(prop_items) + (1 if has_placement else 0)
                      + (1 if exprs else 0) + (2 if is_bool else 0)
                      + (1 if is_linklist else 0) + (1 if is_sheet else 0)
                      + (3 if is_mirror else 0) + (1 if is_sketch else 0)
                      + (6 if is_extrude else 0)
                      + (1 if is_extrude and spec.get("taper") else 0)
                      + (1 if is_extrude and spec.get("symmetric") else 0)
                      + (1 if is_extrude and spec.get("length_rev") else 0)
                      + (6 if is_revolve else 0)
                      + (1 if is_revolve and spec.get("symmetric") else 0)
                      + (4 if is_loft else 0) + (5 if is_sweep else 0)
                      + (3 if is_edge else 0) + (6 if is_thick else 0)
                      + (7 if is_offset else 0) + (3 if is_ruled else 0)
                      + (4 if is_section else 0) + (6 if is_helix else 0)
                      + (3 if is_spiral else 0) + (1 if is_refine else 0))
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
        if is_loft:
            _loft_properties(props_el, spec)
        if is_sweep:
            _sweep_properties(props_el, spec)
        if is_edge:
            edge_triples = _norm_edge_treatment(spec)
            member = "Edges" if not aux_members else "Edges%d" % len(aux_members)
            aux_members[member] = _fillet_edges_blob(edge_triples)
            _edge_treatment_properties(props_el, spec, edge_triples, member)
        if is_thick:
            _thickness_properties(props_el, spec)
        if is_offset:
            sp = ET.SubElement(props_el, "Property",
                               {"name": "Source", "type": "App::PropertyLink"})
            ET.SubElement(sp, "Link", {"value": spec["source"]})
            _offset_properties(props_el, spec)
        if is_ruled:
            _ruled_properties(props_el, spec)
        if is_helix:
            _helix_properties(props_el, spec)
        if is_spiral:
            _spiral_properties(props_el, spec)
        if is_refine:
            rp = ET.SubElement(props_el, "Property",
                               {"name": "Source", "type": "App::PropertyLink"})
            ET.SubElement(rp, "Link", {"value": spec["source"]})
        if is_section:
            for role_name, ref in (("Base", spec["base"]), ("Tool", spec["tool"])):
                lp = ET.SubElement(props_el, "Property",
                                   {"name": role_name, "type": "App::PropertyLink"})
                ET.SubElement(lp, "Link", {"value": ref})
            for pname, flag in (("Approximation", spec.get("approximation", False)),
                                ("Refine", spec.get("refine", False))):
                bp = ET.SubElement(props_el, "Property",
                                   {"name": pname, "type": "App::PropertyBool"})
                ET.SubElement(bp, "Bool", {"value": "true" if flag else "false"})
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
        for member, blob in aux_members.items():
            zo.writestr(member, blob)
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


def bspline(
    name: str,
    poles: "List[List[float]]",
    degree: int = 3,
    weights: "Optional[List[float]]" = None,
    construction: bool = False,
    closed: bool = False,
) -> "Dict[str, Any]":
    """Generate a freeform B-spline curve as a single-edge sketch spec.

    The most general curve: a B-spline of ``degree`` through control ``poles``
    ``[[x, y], ...]`` with an automatic uniform knot vector. When ``closed`` is
    false (default) it is an open, clamped (endpoint-interpolating) curve whose
    end knots carry multiplicity ``degree+1``. When ``closed`` is true it is a
    *periodic* B-spline -- a smooth closed loop through the poles (all knots
    multiplicity 1, count ``poles+1``, uniform on ``[0, 1]``) that can back a
    face, the freeform analogue of a closed polygon. A human pushes/pulls
    control points by hand; here the poles, the degree and the exact
    knot/multiplicity vector come from one description, written byte-exact.
    Optional rational ``weights`` (one per pole) bend it toward heavy poles. The
    result is a ``Part::GeomBSplineCurve`` that extrudes / sweeps straight from
    file. 道法自然.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("bspline: needs a non-empty name")
    if not isinstance(poles, (list, tuple)) or len(poles) < 2:
        raise ValueError("bspline: needs at least 2 'poles'")
    if isinstance(degree, bool) or not isinstance(degree, int) or degree < 1:
        raise ValueError("bspline: 'degree' must be an integer >= 1")
    if len(poles) <= degree:
        raise ValueError("bspline: needs more than 'degree' poles")
    norm_poles: List[List[float]] = []
    for p in poles:
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                           for c in p)):
            raise ValueError("bspline: each pole must be [x, y] numbers")
        norm_poles.append([float(p[0]), float(p[1])])
    if weights is not None and (not isinstance(weights, (list, tuple))
                                or len(weights) != len(norm_poles)):
        raise ValueError("bspline: 'weights' must give one number per pole")
    if not isinstance(construction, bool):
        raise ValueError("bspline: 'construction' must be a bool")
    if not isinstance(closed, bool):
        raise ValueError("bspline: 'closed' must be a bool")
    n = len(norm_poles)
    if closed:
        # periodic uniform knot vector: poles+1 knots, all multiplicity 1,
        # evenly spaced on [0, 1] (knots[i] = i/n) -- sum == poles + 1. This is
        # what the kernel's buildFromPoles(..., periodic=True) yields, matched
        # byte-exact so the closed loop round-trips.
        knots = [float(i) / float(n) for i in range(n + 1)]
        mults = [1] * (n + 1)
    else:
        # clamped uniform knot vector: end knots carry multiplicity degree+1,
        # the (n-degree) interior knots one each -- sum == poles + degree + 1.
        interior = n - degree - 1
        knots = ([0.0] + [float(i + 1) for i in range(interior)]
                 + [float(interior + 1)])
        mults = [degree + 1] + [1] * interior + [degree + 1]
    inner: Dict[str, Any] = {"poles": norm_poles, "knots": knots,
                             "mults": mults, "degree": degree}
    if closed:
        inner["periodic"] = True
    if weights is not None:
        inner["weights"] = [float(w) for w in weights]
    seg: Dict[str, Any] = {"bspline": inner}
    if construction:
        seg["construction"] = True
    return {"type": _SKETCH_TYPE, "name": name, "geometry": [seg]}


def point(
    name: str,
    at: "List[float]",
    construction: bool = False,
) -> "Dict[str, Any]":
    """Generate an isolated sketch point as a single-vertex sketch spec.

    The simplest sketch primitive: a lone ``Part::GeomPoint`` at ``at``
    ``[x, y]`` -- no edge, just a reference / construction vertex that other
    geometry (mirror axes, pattern seeds, dimensional anchors) can hang off. It
    carries the usual optional ``construction`` flag and round-trips byte-exact.
    道法自然 -- 大方無隅.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("point: needs a non-empty name")
    if (not isinstance(at, (list, tuple)) or len(at) != 2
            or not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                       for c in at)):
        raise ValueError("point: 'at' must be [x, y] numbers")
    if not isinstance(construction, bool):
        raise ValueError("point: 'construction' must be a bool")
    seg: Dict[str, Any] = {"point": [float(at[0]), float(at[1])]}
    if construction:
        seg["construction"] = True
    return {"type": _SKETCH_TYPE, "name": name, "geometry": [seg]}


def loft(
    name: str,
    sections: "List[str]",
    solid: bool = True,
    ruled: bool = False,
    closed: bool = False,
) -> "Dict[str, Any]":
    """Generate a ``Part::Loft`` object spec skinning >=2 ``sections``.

    The multi-section solid: a body lofted through the ordered list of section
    profiles ``sections`` (each the name of another object in the same
    document -- a sketch, a face, a point-apex). ``solid`` caps the ends into a
    solid (else an open shell), ``ruled`` joins consecutive sections with
    straight (ruled) surfaces rather than a smooth interpolating spline, and
    ``closed`` wraps the last section back to the first into a loop. The result
    is a ``synthesize`` object spec, not a sketch -- feed it alongside the
    section specs into :func:`synthesize` and the kernel skins the loft on
    recompute. 道生一，一生二，二生三，三生萬物.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("loft: needs a non-empty name")
    if (not isinstance(sections, (list, tuple)) or len(sections) < 2
            or not all(isinstance(r, str) and r.strip() for r in sections)):
        raise ValueError("loft: 'sections' must be a list of >=2 object names")
    for flag, fname in ((solid, "solid"), (ruled, "ruled"), (closed, "closed")):
        if not isinstance(flag, bool):
            raise ValueError("loft: '%s' must be a bool" % fname)
    spec: Dict[str, Any] = {"type": _LOFT_TYPE, "name": name,
                            "sections": [str(r) for r in sections]}
    if not solid:
        spec["solid"] = False
    if ruled:
        spec["ruled"] = True
    if closed:
        spec["closed"] = True
    return spec


def sweep(
    name: str,
    sections: "List[str]",
    spine: str,
    solid: bool = True,
    frenet: bool = False,
    spine_edges: "Optional[List[str]]" = None,
) -> "Dict[str, Any]":
    """Generate a ``Part::Sweep`` object spec piping ``sections`` along ``spine``.

    The path-driven solid: one or more section profiles ``sections`` (each the
    name of another object -- a circle, a sketch) swept along the ``spine``
    object's edge(s), by default its sole ``Edge1`` (override with
    ``spine_edges``). ``solid`` caps the ends into a body (else an open shell),
    ``frenet`` aligns the moving section to the spine's Frenet frame rather than
    a corrected one. The result is a ``synthesize`` object spec -- feed it
    alongside the section + spine specs into :func:`synthesize` and the kernel
    pipes the sweep on recompute. 大道甚夷，其行也遠.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("sweep: needs a non-empty name")
    if (not isinstance(sections, (list, tuple)) or len(sections) < 1
            or not all(isinstance(r, str) and r.strip() for r in sections)):
        raise ValueError("sweep: 'sections' must be a list of >=1 object names")
    if not isinstance(spine, str) or not spine.strip():
        raise ValueError("sweep: 'spine' must be a non-empty object name")
    for flag, fname in ((solid, "solid"), (frenet, "frenet")):
        if not isinstance(flag, bool):
            raise ValueError("sweep: '%s' must be a bool" % fname)
    if spine_edges is not None and (
            not isinstance(spine_edges, (list, tuple)) or not spine_edges
            or not all(isinstance(x, str) and x.strip() for x in spine_edges)):
        raise ValueError(
            "sweep: 'spine_edges' must be a non-empty list of edge names")
    spec: Dict[str, Any] = {"type": _SWEEP_TYPE, "name": name,
                            "sections": [str(r) for r in sections],
                            "spine": str(spine)}
    if not solid:
        spec["solid"] = False
    if frenet:
        spec["frenet"] = True
    if spine_edges is not None:
        spec["spine_edges"] = [str(x) for x in spine_edges]
    return spec


def fillet(name: str, base: str,
           edges: "List[Dict[str, Any]]") -> "Dict[str, Any]":
    """Generate a ``Part::Fillet`` object spec rounding edges of ``base``.

    ``edges`` is a list of per-edge dicts: ``{"edge": <1-based int>, "radius":
    r}`` for a constant round or ``{"edge": int, "radius1": r1, "radius2": r2}``
    for a variable one (the radius eased along the edge). The result is a
    ``synthesize`` object spec -- feed it alongside the ``base`` solid's spec
    into :func:`synthesize` and the kernel rolls a ball of the given radius
    into each named edge on recompute, writing the sizes to the ``Edges`` binary
    side member. 大直若詘.
    """
    return _edge_treatment_spec(_FILLET_TYPE, name, base, edges)


def chamfer(name: str, base: str,
            edges: "List[Dict[str, Any]]") -> "Dict[str, Any]":
    """Generate a ``Part::Chamfer`` object spec bevelling edges of ``base``.

    ``edges`` is a list of per-edge dicts: ``{"edge": <1-based int>, "distance":
    d}`` for a symmetric bevel or ``{"edge": int, "distance1": d1, "distance2":
    d2}`` for an asymmetric one (the two setbacks off the edge). The result is a
    ``synthesize`` object spec -- feed it alongside the ``base`` solid's spec
    into :func:`synthesize` and the kernel planes each named edge back by the
    given setbacks on recompute. The chamfer is the fillet's straight-cut
    sibling; both persist through the same ``Edges`` binary side member.
    """
    return _edge_treatment_spec(_CHAMFER_TYPE, name, base, edges)


def _edge_treatment_spec(otype: str, name: str, base: str,
                         edges: "List[Dict[str, Any]]") -> "Dict[str, Any]":
    """Shared builder for :func:`fillet` / :func:`chamfer`: assemble and validate
    the ``{type, name, base, edges}`` spec (via :func:`_norm_edge_treatment`), so
    a malformed treatment fails at authoring time rather than on recompute."""
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % otype)
    if not isinstance(base, str) or not base.strip():
        raise ValueError("%s: 'base' must be a non-empty object name" % otype)
    spec: Dict[str, Any] = {"type": otype, "name": name, "base": str(base),
                            "edges": edges}
    _norm_edge_treatment(spec)
    return spec


def thickness(name: str, base: str, faces: "List[int]", value: float,
              mode: str = "Skin", join: str = "Arc",
              intersection: bool = False,
              self_intersection: bool = False) -> "Dict[str, Any]":
    """Generate a ``Part::Thickness`` object spec hollowing ``base`` into a shell.

    ``faces`` is the list of 1-based face indices to *remove* (the openings the
    hollow vents through); ``value`` the wall thickness (positive walls inward,
    negative outward). ``mode`` is the offset algorithm (``Skin`` / ``Pipe`` /
    ``RectoVerso``) and ``join`` how offset faces reconnect at a corner (``Arc``
    / ``Tangent`` / ``Intersection``). Feed the result alongside the ``base``
    solid's spec into :func:`synthesize`; the kernel shells it on recompute from
    these scalars + the face link alone. 大成若缺，其用不弊.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _THICKNESS_TYPE)
    if not isinstance(base, str) or not base.strip():
        raise ValueError(
            "%s: 'base' must be a non-empty object name" % _THICKNESS_TYPE)
    spec: Dict[str, Any] = {"type": _THICKNESS_TYPE, "name": name,
                            "base": str(base), "faces": faces,
                            "value": value, "mode": mode, "join": join,
                            "intersection": intersection,
                            "self_intersection": self_intersection}
    _norm_thickness(spec)
    return spec


def offset(name: str, source: str, value: float, mode: str = "Skin",
           join: str = "Arc", fill: bool = False, intersection: bool = False,
           self_intersection: bool = False) -> "Dict[str, Any]":
    """Generate a ``Part::Offset`` object spec growing/shrinking ``source``.

    ``value`` is the signed offset distance (positive grows the solid outward,
    negative shrinks it inward), ``mode`` the offset algorithm (``Skin`` /
    ``Pipe`` / ``RectoVerso``) and ``join`` how offset faces reconnect at a
    corner (``Arc`` / ``Tangent`` / ``Intersection``). ``fill`` walls the gap
    between original and offset into a hollow solid. Feed the result alongside
    the ``source`` object's spec into :func:`synthesize`; the kernel rebuilds the
    offset on recompute from these scalars + the link alone. 大巧若拙.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _OFFSET_TYPE)
    if not isinstance(source, str) or not source.strip():
        raise ValueError(
            "%s: 'source' must be a non-empty object name" % _OFFSET_TYPE)
    spec: Dict[str, Any] = {"type": _OFFSET_TYPE, "name": name,
                            "source": str(source), "value": value,
                            "mode": mode, "join": join, "fill": fill,
                            "intersection": intersection,
                            "self_intersection": self_intersection}
    _norm_offset(spec)
    return spec


def offset2d(name: str, source: str, value: float, mode: str = "Skin",
             join: str = "Arc", fill: bool = False, intersection: bool = False,
             self_intersection: bool = False) -> "Dict[str, Any]":
    """Generate a ``Part::Offset2D`` object spec offsetting a *planar* wire.

    The planar sibling of :func:`offset`: ``source`` must be a planar edge/wire
    (e.g. a sketch or a ``Part::Circle``), which is offset within its own plane
    by ``value`` (outward positive, inward negative). ``fill`` walls the ring
    between the original and offset wire into a face. ``mode`` / ``join`` carry
    the same offset-algorithm / corner-rule meaning as the 3D offset. The kernel
    rebuilds the wire on recompute from these scalars + the link alone. 大方無隅.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _OFFSET2D_TYPE)
    if not isinstance(source, str) or not source.strip():
        raise ValueError(
            "%s: 'source' must be a non-empty object name" % _OFFSET2D_TYPE)
    spec: Dict[str, Any] = {"type": _OFFSET2D_TYPE, "name": name,
                            "source": str(source), "value": value,
                            "mode": mode, "join": join, "fill": fill,
                            "intersection": intersection,
                            "self_intersection": self_intersection}
    _norm_offset(spec)
    return spec


def ruled_surface(name: str, curve1: str, curve2: str,
                  curve1_edges: "Optional[List[str]]" = None,
                  curve2_edges: "Optional[List[str]]" = None,
                  orientation: str = "Automatic") -> "Dict[str, Any]":
    """Generate a ``Part::RuledSurface`` object spec skinning between two curves.

    The elementary loft: join two section edges/wires (``curve1`` / ``curve2``)
    with straight generatrix lines into a single ruled surface. Each curve names
    an object providing an edge (a ``Part::Circle`` / ``Part::Line`` / sketch);
    pass ``curve1_edges`` / ``curve2_edges`` (lists of ``"Edge<n>"``) to pick a
    specific sub-edge of an object that has several, else the object's whole edge
    is used. ``orientation`` matches the two curves' senses -- ``Automatic``
    (kernel-chosen), ``Forward`` or ``Reversed`` (a reversal twists the strip into
    a saddle). Feed the result alongside the two section specs into
    :func:`synthesize`; the kernel rebuilds the surface on recompute from the two
    links + the enum alone. 兩儀生象.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _RULED_TYPE)
    spec: Dict[str, Any] = {"type": _RULED_TYPE, "name": name,
                            "curve1": str(curve1) if curve1 else curve1,
                            "curve2": str(curve2) if curve2 else curve2,
                            "orientation": orientation}
    if curve1_edges:
        spec["curve1_edges"] = list(curve1_edges)
    if curve2_edges:
        spec["curve2_edges"] = list(curve2_edges)
    _norm_ruled(spec)
    return spec


def section(name: str, base: str, tool: str, approximation: bool = False,
            refine: bool = False) -> "Dict[str, Any]":
    """Generate a ``Part::Section`` object spec: the intersection curves of two
    shapes.

    The cross-section boolean: intersect ``base`` with ``tool`` and keep only the
    1-dimensional wire where their boundaries cross (not a solid). ``approximation``
    fits a single B-spline through the section edges instead of keeping the exact
    analytic curves; ``refine`` drops redundant edges/vertices from the result.
    Feed the result alongside the two operand specs into :func:`synthesize`; the
    kernel rebuilds the section on recompute from the two links + two flags alone.
    大成若缺.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _SECTION_TYPE)
    for role, ref in (("base", base), ("tool", tool)):
        if not isinstance(ref, str) or not ref.strip():
            raise ValueError(
                "%s: %r must be a non-empty object name" % (_SECTION_TYPE, role))
    if base == tool:
        raise ValueError(
            "%s: base and tool must be two distinct objects" % _SECTION_TYPE)
    return {"type": _SECTION_TYPE, "name": name, "base": str(base),
            "tool": str(tool), "approximation": bool(approximation),
            "refine": bool(refine)}


def helix(name: str, pitch: float, height: float, radius: float,
          angle: float = 0.0, hand: str = "Right-handed",
          style: str = "Old style",
          placement: "Optional[Dict[str, Any]]" = None) -> "Dict[str, Any]":
    """Generate a ``Part::Helix`` object spec: a parametric helical edge.

    The archetypal spring / thread spine. ``pitch`` is the axial rise per turn and
    ``height`` the total axial length, so the turn count is ``height / pitch``;
    ``radius`` is the helix radius and ``angle`` a cone half-angle taper in degrees
    (``0`` a plain cylindrical helix, non-zero a conical one that spirals the
    radius in/out). ``hand`` sets the chirality -- ``Right-handed`` or
    ``Left-handed`` -- and ``style`` the parametrisation (``Old style`` /
    ``New style``). Feed the result into :func:`synthesize` on its own, or as the
    ``spine`` of a :func:`sweep` to drive screws, springs and threads; the kernel
    rebuilds the edge on recompute from these scalars alone. 綿綿若存.
    """
    spec: Dict[str, Any] = {"type": _HELIX_TYPE, "name": name,
                            "pitch": pitch, "height": height, "radius": radius,
                            "angle": angle, "hand": hand, "style": style}
    if placement:
        spec["placement"] = placement
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _HELIX_TYPE)
    _norm_helix(spec)
    return spec


def spiral(name: str, growth: float, rotations: float, radius: float = 0.0,
           placement: "Optional[Dict[str, Any]]" = None) -> "Dict[str, Any]":
    """Generate a ``Part::Spiral`` object spec: a flat Archimedean spiral edge.

    The planar sibling of :func:`helix`, lying in the XY plane. ``growth`` is the
    radial increase per full turn, ``rotations`` the number of turns, and
    ``radius`` the starting radius at angle 0 (``0`` spirals out from the centre).
    Feed the result into :func:`synthesize` on its own, or as the ``spine`` of a
    :func:`sweep` to drive volutes, clock springs and scroll profiles; the kernel
    rebuilds the edge on recompute from these scalars alone. 大道氾兮.
    """
    spec: Dict[str, Any] = {"type": _SPIRAL_TYPE, "name": name,
                            "growth": growth, "rotations": rotations,
                            "radius": radius}
    if placement:
        spec["placement"] = placement
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _SPIRAL_TYPE)
    _norm_spiral(spec)
    return spec


def refine(name: str, source: str) -> "Dict[str, Any]":
    """Generate a ``Part::Refine`` object spec: a shape-cleanup feature.

    Wraps a single ``source`` object and, on recompute, merges the coplanar faces
    and collinear edges its shape carries (the redundant seams a boolean leaves
    behind -- a cut across a box splits one face in two, refine fuses them back).
    The refinement preserves geometry (same volume), so it is the natural tidy-up
    node to drop downstream of a :func:`synthesize`d CSG tree. 大巧若拙.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("%s: needs a non-empty name" % _REFINE_TYPE)
    if not isinstance(source, str) or not source.strip():
        raise ValueError("%s: %s needs a 'source' object name"
                         % (_REFINE_TYPE, name))
    return {"type": _REFINE_TYPE, "name": name, "source": source}


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
