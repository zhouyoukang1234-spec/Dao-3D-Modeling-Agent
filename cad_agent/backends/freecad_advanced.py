"""Deep native FreeCAD mining (``ss.*`` / ``analyze.*`` / ``draw.*`` / ``mesh.*``).

Runs inside freecadcmd. Reaches past the modelling primitives into FreeCAD's
wider machinery:

* ``ss.*``   — a Spreadsheet whose aliased cells *drive* feature dimensions
               through the ExpressionEngine (a real parameter table).
* ``analyze.*`` — cross-section profiles and minimum-distance measurement.
* ``mesh.*`` — tessellation + watertight / manifold analysis.
* ``draw.*`` — TechDraw 2D drawing pages (multi-view projection, DXF export).

These are the capabilities a human reaches for once the solid exists, now wired
to the agent.
"""
import os

import FreeCAD as App
import Part

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


_MISSING = object()


def _num(a, key, default=_MISSING, label=None):
    """Coerce ``a[key]`` to float with a guided error.

    A bare ``float(a.get(key, d))`` leaks Python's unactionable ``ValueError:
    could not convert string to float: 'x'`` / ``TypeError`` on a non-numeric
    value; name the op-facing argument instead.
    """
    name = label or key
    if key not in a or a[key] is None:
        if default is _MISSING:
            raise ValueError("missing required numeric argument %r" % name)
        return float(default)
    v = a[key]
    if isinstance(v, bool) or not isinstance(v, (int, float, str)):
        raise ValueError("%s must be a number (got %r)" % (name, v))
    try:
        return float(v)
    except (TypeError, ValueError):
        raise ValueError("%s must be a number (got %r)" % (name, v))


def register(state):
    doc = state.doc

    def _shape(name):
        if name in state.shapes and doc.getObject(state.shapes[name]):
            return doc.getObject(state.shapes[name]).Shape
        if name in state.bodies and doc.getObject(state.bodies[name]):
            return doc.getObject(state.bodies[name]).Shape
        raise KeyError("no shape: %s" % name)

    # ---- spreadsheet-driven parameters ----------------------------------- #
    def _require_sheet(op):
        """Return the live spreadsheet, or refuse with guidance.

        ``ss.set``/``ss.bind``/``ss.table`` all read ``state._sheet`` directly;
        when ``ss.create`` was never called that attribute is absent and leaks a
        bare ``AttributeError: 'KernelState' object has no attribute '_sheet'``.
        Point the caller at the missing ``ss.create`` instead.
        """
        name = state.__dict__.get("_sheet")
        sheet = doc.getObject(name) if name else None
        if sheet is None:
            raise ValueError(
                "%s: no parameter spreadsheet yet -- call ss.create first to "
                "define one" % op)
        return sheet

    def op_ss_create(a):
        name = a.get("name", "Spreadsheet")
        # addObject's object name must be a string; a non-string (e.g. an int)
        # otherwise leaks 'TypeError: argument 2 must be str, not int'.
        if not isinstance(name, str):
            raise ValueError(
                "ss.create 'name' must be a string (got %r)" % (name,))
        sheet = doc.getObject(state.__dict__.get("_sheet", "")) if state.__dict__.get("_sheet") else None
        if sheet is None:
            sheet = doc.addObject("Spreadsheet::Sheet", name)
            state._sheet = sheet.Name
            state._cells = {}
        col = "A"
        for i, (alias, value) in enumerate(a.get("cells", {}).items(), start=1):
            cell = "%s%d" % (col, i)
            sheet.set(cell, str(value))
            sheet.setAlias(cell, alias)
            state._cells[alias] = cell
        doc.recompute()
        return {"spreadsheet": sheet.Name, "aliases": list(state._cells.keys())}

    def op_ss_bind(a):
        """Bind a registered param to a spreadsheet alias via the ExpressionEngine."""
        sheet = _require_sheet("ss.bind")
        key = a["param"]
        alias = a["alias"]
        v = state.params.get(key)
        if v is None:
            raise KeyError("no such param: %s" % key)
        obj = doc.getObject(v["obj"])
        expr = u"%s.%s" % (sheet.Name, alias)
        if v["kind"] == "prop":
            obj.setExpression(v["ref"], expr)
        else:  # datum constraint
            obj.setExpression(u"Constraints.%s" % v["ref"], expr)
        doc.recompute()
        return {"bound": key, "to": expr}

    def op_ss_set(a):
        sheet = _require_sheet("ss.set")
        alias = a["alias"]
        # an unknown alias otherwise leaks a bare KeyError; name the valid ones.
        cells = state.__dict__.get("_cells", {})
        if alias not in cells:
            raise ValueError(
                "ss.set: no such alias %r; defined aliases: %s"
                % (alias, sorted(cells)))
        cell = cells[alias]
        sheet.set(cell, str(a["value"]))
        doc.recompute()
        return {"alias": alias, "value": a["value"]}

    def op_ss_table(a):
        sheet = _require_sheet("ss.table")
        out = {}
        for alias, cell in state.__dict__.get("_cells", {}).items():
            try:
                out[alias] = sheet.get(cell)
            except Exception:
                out[alias] = None
        return {"table": out}

    # ---- analysis -------------------------------------------------------- #
    def op_section(a):
        shape = _shape(a["name"])
        plane = a.get("plane", "XY")
        if not isinstance(plane, str):
            raise ValueError(
                "analyze.section 'plane' must be one of 'XY'/'XZ'/'YZ' "
                "(got %r)" % (plane,))
        plane = plane.upper()
        offset = _num(a, "offset", 0, "offset")
        normals = {"XY": V(0, 0, 1), "XZ": V(0, 1, 0), "YZ": V(1, 0, 0)}
        if plane not in normals:
            raise ValueError(
                "analyze.section 'plane' must be one of 'XY'/'XZ'/'YZ' "
                "(got %r)" % (plane,))
        normal = normals[plane]
        wires = shape.slice(normal, offset)
        total_len = sum(w.Length for w in wires)
        # net cross-section area: largest wire is the outer boundary, the rest
        # are holes — build one face with holes so they subtract correctly.
        area = 0.0
        if wires:
            try:
                ordered = sorted(wires, key=lambda w: Part.Face(w).Area, reverse=True)
                area = Part.Face(ordered).Area
            except Exception:
                area = sum(Part.Face(w).Area for w in wires if w.isClosed())
        bb = None
        if wires:
            comp = Part.makeCompound(wires)
            b = comp.BoundBox
            bb = [_round(b.XLength), _round(b.YLength), _round(b.ZLength)]
        return {"plane": plane, "offset": offset, "wires": len(wires),
                "section_length": _round(total_len), "section_area": _round(area), "bbox_size": bb}

    def op_distance(a):
        sa = _shape(a["a"])
        sb = _shape(a["b"])
        info = sa.distToShape(sb)
        return {"min_distance": _round(info[0])}

    # ---- mesh analysis --------------------------------------------------- #
    def op_mesh_analyze(a):
        import Mesh
        shape = _shape(a["name"])
        tol = _num(a, "tolerance", 0.2, "mesh tolerance")
        mesh = Mesh.Mesh(shape.tessellate(tol))
        return {"points": mesh.CountPoints, "facets": mesh.CountFacets,
                "solid": bool(mesh.isSolid()), "has_non_manifolds": bool(mesh.hasNonManifolds()),
                "self_intersections": bool(mesh.hasSelfIntersections()),
                "mesh_volume": _round(mesh.Volume), "brep_volume": _round(shape.Volume),
                "watertight": bool(mesh.isSolid() and not mesh.hasNonManifolds())}

    def op_mesh_export(a):
        import Mesh
        shape = _shape(a["name"])
        path = a.get("path")
        # Mesh.write's path must be a filesystem string; a non-string leaks a
        # raw TypeError.
        if not isinstance(path, str) or not path:
            raise ValueError(
                "mesh.export 'path' must be a non-empty file path string (got %r)"
                % (path,))
        tol = _num(a, "tolerance", 0.1, "mesh tolerance")
        mesh = Mesh.Mesh(shape.tessellate(tol))
        mesh.write(path)
        return {"path": path, "facets": mesh.CountFacets,
                "bytes": os.path.getsize(path) if os.path.exists(path) else 0}

    # ---- TechDraw 2D drawing --------------------------------------------- #
    # standard orthographic / pictorial projection directions (first-angle)
    _DRAW_DIRS = {
        "front": (0, -1, 0), "rear": (0, 1, 0), "back": (0, 1, 0),
        "top": (0, 0, 1), "bottom": (0, 0, -1),
        "right": (1, 0, 0), "left": (-1, 0, 0),
        "iso": (1, -1, 1),
    }
    # page layout (mm) per view in a standard first-angle arrangement
    _DRAW_POS = {
        "front": (110, 150), "top": (110, 230), "right": (210, 150),
        "left": (10, 150), "rear": (290, 150), "bottom": (110, 70),
        "iso": (250, 230),
    }

    def op_techdraw(a):
        obj = None
        name = a["name"]
        if name in state.shapes:
            obj = doc.getObject(state.shapes[name])
        elif name in state.bodies:
            obj = doc.getObject(state.bodies[name])
        if obj is None:
            raise KeyError("no shape: %s" % name)
        page = doc.addObject("TechDraw::DrawPage", a.get("page", "Page"))
        template = doc.addObject("TechDraw::DrawSVGTemplate", "Template")
        tdir = os.path.join(App.getResourceDir(), "Mod", "TechDraw", "Templates")
        tmpl = a.get("template")
        if not tmpl:
            for cand in ("A4_LandscapeTD.svg", "A4_Landscape_blank.svg", "A3_Landscape.svg"):
                if os.path.exists(os.path.join(tdir, cand)):
                    tmpl = os.path.join(tdir, cand)
                    break
        if tmpl and os.path.exists(tmpl):
            template.Template = tmpl
        page.Template = template

        views = a.get("views", ["front"])
        if isinstance(views, str):
            views = [views]
        if not isinstance(views, (list, tuple)):
            raise ValueError(
                "draw.techdraw 'views' must be a view name or list of names "
                "(got %r)" % (views,))
        scale = _num(a, "scale", 1.0, "draw scale")
        made = []
        for vname in views:
            if not isinstance(vname, str):
                raise ValueError(
                    "draw.techdraw view names must be strings like 'front'/'top'"
                    "/'iso' (got %r)" % (vname,))
            key = vname.lower()
            if key not in _DRAW_DIRS:
                raise ValueError("unknown view %r (choose from %s)" % (vname, sorted(_DRAW_DIRS)))
            v = doc.addObject("TechDraw::DrawViewPart", "View" + key.title())
            v.Source = [obj]
            v.Direction = V(*_DRAW_DIRS[key])
            v.Scale = scale
            page.addView(v)
            px, py = _DRAW_POS.get(key, (110, 150))
            v.X, v.Y = float(px), float(py)
            made.append(v.Name)
        doc.recompute()

        out = {"page": page.Name, "views": made, "template": tmpl}

        # cross-section view: cut the part with a plane (default through its
        # centroid, normal +X) and project the hatched section. Reveals internal
        # features (bores, pockets) that an outline view hides. Guarded so a
        # TechDraw-version quirk degrades to a reported error, never a crash.
        if a.get("section") and made:
            sp = a["section"] if isinstance(a.get("section"), dict) else {}
            try:
                base = doc.getObject(made[0])
                sec = doc.addObject("TechDraw::DrawViewSection", "Section")
                page.addView(sec)
                sec.BaseView = base
                sec.Source = base.Source
                sec.SectionNormal = V(*sp.get("normal", (1, 0, 0)))
                c = obj.Shape.BoundBox.Center  # robust for solids and compounds
                origin = sp.get("origin", (c.x, c.y, c.z))
                sec.SectionOrigin = V(*origin)
                doc.recompute()
                sec.X, sec.Y = float(sp.get("at", (110, 70))[0]), float(sp.get("at", (110, 70))[1])
                doc.recompute()
                # validated cross-section geometry (independent of TechDraw's
                # headless renderer): slice the real solid with the cut plane.
                # Internal bores show up as extra closed contours, so a hollow
                # part yields more wires than its solid outline alone.
                nrm = V(*sp.get("normal", (1, 0, 0)))
                nrm = nrm.normalize()
                wires = obj.Shape.slice(nrm, nrm.dot(V(*origin)))
                out["section"] = {"view": sec.Name, "normal": list(sp.get("normal", (1, 0, 0))),
                                  "wires": len(wires)}
            except Exception as exc:
                out["section_error"] = str(exc)

        # overall dimensions block (robust headless: annotate the B-rep extents
        # + mass instead of fragile projected-vertex dimension references)
        if a.get("dimensions"):
            bb = obj.Shape.BoundBox
            note = doc.addObject("TechDraw::DrawViewAnnotation", "Dims")
            lines = ["OVERALL  %.1f x %.1f x %.1f mm"
                     % (bb.XLength, bb.YLength, bb.ZLength),
                     "VOLUME  %.1f mm^3" % obj.Shape.Volume]
            note.Text = lines
            page.addView(note)
            note.X, note.Y = 250.0, 60.0
            doc.recompute()
            out["dimensions"] = {"length": round(bb.XLength, 3), "width": round(bb.YLength, 3),
                                 "height": round(bb.ZLength, 3), "volume": round(obj.Shape.Volume, 3)}

        path = a.get("path")
        if path:
            try:
                import TechDraw
                TechDraw.writeDXFPage(page, path)
                out["path"] = path
                out["bytes"] = os.path.getsize(path) if os.path.exists(path) else 0
            except Exception as exc:
                out["export_error"] = str(exc)
        return out

    return {
        "ss.create": op_ss_create, "ss.bind": op_ss_bind, "ss.set": op_ss_set, "ss.table": op_ss_table,
        "analyze.section": op_section, "analyze.distance": op_distance,
        "mesh.analyze": op_mesh_analyze, "mesh.export": op_mesh_export,
        "draw.techdraw": op_techdraw,
    }
