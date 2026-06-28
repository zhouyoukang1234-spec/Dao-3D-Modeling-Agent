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


def register(state):
    doc = state.doc

    def _shape(name):
        if name in state.shapes and doc.getObject(state.shapes[name]):
            return doc.getObject(state.shapes[name]).Shape
        if name in state.bodies and doc.getObject(state.bodies[name]):
            return doc.getObject(state.bodies[name]).Shape
        raise KeyError("no shape: %s" % name)

    # ---- spreadsheet-driven parameters ----------------------------------- #
    def op_ss_create(a):
        sheet = doc.getObject(state.__dict__.get("_sheet", "")) if state.__dict__.get("_sheet") else None
        if sheet is None:
            sheet = doc.addObject("Spreadsheet::Sheet", a.get("name", "Spreadsheet"))
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
        key = a["param"]
        alias = a["alias"]
        sheet = doc.getObject(state._sheet)
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
        sheet = doc.getObject(state._sheet)
        alias = a["alias"]
        cell = state._cells[alias]
        sheet.set(cell, str(a["value"]))
        doc.recompute()
        return {"alias": alias, "value": a["value"]}

    def op_ss_table(a):
        sheet = doc.getObject(state._sheet)
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
        plane = a.get("plane", "XY").upper()
        offset = float(a.get("offset", 0))
        normal = {"XY": V(0, 0, 1), "XZ": V(0, 1, 0), "YZ": V(1, 0, 0)}[plane]
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
        tol = float(a.get("tolerance", 0.2))
        mesh = Mesh.Mesh(shape.tessellate(tol))
        return {"points": mesh.CountPoints, "facets": mesh.CountFacets,
                "solid": bool(mesh.isSolid()), "has_non_manifolds": bool(mesh.hasNonManifolds()),
                "self_intersections": bool(mesh.hasSelfIntersections()),
                "mesh_volume": _round(mesh.Volume), "brep_volume": _round(shape.Volume),
                "watertight": bool(mesh.isSolid() and not mesh.hasNonManifolds())}

    def op_mesh_export(a):
        import Mesh
        shape = _shape(a["name"])
        mesh = Mesh.Mesh(shape.tessellate(float(a.get("tolerance", 0.1))))
        mesh.write(a["path"])
        return {"path": a["path"], "facets": mesh.CountFacets,
                "bytes": os.path.getsize(a["path"]) if os.path.exists(a["path"]) else 0}

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
        scale = float(a.get("scale", 1.0))
        made = []
        for vname in views:
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
