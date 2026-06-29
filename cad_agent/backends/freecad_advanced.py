"""Deep native FreeCAD mining (``ss.*`` / ``analyze.*`` / ``draw.*`` / ``mesh.*``).

Runs inside freecadcmd. Reaches past the modelling primitives into FreeCAD's
wider machinery:

* ``ss.*``   — a Spreadsheet whose aliased cells *drive* feature dimensions
               through the ExpressionEngine (a real parameter table).
* ``analyze.*`` — cross-section profiles and minimum-distance measurement.
* ``mesh.*`` — tessellation + watertight / manifold analysis, mesh-level
               booleans (robust where BRep booleans choke) and sewing a mesh
               back into a BRep shape (the reverse-engineering bridge).
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

    meshes = {}

    def _shape(name):
        if name in state.shapes and doc.getObject(state.shapes[name]):
            return doc.getObject(state.shapes[name]).Shape
        if name in state.bodies and doc.getObject(state.bodies[name]):
            return doc.getObject(state.bodies[name]).Shape
        raise KeyError("no shape: %s" % name)

    def _named_shape(name, op):
        """Like ``_shape`` but raises a guided ValueError (no raw KeyError)."""
        try:
            return _shape(name)
        except KeyError:
            raise ValueError(
                "%s: no such solid %r -- create it (solid.*/param.*) or import "
                "it (import_step) first" % (op, name))

    def _register_shape(name, shape, kind):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("%s 'out' name must be a non-empty string" % kind)
        if shape is None or shape.isNull():
            raise ValueError("%s produced an empty shape" % kind)
        existing = state.shapes.get(name)
        obj = doc.getObject(existing) if existing else None
        if obj is None:
            obj = doc.addObject("Part::Feature", name)
            state.shapes[name] = obj.Name
        obj.Shape = shape
        doc.recompute()
        return obj

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

    _MESH_BOOL = {
        "union": "unite", "fuse": "unite", "unite": "unite",
        "difference": "difference", "cut": "difference", "subtract": "difference",
        "intersection": "intersect", "intersect": "intersect", "common": "intersect",
    }

    def op_mesh_boolean(a):
        """Mesh-level boolean of two solids' tessellations.

        Robust where exact BRep booleans choke on dirty / non-watertight input:
        operates on triangle meshes, not NURBS. args: a (solid name), b (solid
        name), op (union|difference|intersection), out (mesh name),
        tolerance (tessellation, default 0.1). The result mesh is kept so
        ``mesh.to_shape`` can sew it back into a BRep.
        """
        import Mesh
        op = a.get("op", "union")
        if not isinstance(op, str) or op.lower() not in _MESH_BOOL:
            raise ValueError(
                "mesh.boolean 'op' must be one of union/difference/intersection "
                "(got %r)" % (op,))
        method = _MESH_BOOL[op.lower()]
        sa = _named_shape(a.get("a", a.get("base")), "mesh.boolean 'a'")
        sb = _named_shape(a.get("b", a.get("tool")), "mesh.boolean 'b'")
        out = a.get("out", "MeshBool")
        if not isinstance(out, str) or not out.strip():
            raise ValueError("mesh.boolean 'out' must be a non-empty string")
        tol = _num(a, "tolerance", 0.1, "mesh.boolean tolerance")
        if tol <= 0:
            raise ValueError("mesh.boolean tolerance must be > 0 (got %r)" % tol)
        ma = Mesh.Mesh(sa.tessellate(tol))
        mb = Mesh.Mesh(sb.tessellate(tol))
        try:
            res = getattr(ma, method)(mb)
        except Exception as exc:
            raise ValueError(
                "mesh.boolean %s failed (%s); the meshes may be degenerate" % (op, exc))
        if res is None or res.CountFacets == 0:
            raise ValueError(
                "mesh.boolean %s produced an empty mesh (the inputs may not "
                "overlap)" % op)
        meshes[out] = res
        return {"mesh": out, "op": method, "points": res.CountPoints,
                "facets": res.CountFacets, "solid": bool(res.isSolid()),
                "volume": _round(res.Volume)}

    def op_mesh_to_shape(a):
        """Sew a mesh back into a BRep shape (the reverse-engineering bridge).

        Brings tessellated data (a ``mesh.boolean`` result or any solid's
        tessellation) back into the solid world so BRep ops can consume it.
        args: name (mesh from mesh.boolean OR a solid name), out (shape name),
        tolerance (sew tolerance, default 0.1).
        """
        import Mesh
        name = a.get("name", a.get("mesh"))
        out = a.get("out", "FromMesh")
        sew = _num(a, "tolerance", 0.1, "mesh.to_shape tolerance")
        if sew <= 0:
            raise ValueError("mesh.to_shape tolerance must be > 0 (got %r)" % sew)
        if name in meshes:
            topo = meshes[name].Topology
        else:
            topo = Mesh.Mesh(_named_shape(name, "mesh.to_shape 'name'")
                             .tessellate(sew)).Topology
        shell = Part.Shape()
        try:
            shell.makeShapeFromMesh(topo, sew)
        except Exception as exc:
            raise ValueError(
                "mesh.to_shape could not sew the mesh (%s); try a larger "
                "tolerance" % exc)
        if shell.isNull():
            raise ValueError("mesh.to_shape produced an empty shape")
        result, kind = shell, "Shell"
        try:
            solid = Part.makeSolid(shell)
            if solid is not None and not solid.isNull() and solid.Volume > 1e-6:
                result, kind = solid, "Solid"
        except Exception:
            pass
        obj = _register_shape(out, result, "mesh.to_shape")
        return {"shape": out, "object": obj.Name, "type": kind,
                "faces": len(result.Faces), "volume": _round(result.Volume)}

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
        "mesh.boolean": op_mesh_boolean, "mesh.to_shape": op_mesh_to_shape,
        "draw.techdraw": op_techdraw,
    }
