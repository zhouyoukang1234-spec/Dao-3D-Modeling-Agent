"""Surface / Draft / Points+ReverseEngineering workbench coverage.

Wraps three FreeCAD workbenches that the solid/param families never reached,
so the agent can drive them as first-class, fusable ops:

* ``surface.*`` — the Surface workbench: ``fill`` a (possibly non-planar)
                  boundary loop into a face (``makeFilledFace``), ``ruled`` a
                  surface between two profiles (``makeRuledSurface``),
                  ``interpolate`` a BSpline through a grid (exact fit), and
                  ``offset`` a solid's faces into a parallel shell
                  (``makeOffsetShape``).
* ``draft.*``   — the Draft workbench: orthogonal / polar / ``path`` arrays of
                  an existing solid (the baked compound is re-registered so
                  booleans/FEM can consume it) and ``offset`` of a planar
                  polyline profile (``makeOffset2D``).
* ``points.*``  — a point cloud and its reverse-engineered BSpline surface
                  (``ReverseEngineering.approxSurface``): scan data -> geometry,
                  the reverse-modelling core.

Every numeric / vector / list argument is coerced with a guided ValueError
*before* any kernel object is built, so malformed input never leaks a raw
TypeError / OCCError nor leaves a half-built feature in the document.
"""
import FreeCAD as App
import Part

V = App.Vector
_MISSING = object()


def _round(x, n=4):
    return round(float(x), n)


def _num(a, key, default=_MISSING, label=None):
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


def _int(a, key, default=_MISSING, label=None):
    name = label or key
    f = _num(a, key, default, label)
    if abs(f - round(f)) > 1e-9:
        raise ValueError("%s must be a whole number (got %r)" % (name, a.get(key)))
    return int(round(f))


def _vec(seq, label):
    if isinstance(seq, (str, bytes)) or not isinstance(seq, (list, tuple)) \
            or len(seq) != 3:
        raise ValueError(
            "%s must be a list of 3 numbers [x, y, z] (got %r)" % (label, seq))
    try:
        return V(float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        raise ValueError(
            "%s components must all be numbers (got %r)" % (label, seq))


def _grid(a, key, label, min_rows=2, min_cols=2):
    """Coerce a rectangular grid of [x, y, z] points (rows x cols)."""
    rows = a.get(key)
    if isinstance(rows, (str, bytes)) or not isinstance(rows, (list, tuple)):
        raise ValueError(
            "%s must be a grid (list of rows of [x, y, z] points), got %r"
            % (label, rows))
    if len(rows) < min_rows:
        raise ValueError(
            "%s needs at least %d rows (got %d)" % (label, min_rows, len(rows)))
    width = None
    out = []
    for i, row in enumerate(rows):
        if isinstance(row, (str, bytes)) or not isinstance(row, (list, tuple)):
            raise ValueError("%s row %d must be a list of points (got %r)"
                             % (label, i, row))
        if width is None:
            width = len(row)
            if width < min_cols:
                raise ValueError("%s needs at least %d columns (got %d)"
                                 % (label, min_cols, width))
        elif len(row) != width:
            raise ValueError(
                "%s is not rectangular: row 0 has %d points but row %d has %d"
                % (label, width, i, len(row)))
        orow = []
        for j, p in enumerate(row):
            if isinstance(p, (str, bytes)) or not isinstance(p, (list, tuple)) \
                    or len(p) != 3:
                raise ValueError("%s point [%d][%d] must be [x, y, z] (got %r)"
                                 % (label, i, j, p))
            try:
                orow.append((float(p[0]), float(p[1]), float(p[2])))
            except (TypeError, ValueError):
                raise ValueError(
                    "%s point [%d][%d] components must be numbers (got %r)"
                    % (label, i, j, p))
        out.append(orow)
    return out


def _points(a, key, label, need=3):
    pts = a.get(key)
    if isinstance(pts, (str, bytes)) or not isinstance(pts, (list, tuple)):
        raise ValueError(
            "%s must be a list of [x, y, z] points (got %r)" % (label, pts))
    if len(pts) < need:
        raise ValueError(
            "%s needs at least %d points (got %d)" % (label, need, len(pts)))
    out = []
    for i, p in enumerate(pts):
        if isinstance(p, (str, bytes)) or not isinstance(p, (list, tuple)) \
                or len(p) != 3:
            raise ValueError(
                "%s point %d must be [x, y, z] (got %r)" % (label, i, p))
        try:
            out.append((float(p[0]), float(p[1]), float(p[2])))
        except (TypeError, ValueError):
            raise ValueError(
                "%s point %d components must be numbers (got %r)" % (label, i, p))
    return out


def register(state):
    doc = state.doc
    clouds = {}

    def _shape(name):
        if name in state.shapes and doc.getObject(state.shapes[name]):
            return doc.getObject(state.shapes[name])
        if name in state.bodies and doc.getObject(state.bodies[name]):
            return doc.getObject(state.bodies[name])
        raise ValueError(
            "no such solid %r -- create it first (solid.* / param.*) or import "
            "it (import_step)" % name)

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

    # ---- surface.* -------------------------------------------------------- #
    def op_fill(a):
        """Fill a boundary loop of points into a (possibly non-planar) face.

        args: points [[x,y,z]...>=3] (loop, auto-closed), out (name)
        """
        pts = _points(a, "points", "surface.fill 'points'", need=3)
        out = a.get("out", a.get("name", "Surface"))
        vs = [V(*p) for p in pts]
        edges = []
        for i in range(len(vs)):
            p0, p1 = vs[i], vs[(i + 1) % len(vs)]
            if (p1 - p0).Length < 1e-9:
                raise ValueError(
                    "surface.fill: points %d and %d are coincident; remove "
                    "duplicate boundary points" % (i, (i + 1) % len(vs)))
            edges.append(Part.makeLine(p0, p1))
        try:
            face = Part.makeFilledFace(edges)
        except Exception as exc:
            raise ValueError(
                "surface.fill could not fill the boundary loop (%s); the points "
                "may be degenerate/self-intersecting" % exc)
        if face is None or face.isNull():
            raise ValueError(
                "surface.fill produced no face -- the boundary loop is degenerate")
        obj = _register_shape(out, face, "surface.fill")
        return {"surface": out, "object": obj.Name, "area": _round(face.Area),
                "boundary_points": len(pts)}

    def op_ruled(a):
        """Ruled surface lofted linearly between two boundary curves.

        Unlike ``surface.fill`` (one closed loop) this spans *two* open
        profiles, the Surface-workbench "Sections" primitive
        (``Part.makeRuledSurface``). args: edge1 [[x,y,z]...>=2],
        edge2 [[x,y,z]...>=2], out (name).
        """
        p1 = _points(a, "edge1", "surface.ruled 'edge1'", need=2)
        p2 = _points(a, "edge2", "surface.ruled 'edge2'", need=2)
        out = a.get("out", a.get("name", "Ruled"))
        w1 = Part.makePolygon([V(*p) for p in p1])
        w2 = Part.makePolygon([V(*p) for p in p2])
        try:
            shape = Part.makeRuledSurface(w1, w2)
        except Exception as exc:
            raise ValueError(
                "surface.ruled could not rule between the two profiles (%s); "
                "they may be degenerate or coincident" % exc)
        if shape is None or shape.isNull():
            raise ValueError("surface.ruled produced an empty surface")
        obj = _register_shape(out, shape, "surface.ruled")
        return {"surface": out, "object": obj.Name, "area": _round(shape.Area),
                "faces": len(shape.Faces)}

    def op_interpolate(a):
        """Interpolate a smooth BSpline surface *through* a rectangular grid.

        Distinct from ``points.reverse`` (which *approximates* a scattered
        cloud): here every grid node is hit exactly. args: grid
        [[[x,y,z]...cols]...rows] (>=2x2), out (name).
        """
        grid = _grid(a, "grid", "surface.interpolate 'grid'")
        out = a.get("out", a.get("name", "InterpSurface"))
        vgrid = [[V(*p) for p in row] for row in grid]
        bs = Part.BSplineSurface()
        try:
            bs.interpolate(vgrid)
        except Exception as exc:
            raise ValueError(
                "surface.interpolate could not fit the grid (%s); rows may be "
                "collinear/degenerate or contain coincident nodes" % exc)
        face = bs.toShape()
        if face is None or face.isNull():
            raise ValueError("surface.interpolate produced an empty surface")
        obj = _register_shape(out, face, "surface.interpolate")
        return {"surface": out, "object": obj.Name, "area": _round(face.Area),
                "grid": [len(grid), len(grid[0])]}

    def op_offset(a):
        """Offset every face of an existing solid into a parallel shell.

        The Surface-workbench "Offset" primitive (``makeOffsetShape``): a
        positive distance grows the surface outward, negative shrinks it.
        args: source (solid name), distance (!=0), out (name),
        tol (default 1e-3).
        """
        src = _shape(a.get("source", a.get("body", a.get("name"))))
        out = a.get("out", "Offset")
        dist = _num(a, "distance", label="surface.offset distance")
        if abs(dist) < 1e-9:
            raise ValueError("surface.offset distance must be non-zero")
        tol = _num(a, "tol", 1e-3, "surface.offset tol")
        if tol <= 0:
            raise ValueError("surface.offset tol must be > 0 (got %r)" % tol)
        base = getattr(src, "Shape", None)
        if base is None or base.isNull():
            raise ValueError("surface.offset: source %r has no shape" % src.Name)
        try:
            shell = base.makeOffsetShape(dist, tol, fill=False)
        except Exception as exc:
            raise ValueError(
                "surface.offset could not offset by %g (%s); the distance may "
                "exceed the local radius of curvature" % (dist, exc))
        if shell is None or shell.isNull():
            raise ValueError("surface.offset produced an empty shell")
        obj = _register_shape(out, shell, "surface.offset")
        return {"surface": out, "object": obj.Name, "area": _round(shell.Area),
                "distance": _round(dist), "faces": len(shell.Faces)}

    # ---- draft.* ---------------------------------------------------------- #
    def op_ortho_array(a):
        """Real Draft orthogonal array of an existing solid.

        args: source (solid name), out (name), dx/dy/dz (cell vector spans),
              nx/ny/nz (counts)
        """
        import Draft
        src = _shape(a.get("source", a.get("body", a.get("name"))))
        out = a.get("out", "Array")
        nx = _int(a, "nx", 2, "draft.ortho nx")
        ny = _int(a, "ny", 1, "draft.ortho ny")
        nz = _int(a, "nz", 1, "draft.ortho nz")
        for n, lbl in ((nx, "nx"), (ny, "ny"), (nz, "nz")):
            if n < 1:
                raise ValueError("draft.ortho %s must be >= 1 (got %d)" % (lbl, n))
        iv = V(_num(a, "dx", 10, "draft.ortho dx"), 0, 0)
        jv = V(0, _num(a, "dy", 10, "draft.ortho dy"), 0)
        kv = V(0, 0, _num(a, "dz", 10, "draft.ortho dz"))
        arr = Draft.make_ortho_array(src, iv, jv, kv, nx, ny, nz)
        doc.recompute()
        shape = getattr(arr, "Shape", None)
        if shape is None or shape.isNull():
            raise ValueError("draft.ortho_array produced no geometry")
        # re-register the baked compound so booleans/mesh/FEM can consume it
        obj = _register_shape(out, shape.copy(), "draft.ortho_array")
        try:
            doc.removeObject(arr.Name)
        except Exception:
            pass
        return {"array": out, "object": obj.Name, "count": nx * ny * nz,
                "solids": len(shape.Solids)}

    def op_polar_array(a):
        """Real Draft polar array of an existing solid about a centre.

        args: source (solid name), out (name), count, angle (deg, default 360),
              center [x,y,z]
        """
        import Draft
        src = _shape(a.get("source", a.get("body", a.get("name"))))
        out = a.get("out", "Array")
        count = _int(a, "count", 6, "draft.polar count")
        if count < 1:
            raise ValueError("draft.polar count must be >= 1 (got %d)" % count)
        angle = _num(a, "angle", 360.0, "draft.polar angle")
        center = _vec(a["center"], "draft.polar center") if a.get("center") \
            is not None else V(0, 0, 0)
        arr = Draft.make_polar_array(src, count, angle, center)
        doc.recompute()
        shape = getattr(arr, "Shape", None)
        if shape is None or shape.isNull():
            raise ValueError("draft.polar_array produced no geometry")
        obj = _register_shape(out, shape.copy(), "draft.polar_array")
        try:
            doc.removeObject(arr.Name)
        except Exception:
            pass
        return {"array": out, "object": obj.Name, "count": count,
                "solids": len(shape.Solids)}

    def op_path_array(a):
        """Real Draft path array: distribute a solid evenly along a polyline.

        args: source (solid name), path [[x,y,z]...>=2] (the spine), count
              (>=2), out (name). The baked compound is re-registered so
              booleans/mesh/FEM can consume it.
        """
        import Draft
        src = _shape(a.get("source", a.get("body", a.get("name"))))
        pts = _points(a, "path", "draft.path_array 'path'", need=2)
        out = a.get("out", "PathArray")
        count = _int(a, "count", 4, "draft.path count")
        if count < 2:
            raise ValueError("draft.path count must be >= 2 (got %d)" % count)
        for i in range(len(pts) - 1):
            if (V(*pts[i + 1]) - V(*pts[i])).Length < 1e-9:
                raise ValueError(
                    "draft.path_array: path points %d and %d are coincident; "
                    "remove duplicate spine points" % (i, i + 1))
        path = Draft.make_wire([V(*p) for p in pts])
        doc.recompute()
        arr = Draft.make_path_array(src, path, count)
        doc.recompute()
        shape = getattr(arr, "Shape", None)
        if shape is None or shape.isNull():
            for tmp in (arr, path):
                try:
                    doc.removeObject(tmp.Name)
                except Exception:
                    pass
            raise ValueError("draft.path_array produced no geometry")
        obj = _register_shape(out, shape.copy(), "draft.path_array")
        for tmp in (arr, path):
            try:
                doc.removeObject(tmp.Name)
            except Exception:
                pass
        return {"array": out, "object": obj.Name, "count": count,
                "solids": len(shape.Solids)}

    def op_draft_offset(a):
        """Offset a planar polyline profile inward/outward by a distance.

        The Draft 2D-offset primitive (``Wire.makeOffset2D``): a positive
        distance grows a closed profile outward, negative shrinks it. The
        offset wire registers as a first-class shape (extrudable downstream).
        args: points [[x,y,z]...>=2] (profile), distance (!=0), out (name),
        closed (bool, default true).
        """
        pts = _points(a, "points", "draft.offset 'points'", need=2)
        out = a.get("out", a.get("name", "OffsetWire"))
        dist = _num(a, "distance", label="draft.offset distance")
        if abs(dist) < 1e-9:
            raise ValueError("draft.offset distance must be non-zero")
        closed = a.get("closed", True)
        vs = [V(*p) for p in pts]
        if closed and (vs[-1] - vs[0]).Length > 1e-9:
            vs.append(vs[0])
        wire = Part.makePolygon(vs)
        try:
            off = wire.makeOffset2D(dist)
        except Exception as exc:
            raise ValueError(
                "draft.offset could not offset the profile by %g (%s); a "
                "negative offset may collapse a small/concave loop" % (dist, exc))
        if off is None or off.isNull():
            raise ValueError("draft.offset produced an empty wire")
        obj = _register_shape(out, off, "draft.offset")
        return {"wire": out, "object": obj.Name, "length": _round(off.Length),
                "distance": _round(dist), "closed": bool(off.isClosed())}

    # ---- points.* + reverse engineering ----------------------------------- #
    def op_cloud(a):
        """Create a point cloud (scan data) the reverse op can rebuild.

        args: name, points [[x,y,z]...]
        """
        import Points
        name = a.get("name", a.get("out", "Cloud"))
        if not isinstance(name, str) or not name.strip():
            raise ValueError("points.cloud 'name' must be a non-empty string")
        pts = _points(a, "points", "points.cloud 'points'", need=1)
        po = doc.addObject("Points::Feature", name)
        pk = Points.Points()
        pk.addPoints([V(*p) for p in pts])
        po.Points = pk
        doc.recompute()
        clouds[name] = [tuple(p) for p in pts]
        return {"cloud": name, "object": po.Name, "points": len(pts)}

    def op_reverse(a):
        """Reverse-engineer a BSpline surface from a point cloud / point list.

        args: cloud (name from points.cloud) OR points [[x,y,z]...],
              out (surface name), u_degree/v_degree (default 3),
              u_poles/v_poles (control net size, default 6)
        """
        import ReverseEngineering as Reen
        if a.get("cloud") is not None:
            cname = a["cloud"]
            if cname not in clouds:
                raise ValueError(
                    "points.reverse: no cloud named %r -- create it with "
                    "points.cloud first" % cname)
            pts = clouds[cname]
        else:
            pts = _points(a, "points", "points.reverse 'points'", need=9)
        out = a.get("out", a.get("name", "RevSurface"))
        ud = _int(a, "u_degree", 3, "points.reverse u_degree")
        vd = _int(a, "v_degree", 3, "points.reverse v_degree")
        nu = _int(a, "u_poles", 6, "points.reverse u_poles")
        nv = _int(a, "v_poles", 6, "points.reverse v_poles")
        for val, lbl, lo in ((ud, "u_degree", 1), (vd, "v_degree", 1),
                             (nu, "u_poles", 2), (nv, "v_poles", 2)):
            if val < lo:
                raise ValueError(
                    "points.reverse %s must be >= %d (got %d)" % (lbl, lo, val))
        if nu <= ud or nv <= vd:
            raise ValueError(
                "points.reverse: poles must exceed degree (u_poles>%d, "
                "v_poles>%d); got u_poles=%d v_poles=%d" % (ud, vd, nu, nv))
        if len(pts) < nu * nv:
            raise ValueError(
                "points.reverse: need at least u_poles*v_poles=%d points to fit "
                "the control net (got %d)" % (nu * nv, len(pts)))
        try:
            bs = Reen.approxSurface(Points=[tuple(p) for p in pts],
                                    UDegree=ud, VDegree=vd,
                                    NbUPoles=nu, NbVPoles=nv)
        except Exception as exc:
            raise ValueError(
                "points.reverse could not fit a surface (%s); the points may be "
                "collinear/degenerate or the pole counts too high" % exc)
        face = bs.toShape()
        if face is None or face.isNull():
            raise ValueError("points.reverse produced an empty surface")
        obj = _register_shape(out, face, "points.reverse")
        return {"surface": out, "object": obj.Name, "area": _round(face.Area),
                "fit_points": len(pts), "control_net": [nu, nv]}

    return {
        "surface.fill": op_fill,
        "surface.ruled": op_ruled,
        "surface.interpolate": op_interpolate,
        "surface.offset": op_offset,
        "draft.ortho_array": op_ortho_array,
        "draft.polar_array": op_polar_array,
        "draft.path_array": op_path_array,
        "draft.offset": op_draft_offset,
        "points.cloud": op_cloud,
        "points.reverse": op_reverse,
    }
