"""Direct BREP geometry operations (the ``solid.*`` tool group).

Runs inside freecadcmd. Every shape is a real ``Part::Feature`` object in the
live document, so it is immediately visible in the GUI, exportable, and
measurable. These are non-parametric (explicit BREP) operations — fast, robust,
and the workhorse for boolean modelling, measurement and interference checks.
The PartDesign feature-tree (editable, parametric) lives in ``freecad_parametric``.
"""

import math

import FreeCAD as App
import Part

V = App.Vector


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _round(x, n=4):
    return round(float(x), n)


def _vec(seq, default=(0, 0, 0)):
    if seq is None:
        seq = default
    return V(float(seq[0]), float(seq[1]), float(seq[2]))


def _metrics(shape):
    bb = shape.BoundBox
    data = {
        "valid": bool(shape.isValid()),
        "volume": _round(shape.Volume),
        "area": _round(shape.Area),
        "faces": len(shape.Faces),
        "edges": len(shape.Edges),
        "vertices": len(shape.Vertexes),
        "bbox": [_round(bb.XMin), _round(bb.YMin), _round(bb.ZMin),
                 _round(bb.XMax), _round(bb.YMax), _round(bb.ZMax)],
        "bbox_size": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)],
    }
    try:
        data["closed"] = bool(shape.isClosed())
    except Exception:
        pass
    try:
        com = shape.CenterOfMass
        data["center_of_mass"] = [_round(com.x), _round(com.y), _round(com.z)]
    except Exception:
        pass
    return data


def _center(shape):
    """Centroid of a shape, tolerant of compounds.

    Boolean ops (``cut``/``union``/``common``) routinely return a
    ``Part.Compound`` which — unlike a single ``Solid`` — has no
    ``CenterOfMass``. The mould-half classification only needs a representative
    interior point, so fall back to the bounding-box centre when the true
    centroid is unavailable.
    """
    try:
        return shape.CenterOfMass
    except (AttributeError, RuntimeError):
        bb = shape.BoundBox
        return V(bb.Center.x, bb.Center.y, bb.Center.z)


def _profile_face(spec):
    """Build a planar face (on XY) from a profile spec dict.

    Supported: {"rect":[w,h], "centered":bool}, {"circle":r},
    {"polygon":[[x,y],...]}, {"slot":[length,width]}.
    """
    if "rect" in spec:
        w, h = spec["rect"]
        if spec.get("centered", True):
            x0, y0 = -w / 2.0, -h / 2.0
        else:
            x0, y0 = 0.0, 0.0
        pts = [V(x0, y0, 0), V(x0 + w, y0, 0), V(x0 + w, y0 + h, 0), V(x0, y0 + h, 0), V(x0, y0, 0)]
        wire = Part.makePolygon(pts)
    elif "circle" in spec:
        r = float(spec["circle"])
        wire = Part.Wire(Part.Circle(V(0, 0, 0), V(0, 0, 1), r).toShape())
    elif "polygon" in spec:
        pts = [V(float(p[0]), float(p[1]), 0) for p in spec["polygon"]]
        if pts[0] != pts[-1]:
            pts.append(pts[0])
        wire = Part.makePolygon(pts)
    elif "slot" in spec:
        length, width = spec["slot"]
        r = width / 2.0
        cx = length / 2.0 - r
        e = []
        e.append(Part.LineSegment(V(-cx, -r, 0), V(cx, -r, 0)).toShape())
        e.append(Part.Arc(V(cx, -r, 0), V(cx + r, 0, 0), V(cx, r, 0)).toShape())
        e.append(Part.LineSegment(V(cx, r, 0), V(-cx, r, 0)).toShape())
        e.append(Part.Arc(V(-cx, r, 0), V(-cx - r, 0, 0), V(-cx, -r, 0)).toShape())
        wire = Part.Wire(e)
    else:
        raise ValueError("unknown profile spec: %r" % (spec,))
    return Part.Face(wire)


# --------------------------------------------------------------------------- #
# op registration
# --------------------------------------------------------------------------- #
def register(state):
    doc = state.doc

    def _put(name, shape):
        """Store a shape into a named Part::Feature object (create or update)."""
        # refuse to shadow a parametric body: otherwise the same logical name
        # would resolve to a body via param.* and to this result via solid.*
        # (two different shapes), a silent collision. Force an explicit 'out'.
        if name not in state.shapes and name in state.bodies:
            raise ValueError(
                "%r is a parametric body; pass an explicit 'out' name for the "
                "solid result so it does not shadow the body" % name)
        existing = state.shapes.get(name)
        if existing and doc.getObject(existing):
            obj = doc.getObject(existing)
        else:
            obj = doc.addObject("Part::Feature", name)
            state.shapes[name] = obj.Name
        obj.Shape = shape
        doc.recompute()
        return obj

    def _get(name):
        # resolve a direct solid.* shape, or fall back to a parametric
        # PartDesign body so that explicit ops (export/inspect/booleans) can
        # also consume the parametric feature tree's result.
        oname = state.shapes.get(name) or state.bodies.get(name)
        if not oname:
            raise KeyError("no such solid: %s" % name)
        obj = doc.getObject(oname)
        if obj is None:
            raise KeyError("solid object missing: %s" % name)
        return obj

    # ---- primitives ------------------------------------------------------- #
    def op_box(a):
        s = Part.makeBox(a["length"], a["width"], a["height"], _vec(a.get("pos")))
        _put(a["name"], s)
        return _metrics(s)

    def op_cylinder(a):
        s = Part.makeCylinder(a["radius"], a["height"], _vec(a.get("pos")),
                              _vec(a.get("dir", (0, 0, 1))), a.get("angle", 360))
        _put(a["name"], s)
        return _metrics(s)

    def op_sphere(a):
        s = Part.makeSphere(a["radius"], _vec(a.get("pos")))
        _put(a["name"], s)
        return _metrics(s)

    def op_cone(a):
        s = Part.makeCone(a["radius1"], a["radius2"], a["height"], _vec(a.get("pos")))
        _put(a["name"], s)
        return _metrics(s)

    def op_torus(a):
        s = Part.makeTorus(a["radius1"], a["radius2"], _vec(a.get("pos")))
        _put(a["name"], s)
        return _metrics(s)

    def op_extrude(a):
        face = _profile_face(a["profile"])
        s = face.extrude(_vec(a.get("dir", (0, 0, a.get("height", 10)))))
        _put(a["name"], s)
        return _metrics(s)

    def op_revolve(a):
        face = _profile_face(a["profile"])
        s = face.revolve(_vec(a.get("axis_pos")), _vec(a.get("axis_dir", (0, 1, 0))),
                         a.get("angle", 360))
        _put(a["name"], s)
        return _metrics(s)

    def op_loft(a):
        wires = []
        for sec in a["sections"]:
            face = _profile_face(sec["profile"])
            w = face.Wires[0]
            w.translate(_vec((0, 0, sec.get("offset", 0))))
            wires.append(w)
        s = Part.makeLoft(wires, a.get("solid", True), a.get("ruled", False))
        _put(a["name"], s)
        return _metrics(s)

    def op_shell(a):
        obj = _get(a["name"])
        thickness = float(a["thickness"])
        faces = []
        if a.get("open_faces"):
            faces = [obj.Shape.Faces[i] for i in a["open_faces"]]
        s = obj.Shape.makeThickness(faces, thickness, 1e-3)
        out = a.get("out", a["name"])
        _put(out, s)
        return _metrics(s)

    # ---- transforms ------------------------------------------------------- #
    def op_translate(a):
        obj = _get(a["name"])
        s = obj.Shape.copy()
        s.translate(_vec(a["vector"]))
        _put(a.get("out", a["name"]), s)
        return _metrics(s)

    def op_rotate(a):
        obj = _get(a["name"])
        s = obj.Shape.copy()
        s.rotate(_vec(a.get("center")), _vec(a.get("axis", (0, 0, 1))), a.get("angle", 90))
        _put(a.get("out", a["name"]), s)
        return _metrics(s)

    def op_mirror(a):
        obj = _get(a["name"])
        s = obj.Shape.mirror(_vec(a.get("base")), _vec(a.get("normal", (1, 0, 0))))
        _put(a.get("out", a["name"] + "_m"), s)
        return _metrics(s)

    # ---- booleans --------------------------------------------------------- #
    def _boolean(kind, a):
        base = _get(a["a"]).Shape
        tool = _get(a["b"]).Shape
        if kind == "union":
            s = base.fuse(tool)
        elif kind == "cut":
            s = base.cut(tool)
        elif kind == "common":
            s = base.common(tool)
        else:
            raise ValueError(kind)
        s = s.removeSplitter()
        out = a.get("out", a["a"])
        _put(out, s)
        # absorb the operands like FreeCAD's own Part booleans do: hide any
        # consumed input that is not itself the output, so the live workspace
        # shows the single boolean result rather than overlapping leftovers.
        for operand in (a["a"], a["b"]):
            if operand == out:
                continue
            oname = state.shapes.get(operand) or state.bodies.get(operand)
            obj = doc.getObject(oname) if oname else None
            if obj is not None and hasattr(obj, "Visibility"):
                obj.Visibility = False
        return _metrics(s)

    # ---- fillet / chamfer ------------------------------------------------- #
    def op_fillet(a):
        obj = _get(a["name"])
        edges = obj.Shape.Edges
        if a.get("edges"):
            edges = [obj.Shape.Edges[i] for i in a["edges"]]
        s = obj.Shape.makeFillet(float(a["radius"]), edges)
        _put(a.get("out", a["name"]), s)
        return _metrics(s)

    def op_chamfer(a):
        obj = _get(a["name"])
        edges = obj.Shape.Edges
        if a.get("edges"):
            edges = [obj.Shape.Edges[i] for i in a["edges"]]
        s = obj.Shape.makeChamfer(float(a["size"]), edges)
        _put(a.get("out", a["name"]), s)
        return _metrics(s)

    # ---- patterns --------------------------------------------------------- #
    def op_pattern_linear(a):
        obj = _get(a["name"])
        count = int(a["count"])
        step = _vec(a["step"])
        comp = obj.Shape
        acc = comp
        for i in range(1, count):
            c = comp.copy()
            # build a fresh offset; Vector.multiply mutates in place, which would
            # otherwise accumulate the step factorially across iterations.
            c.translate(_vec((step.x * i, step.y * i, step.z * i)))
            acc = acc.fuse(c)
        acc = acc.removeSplitter()
        _put(a.get("out", a["name"]), acc)
        return _metrics(acc)

    def op_pattern_polar(a):
        obj = _get(a["name"])
        count = int(a["count"])
        total = float(a.get("angle", 360))
        center = _vec(a.get("center"))
        axis = _vec(a.get("axis", (0, 0, 1)))
        full = abs(total - 360) < 1e-6
        n = count if full else count
        ang_step = total / count if full else total / (count - 1)
        comp = obj.Shape
        acc = comp
        for i in range(1, n):
            c = comp.copy()
            c.rotate(center, axis, ang_step * i)
            acc = acc.fuse(c)
        acc = acc.removeSplitter()
        _put(a.get("out", a["name"]), acc)
        return _metrics(acc)

    # ---- inspection ------------------------------------------------------- #
    def op_measure(a):
        return _metrics(_get(a["name"]).Shape)

    def op_inspect(a):
        sh = _get(a["name"]).Shape
        density = float(a.get("density", 1.0))  # g/mm^3 if you like
        m = _metrics(sh)
        m["mass"] = _round(sh.Volume * density)
        try:
            mat = sh.MatrixOfInertia
            m["inertia_diag"] = [_round(mat.A11), _round(mat.A22), _round(mat.A33)]
        except Exception:
            pass
        return m

    def op_interference(a):
        sa = _get(a["a"]).Shape
        sb = _get(a["b"]).Shape
        common = sa.common(sb)
        vol = common.Volume if common.Solids else 0.0
        try:
            dist = sa.distToShape(sb)[0]
        except Exception:
            dist = None
        return {"interfering": vol > 1e-6, "overlap_volume": _round(vol),
                "min_distance": _round(dist) if dist is not None else None}

    def op_draft(a):
        """Mould/casting draft analysis against a pull (de-mould) direction.

        For each face the draft angle is the tilt of the face away from the pull
        axis: beta = asin(|n . pull| / (|n||pull|)). A face perpendicular to the
        pull (a cap / parting face) has beta ~= 90 deg; a vertical side wall has
        beta = 0 and cannot release. Faces with beta < ``min_draft`` (deg) are
        reported as insufficient-draft walls, so a part is ``draftable`` only
        when every side wall carries at least the minimum draft.

        args: name, pull (default +Z), min_draft (deg, default 1.0)
        """
        sh = _get(a["name"]).Shape
        pull = _vec(a.get("pull", (0, 0, 1)))
        plen = pull.Length or 1.0
        com = _center(sh)
        min_draft = float(a.get("min_draft", 1.0))
        sin_min = math.sin(math.radians(min_draft))
        walls, toward, away = [], 0, 0
        for i, f in enumerate(sh.Faces):
            u0, u1, v0, v1 = f.ParameterRange
            n = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
            nlen = n.Length or 1.0
            cos_a = n.dot(pull) / (nlen * plen)          # normal vs pull
            beta = math.degrees(math.asin(min(1.0, abs(cos_a))))
            if abs(cos_a) < sin_min:
                walls.append({"face": "Face%d" % (i + 1), "draft_deg": _round(beta, 3)})
            elif f.CenterOfMass.sub(com).dot(pull) > 0:  # which mould half it parts to
                toward += 1
            else:
                away += 1
        return {"pull": [_round(pull.x), _round(pull.y), _round(pull.z)],
                "min_draft_deg": min_draft, "faces": len(sh.Faces),
                "draftable": len(walls) == 0, "insufficient_draft": len(walls),
                "walls": walls, "toward_pull": toward, "away_pull": away}

    def op_thickness(a):
        """Minimum wall-thickness DFM analysis (mould/casting/print thin walls).

        For each face a grid of sample points is taken; from each point a ray is
        fired straight into the solid along the inward normal and the chord it
        cuts through the material (``edge.common(solid)``) is the local wall
        thickness at that point. The smallest chord over every face is the part's
        minimum wall thickness. A part ``ok`` only when that minimum is at least
        ``min_wall`` (mm); every face thinner than it is reported.

        args: name, min_wall (mm, default 1.0), samples (per-axis, default 3)
        """
        sh = _get(a["name"]).Shape
        min_wall = float(a.get("min_wall", 1.0))
        ns = max(1, int(a.get("samples", 3)))
        diag = sh.BoundBox.DiagonalLength
        eps = max(1e-4, diag * 1e-6)
        worst = None
        thins = []
        for i, f in enumerate(sh.Faces):
            u0, u1, v0, v1 = f.ParameterRange
            face_min = None
            for su in range(ns):
                for sv in range(ns):
                    u = u0 + (u1 - u0) * (su + 0.5) / ns
                    v = v0 + (v1 - v0) * (sv + 0.5) / ns
                    try:
                        p = f.valueAt(u, v)
                        nrm = f.normalAt(u, v)
                    except Exception:
                        continue
                    nl = nrm.Length or 1.0
                    inward = _vec((-nrm.x / nl, -nrm.y / nl, -nrm.z / nl))
                    a_pt = _vec((p.x + inward.x * eps, p.y + inward.y * eps, p.z + inward.z * eps))
                    if not sh.isInside(a_pt, eps * 10, True):  # normal points inward? flip if not
                        inward = _vec((nrm.x / nl, nrm.y / nl, nrm.z / nl))
                        a_pt = _vec((p.x + inward.x * eps, p.y + inward.y * eps, p.z + inward.z * eps))
                        if not sh.isInside(a_pt, eps * 10, True):
                            continue
                    b_pt = _vec((p.x + inward.x * diag * 1.1, p.y + inward.y * diag * 1.1,
                                 p.z + inward.z * diag * 1.1))
                    try:
                        inside = Part.makeLine(a_pt, b_pt).common(sh)
                    except Exception:
                        continue
                    best = None  # chord that starts at this surface point
                    for e in inside.Edges:
                        d0 = min(p.distanceToPoint(vx.Point) for vx in e.Vertexes)
                        if best is None or d0 < best[0]:
                            best = (d0, e.Length)
                    if best is None:
                        continue
                    t = best[1] + eps
                    if face_min is None or t < face_min:
                        face_min = t
            if face_min is None:
                continue
            if worst is None or face_min < worst[0]:
                worst = (face_min, i)
            if face_min < min_wall:
                thins.append({"face": "Face%d" % (i + 1), "thickness": _round(face_min, 3)})
        return {"faces": len(sh.Faces),
                "min_thickness": _round(worst[0], 3) if worst else None,
                "min_thickness_face": ("Face%d" % (worst[1] + 1)) if worst else None,
                "min_wall": min_wall, "ok": bool(worst and worst[0] >= min_wall),
                "thin_walls": thins}

    def op_undercut(a):
        """Undercut detection for a two-plate mould pulled along ``pull``.

        A face can be formed by a simple open/close mould only if it is visible
        from its mould half: a ray fired from the face *outward* along the pull
        axis (toward whichever half that face parts to) must escape without
        re-entering the solid. If that ray hits material again the face is
        shadowed -> it is an undercut needing a side core / lifter. Faces nearly
        parallel to the pull (|n.pull| < sin(parallel_tol)) are side walls handled
        by draft analysis, not undercuts. The part is ``moldable`` (no side
        action) only when no face is an undercut.

        args: name, pull (default +Z), parallel_tol (deg, default 1.0),
              samples (per-axis, default 2)
        """
        sh = _get(a["name"]).Shape
        pull = _vec(a.get("pull", (0, 0, 1)))
        pl = pull.Length or 1.0
        pull = _vec((pull.x / pl, pull.y / pl, pull.z / pl))
        ptol = math.sin(math.radians(float(a.get("parallel_tol", 1.0))))
        ns = max(1, int(a.get("samples", 2)))
        diag = sh.BoundBox.DiagonalLength
        eps = max(1e-4, diag * 1e-6)
        cuts, parallel = [], 0
        for i, f in enumerate(sh.Faces):
            u0, u1, v0, v1 = f.ParameterRange
            nc = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
            ncl = nc.Length or 1.0
            cos_c = nc.dot(pull) / ncl
            if abs(cos_c) < ptol:                       # side wall -> draft domain
                parallel += 1
                continue
            ray = pull if cos_c > 0 else _vec((-pull.x, -pull.y, -pull.z))
            occluded = False
            for su in range(ns):
                for sv in range(ns):
                    u = u0 + (u1 - u0) * (su + 0.5) / ns
                    v = v0 + (v1 - v0) * (sv + 0.5) / ns
                    try:
                        p = f.valueAt(u, v)
                    except Exception:
                        continue
                    a_pt = _vec((p.x + ray.x * eps, p.y + ray.y * eps, p.z + ray.z * eps))
                    b_pt = _vec((p.x + ray.x * diag * 1.1, p.y + ray.y * diag * 1.1,
                                 p.z + ray.z * diag * 1.1))
                    try:
                        inside = Part.makeLine(a_pt, b_pt).common(sh)
                    except Exception:
                        continue
                    if any(e.Length > eps * 10 for e in inside.Edges):
                        occluded = True
                        break
                if occluded:
                    break
            if occluded:
                cuts.append({"face": "Face%d" % (i + 1),
                             "half": "+pull" if cos_c > 0 else "-pull"})
        return {"pull": [_round(pull.x), _round(pull.y), _round(pull.z)],
                "faces": len(sh.Faces), "undercuts": len(cuts),
                "moldable": len(cuts) == 0, "undercut_faces": cuts,
                "parallel_walls": parallel}

    # ---- document management --------------------------------------------- #
    def op_list(a):
        return {"solids": list(state.shapes.keys())}

    def op_delete(a):
        oname = state.shapes.pop(a["name"], None)
        if oname and doc.getObject(oname):
            doc.removeObject(oname)
        doc.recompute()
        return {"deleted": a["name"]}

    def op_export(a):
        names = a.get("names") or list(state.shapes.keys())
        objs = [_get(n) for n in names]
        path = a["path"]
        fmt = a.get("format", path.rsplit(".", 1)[-1]).lower()
        if fmt in ("step", "stp"):
            import Import
            Import.export(objs, path)
        elif fmt == "stl":
            import Mesh
            shapes = [o.Shape for o in objs]
            comp = shapes[0] if len(shapes) == 1 else Part.makeCompound(shapes)
            Mesh.Mesh(comp.tessellate(a.get("tolerance", 0.1))).write(path)
        elif fmt in ("iges", "igs"):
            import Import
            Import.export(objs, path)
        elif fmt == "brep":
            objs[0].Shape.exportBrep(path)
        else:
            raise ValueError("unknown export format: %s" % fmt)
        import os
        return {"path": path, "format": fmt, "bytes": os.path.getsize(path) if os.path.exists(path) else 0}

    def op_import_step(a):
        import Import
        # only register objects this import actually creates, and only real
        # solids -- otherwise pre-existing bodies/sketches/datum planes (which
        # also carry a Shape but live in state.bodies, not state.shapes) get
        # mis-registered as solids, and so do the datum lines/planes inside an
        # imported PartDesign tree.
        before = {o.Name for o in doc.Objects}
        Import.insert(a["path"], doc.Name)
        doc.recompute()
        imported = []
        for o in doc.Objects:
            if o.Name in before:
                continue
            shp = getattr(o, "Shape", None)
            if shp is None or not getattr(shp, "Solids", None):
                continue
            logical = o.Label
            state.shapes[logical] = o.Name
            imported.append(logical)
        return {"imported": imported}

    return {
        "box": op_box, "cylinder": op_cylinder, "sphere": op_sphere, "cone": op_cone,
        "torus": op_torus, "extrude": op_extrude, "revolve": op_revolve, "loft": op_loft,
        "shell": op_shell, "translate": op_translate, "rotate": op_rotate, "mirror": op_mirror,
        "union": lambda a: _boolean("union", a), "cut": lambda a: _boolean("cut", a),
        "common": lambda a: _boolean("common", a), "fillet": op_fillet, "chamfer": op_chamfer,
        "pattern_linear": op_pattern_linear, "pattern_polar": op_pattern_polar,
        "measure": op_measure, "inspect": op_inspect, "interference": op_interference,
        "draft": op_draft, "thickness": op_thickness, "undercut": op_undercut,
        "list": op_list, "delete": op_delete, "export": op_export, "import_step": op_import_step,
    }
