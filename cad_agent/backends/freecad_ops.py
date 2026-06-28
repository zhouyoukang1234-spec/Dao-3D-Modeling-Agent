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


def _cyl_axes(shape, tol=1e-6):
    """Cylindrical faces of a shape as (center, unit-axis, radius) records.

    The raw material for joint inference: a revolute joint shows up as two
    parts sharing a coaxial cylindrical face (a pin in a hole). Coincident
    duplicates (the same axis reported by several faces) are merged.
    """
    out = []
    for f in shape.Faces:
        surf = f.Surface
        if surf.__class__.__name__ != "Cylinder":
            continue
        ax = surf.Axis
        al = ax.Length or 1.0
        ax = (ax.x / al, ax.y / al, ax.z / al)
        c = surf.Center
        rec = {"center": (c.x, c.y, c.z), "dir": ax, "radius": float(surf.Radius)}
        dup = False
        for e in out:
            if (abs(e["radius"] - rec["radius"]) < 1e-4
                    and abs(abs(e["dir"][0] * ax[0] + e["dir"][1] * ax[1]
                                + e["dir"][2] * ax[2]) - 1.0) < 1e-6):
                # same radius & parallel axis: coaxial if centre offset is axial
                dx = (c.x - e["center"][0], c.y - e["center"][1], c.z - e["center"][2])
                cross = (dx[1] * ax[2] - dx[2] * ax[1],
                         dx[2] * ax[0] - dx[0] * ax[2],
                         dx[0] * ax[1] - dx[1] * ax[0])
                if math.sqrt(sum(v * v for v in cross)) < 1e-4:
                    dup = True
                    break
        if not dup:
            out.append(rec)
    return out


def _plane_faces(shape):
    """Planar faces as (outward unit-normal, centre, bbox).

    ``Surface.Axis`` is the underlying plane normal and ignores which side is
    solid, so two faces flat against each other report the *same* sign. Flip by
    the face orientation to get the true outward normal — only then do opposing
    contact faces come out anti-parallel.
    """
    out = []
    for f in shape.Faces:
        if f.Surface.__class__.__name__ != "Plane":
            continue
        n = f.Surface.Axis
        nl = n.Length or 1.0
        sgn = -1.0 if f.Orientation == "Reversed" else 1.0
        c = f.CenterOfMass
        out.append({"n": (sgn * n.x / nl, sgn * n.y / nl, sgn * n.z / nl),
                    "p": (c.x, c.y, c.z), "bb": f.BoundBox})
    return out


def _bb_overlap(b1, b2, tol):
    return (b1.XMin <= b2.XMax + tol and b2.XMin <= b1.XMax + tol
            and b1.YMin <= b2.YMax + tol and b2.YMin <= b1.YMax + tol
            and b1.ZMin <= b2.ZMax + tol and b2.ZMin <= b1.ZMax + tol)


def _contact_normals(sa, sb, gap=1e-3):
    """Unit normals where a planar face of ``sa`` lies flat against an opposing
    face of ``sb`` (anti-parallel, coincident plane, overlapping footprint).

    These are the directions the contact removes from relative translation —
    the raw material for telling a slider (prismatic) from a free part.
    """
    normals = []
    for a in _plane_faces(sa):
        for b in _plane_faces(sb):
            na, nb = a["n"], b["n"]
            dot = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2]
            if dot > -0.999:                       # require facing (opposed) planes
                continue
            dp = (b["p"][0] - a["p"][0], b["p"][1] - a["p"][1], b["p"][2] - a["p"][2])
            if abs(dp[0] * na[0] + dp[1] * na[1] + dp[2] * na[2]) > max(gap, 1e-6):
                continue                            # planes not coincident -> no contact
            if not _bb_overlap(a["bb"], b["bb"], gap):
                continue                            # footprints do not overlap
            # canonicalise direction sign so +n and -n collapse to one
            key = na if (na[0], na[1], na[2]) >= (0, 0, 0) else (-na[0], -na[1], -na[2])
            if not any(abs(abs(key[0] * m[0] + key[1] * m[1] + key[2] * m[2]) - 1.0) < 1e-6
                       for m in normals):
                normals.append(key)
    return normals


def _free_axis(normals):
    """The single free translation axis of a part pinned by ``normals``.

    Contact normals remove translation along themselves. A slider is boxed in by
    contacts spanning exactly two directions, leaving one free line (their cross
    product). Rank<=1 leaves a plane free (planar joint, not prismatic); rank 3
    is fully constrained.
    """
    for i in range(len(normals)):
        for j in range(i + 1, len(normals)):
            a, b = normals[i], normals[j]
            cx = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                  a[0] * b[1] - a[1] * b[0])
            length = math.sqrt(sum(v * v for v in cx))
            if length <= 1e-6:
                continue
            ax = (cx[0] / length, cx[1] / length, cx[2] / length)
            if all(abs(ax[0] * n[0] + ax[1] * n[1] + ax[2] * n[2]) < 1e-6 for n in normals):
                return ax           # rank exactly 2 -> one free axis
            return None             # a third independent normal -> fully constrained
    return None                     # rank <= 1 -> a free plane, not a slider


def _signature(shape):
    """Compact geometry fingerprint of a solid, for reverse-engineering."""
    bb = shape.BoundBox
    com = _center(shape)
    axes = _cyl_axes(shape)
    return {
        "volume": _round(shape.Volume),
        "bbox_size": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)],
        "center_of_mass": [_round(com.x), _round(com.y), _round(com.z)],
        "faces": len(shape.Faces),
        "cyl_axes": [{"center": [_round(c) for c in r["center"]],
                      "dir": [_round(c, 6) for c in r["dir"]],
                      "radius": _round(r["radius"], 4)} for r in axes],
    }


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
        nf = len(obj.Shape.Faces)
        open_faces = a.get("open_faces")
        # makeThickness cannot hollow a solid without at least one removed face:
        # an empty list returns a null shape with an opaque OCC error. Require the
        # opening explicitly and report a clear, actionable message instead.
        if not open_faces:
            raise ValueError(
                "solid.shell needs 'open_faces': indices of the face(s) to remove "
                "to open the shell (a solid cannot be hollowed without an opening)")
        bad = [i for i in open_faces if i < 0 or i >= nf]
        if bad:
            raise ValueError("open_faces %s out of range (solid has %d faces 0..%d)"
                             % (bad, nf, nf - 1))
        faces = [obj.Shape.Faces[i] for i in open_faces]
        s = obj.Shape.makeThickness(faces, thickness, 1e-3)
        if s.isNull() or s.Volume <= 1e-9:
            raise ValueError(
                "solid.shell produced an empty shape — |thickness|=%g is likely too "
                "large for the wall, or the open_faces are wrong" % abs(thickness))
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
    def _pick_edges(shape, idxs):
        """Select edges by index, or all edges when none are given.

        An out-of-range index otherwise leaks a bare ``IndexError`` that the
        caller cannot act on; report which indices were bad and the valid range.
        """
        ne = len(shape.Edges)
        if not idxs:
            return shape.Edges
        bad = [i for i in idxs if i < -ne or i >= ne]
        if bad:
            raise ValueError("edge indices %s out of range (shape has %d edges 0..%d)"
                             % (bad, ne, ne - 1))
        return [shape.Edges[i] for i in idxs]

    def op_fillet(a):
        obj = _get(a["name"])
        edges = _pick_edges(obj.Shape, a.get("edges"))
        s = obj.Shape.makeFillet(float(a["radius"]), edges)
        _put(a.get("out", a["name"]), s)
        return _metrics(s)

    def op_chamfer(a):
        obj = _get(a["name"])
        edges = _pick_edges(obj.Shape, a.get("edges"))
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
        comp = obj.Shape
        acc = comp
        # count <= 1 is the degenerate array (just the original): no copies, and
        # never divide by count-1 (a partial arc with count=1 used to crash).
        if count > 1:
            full = abs(total - 360) < 1e-6
            # full turn -> count copies evenly over 360 (no duplicate at 0/360);
            # partial arc -> count copies spanning the arc inclusive of both ends.
            ang_step = total / count if full else total / (count - 1)
            for i in range(1, count):
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

    def op_overhang(a):
        """Additive-manufacturing overhang / support DFM against a build axis.

        The additive counterpart of the mould trio: a down-facing surface prints
        cleanly only if it is steep enough that each new layer is supported by
        the one below. With the part grown along ``build`` (default +Z), a face's
        inclination from horizontal is ``beta = acos(|n . build|)`` — a vertical
        wall has beta = 90 deg (always fine), a flat ceiling has beta = 0 (worst).
        A *down-facing* face (outward normal opposing the build axis) that is not
        resting on the build plate and whose ``beta < max_overhang`` (deg, default
        45) needs support material. The part is ``printable`` (support-free) only
        when no such face exists.

        Each face is sampled on a grid (``samples`` per parameter, default 5),
        not just at its centre: a single curved face — a sphere, a large fillet,
        a revolved blend — spans both safe and unsupported inclinations, so a
        lone centre normal would miss the overhanging strip. The reported angle
        for a flagged face is the *worst* (smallest) inclination found on it.
        Face normals follow the same ``normalAt`` convention the draft/undercut
        analyses use. args: name, build (default +Z), max_overhang (45), samples.
        """
        sh = _get(a["name"]).Shape
        up = _vec(a.get("build", (0, 0, 1)))
        ul = up.Length or 1.0
        up = _vec((up.x / ul, up.y / ul, up.z / ul))
        limit = float(a.get("max_overhang", 45.0))
        ns = max(2, int(a.get("samples", 5)))
        bb = sh.BoundBox
        plate = bb.XMin * up.x + bb.YMin * up.y + bb.ZMin * up.z
        tol = max(1e-4, bb.DiagonalLength * 1e-5)
        sin_v = math.sin(math.radians(0.5))    # treat near-vertical as safe walls
        overhangs, walls, plate_faces = [], 0, 0

        def _grid(lo, hi):
            if hi - lo < 1e-9:
                return [(lo + hi) / 2.0]
            return [lo + (hi - lo) * (k + 0.5) / ns for k in range(ns)]

        for i, f in enumerate(sh.Faces):
            u0, u1, v0, v1 = f.ParameterRange
            worst = None                       # smallest down-facing beta over plate
            any_down, only_vertical, only_plate = False, True, True
            for u in _grid(u0, u1):
                for v in _grid(v0, v1):
                    try:
                        p = f.valueAt(u, v)
                        n = f.normalAt(u, v)
                    except Exception:
                        continue
                    nl = n.Length or 1.0
                    cos = (n.x * up.x + n.y * up.y + n.z * up.z) / nl
                    if abs(cos) < sin_v:       # vertical -> self-supporting
                        continue
                    only_vertical = False
                    if cos > 0:                # up-facing -> supported from below
                        only_plate = False
                        continue
                    any_down = True
                    proj = p.x * up.x + p.y * up.y + p.z * up.z
                    if proj <= plate + tol:    # this point rests on the plate
                        continue
                    only_plate = False
                    beta = math.degrees(math.acos(min(1.0, abs(cos))))
                    if worst is None or beta < worst:
                        worst = beta
            if worst is not None and worst < limit:
                overhangs.append({"face": "Face%d" % (i + 1), "angle_deg": _round(worst, 3)})
            elif only_vertical:
                walls += 1
            elif any_down and only_plate:
                plate_faces += 1
        return {"build": [_round(up.x), _round(up.y), _round(up.z)],
                "max_overhang_deg": limit, "faces": len(sh.Faces),
                "printable": len(overhangs) == 0, "overhangs": len(overhangs),
                "overhang_faces": overhangs, "vertical_walls": walls,
                "plate_faces": plate_faces}

    def op_section(a):
        """Planar cross-section properties of a solid (beam / structural design).

        Cuts the solid with a plane (``normal`` + offset ``d`` along that normal,
        or a point ``at`` on the plane) and builds the section face — outer
        contour with any interior holes. Reports the engineering section
        properties used for bending and torsion:

          * ``area`` and ``centroid`` of the section;
          * ``Ix`` / ``Iy`` — second moments of area about the centroidal axes
            (∫y²dA, ∫x²dA), i.e. bending stiffness terms;
          * ``J`` — polar second moment about the centroidal normal axis
            (= Ix + Iy for a plane section), the torsion term;
          * ``Ixy`` — product of area (0 for a doubly-symmetric section).

        These come from the face inertia tensor taken about the centroid, so for
        a section lying in a global coordinate plane (the usual axis-perpendicular
        cut) they equal the textbook closed forms exactly. The part is reported
        ``solid`` only when the plane actually intersects material.

        args: name, normal (default +Z), d (offset along normal) | at (point)
        """
        sh = _get(a["name"]).Shape
        n = _vec(a.get("normal", (0, 0, 1)))
        nl = n.Length or 1.0
        n = _vec((n.x / nl, n.y / nl, n.z / nl))
        if a.get("at") is not None:
            p = _vec(a["at"])
            d = p.dot(n)
        else:
            d = float(a.get("d", 0.0))
        wires = sh.slice(n, d)
        if not wires:
            return {"hit": False, "normal": [_round(n.x), _round(n.y), _round(n.z)],
                    "offset": _round(d), "area": 0.0}
        face = Part.Face(wires)
        c = face.CenterOfMass
        m = face.MatrixOfInertia
        diag = [m.A11, m.A22, m.A33]
        # polar second moment about the centroidal normal axis: n . M . n
        mn = (m.A11 * n.x + m.A12 * n.y + m.A13 * n.z,
              m.A12 * n.x + m.A22 * n.y + m.A23 * n.z,
              m.A13 * n.x + m.A23 * n.y + m.A33 * n.z)
        polar = mn[0] * n.x + mn[1] * n.y + mn[2] * n.z
        out = {"hit": True, "normal": [_round(n.x), _round(n.y), _round(n.z)],
               "offset": _round(d), "area": _round(face.Area),
               "centroid": [_round(c.x), _round(c.y), _round(c.z)],
               "J": _round(polar, 3), "loops": len(wires)}
        # for an axis-aligned cut the two in-plane bending moments are exactly
        # the other two tensor-diagonal terms; label them, else leave None.
        axis = next((k for k in range(3) if abs((n.x, n.y, n.z)[k]) > 0.999999), None)
        if axis is not None:
            bend = [diag[k] for k in range(3) if k != axis]
            out["Ix"], out["Iy"] = _round(bend[0], 3), _round(bend[1], 3)
        else:
            out["Ix"] = out["Iy"] = None
        return out

    def op_dfm_report(a):
        """Unified manufacturability report: run the DFM checks a chosen process
        actually cares about and fold them into one verdict.

        This is the orchestration layer over the per-pillar tools — it owns the
        domain knowledge of *which* checks gate *which* process, so a caller asks
        one question ("can I injection-mould / 3D-print / cast this?") instead of
        wiring the trio by hand:

          * ``injection`` — draft + min wall + no undercut (two-plate mould);
          * ``casting``   — draft + min wall (heavier drafts/walls by default);
          * ``print``     — overhang (support-free) + min wall (additive).

        ``axis`` is the pull/build direction (default +Z). Per-process defaults
        can be overridden via ``min_draft`` / ``min_wall`` / ``max_overhang``.
        Returns each check's verdict, the issues found, and ``manufacturable``
        (True only when every gating check passes).

        args: name, process (injection|casting|print), axis, thresholds...
        """
        proc = str(a.get("process", "injection")).lower()
        axis = a.get("axis", (0, 0, 1))
        name = a["name"]
        defaults = {
            "injection": {"min_draft": 1.0, "min_wall": 1.0},
            "casting": {"min_draft": 2.0, "min_wall": 3.0},
            "print": {"max_overhang": 45.0, "min_wall": 0.8},
        }
        if proc not in defaults:
            raise ValueError("unknown process %r (injection|casting|print)" % proc)
        cfg = {**defaults[proc], **{k: a[k] for k in
               ("min_draft", "min_wall", "max_overhang") if k in a}}
        checks, issues = {}, []

        if proc in ("injection", "casting"):
            d = op_draft({"name": name, "pull": axis, "min_draft": cfg["min_draft"]})
            checks["draft"] = {"pass": d["draftable"], "min_draft_deg": cfg["min_draft"],
                               "insufficient": d["insufficient_draft"], "walls": d["walls"]}
            if not d["draftable"]:
                issues.append("%d face(s) below %.1f deg draft" %
                              (d["insufficient_draft"], cfg["min_draft"]))
            u = op_undercut({"name": name, "pull": axis})
            checks["undercut"] = {"pass": u["moldable"], "undercuts": u["undercuts"],
                                  "faces": u["undercut_faces"]}
            if not u["moldable"]:
                issues.append("%d undercut face(s) trap the mould" % u["undercuts"])
        if proc == "print":
            o = op_overhang({"name": name, "build": axis,
                             "max_overhang": cfg["max_overhang"]})
            checks["overhang"] = {"pass": o["printable"], "overhangs": o["overhangs"],
                                  "max_overhang_deg": cfg["max_overhang"],
                                  "faces": o["overhang_faces"]}
            if not o["printable"]:
                issues.append("%d face(s) overhang past %.0f deg (need support)" %
                              (o["overhangs"], cfg["max_overhang"]))

        t = op_thickness({"name": name, "min_wall": cfg["min_wall"]})
        checks["thickness"] = {"pass": len(t["thin_walls"]) == 0,
                               "min_wall_mm": cfg["min_wall"],
                               "min_thickness": t["min_thickness"],
                               "thin_walls": t["thin_walls"]}
        if t["thin_walls"]:
            issues.append("%d region(s) thinner than %.2f mm" %
                          (len(t["thin_walls"]), cfg["min_wall"]))

        ok = all(c["pass"] for c in checks.values())
        return {"process": proc,
                "axis": [_round(axis[0]), _round(axis[1]), _round(axis[2])],
                "manufacturable": ok, "checks": checks, "issues": issues}

    # ---- reverse engineering (庖丁解牛) ----------------------------------- #
    def op_compound(a):
        """Gather several solids into one multi-solid shape.

        The forward counterpart of ``decompose``: it makes the kind of single
        'monolithic' object (many disjoint solids, no part structure) that a
        downloaded model often is, without fusing the parts the way ``union``
        would. Round-trips with ``decompose``.
        """
        names = a["names"]
        comp = Part.makeCompound([_get(n).Shape for n in names])
        _put(a.get("out", "compound"), comp)
        return _metrics(comp)

    def op_decompose(a):
        """Take a monolithic model apart: split a shape into its constituent
        solids, register each as a named part, and fingerprint each one.

        A typical 'downloaded' model is one object holding many disjoint solids
        (a STEP assembly with no part names, a multi-lump import). ``Shape.Solids``
        recovers the individual parts. A single fused solid cannot be split this
        way — that needs feature segmentation — so we flag ``monolithic`` rather
        than silently pretending the model was already one part.
        """
        sh = _get(a["name"]).Shape
        prefix = a.get("prefix", a["name"] + "_part")
        parts = []
        for i, sol in enumerate(sh.Solids):
            nm = "%s%d" % (prefix, i + 1)
            _put(nm, sol)
            parts.append(dict(name=nm, **_signature(sol)))
        # largest part first — usually the frame/block the others hang off of
        parts.sort(key=lambda p: p["volume"], reverse=True)
        return {"source": a["name"], "parts": len(parts),
                "monolithic": len(parts) <= 1, "part_list": parts}

    def op_recognize(a):
        """Recover a primitive's design parameters from raw solid geometry.

        The parametric half of butchering-the-ox: once a part is recovered, name
        *what it is* and its driving dimensions, so it can be re-emitted as a
        clean parametric feature (``solid.box`` / ``cylinder`` / ``sphere``). A
        classification is accepted only if its closed-form volume reproduces the
        measured volume; otherwise the part is reported ``freeform`` rather than
        a primitive it merely resembles — no silent false positives.
        """
        sh = _get(a["name"]).Shape
        bb = sh.BoundBox
        vol = sh.Volume
        faces = sh.Faces
        kinds = {}
        for f in faces:
            k = f.Surface.__class__.__name__
            kinds[k] = kinds.get(k, 0) + 1
        base = {"name": a["name"], "faces": len(faces), "surfaces": kinds,
                "bbox_size": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)],
                "volume": _round(vol)}
        tol = float(a.get("tol", 1e-3))

        def accept(kind, params, pred_vol):
            rel = abs(pred_vol - vol) / max(abs(vol), 1e-9)
            return dict(base, type=kind, params=params,
                        predicted_volume=_round(pred_vol),
                        fit_error=_round(rel, 6), volume_match=rel < tol)

        if len(faces) == 1 and kinds.get("Sphere") == 1:
            r = float(faces[0].Surface.Radius)
            res = accept("sphere", {"radius": _round(r)}, 4.0 / 3.0 * math.pi * r ** 3)
            if res["volume_match"]:
                return res

        def _cap_height(ax):
            caps = [f for f in faces if f.Surface.__class__.__name__ == "Plane"]
            c0, c1 = caps[0].CenterOfMass, caps[1].CenterOfMass
            return abs((c1.x - c0.x) * ax[0] + (c1.y - c0.y) * ax[1] + (c1.z - c0.z) * ax[2])

        if len(faces) == 3 and kinds.get("Cylinder") == 1 and kinds.get("Plane") == 2:
            cyl = _cyl_axes(sh)[0]
            ax, r = cyl["dir"], cyl["radius"]
            h = _cap_height(ax)
            res = accept("cylinder", {"radius": _round(r), "height": _round(h),
                                      "axis": [_round(c, 6) for c in ax]}, math.pi * r * r * h)
            if res["volume_match"]:
                return res
        # bored cylinder (tube / bushing / rod-eye): two coaxial cylindrical
        # walls (outer + through-bore) closed by two annular planar caps.
        if len(faces) == 4 and kinds.get("Cylinder") == 2 and kinds.get("Plane") == 2:
            walls = _cyl_axes(sh)
            if len(walls) == 2:
                ax = walls[0]["dir"]
                radii = sorted(w["radius"] for w in walls)
                ri, ro = radii[0], radii[1]
                h = _cap_height(ax)
                res = accept("tube", {"outer_radius": _round(ro), "inner_radius": _round(ri),
                                      "height": _round(h), "axis": [_round(c, 6) for c in ax]},
                             math.pi * (ro * ro - ri * ri) * h)
                if res["volume_match"]:
                    return res
        if len(faces) == 6 and kinds.get("Plane") == 6:
            lx, ly, lz = bb.XLength, bb.YLength, bb.ZLength
            res = accept("box", {"length": _round(lx), "width": _round(ly), "height": _round(lz)},
                         lx * ly * lz)
            if res["volume_match"]:
                return res
        return dict(base, type="freeform", params=None, volume_match=False)

    def op_joints(a):
        """Infer revolute joints between parts from shared coaxial cylinders.

        A pin riding in a hole is two parts whose cylindrical faces are coaxial
        (parallel axes, the same line) with matching radius. That is exactly a
        revolute (hinge) joint, and its axis is the shared cylinder axis.
        """
        names = a.get("parts") or list(state.shapes.keys())
        axes = []
        for n in names:
            for r in _cyl_axes(_get(n).Shape):
                axes.append((n, r))
        rtol = float(a.get("radius_tol", 0.6))
        atol = float(a.get("axis_tol", 1e-3))
        gap = float(a.get("contact_gap", 1e-3))
        seen, joints = set(), []
        for i in range(len(axes)):
            for j in range(i + 1, len(axes)):
                na, ra = axes[i]
                nb, rb = axes[j]
                if na == nb:
                    continue
                ax = ra["dir"]
                dot = abs(ax[0] * rb["dir"][0] + ax[1] * rb["dir"][1] + ax[2] * rb["dir"][2])
                if abs(dot - 1.0) > 1e-3:
                    continue
                if abs(ra["radius"] - rb["radius"]) > rtol:
                    continue
                dx = tuple(rb["center"][k] - ra["center"][k] for k in range(3))
                cross = (dx[1] * ax[2] - dx[2] * ax[1],
                         dx[2] * ax[0] - dx[0] * ax[2],
                         dx[0] * ax[1] - dx[1] * ax[0])
                if math.sqrt(sum(v * v for v in cross)) > max(atol, 1e-4):
                    continue
                key = tuple(sorted((na, nb)))
                if key in seen:
                    continue
                seen.add(key)
                joints.append({
                    "type": "revolute", "parts": list(key),
                    "axis_point": [_round(c) for c in ra["center"]],
                    "axis_dir": [_round(c, 6) for c in ax],
                    "radius": _round((ra["radius"] + rb["radius"]) / 2.0, 4)})
        # prismatic (slider) joints: a part boxed in by planar contacts that
        # leave exactly one free translation axis.
        seen_p = set()
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                key = tuple(sorted((names[i], names[j])))
                if key in seen_p:
                    continue
                normals = _contact_normals(_get(key[0]).Shape, _get(key[1]).Shape, gap)
                fax = _free_axis(normals)
                if fax is None:
                    continue
                seen_p.add(key)
                joints.append({
                    "type": "prismatic", "parts": list(key),
                    "axis_dir": [_round(c, 6) for c in fax],
                    "contacts": len(normals)})
        return {"parts": names, "joints": len(joints), "joint_list": joints}

    def op_mechanism(a):
        """Assemble inferred joints into a kinematic graph and report mobility.

        Closes the reverse loop: decompose -> joints -> *how it moves*. The
        Kutzbach–Grübler criterion gives the degrees of freedom of the linkage
        from the link and joint counts, so a recovered slider-crank (4 links, 3
        revolute + 1 prismatic, all 1-DOF) must come out at mobility 1 — a single
        crank angle drives the whole chain.
        """
        names = a.get("parts") or list(state.shapes.keys())
        jspec = a.get("joint_list")
        if jspec is None:
            sub = {k: a[k] for k in ("radius_tol", "axis_tol", "contact_gap") if k in a}
            sub["parts"] = names
            jspec = op_joints(sub)["joint_list"]
        # spatial freedoms per lower-pair joint
        fmap = {"revolute": 1, "prismatic": 1, "cylindrical": 2, "spherical": 3, "planar": 3}
        n = len(names)
        lower = [j for j in jspec if j["type"] in ("revolute", "prismatic")]
        mobility_planar = 3 * (n - 1) - 2 * len(lower)
        mobility_spatial = 6 * (n - 1) - sum(6 - fmap.get(j["type"], 1) for j in jspec)
        graph = {nm: [] for nm in names}
        for j in jspec:
            x, y = j["parts"]
            graph[x].append(y)
            graph[y].append(x)
        types = {}
        for j in jspec:
            types[j["type"]] = types.get(j["type"], 0) + 1
        return {"links": n, "joints": len(jspec), "joint_types": types,
                "mobility_planar": mobility_planar,
                "mobility_spatial": mobility_spatial,
                "graph": {k: sorted(v) for k, v in graph.items()}}

    def op_drive(a):
        """Drive a recovered planar slider-crank through a crank angle.

        Closes the reverse loop end to end: the geometry told us the pivot O, the
        slider's guide line, and (from the joint points) the crank length
        R=|OA| and rod length L=|AB|; this turns that into motion. For crank
        angle theta the crank pin is A = O + R(cos, sin); the wrist pin B is where
        the rod of length L meets the guide line, i.e. the line/circle
        intersection — which handles an offset guide too, not just the centred
        case. Returns the pose with |AB| held at L so callers can place parts.
        """
        ox, oy = [float(v) for v in a["ground_point"][:2]]
        gx, gy = [float(v) for v in a.get("guide_point", (ox, oy))[:2]]
        ux, uy = [float(v) for v in a["guide_dir"][:2]]
        un = math.hypot(ux, uy) or 1.0
        ux, uy = ux / un, uy / un
        R = float(a["crank_len"])
        L = float(a["rod_len"])
        th = math.radians(float(a["angle"]))
        ax, ay = ox + R * math.cos(th), oy + R * math.sin(th)
        # B = G + t*u with |A-B| = L  ->  t^2 - 2 t (w.u) + (|w|^2 - L^2) = 0,
        # where w = A - G. Pick the branch reached from top dead centre (larger t).
        wx, wy = ax - gx, ay - gy
        b = wx * ux + wy * uy
        c = wx * wx + wy * wy - L * L
        disc = b * b - c
        if disc < 0:
            raise ValueError(
                "slider-crank cannot close: rod L=%g too short to reach the guide "
                "at crank angle %g deg (need L >= perpendicular offset)" % (L, float(a["angle"])))
        # two line/circle intersections along the guide. With ``guide_dir``
        # oriented toward the slider's travel, "far" (default) = the outboard
        # branch b+sqrt (standard slider-crank), "near" = the inboard branch.
        sq = math.sqrt(disc)
        t = (b - sq) if a.get("branch") == "near" else (b + sq)
        bx, by = gx + t * ux, gy + t * uy
        rod = math.hypot(ax - bx, ay - by)
        return {"A": [_round(ax), _round(ay)], "B": [_round(bx), _round(by)],
                "piston": _round(t), "crank_angle": _round(float(a["angle"])),
                "rod_len": _round(rod), "rod_len_ok": abs(rod - L) < 1e-6}

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
            # A STEP assembly imports as an App::Part container *plus* its leaf
            # parts; the container carries a compound Shape of all children, so
            # without this guard it registers as a phantom extra "solid" that
            # overlaps every real part and invents spurious joints. Skip any
            # object that groups other objects -- keep only the leaf solids.
            if getattr(o, "Group", None):
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
        "overhang": op_overhang, "section": op_section, "dfm_report": op_dfm_report,
        "compound": op_compound, "decompose": op_decompose, "joints": op_joints,
        "mechanism": op_mechanism, "drive": op_drive, "recognize": op_recognize,
        "list": op_list, "delete": op_delete, "export": op_export, "import_step": op_import_step,
    }
