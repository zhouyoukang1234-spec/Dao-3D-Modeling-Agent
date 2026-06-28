"""Direct BREP geometry operations (the ``solid.*`` tool group).

Runs inside freecadcmd. Every shape is a real ``Part::Feature`` object in the
live document, so it is immediately visible in the GUI, exportable, and
measurable. These are non-parametric (explicit BREP) operations — fast, robust,
and the workhorse for boolean modelling, measurement and interference checks.
The PartDesign feature-tree (editable, parametric) lives in ``freecad_parametric``.
"""

import hashlib
import itertools
import json
import math
import os

import FreeCAD as App
import Part

V = App.Vector


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _round(x, n=4):
    return round(float(x), n)


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n else tuple(v)


def _unit_v(v):
    """Normalise an ``App.Vector`` (returns it unchanged if it has zero length)."""
    n = v.Length
    return V(v.x / n, v.y / n, v.z / n) if n else v


def _vec(seq, default=(0, 0, 0)):
    if seq is None:
        seq = default
    return V(float(seq[0]), float(seq[1]), float(seq[2]))


def _proper_rotations():
    """The 24 axis-aligned proper rotations (signed permutation matrices, det +1).

    These are exactly the rigid rotations that map an axis-aligned inertia frame
    onto itself, so testing all of them aligns two bodies brought into their own
    principal frames regardless of any moment degeneracy.
    """
    mats = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            cols = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
            for i, p in enumerate(perm):
                cols[i][p] = signs[i]
            m = [[cols[r][c] for c in range(3)] for r in range(3)]
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                   - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            if det == 1:
                mats.append(App.Matrix(
                    m[0][0], m[0][1], m[0][2], 0,
                    m[1][0], m[1][1], m[1][2], 0,
                    m[2][0], m[2][1], m[2][2], 0, 0, 0, 0, 1))
    return mats


_PROPER_ROTATIONS = _proper_rotations()


def _face_entries(shape):
    """(centroid, area, surface-type) per face — the rotation/reflection-stable
    signature an isometry must preserve. Shared by the fast 'invariant' paths of
    ``symmetry`` and ``chirality`` (no BREP booleans, so it scales to high-face
    real parts the volumetric proof must refuse)."""
    return [(f.CenterOfMass, f.Area, f.Surface.__class__.__name__)
            for f in shape.Faces]


def _face_bijection(src, dst, tol, dtol):
    """True iff every ``src`` face maps one-to-one onto a ``dst`` face of the
    same surface type and (relatively) equal area whose centroid lands within
    ``dtol``. Returns ``(ok, max_centroid_deviation)``. A necessary condition
    for two face sets to be the same shape under an isometry — strong for real
    parts, but not a volumetric proof, so callers mark ``proven=False``."""
    if len(src) != len(dst):
        return False, None
    used = [False] * len(dst)
    maxdev = 0.0
    for c, ar, ty in src:
        best, bestd = None, None
        for i, (c0, ar0, ty0) in enumerate(dst):
            if used[i] or ty0 != ty:
                continue
            if abs(ar0 - ar) > tol * max(ar, ar0, 1e-9):
                continue
            d = c.distanceToPoint(c0)
            if bestd is None or d < bestd:
                best, bestd = i, d
        if best is None or bestd > dtol:
            return False, None
        used[best] = True
        if bestd > maxdev:
            maxdev = bestd
    return True, maxdev


def _guard_boolean_budget(op, body, a, default_max=120):
    """Refuse loudly (not with an opaque RPC timeout) when a boolean-proof
    operation would be too expensive.

    ``solid.symmetry`` / ``solid.chirality`` prove their result with dozens of
    full BREP boolean cuts, each O(faces). On a high-face real part (e.g. a
    toothed pulley with hundreds of cylindrical faces) that silently blows the
    request budget and surfaces as an unactionable timeout -- the very "silent
    failure" we forbid. So we check the face count up front and raise a clear,
    actionable error. ``max_faces`` tunes the ceiling; ``force=True`` runs it
    anyway when the caller knowingly accepts the cost.
    """
    if a.get("force"):
        return
    limit = int(a.get("max_faces", default_max))
    nf = len(body.Faces)
    if nf > limit:
        raise ValueError(
            "%s proves its result with O(faces) boolean cuts; this part has %d "
            "faces (> max_faces=%d) and would exceed the time budget. Defeature "
            "or simplify the part first, raise max_faces, or pass force=true to "
            "run it anyway." % (op, nf, limit))


def _metrics(shape):
    bb = shape.BoundBox
    data = {
        "valid": bool(shape.isValid()),
        "volume": _round(shape.Volume),
        "area": _round(shape.Area),
        "solids": len(shape.Solids),
        "shells": len(shape.Shells),
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


def _inertia_about(shape, density, about):
    """Mass and inertia tensor of a solid about a chosen reference point.

    FreeCAD's ``Shape.MatrixOfInertia`` is the *geometric* (density = 1, i.e.
    mass = volume) inertia tensor taken about the **centroid** — it silently
    ignores both the material density and where you actually want the moments.
    A real rigid-body calculation needs neither assumption: scale by density and
    shift the reference with the parallel-axis theorem

        I_P = I_cm + m(|d|^2 E - d (x) d),   d = com - P.

    ``about`` is ``"centroid"`` (default), ``"origin"`` or an explicit
    ``[x, y, z]`` point. Returns ``(mass, com, tensor3x3, ref_point)``.
    """
    m = float(shape.Volume) * density
    com = _center(shape)
    mat = shape.MatrixOfInertia
    tensor = [[mat.A11 * density, mat.A12 * density, mat.A13 * density],
              [mat.A12 * density, mat.A22 * density, mat.A23 * density],
              [mat.A13 * density, mat.A23 * density, mat.A33 * density]]
    if about in (None, "centroid", "center", "com"):
        ref = com
    elif about == "origin":
        ref = V(0, 0, 0)
    else:
        ref = _vec(about)
    d = (com.x - ref.x, com.y - ref.y, com.z - ref.z)
    d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
    for i in range(3):
        for j in range(3):
            tensor[i][j] += m * ((d2 if i == j else 0.0) - d[i] * d[j])
    return m, com, tensor, ref


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

    def op_inertia(a):
        """Full rigid-body mass properties of a solid.

        Returns the mass, centre of mass, the 3x3 inertia tensor about a chosen
        reference (``about`` = centroid / origin / explicit point), and the
        density-scaled principal moments + principal axes + radii of gyration.
        Unlike ``inspect`` (geometric diagonal only) this honours material
        ``density`` and the parallel-axis shift, so it is usable for real
        dynamics, rotor balancing and FEM mass lumping.
        """
        sh = _get(a["name"]).Shape
        if not sh.Solids:
            raise ValueError(
                "solid.inertia needs a solid (got a shell/compound with no "
                "volume); inertia is undefined without an enclosed mass")
        density = float(a.get("density", 1.0))
        about = a.get("about", "centroid")
        m, com, tensor, ref = _inertia_about(sh, density, about)
        pr = sh.PrincipalProperties  # always centroid-relative
        moments = [x * density for x in pr["Moments"]]
        axes = [pr["FirstAxisOfInertia"], pr["SecondAxisOfInertia"],
                pr["ThirdAxisOfInertia"]]
        return {
            "mass": _round(m), "density": density,
            "center_of_mass": [_round(com.x), _round(com.y), _round(com.z)],
            "about": [_round(ref.x), _round(ref.y), _round(ref.z)],
            "tensor": [[_round(v, 3) for v in row] for row in tensor],
            "principal_moments": [_round(x, 3) for x in moments],
            "principal_axes": [[_round(c, 6) for c in (ax.x, ax.y, ax.z)]
                               for ax in axes],
            "radius_of_gyration": [_round(x, 4) for x in pr["RadiusOfGyration"]],
        }

    def op_curvature(a):
        """Differential-geometry surface analysis of a solid — the curvature channel.

        For every face it samples the two principal curvatures k1,k2 (FreeCAD's
        ``curvatureAt``, units 1/mm) and from them the Gaussian K=k1*k2 and the
        mean H=(k1+k2)/2. This is the *quantitative* complement to ``recognize``
        (which only names the surface type): a sphere has K=1/R^2>0 everywhere
        (elliptic), a cylinder or cone is developable with K=0 (parabolic), a
        plane has K=H=0, a saddle/throat has K<0 (hyperbolic). Because curvature
        on a freeform or toroidal face varies, each face is scanned on a
        ``grid``x``grid`` (default 3) parameter lattice and the local extreme
        |curvature| is kept. The global minimum radius of curvature
        ``1/max|k|`` is the tightest feature in the part — it bounds the usable
        tool radius, the FEM mesh size and the printable detail — so it is
        reported up front instead of buried per-face.
        """
        sh = _get(a["name"]).Shape
        if not sh.Faces:
            raise ValueError(
                "solid.curvature needs a shape with faces (got a wire/vertex); "
                "curvature is undefined without a surface")
        grid = int(a.get("grid", 3))
        if grid < 1:
            raise ValueError("solid.curvature: grid must be >= 1, got %d" % grid)
        detail = []
        kmax_abs = 0.0
        kmax_face = None
        for idx, f in enumerate(sh.Faces):
            kind = f.Surface.__class__.__name__
            u0, u1, v0, v1 = f.ParameterRange
            k1c, k2c = f.curvatureAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
            loc = max(abs(k1c), abs(k2c))
            for i in range(grid):
                for j in range(grid):
                    u = u0 + (u1 - u0) * (i + 0.5) / grid
                    v = v0 + (v1 - v0) * (j + 0.5) / grid
                    s1, s2 = f.curvatureAt(u, v)
                    loc = max(loc, abs(s1), abs(s2))
            gauss = k1c * k2c
            mean = (k1c + k2c) / 2.0
            if gauss > 1e-9:
                cls = "elliptic"
            elif gauss < -1e-9:
                cls = "hyperbolic"
            elif loc > 1e-9:
                cls = "parabolic"
            else:
                cls = "planar"
            rec = {"face": idx, "surface": kind, "area": _round(f.Area),
                   "principal": [_round(k1c, 6), _round(k2c, 6)],
                   "gaussian": _round(gauss, 8), "mean": _round(mean, 6),
                   "class": cls,
                   "min_radius": _round(1.0 / loc, 4) if loc > 1e-9 else None}
            su = f.Surface
            if kind in ("Sphere", "Cylinder"):
                rec["radius"] = _round(float(su.Radius), 4)
            elif kind == "Toroid":
                rec["major_radius"] = _round(float(su.MajorRadius), 4)
                rec["minor_radius"] = _round(float(su.MinorRadius), 4)
            detail.append(rec)
            if loc > kmax_abs:
                kmax_abs, kmax_face = loc, idx
        return {
            "name": a["name"], "faces": len(detail),
            "max_abs_curvature": _round(kmax_abs, 6),
            "min_radius_of_curvature": _round(1.0 / kmax_abs, 4) if kmax_abs > 1e-9 else None,
            "tightest_face": kmax_face,
            "detail": detail,
        }

    def op_obb(a):
        """Oriented bounding box — recover a part's natural frame and true size.

        An imported or downloaded model arrives at some arbitrary placement, so
        its axis-aligned bounding box (``measure``'s ``bbox``) is inflated and
        tells you nothing about the real dimensions. This finds the part's own
        coordinate frame from the principal axes of inertia, re-expresses the
        solid in that frame with a *rigid* transform (so analytic faces stay
        analytic and the box stays tight — baking to NURBS would balloon it),
        and reports the tight oriented box: its three edge lengths, the three
        unit axes, the world-space centre and the fill ratio Vol/Vol_obb. The
        fill ratio is itself a closed-form fingerprint: 1 for a box, pi/4 for a
        cylinder, pi/6 for a sphere — the reverse half's first read on "what
        rough stock does this part come from, and how is it oriented".
        """
        sh = _get(a["name"]).Shape
        sols = sh.Solids
        if not sols:
            raise ValueError(
                "solid.obb needs a solid (got a shell/compound with no volume); "
                "the natural frame comes from the mass distribution")
        if len(sols) != 1:
            raise ValueError(
                "solid.obb expects a single solid (got %d); the natural frame is "
                "one body's principal axes - run solid.decompose first and orient "
                "each part" % len(sols))
        # work on the solid itself, not its enclosing compound: a boolean result
        # or an imported STEP arrives as a single-solid Part.Compound, and
        # Compound has no PrincipalProperties.
        body = sols[0]
        pr = body.PrincipalProperties
        a1 = _unit_v(pr["FirstAxisOfInertia"])
        a2 = pr["SecondAxisOfInertia"]
        a2 = _unit_v(a2 - a1 * a2.dot(a1))          # Gram-Schmidt: kill the
        a3 = a1.cross(a2)                            # degeneracy of symmetric
        mat = App.Matrix(a1.x, a1.y, a1.z, 0,        # bodies, force orthonormal
                         a2.x, a2.y, a2.z, 0,
                         a3.x, a3.y, a3.z, 0, 0, 0, 0, 1)
        local = body.copy()
        local.transformShape(mat, True, False)       # rigid: analytic stays tight
        bb = local.BoundBox
        dims = [bb.XLength, bb.YLength, bb.ZLength]
        cworld = mat.inverse().multiply(bb.Center)
        obb_vol = dims[0] * dims[1] * dims[2]
        ab = body.BoundBox
        return {
            "name": a["name"],
            "dimensions": [_round(d) for d in dims],
            "sorted_dimensions": [_round(d) for d in sorted(dims)],
            "axes": [[_round(c, 6) for c in (ax.x, ax.y, ax.z)]
                     for ax in (a1, a2, a3)],
            "obb_center": [_round(cworld.x), _round(cworld.y), _round(cworld.z)],
            "obb_volume": _round(obb_vol),
            "fill_ratio": _round(body.Volume / obb_vol, 6) if obb_vol > 1e-12 else None,
            "aabb_size": [_round(ab.XLength), _round(ab.YLength), _round(ab.ZLength)],
        }

    def op_symmetry(a):
        """Recover a solid's symmetry — mirror planes, rotation axes, inversion.

        Symmetry is design intent made visible: a balanced part betrays how few
        parameters truly drive it. Working in the natural frame (principal axes
        of inertia through the mass centroid), this probes each principal plane
        for mirror symmetry and each principal axis for n-fold rotational
        symmetry, by reflecting/rotating the real BREP and demanding the
        symmetric difference vanish (volume of S\\S' and S'\\S both ~0 relative
        to V) — a geometric proof, not a guess. It also tests central inversion.
        A box returns 3 mirror planes and 2-fold about each axis; a cylinder
        adds a continuous (highest-order) axis; an L-bracket keeps a single
        plane and is not centro-symmetric. ``orders`` (default 2..8) bounds the
        rotational search; hitting the top order is reported as ``continuous``.
        """
        sh = _get(a["name"]).Shape
        sols = sh.Solids
        if not sols:
            raise ValueError(
                "solid.symmetry needs a solid (got a shell/compound with no "
                "volume); symmetry is measured against an enclosed mass")
        tol = float(a.get("tol", 1e-3))
        orders = a.get("orders", [2, 3, 4, 5, 6, 8])
        if len(sols) != 1:
            raise ValueError(
                "solid.symmetry expects a single solid (got %d); the natural "
                "frame is one body's principal axes — analyse one part at a "
                "time" % len(sols))
        # work on the solid itself, not its enclosing compound: a boolean
        # result is a single-solid Part.Compound, and Compound has no
        # PrincipalProperties/CenterOfMass.
        body = sols[0]
        com = body.CenterOfMass
        pr = body.PrincipalProperties
        a1 = _unit_v(pr["FirstAxisOfInertia"])
        a2 = pr["SecondAxisOfInertia"]
        a2 = _unit_v(a2 - a1 * a2.dot(a1))
        a3 = a1.cross(a2)
        frame = [a1, a2, a3]
        vol = body.Volume

        def _ax(v):
            return [_round(v.x, 6), _round(v.y, 6), _round(v.z, 6)]

        method = a.get("method", "exact")
        if method not in ("exact", "invariant"):
            raise ValueError(
                "solid.symmetry method must be 'exact' (BREP boolean proof, "
                "default) or 'invariant' (fast face-centroid test, works at any "
                "face count but proven=False); got %r" % method)

        if method == "invariant":
            # An isometry that leaves the part invariant must permute its faces:
            # each face maps to a face of equal area and the same surface type,
            # with its centroid landing on the matched face's centroid. Checking
            # that bijection is O(faces^2) and uses no BREP booleans, so it scales
            # to the high-face real parts that the boolean proof must refuse. It
            # is a strong *necessary* condition, not a volumetric proof -- hence
            # ``proven=False``; use the default exact method when you need the
            # proof and the part is within budget.
            diag = body.BoundBox.DiagonalLength or 1.0
            dtol = max(tol * diag, 1e-6)
            entries = _face_entries(body)

            def _invariant_under(pointmap):
                mapped = [(pointmap(c), ar, ty) for c, ar, ty in entries]
                return _face_bijection(mapped, entries, tol, dtol)

            def _mirror_map(n):
                return lambda p: p - n * (2.0 * (p - com).dot(n))

            def _rot_map(ax, deg):
                rot = App.Rotation(ax, deg)
                return lambda p: com + rot.multVec(p - com)

            mirrors, devs = [], []
            for ax in frame:
                ok, dev = _invariant_under(_mirror_map(ax))
                if ok:
                    mirrors.append(_ax(ax))
                    devs.append(dev)
            top = max(orders)
            rotational = []
            for ax in frame:
                best = 1
                for n in orders:
                    ok, dev = _invariant_under(_rot_map(ax, 360.0 / n))
                    if ok:
                        best = n
                        devs.append(dev)
                if best > 1:
                    rotational.append({"axis": _ax(ax), "order": best,
                                       "continuous": best == top})
            inv_ok, inv_dev = _invariant_under(lambda p: com * 2.0 - p)
            if inv_ok:
                devs.append(inv_dev)
            return {
                "name": a["name"], "method": "face-invariant", "proven": False,
                "centroid": [_round(com.x), _round(com.y), _round(com.z)],
                "mirror_planes": mirrors, "mirror_plane_count": len(mirrors),
                "rotational_axes": rotational,
                "max_rotational_order": max((r["order"] for r in rotational), default=1),
                "point_symmetric": inv_ok,
                "max_face_deviation": _round(max(devs), 6) if devs else 0.0,
                "orders_tested": list(orders),
            }

        _guard_boolean_budget("solid.symmetry", body, a)

        def _symdiff(other):
            return max(body.cut(other).Volume, other.cut(body).Volume) / vol

        mirrors = [_ax(ax) for ax in frame if _symdiff(body.mirror(com, ax)) < tol]
        top = max(orders)
        rotational = []
        for ax in frame:
            best = 1
            for n in orders:
                r = body.copy()
                r.rotate(com, ax, 360.0 / n)
                if _symdiff(r) < tol:
                    best = n
            if best > 1:
                rotational.append({"axis": _ax(ax), "order": best,
                                   "continuous": best == top})
        mat = App.Matrix(-1, 0, 0, 2 * com.x, 0, -1, 0, 2 * com.y,
                         0, 0, -1, 2 * com.z, 0, 0, 0, 1)
        inv = body.copy()
        inv.transformShape(mat, True, True)
        return {
            "name": a["name"], "method": "exact-boolean", "proven": True,
            "centroid": [_round(com.x), _round(com.y), _round(com.z)],
            "mirror_planes": mirrors, "mirror_plane_count": len(mirrors),
            "rotational_axes": rotational,
            "max_rotational_order": max((r["order"] for r in rotational), default=1),
            "point_symmetric": _symdiff(inv) < tol,
            "orders_tested": list(orders),
        }

    def _shape_fingerprint(name):
        """Shared fingerprint kernel used by ``fingerprint`` and ``match``.

        Returns the raw (un-rounded) invariants so callers can both report them
        and compute distances. Raises loudly unless ``name`` is a single solid.
        """
        sh = _get(name).Shape
        sols = sh.Solids
        if len(sols) != 1:
            raise ValueError(
                "solid.fingerprint expects a single solid (got %d); fingerprint "
                "one part at a time" % len(sols))
        return _fingerprint_body(sols[0])

    def _fingerprint_body(body):
        """Fingerprint a raw solid ``Shape`` — the kernel behind ``fingerprint``,
        ``match`` and the model-library search (which feeds it shapes loaded from
        STEP files, not session objects)."""
        vol = body.Volume
        area = body.Area
        iso = area ** 3 / (vol * vol) if vol > 1e-12 else None
        pr = body.PrincipalProperties
        a1 = _unit_v(pr["FirstAxisOfInertia"])
        a2 = pr["SecondAxisOfInertia"]
        a2 = _unit_v(a2 - a1 * a2.dot(a1))
        a3 = a1.cross(a2)
        mat = App.Matrix(a1.x, a1.y, a1.z, 0, a2.x, a2.y, a2.z, 0,
                         a3.x, a3.y, a3.z, 0, 0, 0, 0, 1)
        loc = body.copy()
        loc.transformShape(mat, True, False)
        bb = loc.BoundBox
        dims = sorted([bb.XLength, bb.YLength, bb.ZLength])
        obb_aspect = [dims[1] / dims[0], dims[2] / dims[0]] if dims[0] > 1e-9 else [1.0, 1.0]
        mom = sorted(float(m) for m in pr["Moments"])
        mom_ratio = [mom[1] / mom[0], mom[2] / mom[0]] if mom[0] > 1e-9 else [1.0, 1.0]
        hist = {}
        for f in body.Faces:
            kind = f.Surface.__class__.__name__
            hist[kind] = hist.get(kind, 0) + 1
        counts = [len(body.Vertexes), len(body.Edges), len(body.Faces)]
        invariants = (
            round(iso, 3) if iso is not None else None,
            tuple(round(x, 4) for x in obb_aspect),
            tuple(round(x, 4) for x in mom_ratio),
            tuple(sorted(hist.items())),
            tuple(counts),
        )
        shape_key = hashlib.sha1(repr(invariants).encode()).hexdigest()[:16]
        return {"shape_key": shape_key, "iso": iso, "obb_aspect": obb_aspect,
                "mom_ratio": mom_ratio, "hist": hist, "counts": counts,
                "volume": vol, "area": area, "dims": dims}

    def op_fingerprint(a):
        """Pose- and scale-invariant shape signature — the model-library key.

        To integrate the world's models you must be able to *recognise the same
        part again* regardless of how it was placed, scaled or named. This
        distils a solid into invariants that survive rigid motion and uniform
        scaling: the dimensionless isoperimetric ratio A^3/V^2 (a sphere's
        minimum is 36*pi, a cube is 216), the sorted OBB aspect ratios, the
        ratios of the principal moments of inertia, the surface-type histogram
        and the V/E/F topology counts. Their hash is a ``shape_key`` — two
        copies of one design in any pose or size collapse to the same key, while
        genuinely different parts diverge — so a downloaded STEP can be matched
        against everything already seen instead of being re-modelled from zero.
        The raw size (volume, area, true OBB dimensions) is reported alongside.
        """
        fp = _shape_fingerprint(a["name"])
        return {
            "name": a["name"],
            "shape_key": fp["shape_key"],
            "isoperimetric": _round(fp["iso"], 4) if fp["iso"] is not None else None,
            "obb_aspect": [_round(x, 4) for x in fp["obb_aspect"]],
            "moment_ratio": [_round(x, 4) for x in fp["mom_ratio"]],
            "surface_histogram": fp["hist"],
            "topology": {"vertices": fp["counts"][0], "edges": fp["counts"][1],
                         "faces": fp["counts"][2]},
            "volume": _round(fp["volume"]),
            "area": _round(fp["area"]),
            "obb_dimensions": [_round(d) for d in fp["dims"]],
        }

    def _fp_distance(q, c):
        """Scale-invariant dissimilarity between two fingerprints (0 = same family)."""
        d = 0.0
        if q["iso"] and c["iso"]:
            d += abs(math.log(q["iso"] / c["iso"]))
        d += sum(abs(x - y) for x, y in zip(q["obb_aspect"], c["obb_aspect"]))
        d += sum(abs(x - y) for x, y in zip(q["mom_ratio"], c["mom_ratio"]))
        kinds = set(q["hist"]) | set(c["hist"])
        d += 0.1 * sum(abs(q["hist"].get(k, 0) - c["hist"].get(k, 0)) for k in kinds)
        return d

    def op_match(a):
        """Retrieve the closest-shaped solids to a query — search before building.

        反者道之动: the cheapest part to make is the one you already have. Given a
        query solid and a set of candidates (``against`` names, else every other
        solid in the document), this ranks them by a scale-invariant distance
        over their fingerprints — identical shape families (a box vs the same box
        in any pose or size) collapse to distance ~0 and share a ``shape_key``;
        a sphere sits far from a box. The top hit, when ``same_key`` is true, is
        a reuse candidate; ``volume_ratio`` tells you how to scale it to size.
        """
        q = _shape_fingerprint(a["name"])
        names = a.get("against")
        if names is None:
            names = [n for n in state.shapes if n != a["name"]]
        if not names:
            raise ValueError(
                "solid.match has nothing to compare against - pass 'against': "
                "[names] or load more solids into the document")
        ranked = []
        for nm in names:
            c = _shape_fingerprint(nm)
            ranked.append({
                "name": nm,
                "distance": _round(_fp_distance(q, c), 6),
                "same_key": c["shape_key"] == q["shape_key"],
                "volume_ratio": _round(c["volume"] / q["volume"], 4) if q["volume"] > 1e-12 else None,
            })
        ranked.sort(key=lambda r: r["distance"])
        return {"name": a["name"], "query_key": q["shape_key"],
                "candidates": len(ranked), "best": ranked[0]["name"],
                "ranking": ranked}

    def _candidate_record(body, path, label):
        """A JSON-serialisable fingerprint record: enough to rank against a query
        without re-opening the file, so a library can be indexed once and queried
        many times."""
        c = _fingerprint_body(body)
        return {"path": path, "label": label, "shape_key": c["shape_key"],
                "iso": c["iso"], "obb_aspect": c["obb_aspect"],
                "mom_ratio": c["mom_ratio"], "hist": c["hist"],
                "volume": c["volume"]}

    def _load_candidates(paths, skipped):
        """Fingerprint every solid in every model file in ``paths``. A file that
        will not load (corrupt download, unknown format, surface-only) is logged
        in ``skipped`` rather than aborting the whole scan."""
        recs = []
        for path in paths:
            if not os.path.isfile(path):
                skipped.append({"path": path, "reason": "no such file"})
                continue
            try:
                shp = Part.Shape()
                shp.read(path)
            except Exception as exc:
                skipped.append({"path": path, "reason": "unreadable: %s" % exc})
                continue
            sols = shp.Solids
            if not sols:
                skipped.append({"path": path, "reason": "no solid in file"})
                continue
            base = os.path.basename(path)
            for idx, body in enumerate(sols):
                label = base if len(sols) == 1 else "%s#%d" % (base, idx)
                try:
                    recs.append(_candidate_record(body, path, label))
                except Exception as exc:
                    skipped.append({"path": path, "reason": "fingerprint failed: %s" % exc})
        return recs

    def _rank_records(q, records):
        ranked = []
        for c in records:
            ranked.append({
                "path": c.get("path"), "label": c["label"],
                "shape_key": c["shape_key"],
                "distance": _round(_fp_distance(q, c), 6),
                "same_key": c["shape_key"] == q["shape_key"],
                "volume_ratio": _round(c["volume"] / q["volume"], 4) if q["volume"] > 1e-12 else None,
            })
        ranked.sort(key=lambda r: r["distance"])
        return ranked

    def _collect_paths(a, op):
        """Resolve a model-file list from explicit ``paths`` and/or a ``dir`` to
        walk (``recursive`` default True, filtered by ``exts``)."""
        paths = list(a.get("paths") or [])
        d = a.get("dir")
        if d:
            if not os.path.isdir(d):
                raise ValueError("%s 'dir' is not a directory: %r" % (op, d))
            exts = tuple(e.lower() for e in a.get(
                "exts", [".step", ".stp", ".brep", ".brp", ".iges", ".igs"]))
            if a.get("recursive", True):
                for root, _dirs, files in os.walk(d):
                    for fn in files:
                        if fn.lower().endswith(exts):
                            paths.append(os.path.join(root, fn))
            else:
                for fn in sorted(os.listdir(d)):
                    fp = os.path.join(d, fn)
                    if os.path.isfile(fp) and fn.lower().endswith(exts):
                        paths.append(fp)
        return paths

    def op_library_index(a):
        """Fingerprint a whole library of model files once and persist the index.

        整合市面一切 3D 资源 begins with cataloguing it: point this at a list of
        ``paths`` and/or a ``dir`` to walk, and it loads every model, fingerprints
        every solid, and (if ``out`` is given) writes a JSON index of scale-/pose-
        invariant signatures. ``solid.library_match`` can then query that index in
        memory without re-opening a single file — so a downloaded library is
        parsed once and reused forever. Junk files land in ``skipped``.
        """
        skipped = []
        paths = _collect_paths(a, "solid.library_index")
        if not paths:
            raise ValueError(
                "solid.library_index needs 'paths': [files] or 'dir': a folder "
                "of model files (STEP/BREP/IGES) to catalogue")
        records = _load_candidates(paths, skipped)
        if not records:
            raise ValueError(
                "solid.library_index found no usable solid in %d path(s); "
                "skipped=%r" % (len(paths), skipped))
        out = a.get("out")
        if out:
            with open(out, "w", encoding="utf-8") as fh:
                json.dump({"version": 1, "records": records}, fh)
        return {"indexed": len(records), "files": len(paths), "out": out,
                "shape_keys": sorted({r["shape_key"] for r in records}),
                "skipped": skipped}

    def op_library_match(a):
        """Search a library of model files (or a prebuilt index) for the part you
        already need.

        This is ``match`` pointed at the world instead of the open document:
        given a query solid and either a list of model file ``paths`` (STEP/BREP),
        a ``dir`` to walk, or a prebuilt ``index`` (from ``solid.library_index``),
        it ranks every catalogued solid against the query by the same scale-
        invariant distance. The point of integrating the world's models is exactly
        this — before modelling a part from zero, ask whether a downloaded library
        already holds the same shape family (a ``same_key`` hit), and if so how to
        scale it (``volume_ratio``). Files that fail to load are reported in
        ``skipped`` rather than aborting the search.
        """
        q = _shape_fingerprint(a["name"])
        skipped = []
        index = a.get("index")
        if index:
            if not os.path.isfile(index):
                raise ValueError(
                    "solid.library_match 'index' file not found: %r (build it "
                    "with solid.library_index)" % index)
            with open(index, encoding="utf-8") as fh:
                records = (json.load(fh) or {}).get("records") or []
            if not records:
                raise ValueError(
                    "solid.library_match index %r holds no records" % index)
        else:
            paths = _collect_paths(a, "solid.library_match")
            if not paths:
                raise ValueError(
                    "solid.library_match needs 'paths': [files], 'dir': a folder, "
                    "or 'index': a prebuilt library index to search")
            records = _load_candidates(paths, skipped)
            if not records:
                raise ValueError(
                    "solid.library_match found no usable solid in %d path(s); "
                    "skipped=%r" % (len(paths), skipped))
        ranked = _rank_records(q, records)
        return {"name": a["name"], "query_key": q["shape_key"],
                "matches": len(ranked), "best": ranked[0]["label"],
                "best_distance": ranked[0]["distance"],
                "ranking": ranked, "skipped": skipped}

    def _in_principal_frame(body):
        """Return a copy of ``body`` moved to its centroid and rotated so its
        principal axes coincide with the world axes (rigid, analytic-preserving)."""
        com = body.CenterOfMass
        pr = body.PrincipalProperties
        a1 = _unit_v(pr["FirstAxisOfInertia"])
        a2 = pr["SecondAxisOfInertia"]
        a2 = _unit_v(a2 - a1 * a2.dot(a1))
        a3 = a1.cross(a2)
        m = App.Matrix(a1.x, a1.y, a1.z, 0, a2.x, a2.y, a2.z, 0,
                       a3.x, a3.y, a3.z, 0, 0, 0, 0, 1)
        t = body.copy()
        t.translate(V(-com.x, -com.y, -com.z))
        t.transformShape(m, True, False)
        return t

    def op_chirality(a):
        """Decide whether a solid is the same as its mirror image — handedness.

        A fingerprint is mirror-blind: a left- and right-hand part share every
        scale/pose invariant, yet on the shop floor they are *different parts* —
        you cannot fit a left glove on a right hand. This settles it by proof. A
        solid is achiral iff it can be superimposed on its own mirror by a rigid
        motion. We reflect the body, bring both the original and the reflection
        into their principal frames, then try to align them with each of the 24
        axis-aligned proper rotations (which also covers any inertia-moment
        degeneracy); if the symmetric-difference volume vanishes for some
        rotation the part is achiral, otherwise it is chiral and its mirror is a
        genuinely distinct enantiomer. ``mirror_distance`` is that best residual.
        """
        sh = _get(a["name"]).Shape
        sols = sh.Solids
        if len(sols) != 1:
            raise ValueError(
                "solid.chirality expects a single solid (got %d); handedness is "
                "a property of one part" % len(sols))
        tol = float(a.get("tol", 1e-3))
        body = sols[0]
        method = a.get("method", "exact")
        if method not in ("exact", "invariant"):
            raise ValueError(
                "solid.chirality method must be 'exact' (BREP boolean proof, "
                "default) or 'invariant' (fast face-centroid test, works at any "
                "face count but proven=False); got %r" % method)
        base = _in_principal_frame(body)
        mir = body.copy()
        mir.transformShape(App.Matrix(1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, -1, 0, 0, 0, 0, 1), True, True)
        mir = _in_principal_frame(mir)

        if method == "invariant":
            # Achiral iff the mirror image can be brought back onto the original
            # by a proper rotation. Instead of a volumetric symmetric-difference
            # per rotation (dozens of BREP booleans), match the two face sets
            # under each of the 24 axis-aligned proper rotations -- O(faces^2)
            # each, no booleans, so it scales to high-face real parts. Necessary
            # condition only, hence proven=False.
            diag = base.BoundBox.DiagonalLength or 1.0
            dtol = max(tol * diag, 1e-6)
            baseE = _face_entries(base)
            mirE = _face_entries(mir)
            best = None
            for rm in _PROPER_ROTATIONS:
                rotated = [(rm.multVec(c), ar, ty) for c, ar, ty in mirE]
                ok, dev = _face_bijection(rotated, baseE, tol, dtol)
                if ok:
                    best = dev if best is None else min(best, dev)
                    if best <= dtol:
                        break
            achiral = best is not None
            return {
                "name": a["name"], "method": "face-invariant", "proven": False,
                "achiral": achiral, "chiral": not achiral,
                "mirror_distance": _round(best, 6) if achiral else None,
                "tol": tol,
            }

        _guard_boolean_budget("solid.chirality", body, a)
        vol = body.Volume
        best = None
        for rm in _PROPER_ROTATIONS:
            t = mir.copy()
            t.transformShape(rm, True, False)
            d = max(base.cut(t).Volume, t.cut(base).Volume) / vol
            best = d if best is None else min(best, d)
            if best < tol:
                break
        return {
            "name": a["name"], "method": "exact-boolean", "proven": True,
            "achiral": best < tol,
            "chiral": best >= tol,
            "mirror_distance": _round(best, 6),
            "tol": tol,
        }

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

        # torus / O-ring / ring-gasket: a single toroidal face.
        if len(faces) == 1 and kinds.get("Toroid") == 1:
            su = faces[0].Surface
            big, small = float(su.MajorRadius), float(su.MinorRadius)
            ax = _unit((su.Axis.x, su.Axis.y, su.Axis.z))
            res = accept("torus", {"major_radius": _round(big), "minor_radius": _round(small),
                                   "axis": [_round(c, 6) for c in ax]},
                         2.0 * math.pi ** 2 * big * small * small)
            if res["volume_match"]:
                return res

        def _cap_circle(f):
            for e in f.Edges:
                cur = e.Curve
                if cur.__class__.__name__ == "Circle":
                    c = cur.Center
                    return float(cur.Radius), (c.x, c.y, c.z)
            return None

        # full cone (nozzle / point): one conical face closed by one planar base.
        if len(faces) == 2 and kinds.get("Cone") == 1 and kinds.get("Plane") == 1:
            cone = next(f.Surface for f in faces if f.Surface.__class__.__name__ == "Cone")
            plane = next(f for f in faces if f.Surface.__class__.__name__ == "Plane")
            ax = _unit((cone.Axis.x, cone.Axis.y, cone.Axis.z))
            cc = _cap_circle(plane)
            if cc is not None:
                rad, ctr = cc
                ap = cone.Apex
                h = abs((ap.x - ctr[0]) * ax[0] + (ap.y - ctr[1]) * ax[1] + (ap.z - ctr[2]) * ax[2])
                res = accept("cone", {"radius": _round(rad), "height": _round(h),
                                      "axis": [_round(c, 6) for c in ax]},
                             math.pi * rad * rad * h / 3.0)
                if res["volume_match"]:
                    return res

        # truncated cone / frustum (tapered boss): one conical wall, two circular
        # caps of differing radius.
        if len(faces) == 3 and kinds.get("Cone") == 1 and kinds.get("Plane") == 2:
            cone = next(f.Surface for f in faces if f.Surface.__class__.__name__ == "Cone")
            ax = _unit((cone.Axis.x, cone.Axis.y, cone.Axis.z))
            caps = [c for c in (_cap_circle(f) for f in faces
                                if f.Surface.__class__.__name__ == "Plane") if c is not None]
            if len(caps) == 2:
                (r1, c1), (r2, c2) = caps
                big, small = max(r1, r2), min(r1, r2)
                h = abs((c2[0] - c1[0]) * ax[0] + (c2[1] - c1[1]) * ax[1] + (c2[2] - c1[2]) * ax[2])
                res = accept("frustum", {"base_radius": _round(big), "top_radius": _round(small),
                                         "height": _round(h), "axis": [_round(c, 6) for c in ax]},
                             math.pi * h * (big * big + big * small + small * small) / 3.0)
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
        # general prism: an extrusion of an arbitrary polygonal profile -- two
        # congruent parallel cap faces (normal = extrusion axis) and side walls
        # all parallel to that axis. Covers hex bars, L/T/U brackets, etc. The
        # cap's own face area times the cap separation must equal the volume.
        if len(faces) >= 5 and kinds.get("Plane") == len(faces):
            pf = _plane_faces(sh)
            for cand in pf:
                ax = cand["n"]
                caps = [f for f in pf if abs(abs(f["n"][0] * ax[0] + f["n"][1] * ax[1]
                                                 + f["n"][2] * ax[2]) - 1.0) < 1e-6]
                sides = [f for f in pf if abs(f["n"][0] * ax[0] + f["n"][1] * ax[1]
                                              + f["n"][2] * ax[2]) < 1e-6]
                if len(caps) != 2 or len(caps) + len(sides) != len(pf):
                    continue
                fcaps = [f for f in faces if f.Surface.__class__.__name__ == "Plane"
                         and abs(abs(f.Surface.Axis.normalize().dot(V(*ax))) - 1.0) < 1e-6]
                if len(fcaps) != 2 or abs(fcaps[0].Area - fcaps[1].Area) > 1e-6 * max(fcaps[0].Area, 1.0):
                    continue
                c0, c1 = caps[0]["p"], caps[1]["p"]
                length = abs(sum((c1[k] - c0[k]) * ax[k] for k in range(3)))
                area = fcaps[0].Area
                res = accept("prism", {"profile_area": _round(area), "length": _round(length),
                                       "axis": [_round(c, 6) for c in ax],
                                       "sides": len(sides)}, area * length)
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

    def op_coaxial(a):
        """Group parts that share a common cylinder axis line -- the rotational
        backbone of an assembly.

        A real rotary assembly (a gearmotor's output shaft carrying gears and
        bearings, a hinge stack) is a set of parts threaded onto one axis. Unlike
        ``joints``, this does *not* require matching radii or a pin-in-hole fit:
        any parts whose cylindrical faces are collinear count, so a thin shaft
        and the wide gears around it are recognised as one spindle. Each returned
        group lists the parts on that axis and the radii present (smallest =
        shaft/bore, larger = gears/hubs).
        """
        names = a.get("parts") or list(state.shapes.keys())
        atol = float(a.get("axis_tol", 1e-3))
        recs = []
        for n in names:
            for r in _cyl_axes(_get(n).Shape):
                recs.append((n, r))

        def collinear(ra, rb):
            ax = ra["dir"]
            if abs(abs(ax[0] * rb["dir"][0] + ax[1] * rb["dir"][1]
                       + ax[2] * rb["dir"][2]) - 1.0) > 1e-3:
                return False
            dx = tuple(rb["center"][k] - ra["center"][k] for k in range(3))
            cross = (dx[1] * ax[2] - dx[2] * ax[1], dx[2] * ax[0] - dx[0] * ax[2],
                     dx[0] * ax[1] - dx[1] * ax[0])
            return math.sqrt(sum(v * v for v in cross)) <= max(atol, 1e-4)

        groups = []        # each: {"dir","center","parts":set,"radii":[]}
        for n, r in recs:
            for g in groups:
                if collinear(g["rep"], r):
                    g["parts"].add(n)
                    g["radii"].append(r["radius"])
                    break
            else:
                groups.append({"rep": r, "dir": r["dir"], "center": r["center"],
                               "parts": {n}, "radii": [r["radius"]]})
        out = []
        for g in groups:
            if len(g["parts"]) < 2:           # a lone part is not an assembly axis
                continue
            out.append({"axis_dir": [_round(c, 6) for c in g["dir"]],
                        "axis_point": [_round(c) for c in g["center"]],
                        "parts": sorted(g["parts"]),
                        "radii": sorted(_round(x, 4) for x in set(g["radii"]))})
        out.sort(key=lambda x: len(x["parts"]), reverse=True)
        return {"parts": names, "groups": len(out), "group_list": out}

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

    def op_spatial_mobility(a):
        """General spatial Kutzbach-Grubler mobility for an arbitrary joint set.

        Where ``mechanism`` derives mobility from geometry-recovered revolute/
        prismatic joints, this evaluates the closed-form criterion for any list
        of named joint types in 3-D:

            M = 6 (n - 1) - sum_over_joints (6 - f_i)

        with the standard lower/higher-pair freedoms f: revolute/prismatic/
        helical = 1, cylindrical/universal/gear = 2, spherical/planar = 3. ``n``
        counts all links including ground. Optional ``idle_dof`` (e.g. the free
        spin of a binary S-S coupler) is subtracted to give the *effective*
        mobility. A non-positive gross M with a known-mobile mechanism is the
        classic Kutzbach *paradox* (special geometry beats the generic count) --
        we flag it rather than hide it.
        """
        fmap = {"revolute": 1, "prismatic": 1, "helical": 1, "screw": 1,
                "cylindrical": 2, "universal": 2, "gear": 2, "cam": 2,
                "spherical": 3, "planar": 3}
        n = int(a["links"])
        if n < 1:
            raise ValueError("need at least one link (ground)")
        joints = a["joints"]
        terms, total_f, j = [], 0, 0
        for spec in joints:
            t = spec["type"] if isinstance(spec, dict) else spec
            cnt = int(spec.get("count", 1)) if isinstance(spec, dict) else 1
            if t not in fmap:
                raise ValueError("unknown joint type %r" % t)
            total_f += fmap[t] * cnt
            j += cnt
            terms.append({"type": t, "count": cnt, "f": fmap[t]})
        gross = 6 * (n - 1) - sum((6 - fmap[x["type"]]) * x["count"] for x in terms)
        idle = int(a.get("idle_dof", 0))
        return {"links": n, "joints": j, "sum_freedoms": total_f,
                "mobility": gross, "idle_dof": idle, "effective_mobility": gross - idle,
                "overconstrained": gross <= 0, "joint_table": terms}

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

    def op_fourbar(a):
        """Drive a planar four-bar linkage through an input-crank angle.

        The four-bar is the workhorse planar linkage: a ground link ``g`` between
        two fixed pivots O2 and O4, an input crank ``a`` (O2->A), a coupler ``b``
        (A->B) and an output rocker ``c`` (O4->B). For input angle theta the
        crank pin is A = O2 + a(cos, sin); the coupler pin B is the intersection
        of the circle of radius ``b`` about A with the circle of radius ``c``
        about O4 (the loop-closure constraint). Two assembly modes exist -- the
        ``open`` and ``crossed`` circuits -- selected by ``branch``.

        Also reports the Grashof classification from the link lengths, which
        decides whether the input can fully rotate (crank-rocker) or only rock.
        """
        o2x, o2y = [float(v) for v in a.get("ground_point", (0.0, 0.0))[:2]]
        gdx, gdy = [float(v) for v in a.get("ground_dir", (1.0, 0.0))[:2]]
        gn = math.hypot(gdx, gdy) or 1.0
        gdx, gdy = gdx / gn, gdy / gn
        la = float(a["crank"])
        lb = float(a["coupler"])
        lc = float(a["rocker"])
        lg = float(a["ground"])
        th = math.radians(float(a["angle"]))
        # crank turns in the linkage plane; +y is the in-plane normal of ground dir
        nx, ny = -gdy, gdx
        ax = o2x + la * (math.cos(th) * gdx + math.sin(th) * nx)
        ay = o2y + la * (math.cos(th) * gdy + math.sin(th) * ny)
        o4x, o4y = o2x + lg * gdx, o2y + lg * gdy
        # circle(A, lb) ∩ circle(O4, lc)
        dx, dy = o4x - ax, o4y - ay
        d = math.hypot(dx, dy)
        if d > lb + lc + 1e-9 or d < abs(lb - lc) - 1e-9 or d == 0:
            raise ValueError(
                "four-bar cannot assemble at theta=%g deg: coupler %g and rocker "
                "%g cannot span A->O4 distance %g" % (float(a["angle"]), lb, lc, d))
        aa = (lb * lb - lc * lc + d * d) / (2 * d)
        h = math.sqrt(max(lb * lb - aa * aa, 0.0))
        mx, my = ax + aa * dx / d, ay + aa * dy / d
        sign = -1.0 if a.get("branch") == "crossed" else 1.0
        bx = mx + sign * h * (-dy / d)
        by = my + sign * h * (dx / d)
        # Grashof: s + l <= p + q  => at least one link fully rotates
        links = sorted([la, lb, lc, lg])
        grashof = (links[0] + links[3]) <= (links[1] + links[2]) + 1e-9
        coupler = math.hypot(bx - ax, by - ay)
        rocker = math.hypot(bx - o4x, by - o4y)
        return {"O2": [_round(o2x), _round(o2y)], "O4": [_round(o4x), _round(o4y)],
                "A": [_round(ax), _round(ay)], "B": [_round(bx), _round(by)],
                "crank_angle": _round(float(a["angle"])),
                "coupler_len": _round(coupler), "rocker_len": _round(rocker),
                "coupler_ok": abs(coupler - lb) < 1e-6, "rocker_ok": abs(rocker - lc) < 1e-6,
                "grashof": grashof,
                "grashof_type": ("crank-rocker" if grashof else "double-rocker")}

    def op_gearmesh(a):
        """Detect meshing gear pairs purely from geometry (reverse inference).

        The inter-axis complement to ``coaxial`` (which finds parts stacked on
        one shaft). Two gears mesh when their axes are *parallel but offset* and
        the centre distance equals the sum of their pitch radii (external mesh)
        or their difference (a pinion inside a ring -> internal mesh). With only
        recovered geometry we approximate the pitch radius by the gear-blank
        cylinder radius. Each candidate mesh reports the centre distance, both
        radii and the speed ratio r_i/r_j -- feed these straight into
        ``geartrain`` to get the train value.
        """
        names = a.get("parts") or list(state.shapes.keys())
        tol = float(a.get("tol", 1e-2))
        parts = []
        for n in names:
            cyls = _cyl_axes(_get(n).Shape)
            if cyls:
                parts.append((n, max(cyls, key=lambda c: c["radius"])))
        meshes = []
        for i in range(len(parts)):
            for j in range(i + 1, len(parts)):
                ni, ri = parts[i]
                nj, rj = parts[j]
                di, dj = ri["dir"], rj["dir"]
                if abs(abs(di[0] * dj[0] + di[1] * dj[1] + di[2] * dj[2]) - 1.0) > 1e-3:
                    continue                       # axes not parallel
                w = tuple(rj["center"][k] - ri["center"][k] for k in range(3))
                wd = w[0] * di[0] + w[1] * di[1] + w[2] * di[2]
                perp = tuple(w[k] - wd * di[k] for k in range(3))
                dist = math.sqrt(sum(v * v for v in perp))
                ext = ri["radius"] + rj["radius"]
                diff = abs(ri["radius"] - rj["radius"])
                reltol = tol * max(ext, 1.0)
                if dist > 1e-6 and abs(dist - ext) <= reltol:
                    kind = "external"
                elif diff > 1e-6 and abs(dist - diff) <= reltol:
                    kind = "internal"
                else:
                    continue
                meshes.append({"gears": [ni, nj], "type": kind,
                               "center_distance": _round(dist),
                               "radii": [_round(ri["radius"]), _round(rj["radius"])],
                               "ratio": _round(ri["radius"] / rj["radius"], 6)})
        return {"parts": names, "meshes": len(meshes), "mesh_list": meshes}

    def op_geneva(a):
        """External Geneva (Maltese-cross) intermittent indexer.

        A continuously turning drive crank with one pin enters the radial slots
        of an ``n``-slot wheel, rotating it one step (360/n deg) then leaving it
        locked for the rest of the revolution -- intermittent indexing. For a
        right-angle entry/exit (no shock) the centre distance d, crank-pin radius
        r and slot count obey r = d sin(pi/n); let m = d/r = 1/sin(pi/n). With
        the drive angle ``alpha`` measured from the symmetric centre, the driven
        angle is

            phi(alpha) = atan2(sin alpha, m - cos alpha)

        swinging from -pi/n to +pi/n over the engagement (so each index is
        exactly 2 pi/n). The crank is engaged only for |alpha| <= 90 - 180/n
        degrees; outside that the wheel is geometrically locked (dwell). The
        angular-velocity ratio is (m cos a - 1)/(m^2 - 2 m cos a + 1), peaking at
        1/(m-1) at centre.
        """
        n = int(a["slots"])
        if n < 3:
            raise ValueError("Geneva wheel needs at least 3 slots")
        s = math.sin(math.pi / n)
        if "center_distance" in a:
            d = float(a["center_distance"])
            r = d * s
        elif "crank_radius" in a:
            r = float(a["crank_radius"])
            d = r / s
        else:
            raise ValueError("geneva needs center_distance or crank_radius")
        m = d / r                                       # = 1/sin(pi/n)
        alpha0 = 90.0 - 180.0 / n                        # half engagement (deg)
        out = {"slots": n, "center_distance": _round(d), "crank_radius": _round(r),
               "index_angle": _round(360.0 / n), "engagement_angle": _round(2 * alpha0),
               "max_velocity_ratio": _round(1.0 / (m - 1.0), 6)}
        if "angle" in a:
            al = float(a["angle"])
            ar = math.radians(al)
            if abs(al) <= alpha0 + 1e-9:
                phi = math.degrees(math.atan2(math.sin(ar), m - math.cos(ar)))
                vr = (m * math.cos(ar) - 1.0) / (m * m - 2 * m * math.cos(ar) + 1.0)
                out.update(engaged=True, driven_angle=_round(phi, 6), velocity_ratio=_round(vr, 6))
            else:
                locked = 180.0 / n if al > 0 else -180.0 / n
                out.update(engaged=False, driven_angle=_round(locked, 6), velocity_ratio=0.0)
        return out

    def op_planetary(a):
        """Solve a sun-planet-ring epicyclic set via the Willis equation.

        An epicyclic (planetary) train has a moving carrier, so the ordinary
        train value is taken *relative to the carrier*:

            (w_sun - w_carrier) / (w_ring - w_carrier) = - N_ring / N_sun

        Three members (sun, ring, carrier) and one constraint -> 2 DOF: give any
        two of the three speeds and the third is solved exactly. Tooth counts
        obey N_ring = N_sun + 2 N_planet; if ``teeth_planet`` is given it is
        checked. Classic operating modes fall straight out: ring fixed gives the
        reduction w_sun/w_carrier = 1 + N_ring/N_sun; carrier fixed degenerates
        to the ordinary train value -N_sun/N_ring (sun -> ring).
        """
        ns = float(a["teeth_sun"])
        nr = float(a["teeth_ring"])
        if ns <= 0 or nr <= 0:
            raise ValueError("sun/ring teeth must be positive")
        if nr <= ns:
            raise ValueError("ring must have more teeth than sun (N_ring > N_sun)")
        np_ = a.get("teeth_planet")
        if np_ is not None and abs(float(np_) - (nr - ns) / 2.0) > 1e-9:
            raise ValueError("tooth constraint violated: N_ring != N_sun + 2*N_planet")
        ws, wr, wc = a.get("sun_rpm"), a.get("ring_rpm"), a.get("carrier_rpm")
        known = [x is not None for x in (ws, wr, wc)]
        if sum(known) != 2:
            raise ValueError("give exactly two of sun_rpm/ring_rpm/carrier_rpm")
        if wc is None:                       # carrier from sun & ring
            wc = (ns * float(ws) + nr * float(wr)) / (ns + nr)
        elif ws is None:                     # sun from carrier & ring
            wc = float(wc)
            ws = wc - (nr / ns) * (float(wr) - wc)
        else:                                # ring from carrier & sun
            wc = float(wc)
            wr = wc - (ns / nr) * (float(ws) - wc)
        ws, wr, wc = float(ws), float(wr), float(wc)
        return {"teeth_sun": _round(ns), "teeth_ring": _round(nr),
                "teeth_planet": _round((nr - ns) / 2.0),
                "sun_rpm": _round(ws), "ring_rpm": _round(wr),
                "carrier_rpm": _round(wc)}

    def op_cam(a):
        """Disc-cam follower lift for a rise-dwell-fall-dwell (RDFD) profile.

        A rotating cam pushes a follower through a programmed motion. The lift
        over the rise is set by a displacement *law*; we support the two classic
        smooth ones:

          * ``harmonic``  s = S/2 (1 - cos(pi u))           -- zero velocity at
            the ends but a finite (step) acceleration there;
          * ``cycloidal`` s = S (u - sin(2 pi u)/(2 pi))    -- zero velocity AND
            zero acceleration at the ends (shock-free), u = theta_local/beta.

        The cam angle theta is split into rise (``rise_angle``), top dwell
        (``dwell_angle``), fall (``fall_angle``, mirror of the rise) and the
        remaining bottom dwell. Returns the follower lift, the analytic velocity
        d(lift)/d(theta) (so callers can verify end-smoothness) and the radial
        cam-profile radius base_radius + lift -- the parametric design output.
        """
        S = float(a["rise"])
        law = a.get("law", "cycloidal")
        br = float(a.get("rise_angle", 90.0))
        bd = float(a.get("dwell_angle", 90.0))
        bf = float(a.get("fall_angle", br))
        base = float(a.get("base_radius", 0.0))
        th = float(a["angle"]) % 360.0

        def lift_frac(u):
            if law == "harmonic":
                return 0.5 * (1.0 - math.cos(math.pi * u))
            if law == "cycloidal":
                return u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
            raise ValueError("unknown cam law %r (use harmonic|cycloidal)" % law)

        def dfrac(u, beta):                       # d(frac)/d(theta_deg)
            if law == "harmonic":
                return 0.5 * math.pi * math.sin(math.pi * u) / beta
            return (1.0 - math.cos(2.0 * math.pi * u)) / beta

        def d2frac(u, beta):                      # d2(frac)/d(theta_deg)^2
            if law == "harmonic":
                return 0.5 * math.pi * math.pi * math.cos(math.pi * u) / (beta * beta)
            return 2.0 * math.pi * math.sin(2.0 * math.pi * u) / (beta * beta)

        if th < br:
            u = th / br if br else 0.0
            lift, vel, acc, seg = S * lift_frac(u), S * dfrac(u, br), S * d2frac(u, br), "rise"
        elif th < br + bd:
            lift, vel, acc, seg = S, 0.0, 0.0, "dwell-top"
        elif th < br + bd + bf:
            u = (th - br - bd) / bf if bf else 0.0
            lift, vel, acc, seg = S * (1.0 - lift_frac(u)), -S * dfrac(u, bf), -S * d2frac(u, bf), "fall"
        else:
            lift, vel, acc, seg = 0.0, 0.0, 0.0, "dwell-bottom"
        return {"angle": _round(th), "segment": seg, "law": law,
                "lift": _round(lift, 6), "velocity": _round(vel, 6),
                "acceleration": _round(acc, 6),
                "cam_radius": _round(base + lift, 6), "rise": _round(S)}

    def op_cam_profile(a):
        """Build the real cam-disc solid from the displacement law (parametric).

        Turns ``cam``'s analytic lift into actual CAD geometry: it samples the
        radial profile r(theta) = base_radius + lift(theta) every ``step``
        degrees, sweeps a closed polar polygon, faces it and extrudes by
        ``thickness`` to a solid stored under ``name``. This is the parametric
        design output -- regenerate any cam by changing rise/angles/law. The
        returned bounds let callers confirm the kernel-built profile matches the
        law (max radius = base + rise, min radius = base).
        """
        S = float(a["rise"])
        base = float(a.get("base_radius", 0.0))
        thick = float(a.get("thickness", 5.0))
        step = float(a.get("step", 2.0))
        if base <= 0 or thick <= 0 or step <= 0:
            raise ValueError("base_radius, thickness and step must be positive")
        camargs = {k: a[k] for k in ("law", "rise_angle", "dwell_angle", "fall_angle")
                   if k in a}
        pts, th = [], 0.0
        while th < 360.0 - 1e-9:
            cr = op_cam(dict(camargs, rise=S, base_radius=base, angle=th))["cam_radius"]
            pts.append(V(cr * math.cos(math.radians(th)), cr * math.sin(math.radians(th)), 0.0))
            th += step
        pts.append(pts[0])
        sol = Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, thick))
        _put(a["name"], sol)
        radii = [math.hypot(p.x, p.y) for p in pts[:-1]]
        return {"name": a["name"], "samples": len(pts) - 1,
                "min_radius": _round(min(radii), 6), "max_radius": _round(max(radii), 6),
                "base_radius": _round(base), "rise": _round(S),
                "thickness": _round(thick), "volume": _round(sol.Volume)}

    def op_rackpinion(a):
        """Rack-and-pinion: convert pinion rotation to/from rack travel.

        The drivetrain's last stage often turns rotation into straight-line
        motion. A pinion of pitch radius r rolls without slipping on a rack, so
        the rack advances exactly the pitch-circle arc: x = r * theta (theta in
        radians), and one full pinion revolution moves the rack one pitch
        circumference 2*pi*r. The pitch radius may be given directly or as
        module * teeth / 2. Give ``angle`` (deg) to get rack travel, or
        ``travel`` to get the pinion angle -- the map is exact and invertible.
        """
        if "pitch_radius" in a:
            r = float(a["pitch_radius"])
        elif "module" in a and "teeth" in a:
            r = float(a["module"]) * float(a["teeth"]) / 2.0
        else:
            raise ValueError("rackpinion needs pitch_radius or (module, teeth)")
        if r <= 0:
            raise ValueError("pitch radius must be positive")
        out = {"pitch_radius": _round(r), "travel_per_rev": _round(2 * math.pi * r)}
        if "angle" in a:
            th = math.radians(float(a["angle"]))
            out["angle"] = _round(float(a["angle"]))
            out["travel"] = _round(r * th)
        elif "travel" in a:
            x = float(a["travel"])
            out["travel"] = _round(x)
            out["angle"] = _round(math.degrees(x / r))
        else:
            raise ValueError("rackpinion needs angle (deg) or travel")
        return out

    def op_geartrain(a):
        """Compute the train value (speed ratio) of an ordinary gear train.

        A gear train is a sequence of meshes. Each mesh multiplies the train
        value by (driver_teeth / driven_teeth); an *external* mesh also flips the
        sign (the gears spin opposite ways), while an *internal*/ring mesh keeps
        it. Idlers fall out of the magnitude automatically (their teeth appear
        once as driven and once as driver) but still flip the sign; compound
        gears (two gears keyed to one shaft) are expressed as consecutive meshes.

        train_value e = ω_out/ω_in = Π (± N_driver / N_driven). Tooth counts may
        be replaced by pitch radii -- the ratio is identical.
        """
        meshes = a["meshes"]
        if not meshes:
            raise ValueError("gear train needs at least one mesh")
        e = 1.0
        for m in meshes:
            drv = float(m.get("driver", m.get("driver_radius")))
            dvn = float(m.get("driven", m.get("driven_radius")))
            if drv <= 0 or dvn <= 0:
                raise ValueError("gear teeth/radius must be positive: %r" % (m,))
            f = drv / dvn
            e *= f if m.get("internal") else -f
        inp = float(a.get("input_rpm", 1.0))
        return {"stages": len(meshes), "train_value": _round(e, 6),
                "ratio_magnitude": _round(abs(e), 6),
                "reduction": _round(1.0 / abs(e), 6) if e else None,
                "reversing": e < 0, "input_rpm": _round(inp),
                "output_rpm": _round(inp * e)}

    def op_reverse(a):
        """One-shot 'butcher the ox': the whole reverse chain in a single call.

        Given a monolithic model (``name``) or an explicit ``parts`` list, this
        orchestrates decompose -> recognize each part (a parametric BOM naming
        what every part *is* and its driving dimensions) -> infer joints ->
        Kutzbach mobility. The DFM layer has ``dfm_report`` as its editorial
        front; this is the reverse pipeline's. Parts that are not clean
        primitives come back ``freeform`` rather than mislabelled.
        """
        if a.get("parts"):
            names = list(a["parts"])
        else:
            names = [p["name"] for p in op_decompose({"name": a["name"],
                                                       "prefix": a.get("prefix", a["name"] + "_part")})["part_list"]]
        bom = []
        for nm in names:
            r = op_recognize({"name": nm, "tol": a.get("tol", 1e-3)})
            bom.append({"name": nm, "type": r["type"], "params": r["params"],
                        "volume": r["volume"]})
        jspec = op_joints({"parts": names})["joint_list"]
        mech = op_mechanism({"parts": names, "joint_list": jspec})
        coax = op_coaxial({"parts": names})["group_list"]
        meshes = op_gearmesh({"parts": names})["mesh_list"]
        kinds = {}
        for e in bom:
            kinds[e["type"]] = kinds.get(e["type"], 0) + 1
        return {"parts": len(names), "part_types": kinds, "bom": bom,
                "joints": len(jspec), "joint_list": jspec,
                "joint_types": mech["joint_types"],
                "coaxial_groups": coax,
                "gear_meshes": meshes,
                "mobility_planar": mech["mobility_planar"],
                "mobility_spatial": mech["mobility_spatial"]}

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
        # Honour the caller's requested handle for a single-solid import: a
        # downloaded part arrives labelled by its vendor ("Aluminum Timing
        # Pulley v1"), but a caller that said name="q" expects to refer to it as
        # "q". Silently keeping the STEP label made every downstream op fail with
        # "no such solid: q". For multi-solid assemblies a single name is
        # ambiguous, so we refuse loudly and point at ``out`` (which bundles
        # them) instead of guessing.
        name = a.get("name")
        if name:
            if len(imported) == 1:
                state.shapes[name] = state.shapes.pop(imported[0])
                imported = [name]
            else:
                raise ValueError(
                    "import_step got name=%r but the file holds %d solids; a "
                    "single name is ambiguous - drop 'name' to keep each part's "
                    "label, or use 'out' to bundle them into one handle"
                    % (name, len(imported)))
        out = a.get("out")
        if out:
            # Bundle every imported leaf solid into one named handle so the
            # reverse pipeline can butcher a downloaded assembly directly --
            # ``reverse`` will ``decompose`` it back into the same parts. With no
            # leaves (a surface/shell-only STEP) we refuse rather than emit an
            # empty compound that would silently "succeed".
            if not imported:
                raise ValueError("no solids imported from %r; cannot build %r "
                                 "(surface/shell-only STEP?)" % (a["path"], out))
            comp = Part.makeCompound([_get(n).Shape for n in imported])
            _put(out, comp)
        return {"imported": imported, "out": out, "solids": len(imported)}

    return {
        "box": op_box, "cylinder": op_cylinder, "sphere": op_sphere, "cone": op_cone,
        "torus": op_torus, "extrude": op_extrude, "revolve": op_revolve, "loft": op_loft,
        "shell": op_shell, "translate": op_translate, "rotate": op_rotate, "mirror": op_mirror,
        "union": lambda a: _boolean("union", a), "cut": lambda a: _boolean("cut", a),
        "common": lambda a: _boolean("common", a), "fillet": op_fillet, "chamfer": op_chamfer,
        "pattern_linear": op_pattern_linear, "pattern_polar": op_pattern_polar,
        "measure": op_measure, "inspect": op_inspect, "inertia": op_inertia,
        "curvature": op_curvature, "obb": op_obb, "symmetry": op_symmetry,
        "fingerprint": op_fingerprint, "match": op_match, "chirality": op_chirality,
        "library_match": op_library_match, "library_index": op_library_index,
        "interference": op_interference,
        "draft": op_draft, "thickness": op_thickness, "undercut": op_undercut,
        "overhang": op_overhang, "section": op_section, "dfm_report": op_dfm_report,
        "compound": op_compound, "decompose": op_decompose, "joints": op_joints,
        "mechanism": op_mechanism, "drive": op_drive, "recognize": op_recognize,
        "reverse": op_reverse, "coaxial": op_coaxial, "fourbar": op_fourbar,
        "geartrain": op_geartrain, "gearmesh": op_gearmesh,
        "rackpinion": op_rackpinion, "cam": op_cam, "planetary": op_planetary,
        "geneva": op_geneva, "cam_profile": op_cam_profile,
        "spatial_mobility": op_spatial_mobility,
        "list": op_list, "delete": op_delete, "export": op_export, "import_step": op_import_step,
    }
