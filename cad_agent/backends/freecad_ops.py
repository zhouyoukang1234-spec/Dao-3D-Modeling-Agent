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
import urllib.parse
import urllib.request

import numpy as np

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
        sols = getattr(shape, "Solids", None)
        if sols:
            tv = sum(s.Volume for s in sols) or 1.0
            return V(sum(s.CenterOfMass.x * s.Volume for s in sols) / tv,
                     sum(s.CenterOfMass.y * s.Volume for s in sols) / tv,
                     sum(s.CenterOfMass.z * s.Volume for s in sols) / tv)
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
    if about in (None, "centroid", "center", "com"):
        ref = com
    elif about == "origin":
        ref = V(0, 0, 0)
    else:
        ref = _vec(about)
    # Boolean results are routinely a Part.Compound, which (unlike a single
    # Solid) exposes no MatrixOfInertia. Accumulate each constituent solid's
    # centroidal tensor and parallel-axis-shift it to the common reference, so
    # the op works on cut/union/multi-body shapes, not just primitives.
    solids = shape.Solids or [shape]
    tensor = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for s in solids:
        mi = float(s.Volume) * density
        ci = s.CenterOfMass
        mat = s.MatrixOfInertia
        ti = [[mat.A11 * density, mat.A12 * density, mat.A13 * density],
              [mat.A12 * density, mat.A22 * density, mat.A23 * density],
              [mat.A13 * density, mat.A23 * density, mat.A33 * density]]
        d = (ci.x - ref.x, ci.y - ref.y, ci.z - ref.z)
        d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
        for i in range(3):
            for j in range(3):
                tensor[i][j] += ti[i][j] + mi * ((d2 if i == j else 0.0)
                                                 - d[i] * d[j])
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
        # Principal axes are an eigendecomposition of the *centroidal* tensor.
        # Diagonalising it ourselves (rather than Shape.PrincipalProperties)
        # keeps this working for booleans/multi-body compounds, which expose no
        # PrincipalProperties; eigh gives ascending real eigenvalues for the
        # symmetric tensor.
        _, _, tcm, _ = _inertia_about(sh, density, "centroid")
        vals, vecs = np.linalg.eigh(np.array(tcm))
        moments = [float(v) for v in vals]
        axes = [[float(vecs[r][c]) for r in range(3)] for c in range(3)]
        rog = [math.sqrt(v / m) if m > 0 and v > 0 else 0.0 for v in moments]
        return {
            "mass": _round(m), "density": density,
            "center_of_mass": [_round(com.x), _round(com.y), _round(com.z)],
            "about": [_round(ref.x), _round(ref.y), _round(ref.z)],
            "tensor": [[_round(v, 3) for v in row] for row in tensor],
            "principal_moments": [_round(x, 3) for x in moments],
            "principal_axes": [[_round(c, 6) for c in ax] for ax in axes],
            "radius_of_gyration": [_round(x, 4) for x in rog],
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

    def _feature_signature(body):
        """A compact, JSON-serialisable feature read of one library body: how many
        holes/bosses, their diameters, the through count, and the edge-break tally
        -- enough to ask a catalogue "which parts carry a phi3.2 through hole?"
        without re-opening or re-analysing a file. Computed by temporarily binding
        the body to a name and reusing the closed-form ``holes`` / ``fillets``
        operators, so the index agrees with what design_intent would report."""
        tmp = "__lib_feat_tmp__"
        _put(tmp, body)
        try:
            h = op_holes({"name": tmp})
            fb = op_fillets({"name": tmp})
        finally:
            existing = state.shapes.pop(tmp, None)
            if existing and doc.getObject(existing):
                doc.removeObject(existing)
        holes = [x for x in h["features"] if x["kind"] == "hole"]
        bosses = [x for x in h["features"] if x["kind"] == "boss"]
        return {
            "holes": len(holes), "through_holes": h["through_holes"],
            "bosses": len(bosses),
            "hole_diams": sorted(_round(2.0 * x["radius"]) for x in holes),
            "boss_diams": sorted(_round(2.0 * x["radius"]) for x in bosses),
            "rounds": fb["round_count"], "fillets": fb["fillet_count"],
            "blend_radii": fb["radii"],
        }

    def _candidate_record(body, path, label, features=False):
        """A JSON-serialisable fingerprint record: enough to rank against a query
        without re-opening the file, so a library can be indexed once and queried
        many times. With ``features=True`` it also carries a feature signature so
        the catalogue can be searched by mounting feature, not just shape."""
        c = _fingerprint_body(body)
        rec = {"path": path, "label": label, "shape_key": c["shape_key"],
               "iso": c["iso"], "obb_aspect": c["obb_aspect"],
               "mom_ratio": c["mom_ratio"], "hist": c["hist"],
               "volume": c["volume"]}
        if features:
            rec["features"] = _feature_signature(body)
        return rec

    def _load_candidates(paths, skipped, features=False):
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
                    recs.append(_candidate_record(body, path, label, features))
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
        want_features = bool(a.get("features"))
        records = _load_candidates(paths, skipped, features=want_features)
        if not records:
            raise ValueError(
                "solid.library_index found no usable solid in %d path(s); "
                "skipped=%r" % (len(paths), skipped))
        out = a.get("out")
        if out:
            with open(out, "w", encoding="utf-8") as fh:
                json.dump({"version": 1, "features": want_features,
                           "records": records}, fh)
        return {"indexed": len(records), "files": len(paths), "out": out,
                "features": want_features,
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

    def op_library_query(a):
        """Search a feature-indexed library by *mounting feature*, not by shape.

        ``library_match`` answers "what in the world looks like this part?";
        this answers the complementary, intent-level question a designer actually
        asks -- "which catalogued parts carry the feature I need?": every part
        with a phi3.2 through-hole, with at least two holes, with a boss. It reads
        an ``index`` built by ``solid.library_index`` with ``features=True`` (or
        scans ``paths`` / ``dir`` on the fly), then filters the feature signatures
        by the given spec. Predicates (all optional, AND-combined): ``min_holes``,
        ``through`` (require >=1 through-hole), ``boss`` (True/False require / for
        bid a boss), ``hole_diam`` with ``diam_tol`` (a hole of that diameter),
        ``boss_diam`` with ``diam_tol``. An index with no feature signatures is a
        loud error -- it must be rebuilt with features=True.
        """
        skipped = []
        index = a.get("index")
        if index:
            if not os.path.isfile(index):
                raise ValueError(
                    "solid.library_query 'index' file not found: %r (build it "
                    "with solid.library_index features=True)" % index)
            with open(index, encoding="utf-8") as fh:
                records = (json.load(fh) or {}).get("records") or []
        else:
            paths = _collect_paths(a, "solid.library_query")
            if not paths:
                raise ValueError(
                    "solid.library_query needs 'index': a feature index, or "
                    "'paths'/'dir' to scan with feature extraction")
            records = _load_candidates(paths, skipped, features=True)
        feat_recs = [r for r in records if r.get("features")]
        if not feat_recs:
            raise ValueError(
                "solid.library_query found no feature signatures in the library; "
                "rebuild the index with solid.library_index features=True")

        dtol = float(a.get("diam_tol", 0.2))
        min_holes = a.get("min_holes")
        need_through = a.get("through")
        need_boss = a.get("boss")
        hole_diam = a.get("hole_diam")
        boss_diam = a.get("boss_diam")
        hits = []
        for r in feat_recs:
            fs = r["features"]
            if min_holes is not None and fs["holes"] < int(min_holes):
                continue
            if need_through and fs["through_holes"] < 1:
                continue
            if need_boss is not None and bool(fs["bosses"]) != bool(need_boss):
                continue
            if hole_diam is not None and not any(
                    abs(d - float(hole_diam)) <= dtol for d in fs["hole_diams"]):
                continue
            if boss_diam is not None and not any(
                    abs(d - float(boss_diam)) <= dtol for d in fs["boss_diams"]):
                continue
            hits.append({"label": r["label"], "path": r.get("path"),
                         "features": fs})
        hits.sort(key=lambda h: h["label"])
        return {"matched": len(hits), "scanned": len(feat_recs),
                "spec": {"min_holes": min_holes, "through": need_through,
                         "boss": need_boss, "hole_diam": hole_diam,
                         "boss_diam": boss_diam, "diam_tol": dtol},
                "hits": hits, "skipped": skipped}

    def _fetch_one(src, cache_dir, timeout, max_bytes, skipped):
        """Download one community/online model into ``cache_dir`` and return its
        local path (or ``None`` on failure, logged in ``skipped``).

        ``src`` is a URL string or a ``{"url", "label"|"name"}`` dict. ``file://``
        and bare local paths are accepted too, so an offline mirror behaves like
        an online one. The body is streamed with a hard ``max_bytes`` ceiling so a
        runaway download cannot exhaust the box, and the on-disk name is derived
        from the URL (falling back to a content hash) to keep the cache stable and
        de-duplicated across repeated fetches.
        """
        url = src.get("url") if isinstance(src, dict) else src
        label = (src.get("label") or src.get("name")) if isinstance(src, dict) else None
        if not url:
            skipped.append({"url": src, "reason": "no url"})
            return None
        parsed = urllib.parse.urlparse(url)
        if parsed.scheme in ("", "file"):                 # local / file:// mirror
            local = parsed.path if parsed.scheme == "file" else url
            if not os.path.isfile(local):
                skipped.append({"url": url, "reason": "no such file"})
                return None
            return local
        name = label or os.path.basename(parsed.path) or hashlib.sha1(
            url.encode()).hexdigest()
        if "." not in os.path.basename(name):
            name += ".step"
        dest = os.path.join(cache_dir, name)
        if os.path.isfile(dest) and os.path.getsize(dest) > 0:
            return dest                                   # cached -> never refetch
        # Honour an explicit no-proxy world (mirrors the DAO Bridge SDK pattern)
        # so internal/community mirrors resolve directly.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "dao-cad/1.0"})
            with opener.open(req, timeout=timeout) as resp:
                buf = bytearray()
                while True:
                    chunk = resp.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    if len(buf) > max_bytes:
                        skipped.append({"url": url,
                                        "reason": "exceeds max_bytes=%d" % max_bytes})
                        return None
            with open(dest, "wb") as fh:
                fh.write(buf)
            return dest
        except Exception as exc:
            skipped.append({"url": url, "reason": "fetch failed: %s" % exc})
            return None

    def op_library_fetch(a):
        """Pull 3D models from community/online sources into the local library.

        取之尽锱铢: this is the bridge between the world's model repositories and
        the in-box matching pipeline. Give it ``urls`` (a list of direct download
        URLs, or ``{"url", "label"}`` dicts) and it streams each model into a
        ``cache`` directory (default ``~/.dao_cad/library``), de-duplicating by
        name and capping every download at ``max_bytes``. The freshly cached files
        then flow straight into the existing pipeline:

          * always: fingerprint each fetched solid (so the catalogue is usable);
          * with ``name``: rank the fetched models against that open query solid
            (same scale-invariant distance as ``library_match``);
          * with ``out``: persist a JSON ``solid.library_index`` over the cache,
            optionally with ``features=True`` for ``library_query``.

        Unreachable / oversized / unreadable sources are reported in ``skipped``
        rather than aborting, so one dead link never sinks the whole harvest.

        args: urls(list), cache/dir(optional), name(optional query solid),
              out(optional index path), features(bool), timeout(s), max_bytes
        """
        urls = a.get("urls") or []
        if not urls:
            raise ValueError(
                "solid.library_fetch needs 'urls': a list of model download URLs "
                "(or {'url','label'} dicts) to pull into the local library")
        cache_dir = a.get("cache") or a.get("dir") or os.path.join(
            os.path.expanduser("~"), ".dao_cad", "library")
        os.makedirs(cache_dir, exist_ok=True)
        timeout = float(a.get("timeout", 30))
        max_bytes = int(a.get("max_bytes", 64 * 1024 * 1024))
        skipped = []
        fetched = []
        for src in urls:
            local = _fetch_one(src, cache_dir, timeout, max_bytes, skipped)
            if local:
                fetched.append(local)
        if not fetched:
            raise ValueError(
                "solid.library_fetch downloaded no usable model from %d source(s);"
                " skipped=%r" % (len(urls), skipped))
        want_features = bool(a.get("features"))
        records = _load_candidates(fetched, skipped, features=want_features)
        if not records:
            raise ValueError(
                "solid.library_fetch fetched %d file(s) but found no solid; "
                "skipped=%r" % (len(fetched), skipped))
        out = {"fetched": len(fetched), "cache": cache_dir,
               "indexed": len(records),
               "labels": [r["label"] for r in records], "skipped": skipped}
        index_path = a.get("out")
        if index_path:
            with open(index_path, "w", encoding="utf-8") as fh:
                json.dump({"version": 1, "features": want_features,
                           "records": records}, fh)
            out["out"] = index_path
        if "name" in a:
            q = _shape_fingerprint(a["name"])
            ranked = _rank_records(q, records)
            out["query_key"] = q["shape_key"]
            out["best"] = ranked[0]["label"]
            out["best_distance"] = ranked[0]["distance"]
            out["ranking"] = ranked
        return out

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

    def op_holes(a):
        """Recover cylindrical holes and bosses from a solid — the mounting-feature
        channel of butchering-the-ox.

        ``recognize`` names a whole simple part; a real bracket is a block *minus
        holes plus bosses*, and those round features are the design intent you
        most want back when reverse-engineering. This scans every cylindrical
        face, decides hole-vs-boss from the true outward normal (a hole's normal
        points *toward* its axis — the solid is outside the bore; a boss's points
        away), and merges coaxial faces into one feature (so a counterbore comes
        back as one feature carrying both radii). For each feature it reports the
        axis, a point on it, the radius (radii, sorted, when stepped), the axial
        depth, and whether it runs ``through`` the part (depth ~ the part's extent
        along that axis). args: name, tol.
        """
        sh = _get(a["name"]).Shape
        sols = sh.Solids
        if not sols:
            raise ValueError(
                "solid.holes needs a solid (got a shell/compound with no "
                "volume); round features are bored into a body")
        if len(sols) != 1:
            raise ValueError(
                "solid.holes expects a single solid (got %d); run solid.decompose "
                "and analyse one part at a time" % len(sols))
        body = sols[0]
        diag = body.BoundBox.DiagonalLength or 1.0
        ptol = max(float(a.get("tol", 1e-4)) * diag, 1e-6)
        eps = max(1e-3, 1e-4 * diag)
        raw = []
        for idx, f in enumerate(body.Faces):
            su = f.Surface
            if su.__class__.__name__ != "Cylinder":
                continue
            ax = _unit_v(su.Axis)
            ctr = su.Center
            u0, u1, v0, v1 = f.ParameterRange
            um, vm = (u0 + u1) / 2.0, (v0 + v1) / 2.0
            try:
                n = f.normalAt(um, vm)
                p = f.valueAt(um, vm)
            except Exception:
                continue
            # Face.normalAt already returns the orientation-aware outward normal
            # (out of the solid): for a bore it points toward the axis, for a
            # boss away from it -- that sign alone separates hole from boss.
            radial = p - ctr
            radial = radial - ax * radial.dot(ax)
            rl = radial.Length or 1.0
            outward = (n.x * radial.x + n.y * radial.y + n.z * radial.z) / rl
            ts = [(vx.Point - ctr).dot(ax) for vx in f.Vertexes]
            vmin, vmax = (min(ts), max(ts)) if ts else (0.0, 0.0)
            raw.append({"face": idx, "axis": ax, "center": ctr,
                        "radius": float(su.Radius), "vmin": vmin, "vmax": vmax,
                        "span": abs(u1 - u0),
                        "kind": "boss" if outward > 0 else "hole"})

        def _coaxial(ax0, p0, ax1, p1):
            return (abs(abs(ax0.dot(ax1)) - 1.0) <= 1e-6
                    and (p1 - p0).cross(ax0).Length < ptol)

        # phase 1 — rings: coaxial faces of *equal* radius are one turned/bored
        # cylindrical surface, even when the modeller split them into faces with
        # an axial gap (a pin interrupted by a collar, a bore split by a groove):
        # one radius on one axis is one cylinder. The summed angular span then
        # tells a complete 2*pi cylinder (a real bore/boss) from a partial patch
        # (an edge fillet, a gear-tooth flank).
        rings = []
        for cf in raw:
            tgt = None
            for rg in rings:
                if (rg["kind"] == cf["kind"]
                        and abs(rg["radius"] - cf["radius"]) <= ptol
                        and _coaxial(rg["axis"], rg["pt"], cf["axis"], cf["center"])):
                    tgt = rg
                    break
            if tgt is None:
                tgt = {"kind": cf["kind"], "axis": cf["axis"], "pt": cf["center"],
                       "radius": cf["radius"], "vmin": cf["vmin"], "vmax": cf["vmax"],
                       "faces": [], "span": 0.0}
                rings.append(tgt)
            tgt["faces"].append(cf["face"])
            tgt["vmin"] = min(tgt["vmin"], cf["vmin"])
            tgt["vmax"] = max(tgt["vmax"], cf["vmax"])
            tgt["span"] += cf["span"]

        # phase 2 — features: telescoping rings of *different* radius on one axis
        # are a single stepped feature (a counterbore); concentric rings that
        # share the same axial extent (a gear's tooth lands at r10/r11.2/r12.75)
        # stay distinct, so a gear is not reported as one 75-radius "counterbore".
        feats = []
        for rg in sorted(rings, key=lambda r: r["radius"]):
            tgt = None
            for ft in feats:
                if ft["kind"] != rg["kind"] or not _coaxial(ft["axis"], ft["pt"],
                                                            rg["axis"], rg["pt"]):
                    continue
                # a counterbore's steps abut (the wider recess meets the narrower
                # bore at one plane), so accept touching as well as overlapping;
                # only reject concentric rings of identical extent (gear lands)
                # and rings too far apart to be one feature.
                overlap = min(rg["vmax"], ft["vmax"]) - max(rg["vmin"], ft["vmin"])
                concentric = (abs(rg["vmin"] - ft["vmin"]) <= eps
                              and abs(rg["vmax"] - ft["vmax"]) <= eps)
                if overlap >= -eps and not concentric:
                    tgt = ft
                    break
            if tgt is None:
                tgt = {"kind": rg["kind"], "axis": rg["axis"], "pt": rg["pt"],
                       "radii": [], "faces": [], "rings": [], "vmin": rg["vmin"],
                       "vmax": rg["vmax"], "span": 0.0}
                feats.append(tgt)
            tgt["radii"].append(rg["radius"])
            tgt["faces"].extend(rg["faces"])
            tgt["rings"].append({"radius": rg["radius"], "faces": list(rg["faces"])})
            tgt["vmin"] = min(tgt["vmin"], rg["vmin"])
            tgt["vmax"] = max(tgt["vmax"], rg["vmax"])
            tgt["span"] = max(tgt["span"], rg["span"])

        features, blends = [], []
        for ft in feats:
            ax = ft["axis"]
            depth = ft["vmax"] - ft["vmin"]
            radii = sorted(round(r, 4) for r in ft["radii"])
            full_round = ft["span"] >= 2.0 * math.pi - 1e-2
            through = False
            if ft["kind"] == "hole" and full_round:
                # a bore is through iff it opens to the outside at both ends:
                # a point just past each end, on the axis, lies outside the solid
                # (a blind hole is capped by material at one end). This is exact
                # regardless of how the part's thickness varies elsewhere.
                lo = ft["pt"] + ax * (ft["vmin"] - eps)
                hi = ft["pt"] + ax * (ft["vmax"] + eps)
                through = (not body.isInside(lo, eps, True)
                           and not body.isInside(hi, eps, True))
            # the feature's true axial extent: project every vertex of every face
            # in the feature onto the axis and take the span. The two endpoints
            # (on the axis) are what a reconstruction needs to place the cylinder.
            ts = [(body.Faces[fi].Vertexes[k].Point - ft["pt"]).dot(ax)
                  for fi in ft["faces"]
                  for k in range(len(body.Faces[fi].Vertexes))]
            tmin, tmax = (min(ts), max(ts)) if ts else (0.0, 0.0)
            p0 = ft["pt"] + ax * tmin
            p1 = ft["pt"] + ax * tmax
            # a counterbore is several coaxial steps of different radius; record
            # each step's own radius and axial endpoints so a reconstruction can
            # cut the recess and the bore separately rather than guessing.
            steps = None
            if len(set(radii)) > 1:
                byr = {}
                for rg in ft["rings"]:
                    ets = [(body.Faces[fi].Vertexes[k].Point - ft["pt"]).dot(ax)
                           for fi in rg["faces"]
                           for k in range(len(body.Faces[fi].Vertexes))]
                    if not ets:
                        continue
                    key = round(rg["radius"], 4)
                    cur = byr.setdefault(key, [min(ets), max(ets)])
                    cur[0] = min(cur[0], min(ets))
                    cur[1] = max(cur[1], max(ets))
                steps = []
                for rr in sorted(byr):
                    smn, smx = byr[rr]
                    sp0 = ft["pt"] + ax * smn
                    sp1 = ft["pt"] + ax * smx
                    steps.append({"radius": rr,
                                  "ends": [[_round(sp0.x), _round(sp0.y), _round(sp0.z)],
                                           [_round(sp1.x), _round(sp1.y), _round(sp1.z)]]})
            rec = {
                "kind": ft["kind"],
                "axis": [_round(ax.x, 6), _round(ax.y, 6), _round(ax.z, 6)],
                "point": [_round(ft["pt"].x), _round(ft["pt"].y), _round(ft["pt"].z)],
                "ends": [[_round(p0.x), _round(p0.y), _round(p0.z)],
                         [_round(p1.x), _round(p1.y), _round(p1.z)]],
                "radius": radii[0], "radii": radii,
                "counterbored": len(set(radii)) > 1,
                "steps": steps,
                "depth": _round(depth),
                "through": through,
                "full_round": full_round,
                "faces": sorted(ft["faces"]),
            }
            # a partial cylinder (sum of spans < 2*pi) is an edge blend, not a
            # drilled hole or a turned boss -- report it apart so the hole/boss
            # tally is not swamped by every fillet on the part.
            (features if full_round else blends).append(rec)
        features.sort(key=lambda x: (x["kind"], -x["radius"]))
        blends.sort(key=lambda x: -x["radius"])
        holes = [f for f in features if f["kind"] == "hole"]
        bosses = [f for f in features if f["kind"] == "boss"]
        return {"name": a["name"], "features": features, "blends": blends,
                "hole_count": len(holes), "boss_count": len(bosses),
                "blend_count": len(blends),
                "through_holes": sum(1 for h in holes if h["through"])}

    def op_fillets(a):
        """Recover edge blends -- fillets and rounds -- from a solid.

        ``holes`` recovers full-round bores and turned bosses; the other
        pervasive manufacturing intent is "break every sharp edge". A *round*
        softens a convex edge (a partial cylinder swept along a straight edge, a
        sphere patch over a convex corner); a *fillet* fills a concave re-entrant
        edge (where a boss meets a floor -- a circular edge yields a toroidal
        fillet whose *minor* radius is the blend radius). Every case is an
        analytic surface read exactly: straight-edge blend = partial cylinder of
        radius = blend radius; corner = sphere of that radius; circular-edge
        fillet = torus, minor radius = blend radius. The solid's true outward
        normal sign separates round (points away from the blend's curvature
        centre -- material is inside) from fillet (points toward it). Faces are
        grouped by (kind, geometry, radius) so a "fillet all 12 edges at r2"
        comes back as one group with count 12. args: name, tol.
        """
        sh = _get(a["name"]).Shape
        sols = sh.Solids
        if not sols:
            raise ValueError(
                "solid.fillets needs a solid (got a shell/compound with no "
                "volume); edge blends live on a body")
        if len(sols) != 1:
            raise ValueError(
                "solid.fillets expects a single solid (got %d); run "
                "solid.decompose and analyse one part at a time" % len(sols))
        body = sols[0]
        diag = body.BoundBox.DiagonalLength or 1.0
        ptol = max(float(a.get("tol", 1e-4)) * diag, 1e-6)

        def _coaxial(ax0, p0, ax1, p1):
            return (abs(abs(ax0.dot(ax1)) - 1.0) <= 1e-6
                    and (p1 - p0).cross(ax0).Length < ptol)

        # cylinders: group coaxial same-radius faces and sum the angular span, so
        # a full bore split into faces is recognised as a bore (skipped) rather
        # than mistaken for a blend -- only sub-2*pi sweeps are edge blends.
        cyl_rings, blends = [], []
        for idx, f in enumerate(body.Faces):
            su = f.Surface
            kind = su.__class__.__name__
            u0, u1, v0, v1 = f.ParameterRange
            um, vm = (u0 + u1) / 2.0, (v0 + v1) / 2.0
            try:
                n = f.normalAt(um, vm)
                p = f.valueAt(um, vm)
            except Exception:
                continue
            if kind == "Cylinder":
                ax = _unit_v(su.Axis)
                ctr = su.Center
                radial = p - ctr
                radial = radial - ax * radial.dot(ax)
                rl = radial.Length or 1.0
                outward = (n.x * radial.x + n.y * radial.y + n.z * radial.z) / rl
                ts = [(vx.Point - ctr).dot(ax) for vx in f.Vertexes]
                vmin, vmax = (min(ts), max(ts)) if ts else (0.0, 0.0)
                tgt = None
                for rg in cyl_rings:
                    if (abs(rg["radius"] - su.Radius) <= ptol
                            and _coaxial(rg["axis"], rg["pt"], ax, ctr)):
                        tgt = rg
                        break
                if tgt is None:
                    tgt = {"axis": ax, "pt": ctr, "radius": float(su.Radius),
                           "span": 0.0, "vmin": vmin, "vmax": vmax,
                           "faces": [], "outward": outward}
                    cyl_rings.append(tgt)
                tgt["span"] += abs(u1 - u0)
                tgt["vmin"] = min(tgt["vmin"], vmin)
                tgt["vmax"] = max(tgt["vmax"], vmax)
                tgt["faces"].append(idx)
            elif kind == "Sphere":
                ctr = su.Center
                radial = p - ctr
                rl = radial.Length or 1.0
                outward = (n.x * radial.x + n.y * radial.y + n.z * radial.z) / rl
                blends.append({"geom": "sphere", "radius": float(su.Radius),
                               "edge_length": 0.0, "faces": [idx],
                               "kind": "round" if outward > 0 else "fillet"})
            elif kind == "Toroid":
                ctr = su.Center
                ax = _unit_v(su.Axis)
                rad = p - ctr
                rad = rad - ax * rad.dot(ax)
                rl = rad.Length or 1.0
                circ = ctr + rad * (float(su.MajorRadius) / rl)
                tube = p - circ
                tl = tube.Length or 1.0
                outward = (n.x * tube.x + n.y * tube.y + n.z * tube.z) / tl
                arc = 2.0 * math.pi * float(su.MajorRadius) * (abs(u1 - u0) / (2.0 * math.pi))
                blends.append({"geom": "torus", "radius": float(su.MinorRadius),
                               "edge_length": arc, "faces": [idx],
                               "kind": "round" if outward > 0 else "fillet"})

        for rg in cyl_rings:
            if rg["span"] >= 2.0 * math.pi - 1e-2:
                continue                       # a complete bore/boss, not a blend
            blends.append({"geom": "cylinder", "radius": rg["radius"],
                           "edge_length": rg["vmax"] - rg["vmin"], "faces": rg["faces"],
                           "kind": "round" if rg["outward"] > 0 else "fillet"})

        groups = []
        for b in blends:
            g = None
            for e in groups:
                if (e["kind"] == b["kind"] and e["geom"] == b["geom"]
                        and abs(e["radius"] - b["radius"]) <= ptol):
                    g = e
                    break
            if g is None:
                g = {"kind": b["kind"], "geom": b["geom"], "radius": _round(b["radius"]),
                     "count": 0, "faces": [], "edge_length": 0.0}
                groups.append(g)
            g["count"] += 1
            g["faces"].extend(b["faces"])
            g["edge_length"] += b["edge_length"]
        for g in groups:
            g["edge_length"] = _round(g["edge_length"])
            g["faces"] = sorted(g["faces"])
        groups.sort(key=lambda x: (x["kind"], -x["radius"]))
        rounds = [g for g in groups if g["kind"] == "round"]
        fillets = [g for g in groups if g["kind"] == "fillet"]
        return {"name": a["name"], "blend_groups": groups,
                "blend_face_count": len(blends),
                "round_count": sum(g["count"] for g in rounds),
                "fillet_count": sum(g["count"] for g in fillets),
                "radii": sorted({g["radius"] for g in groups})}

    def op_design_intent(a):
        """Fuse the reverse-engineering reads into one design-intent digest.

        The reverse half walks a real part back to "the first thought": what
        rough stock, what primitive, which symmetry, which drilled holes and
        turned bosses, which broken edges. Each sub-read is an existing, closed
        -form-verified operator (``obb`` for stock+frame, ``recognize`` for a
        primitive guess, ``symmetry`` for the design's regularity, ``holes`` for
        mounting features, ``fillets`` for edge breaks); this composes them into
        one card plus an ordered, human-readable build ``recipe`` -- the forward
        program a clean CAD model would run to reproduce the part. args: name,
        tol. Sub-reads that a part defeats (e.g. symmetry on a huge mesh) degrade
        to null rather than failing the whole digest.
        """
        name = a["name"]
        sols = _get(name).Shape.Solids
        if not sols:
            raise ValueError(
                "solid.design_intent needs a solid (got a shell/compound with "
                "no volume)")
        if len(sols) != 1:
            raise ValueError(
                "solid.design_intent expects a single solid (got %d); run "
                "solid.decompose and digest one part at a time" % len(sols))
        prim = op_recognize({"name": name})
        box = op_obb({"name": name})
        try:
            sym = op_symmetry({"name": name, "method": "invariant"})
        except Exception:
            sym = None
        feats = op_holes({"name": name})
        bl = op_fillets({"name": name})

        dims = box["sorted_dimensions"]
        is_prim = (prim.get("type") not in (None, "freeform")
                   and prim.get("volume_match"))
        recipe = []
        # a volume-matched primitive *is* the whole part: its closed-form volume
        # equals the measured volume, so there are no subtractions (a hole) or
        # additions (a boss) -- the cylindrical body's own face that ``holes``
        # honestly reports as a boss is the body, not an added feature.
        if is_prim:
            recipe.append("start from a %s %s" % (prim["type"], prim.get("params")))
            holes, bosses = [], []
            rounds = fillets = 0
            radii = []
        else:
            recipe.append("start from a %g x %g x %g block (rough stock)"
                          % (dims[0], dims[1], dims[2]))
            holes = [f for f in feats["features"] if f["kind"] == "hole"]
            bosses = [f for f in feats["features"] if f["kind"] == "boss"]
            rounds, fillets, radii = bl["round_count"], bl["fillet_count"], bl["radii"]
        for h in holes:
            recipe.append("drill phi%g %s hole, depth %g"
                          % (2 * h["radius"], "through" if h["through"] else "blind",
                             h["depth"]))
        for b in bosses:
            recipe.append("add phi%g boss, height %g%s"
                          % (2 * b["radius"], b["depth"],
                             " (stepped)" if b["counterbored"] else ""))
        if rounds or fillets:
            recipe.append("break edges: %d round(s) + %d fillet(s) at r=%s"
                          % (rounds, fillets, radii))
        return {
            "name": name,
            "stock": {"size": dims, "fill_ratio": box["fill_ratio"],
                      "axes": box["axes"]},
            "primitive": {"type": prim.get("type"), "params": prim.get("params"),
                          "volume_match": prim.get("volume_match")},
            "symmetry": None if sym is None else {
                "mirror_planes": sym.get("mirror_plane_count"),
                "max_rotation": sym.get("max_rotational_order")},
            "holes": {"count": len(holes),
                      "through": sum(1 for h in holes if h["through"]),
                      "bosses": len(bosses)},
            "blends": {"rounds": rounds, "fillets": fillets, "radii": radii},
            "recipe": recipe,
        }

    def _prim_shape(spec):
        """Build one solid from a frame-agnostic recipe step. The same vocabulary
        backs the replayable build program and reverse_build's own rebuild, so the
        emitted program is guaranteed to be what was actually built."""
        mk = spec["make"]
        if mk == "box":
            sz = spec["size"]
            pos = spec.get("pos", [0, 0, 0])
            return Part.makeBox(sz[0], sz[1], sz[2], V(*pos))
        if mk == "cylinder":
            pos = spec.get("pos", [0, 0, 0])
            dirv = spec.get("dir", [0, 0, 1])
            return Part.makeCylinder(spec["r"], spec["h"], V(*pos), V(*dirv))
        if mk == "sphere":
            return Part.makeSphere(spec["r"])
        if mk == "cone":
            pos = spec.get("pos", [0, 0, 0])
            dirv = spec.get("dir", [0, 0, 1])
            return Part.makeCone(spec["r1"], spec["r2"], spec["h"],
                                 V(*pos), V(*dirv))
        if mk == "torus":
            return Part.makeTorus(spec["R"], spec["r"])
        raise ValueError("unknown build step %r" % mk)

    def _run_program(prog):
        """Replay a {stock, cuts} build program into a single solid."""
        shp = _prim_shape(prog["stock"])
        for c in prog.get("cuts") or []:
            shp = shp.cut(_prim_shape(c))
        return shp

    def op_replay(a):
        """Re-execute a replayable build program (as emitted by reverse_build) and
        bind the resulting solid -- so a recovered design intent is not just an
        internal rebuild but an editable, reusable forward recipe a user can keep,
        tweak (change a diameter, a stock size) and re-run. args: program, out."""
        prog = a.get("program")
        if not prog or "stock" not in prog:
            raise ValueError(
                "solid.replay needs a 'program' with a 'stock' step (get one "
                "from solid.reverse_build's 'program' field)")
        shp = _run_program(prog)
        sol = shp.Solids[0] if shp.Solids else shp
        out = a.get("out") or "replayed"
        _put(out, sol)
        return {"out": out, "volume": _round(sol.Volume),
                "cuts": len(prog.get("cuts") or [])}

    def op_reverse_build(a):
        """Close the forward-reverse loop: rebuild a clean model from the
        recovered design intent and *prove* it reproduces the original.

        正反相呼应 ends here. The reverse half (recognize/obb/holes) reads a part
        back to its intent; this re-runs that intent forward as real geometry --
        a recognised primitive is re-emitted with ``Part.make*`` exactly, an
        engineered part is rebuilt as its stock block minus each drilled hole
        plus each turned boss (placed at the recovered axes/ends/radii) -- then
        checks the rebuild against the original by the two invariants that cannot
        be fudged: relative volume error and the scale-/pose-invariant shape key.
        Work is done in the part's own principal frame so the stock block is
        axis-aligned. Stepped (counterbored) features are not yet reconstructed
        exactly and are reported in ``skipped`` with ``volume_match`` telling the
        honest truth rather than a silent near-miss. args: name, out, tol.
        """
        name = a["name"]
        out = a.get("out") or (name + "_rebuilt")
        tol = float(a.get("tol", 1e-3))
        sols = _get(name).Shape.Solids
        if len(sols) != 1:
            raise ValueError(
                "solid.reverse_build expects a single solid (got %d); run "
                "solid.decompose and rebuild one part at a time" % len(sols))
        body0 = _in_principal_frame(sols[0])
        diag = body0.BoundBox.DiagonalLength or 1.0
        skipped = []

        prim = op_recognize({"name": name})
        ptype = prim.get("type")
        pp = prim.get("params") or {}
        program = None
        kind = None
        if prim.get("volume_match") and ptype == "box":
            program = {"stock": {"make": "box",
                                 "size": [pp["length"], pp["width"], pp["height"]]}}
            kind = "primitive:box"
        elif prim.get("volume_match") and ptype == "cylinder":
            program = {"stock": {"make": "cylinder", "r": pp["radius"], "h": pp["height"]}}
            kind = "primitive:cylinder"
        elif prim.get("volume_match") and ptype == "sphere":
            program = {"stock": {"make": "sphere", "r": pp["radius"]}}
            kind = "primitive:sphere"
        elif prim.get("volume_match") and ptype == "cone":
            program = {"stock": {"make": "cone", "r1": pp["radius"], "r2": 0.0,
                                 "h": pp["height"]}}
            kind = "primitive:cone"
        elif prim.get("volume_match") and ptype == "frustum":
            program = {"stock": {"make": "cone", "r1": pp["base_radius"],
                                 "r2": pp["top_radius"], "h": pp["height"]}}
            kind = "primitive:frustum"
        elif prim.get("volume_match") and ptype == "torus":
            program = {"stock": {"make": "torus", "R": pp["major_radius"],
                                 "r": pp["minor_radius"]}}
            kind = "primitive:torus"
        elif prim.get("volume_match") and ptype == "tube":
            program = {"stock": {"make": "cylinder", "r": pp["outer_radius"],
                                 "h": pp["height"]},
                       "cuts": [{"make": "cylinder", "r": pp["inner_radius"],
                                 "h": pp["height"]}]}
            kind = "primitive:tube"

        if program is None:
            # engineered part: a stock billet minus each drilled hole, placed in
            # the principal frame where the billet is axis-aligned. The billet is
            # a block by default, but a *turned* part (two equal cross-section
            # dimensions) gets a cylindrical billet instead -- a bbox block would
            # massively overestimate a shaft/bushing's stock.
            tmp = "__rev_build_tmp__"
            _put(tmp, body0)
            try:
                feats = op_holes({"name": tmp})
            finally:
                ex = state.shapes.pop(tmp, None)
                if ex and doc.getObject(ex):
                    doc.removeObject(ex)
            bb = body0.BoundBox
            cen = bb.Center
            dims = [("x", bb.XLength), ("y", bb.YLength), ("z", bb.ZLength)]
            _AX = {"x": V(1, 0, 0), "y": V(0, 1, 0), "z": V(0, 0, 1)}
            # a body of revolution has two equal transverse extents AND fits
            # inside the resulting cylinder. The volume guard rejects a square
            # prism (equal footprint, but its corners stick out past any inscribed
            # cylinder, so part volume exceeds the cylinder's) -- only a genuinely
            # round part has part_volume <= cylinder_volume.
            billet_axis = None
            billet_r = None
            for i in range(3):
                a0, a1, a2 = dims[i], dims[(i + 1) % 3], dims[(i + 2) % 3]
                if abs(a1[1] - a2[1]) <= 0.02 * max(a1[1], a2[1], 1e-9):
                    r = 0.5 * (a1[1] + a2[1]) / 2.0
                    if body0.Volume <= math.pi * r * r * a0[1] * (1.0 + 1e-3):
                        billet_axis = a0[0]
                        billet_r = r
                        billet_h = a0[1]
                        break
            cuts = []
            if billet_axis is not None:
                ax = _AX[billet_axis]
                lo = getattr(bb, billet_axis.upper() + "Min")
                base = V(cen.x, cen.y, cen.z) - ax * (cen.dot(ax)) + ax * lo
                stock = {"make": "cylinder", "r": billet_r, "h": billet_h,
                         "pos": [base.x, base.y, base.z],
                         "dir": [ax.x, ax.y, ax.z]}
                kind = "billet:cylinder-minus-holes"
            else:
                stock = {"make": "box",
                         "size": [bb.XLength, bb.YLength, bb.ZLength],
                         "pos": [bb.XMin, bb.YMin, bb.ZMin]}
                ax = None
                kind = "billet:box-minus-holes"
            mtol = 1e-3 * diag

            def _cut_spec(r, p0, p1, ext0, ext1):
                # ext0/ext1 push the cut a touch past an *open* end so the cut
                # face is not coplanar with the part face (a robust boolean);
                # an interior step boundary is cut exactly so abutting steps meet.
                seg = p1 - p0
                length = seg.Length
                if length <= 1e-9:
                    return None
                dirv = _unit_v(seg)
                m0 = 0.05 * diag if ext0 else 0.0
                m1 = 0.05 * diag if ext1 else 0.0
                b = p0 - dirv * m0
                return {"make": "cylinder", "r": r, "h": length + m0 + m1,
                        "pos": [b.x, b.y, b.z], "dir": [dirv.x, dirv.y, dirv.z]}

            for f in feats["features"]:
                if f["kind"] == "boss":
                    if f["counterbored"]:
                        skipped.append({"feature": "boss", "radii": f["radii"],
                                        "reason": "stepped boss not reconstructed"})
                        continue
                    fax = V(*f["axis"])
                    # the cylindrical billet's own outer wall reads as a boss --
                    # but it is the stock, not an added feature, so skip it (same
                    # axis, same radius). Any *other* protruding boss genuinely
                    # cannot be recovered from a convex billet.
                    if (ax is not None
                            and abs(abs(fax.dot(ax)) - 1.0) < 1e-3
                            and abs(f["radius"] - billet_r) <= 1e-2 * max(billet_r, 1e-9)):
                        continue
                    skipped.append({"feature": "boss", "radius": f["radius"],
                                    "reason": "protruding boss not recoverable from a convex billet"})
                    continue
                # a hole, possibly counterbored: cut each step (recess + bore)
                # over its own axial extent; a plain hole is one step. A step end
                # that lands on the feature's open mouth/exit gets the overshoot.
                gp0 = V(*f["ends"][0])
                gp1 = V(*f["ends"][1])
                if f["counterbored"] and f["steps"]:
                    for st in f["steps"]:
                        sp0 = V(*st["ends"][0])
                        sp1 = V(*st["ends"][1])
                        e0 = f["through"] and (sp0 - gp0).Length <= mtol
                        e1 = f["through"] and (sp1 - gp1).Length <= mtol
                        c = _cut_spec(st["radius"], sp0, sp1, e0, e1)
                        if c:
                            cuts.append(c)
                else:
                    c = _cut_spec(f["radius"], gp0, gp1, f["through"], f["through"])
                    if c:
                        cuts.append(c)
            program = {"stock": stock, "cuts": cuts}

        shape = _run_program(program)
        rebuilt = shape.Solids[0] if shape.Solids else shape
        vol_err = abs(rebuilt.Volume - body0.Volume) / max(body0.Volume, 1e-9)
        q_orig = _fingerprint_body(body0)
        q_new = _fingerprint_body(rebuilt)
        same_key = q_orig["shape_key"] == q_new["shape_key"]
        _put(out, rebuilt)
        return {
            "name": name, "out": out, "recipe_kind": kind,
            "original_volume": _round(body0.Volume),
            "rebuilt_volume": _round(rebuilt.Volume),
            "volume_error": _round(vol_err, 6),
            "volume_match": vol_err < tol and not skipped,
            "same_shape_key": same_key,
            "skipped": skipped,
            "program": program,
        }

    def op_reuse(a):
        """先检索复用、再从零建模 -- end to end. Given a target (a loaded solid to
        match by *shape*, or a set of *feature* predicates) and a library (an
        ``index``/``dir``/``paths``), find the closest catalogued parts and hand
        each back as an *editable replay program* recovered by ``reverse_build``.
        A new requirement thus starts from an existing design to adapt (change a
        diameter, a stock size, then ``solid.replay``) instead of a blank sheet.

        Shape mode (``name`` given, no feature predicate) ranks by the scale-
        invariant fingerprint distance; feature mode (``min_holes``/``through``/
        ``boss``/``hole_diam``/``boss_diam``) filters by mounting feature. Parts
        that cannot be reverse-built exactly are still returned, flagged with a
        note rather than dropped silently. args: name|<feature preds>,
        index|dir|paths, top (default 3), diam_tol.
        """
        top = int(a.get("top", 3))
        skipped = []
        feat_keys = ("min_holes", "through", "boss", "hole_diam", "boss_diam")
        by_feature = any(a.get(k) is not None for k in feat_keys)
        lib = {k: a[k] for k in ("index", "dir", "paths", "exts", "recursive")
               if a.get(k) is not None}

        if a.get("name") and not by_feature:
            ranked = op_library_match(dict(lib, name=a["name"]))
            cands = [{"label": r["label"], "path": r.get("path"),
                      "distance": r["distance"], "same_key": r["same_key"]}
                     for r in ranked["ranking"]]
            skipped += ranked.get("skipped") or []
            mode = "shape"
        else:
            qa = dict(lib)
            for k in feat_keys + ("diam_tol",):
                if a.get(k) is not None:
                    qa[k] = a[k]
            q = op_library_query(qa)
            cands = [{"label": h["label"], "path": h.get("path"),
                      "features": h.get("features")} for h in q["hits"]]
            skipped += q.get("skipped") or []
            mode = "feature"

        def _drop(n):
            ex = state.shapes.pop(n, None)
            if ex and doc.getObject(ex):
                doc.removeObject(ex)

        reusable = []
        seen = set()
        for c in cands:
            if len(reusable) >= top:
                break
            path = c.get("path")
            if not path or path in seen:
                continue
            seen.add(path)
            try:
                imp = op_import_step({"path": path})
            except Exception as exc:
                skipped.append({"path": path, "reason": "import failed: %s" % exc})
                continue
            names = imp["imported"]
            if len(names) != 1:
                for n in names:
                    _drop(n)
                skipped.append({"path": path,
                                "reason": "%d solids; reuse one part at a time "
                                          "(decompose first)" % len(names)})
                continue
            use = names[0]
            rbout = use + "__reuse_rb"
            try:
                rb = op_reverse_build({"name": use, "out": rbout})
            except Exception as exc:
                skipped.append({"path": path,
                                "reason": "reverse_build failed: %s" % exc})
                _drop(use)
                continue
            entry = {"label": c["label"], "path": path,
                     "recipe_kind": rb["recipe_kind"],
                     "volume_match": rb["volume_match"],
                     "program": rb["program"]}
            if mode == "shape":
                entry["distance"] = c["distance"]
                entry["same_key"] = c["same_key"]
            else:
                entry["features"] = c.get("features")
            if not rb["volume_match"]:
                entry["note"] = ("reverse_build could not reproduce this part "
                                 "exactly; the program is an approximate stock "
                                 "recipe (see reverse_build skipped reasons)")
            reusable.append(entry)
            _drop(use)
            _drop(rbout)

        return {"mode": mode, "library_candidates": len(cands),
                "returned": len(reusable), "reusable": reusable,
                "skipped": skipped}

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

    # ---- projection / hydrostatics / tolerance (engineering analysis) ----- #
    def op_projected_area(a):
        """Silhouette (shadow) area of a solid projected onto the plane
        perpendicular to ``dir`` — the footprint a part casts for laser/water-jet
        nesting, 3D-print bed area, casting draw projection and sun/wind load.

        Computed from the surface-divergence identity: for a solid whose outline
        along ``dir`` is covered front-and-back exactly once (convex parts and
        the prismatic/blocky parts that dominate mechanical design), the projected
        area equals ``(1/2) * integral |n . d| dA`` over the closed boundary. We
        evaluate that surface integral by tessellating every face, so planar caps
        give the exact closed form and curved walls integrate to their analytic
        value. ``exact`` is True when every face is planar (result is then
        closed-form exact); curved parts converge with ``deflection``.

        args: name, dir (projection direction, default +Z),
              deflection (mesh tolerance for curved faces, default 0.05)
        """
        sh = _get(a["name"]).Shape
        d = _unit_v(_vec(a.get("dir", (0, 0, 1))))
        defl = float(a.get("deflection", 0.05))
        acc = 0.0
        all_planar = True
        for f in sh.Faces:
            if f.Surface.__class__.__name__ != "Plane":
                all_planar = False
            pts, tris = f.tessellate(defl)
            for ia, ib, ic in tris:
                cross = (pts[ib] - pts[ia]).cross(pts[ic] - pts[ia])
                acc += 0.5 * abs(cross.dot(d))  # |n.d| * triangle_area
        return {"name": a["name"],
                "dir": [_round(d.x), _round(d.y), _round(d.z)],
                "projected_area": _round(acc / 2.0, 3),
                "method": "surface-integral", "exact": all_planar,
                "faces": len(sh.Faces)}

    def _section_props(sh, n, pt, op):
        """Cut ``sh`` with the plane (normal ``n`` through ``pt``) and return the
        section's area, centroid, in-plane principal second moments of area
        (ascending eigenvalues of ``e . M . e`` for the face inertia ``M``), their
        world-space axes, and the discretised boundary points (centred). Shared by
        section-modulus and buckling so both read the same kernel section."""
        wires = sh.slice(n, n.dot(pt))
        if not wires:
            raise ValueError(
                "%s: the plane (normal=%r through %r) misses the solid"
                % (op, [_round(n.x), _round(n.y), _round(n.z)],
                   [_round(pt.x), _round(pt.y), _round(pt.z)]))
        face = Part.makeFace(wires, "Part::FaceMakerBullseye")
        c = face.CenterOfMass
        mat = face.MatrixOfInertia
        m = np.array([[mat.A11, mat.A12, mat.A13],
                      [mat.A12, mat.A22, mat.A23],
                      [mat.A13, mat.A23, mat.A33]])
        seed = V(1, 0, 0) if abs(n.x) < 0.9 else V(0, 1, 0)
        u = _unit_v(seed - n * seed.dot(n))
        vv = n.cross(u)
        U = np.array([u.x, u.y, u.z])
        Vn = np.array([vv.x, vv.y, vv.z])
        block = np.array([[U @ m @ U, U @ m @ Vn], [Vn @ m @ U, Vn @ m @ Vn]])
        vals, vecs = np.linalg.eigh(block)               # ascending eigenvalues
        axes = [U * vecs[0, k] + Vn * vecs[1, k] for k in range(2)]
        cc = np.array([c.x, c.y, c.z])
        # Discretise the boundary (not just vertices) so curved sections get a
        # faithful extreme-fibre distance.
        pts = [np.array([p.x, p.y, p.z]) - cc
               for w in wires for e in w.Edges for p in e.discretize(64)]
        return {"area": face.Area, "centroid": c, "vals": vals, "axes": axes,
                "boundary": pts, "regions": len(wires)}

    def op_section_modulus(a):
        """Cross-section bending properties of a solid cut by a plane — the beam
        numbers a structural engineer actually sizes with.

        Slices the solid with the plane through ``point`` (default the solid's
        centroid) with normal ``normal`` (default +Z), builds the section face
        (holes handled), and reads off, about the section centroid: area ``A``,
        the two principal second moments of area I (eigenvalues of the in-plane
        block of the face inertia, since the area moment about a centroidal axis
        ``e`` is ``e . M . e``), the polar moment ``J = I1 + I2`` (perpendicular-
        axis theorem), the section modulus ``S = I / c`` (``c`` the extreme-fibre
        distance measured *perpendicular* to each neutral axis), and the radius
        of gyration ``r = sqrt(I/A)``. Matches the textbook closed forms exactly
        for a rectangle (``I = b h^3/12``, ``S = b h^2/6``) and, to tessellation,
        a circle (``I = pi r^4/4``, ``S = pi r^3/4``).

        args: name, normal (cut-plane normal, default +Z),
              point (a point on the plane, default the solid centroid)
        """
        sh = _get(a["name"]).Shape
        if not sh.Solids:
            raise ValueError("section_modulus needs a solid")
        n = _unit_v(_vec(a.get("normal", (0, 0, 1))))
        pt = _vec(a["point"]) if "point" in a else _center(sh)
        sp = _section_props(sh, n, pt, "section_modulus")
        area, c, vals, axes, pts = (sp["area"], sp["centroid"], sp["vals"],
                                    sp["axes"], sp["boundary"])
        props = []
        for k in range(2):
            mom = float(vals[k])
            perp = axes[1 - k]                           # neutral axis is axes[k]
            ax = axes[k]
            cmax = max(abs(p @ perp) for p in pts)
            props.append({
                "second_moment": _round(mom, 3),
                "neutral_axis": [_round(float(ax[0]), 6), _round(float(ax[1]), 6),
                                 _round(float(ax[2]), 6)],
                "extreme_fiber": _round(cmax, 4),
                "section_modulus": _round(mom / cmax, 3) if cmax > 1e-9 else None,
                "radius_of_gyration": _round(math.sqrt(mom / area), 4) if area > 0 else None,
            })
        return {"name": a["name"],
                "normal": [_round(n.x), _round(n.y), _round(n.z)],
                "centroid": [_round(c.x), _round(c.y), _round(c.z)],
                "area": _round(area, 3),
                "polar_moment": _round(float(vals[0] + vals[1]), 3),
                "principal": props, "regions": sp["regions"]}

    def op_buckling(a):
        """Euler critical buckling load of a slender column (the solid itself).

        A compression member fails not by crushing but by buckling once the load
        reaches the Euler critical value ``P_cr = pi^2 E I / (K L)^2``, governed by
        the *smallest* principal second moment of the cross-section (it buckles
        about its weakest axis). The op cuts the solid mid-length perpendicular to
        the column ``axis`` to read the real ``I_min`` and area, takes the column
        length ``L`` along that axis (the bounding extent unless given), and
        returns the critical load, the critical (compressive) stress
        ``sigma_cr = P_cr/A = pi^2 E/lambda^2`` and the slenderness ratio
        ``lambda = K L / r_min`` (``r_min = sqrt(I_min/A)``). ``K`` is the
        end-fixity factor (1.0 pinned-pinned, 0.5 fixed-fixed, 0.699 fixed-pinned,
        2.0 fixed-free). For a rectangular column this matches the closed form
        exactly (``I_min = min(b h^3, h b^3)/12``).

        args: name, modulus E (required, e.g. MPa), axis (default +Z),
              length L (default = extent along axis), K (end-fixity, default 1.0)
        """
        sh = _get(a["name"]).Shape
        if not sh.Solids:
            raise ValueError("buckling needs a solid column")
        if "modulus" not in a:
            raise ValueError(
                "buckling needs 'modulus': Young's modulus E (consistent units, "
                "e.g. MPa with mm gives N)")
        E = float(a["modulus"])
        K = float(a.get("K", 1.0))
        axis = _unit_v(_vec(a.get("axis", (0, 0, 1))))
        corners = [V(x, y, z)
                   for x in (sh.BoundBox.XMin, sh.BoundBox.XMax)
                   for y in (sh.BoundBox.YMin, sh.BoundBox.YMax)
                   for z in (sh.BoundBox.ZMin, sh.BoundBox.ZMax)]
        proj = [p.dot(axis) for p in corners]
        L = float(a["length"]) if "length" in a else max(proj) - min(proj)
        if L <= 0:
            raise ValueError("buckling: column length along axis is zero")
        sp = _section_props(sh, axis, _center(sh), "buckling")
        area = sp["area"]
        i_min = float(sp["vals"][0])
        i_max = float(sp["vals"][1])
        r_min = math.sqrt(i_min / area) if area > 0 else 0.0
        eff = K * L
        p_cr = math.pi ** 2 * E * i_min / (eff ** 2)
        slender = eff / r_min if r_min > 0 else None
        sigma_cr = p_cr / area if area > 0 else None
        return {"name": a["name"], "modulus": E, "K": K,
                "axis": [_round(axis.x), _round(axis.y), _round(axis.z)],
                "length": _round(L, 4), "effective_length": _round(eff, 4),
                "area": _round(area, 3),
                "I_min": _round(i_min, 3), "I_max": _round(i_max, 3),
                "radius_of_gyration_min": _round(r_min, 4),
                "slenderness_ratio": _round(slender, 3) if slender else None,
                "critical_load": _round(p_cr, 3),
                "critical_stress": _round(sigma_cr, 4) if sigma_cr else None}

    def op_hydrostatics(a):
        """Free-floating hydrostatics of a solid in a fluid (naval / buoyancy).

        Given part ``density`` and ``fluid_density`` it solves the still-water
        plane (perpendicular to ``up``, default +Z) where buoyancy balances
        weight (Archimedes: rho_part*V_total = rho_fluid*V_submerged), then reads
        every quantity off the real cut solids: draft, submerged volume, centre
        of buoyancy B, centre of gravity G, waterplane area A_wp, the transverse
        metacentric radius BM = I_wp / V_sub and the metacentric height
        GM = KB + BM - KG (GM > 0 => statically stable). For a box this matches
        the closed forms exactly (T = (rho_part/rho_fluid)*H, BMt = b^2/(12 T)).

        args: name, density (part, default 1.0), fluid_density (default 1.0),
              up (default +Z)
        """
        sh = _get(a["name"]).Shape
        if not sh.Solids:
            raise ValueError("hydrostatics needs a solid with volume")
        up = _unit_v(_vec(a.get("up", (0, 0, 1))))
        rho = float(a.get("density", 1.0))
        rho_f = float(a.get("fluid_density", 1.0))
        vtot = sh.Volume
        ratio = rho / rho_f
        if ratio >= 1.0:
            return {"name": a["name"], "floats": False,
                    "ratio": _round(ratio, 4),
                    "note": "part density >= fluid density; it sinks"}
        target = ratio * vtot
        bb = sh.BoundBox
        corners = [V(x, y, z) for x in (bb.XMin, bb.XMax)
                   for y in (bb.YMin, bb.YMax) for z in (bb.ZMin, bb.ZMax)]
        s = [c.dot(up) for c in corners]
        smin, smax = min(s), max(s)
        c = V(bb.Center.x, bb.Center.y, bb.Center.z)
        big = bb.DiagonalLength * 4 + 1.0
        rot = App.Rotation(V(0, 0, 1), up)

        def submerged(w):
            # half-space below the waterline: a large box whose local +Z is the
            # ``up`` axis and whose top face sits on the plane p.up == w.
            box = Part.makeBox(2 * big, 2 * big, big, V(-big, -big, -big))
            box.Placement = App.Placement(c + up * (w - c.dot(up)), rot)
            return sh.common(box)

        lo, hi = smin, smax
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if submerged(mid).Volume < target:
                lo = mid
            else:
                hi = mid
        w = 0.5 * (lo + hi)
        sub = submerged(w)
        # a boolean ``common`` yields a Compound (no direct CenterOfMass), so
        # take the volume-weighted centroid of its solids.
        sols = sub.Solids or [sub]
        tv = sum(x.Volume for x in sols) or 1.0
        b = V(sum(x.CenterOfMass.x * x.Volume for x in sols) / tv,
              sum(x.CenterOfMass.y * x.Volume for x in sols) / tv,
              sum(x.CenterOfMass.z * x.Volume for x in sols) / tv)
        g = sh.CenterOfMass
        draft = w - smin
        # waterplane section at the solved waterline
        wires = sh.slice(up, w)
        a_wp = i_t = None
        if wires:
            face = Part.Face(wires)
            a_wp = face.Area
            m = face.MatrixOfInertia
            # in-plane second moments are the two tensor-diagonal terms not along up
            axis = next((k for k in range(3)
                         if abs((up.x, up.y, up.z)[k]) > 0.999999), None)
            if axis is not None:
                diag = [m.A11, m.A22, m.A33]
                i_t = min(d_ for k, d_ in enumerate(diag) if k != axis)
        kb = b.dot(up) - smin
        kg = g.dot(up) - smin
        out = {"name": a["name"], "floats": True, "ratio": _round(ratio, 4),
               "draft": _round(draft, 4),
               "submerged_volume": _round(sub.Volume, 4),
               "waterline": _round(w, 4),
               "center_of_buoyancy": [_round(b.x), _round(b.y), _round(b.z)],
               "center_of_gravity": [_round(g.x), _round(g.y), _round(g.z)],
               "KB": _round(kb, 4), "KG": _round(kg, 4)}
        if a_wp is not None:
            out["waterplane_area"] = _round(a_wp, 3)
        if i_t is not None and sub.Volume > 1e-9:
            bm = i_t / sub.Volume
            out["I_waterplane"] = _round(i_t, 3)
            out["BM"] = _round(bm, 4)
            out["GM"] = _round(kb + bm - kg, 4)
            out["stable"] = bool(kb + bm - kg > 0)
        return out

    def op_tolerance_stack(a):
        """1-D dimensional tolerance stack-up for an assembly chain.

        Each ``link`` is ``{nominal, plus, minus | tol, sign(=+1), name}`` where
        ``plus``/``minus`` are the unsigned upper/lower tolerances (symmetric +/-t
        may be given as ``tol``) and ``sign`` is +1 if the dimension grows the gap
        or -1 if it closes it. Reports the gap ``nominal``, the worst-case limits
        (arithmetic sum of tolerances), the statistical RSS limits (root-sum-square
        — the realistic spread for independent dimensions) and the dominant
        contributor. Pure analytic, so exact.

        args: links: [{nominal, plus, minus | tol, sign, name}], sigma(=3)
        """
        links = a["links"]
        if not links:
            raise ValueError("tolerance_stack needs at least one link")
        sigma = float(a.get("sigma", 3))
        nom = wc_plus = wc_minus = var_plus = var_minus = 0.0
        detail = []
        for i, lk in enumerate(links):
            sign = float(lk.get("sign", 1))
            n = float(lk["nominal"])
            if "tol" in lk:
                p = m = abs(float(lk["tol"]))
            else:
                p, m = abs(float(lk["plus"])), abs(float(lk["minus"]))
            nom += sign * n
            # a -1 link flips which gap extreme each tolerance pushes toward.
            tp, tm = (p, m) if sign >= 0 else (m, p)
            wc_plus += tp
            wc_minus += tm
            var_plus += tp * tp
            var_minus += tm * tm
            detail.append({"name": lk.get("name", "L%d" % (i + 1)),
                           "sign": sign, "nominal": n, "plus": p, "minus": m})
        rss_plus, rss_minus = math.sqrt(var_plus), math.sqrt(var_minus)
        dom = max(detail, key=lambda d: d["plus"] + d["minus"])
        return {"links": len(links), "nominal": _round(nom, 4),
                "worst_case": {"max": _round(nom + wc_plus, 4),
                               "min": _round(nom - wc_minus, 4),
                               "plus": _round(wc_plus, 4),
                               "minus": _round(wc_minus, 4)},
                "rss": {"max": _round(nom + rss_plus, 4),
                        "min": _round(nom - rss_minus, 4),
                        "plus": _round(rss_plus, 4),
                        "minus": _round(rss_minus, 4), "sigma": sigma},
                "dominant": dom["name"], "detail": detail}

    def op_clearance(a):
        """Minimum gap (and the closest points) between two separate solids — the
        airgap/clearance complement of ``interference``.

        Uses OCCT's exact BRep extrema (``Shape.distToShape``) so the result is
        the true geometric minimum distance between the boundaries, not a mesh
        approximation: e.g. two spheres give ``center_distance - r1 - r2`` and
        two axis-aligned boxes give the exact face-to-face air gap. When the
        solids touch or overlap the distance is ~0 and ``touching`` is True; the
        signed-style ``interfering`` flag is raised when they share volume.

        args: a, b (object names)
        returns: distance, touching, interfering, point_a, point_b
        """
        sa = _get(a["a"]).Shape
        sb = _get(a["b"]).Shape
        dist, pts, _info = sa.distToShape(sb)
        pa, pb = pts[0]
        overlap = sa.common(sb)
        interfering = bool(overlap.Solids) and overlap.Volume > 1e-6
        return {"a": a["a"], "b": a["b"], "distance": _round(dist, 4),
                "touching": dist < 1e-6, "interfering": interfering,
                "point_a": [_round(pa.x), _round(pa.y), _round(pa.z)],
                "point_b": [_round(pb.x), _round(pb.y), _round(pb.z)]}

    def op_thermal_expansion(a):
        """Free thermal growth of a solid for a uniform temperature change.

        Isotropic linear expansion: every length scales by ``1 + alpha*dT`` so
        each bounding dimension grows by ``alpha*dT*L`` and the volume by
        ``(1+alpha*dT)^3 - 1`` (~= ``3*alpha*dT`` for small strains). The op reads
        the real bounding box and volume off the kernel, then reports linear and
        volumetric strain plus the grown dimensions — the closed form behind
        shrink-fits, clearance loss at temperature and CTE-mismatch checks.

        args: name, cte (1/K, e.g. 23e-6 for Al), delta_t (K)
        """
        sh = _get(a["name"]).Shape
        alpha = float(a["cte"])
        dt = float(a["delta_t"])
        eps = alpha * dt                       # linear strain
        s = 1.0 + eps
        bb = sh.BoundBox
        dims = [bb.XLength, bb.YLength, bb.ZLength]
        v0 = sh.Volume
        return {"name": a["name"], "cte": alpha, "delta_t": dt,
                "linear_strain": _round(eps, 8),
                "volumetric_strain": _round(s**3 - 1.0, 8),
                "dims": [_round(d) for d in dims],
                "expanded_dims": [_round(d * s) for d in dims],
                "delta_dims": [_round(d * eps) for d in dims],
                "volume": _round(v0), "expanded_volume": _round(v0 * s**3)}

    def op_pressure_vessel(a):
        """Thin-wall pressure-vessel membrane stresses (Barlow / boiler formula).

        For a thin shell (``r/t >= 10``) under internal gauge ``pressure`` the
        membrane stresses are, for a ``cylinder``: hoop ``sigma_h = p*r/t`` and
        longitudinal ``sigma_l = p*r/(2t)``; for a ``sphere`` both equal
        ``p*r/(2t)``. The governing (max) stress drives a von-Mises equivalent
        and, when a material ``yield_strength`` is given, a safety factor. ``r``
        and ``t`` may be passed directly or derived from a hollow round ``name``.

        args: pressure, radius|r, thickness|t, kind(cylinder|sphere),
              yield_strength(optional), name(optional, to read r,t)
        """
        kind = a.get("kind", "cylinder")
        p = float(a["pressure"])
        r = float(a.get("radius", a.get("r")))
        t = float(a.get("thickness", a.get("t")))
        if t <= 0:
            raise ValueError("pressure_vessel needs a positive wall thickness")
        if kind == "sphere":
            sh_hoop = sl = p * r / (2.0 * t)
        else:
            sh_hoop = p * r / t
            sl = p * r / (2.0 * t)
        # plane-stress von Mises of the (sigma_h, sigma_l) membrane state.
        vm = math.sqrt(sh_hoop**2 - sh_hoop * sl + sl**2)
        out = {"kind": kind, "pressure": p, "radius": _round(r),
               "thickness": _round(t), "r_over_t": _round(r / t, 2),
               "thin_wall": (r / t) >= 10.0,
               "hoop_stress": _round(sh_hoop, 4),
               "longitudinal_stress": _round(sl, 4),
               "von_mises": _round(vm, 4)}
        if "yield_strength" in a:
            sy = float(a["yield_strength"])
            out["yield_strength"] = sy
            out["safety_factor"] = _round(sy / vm, 3) if vm > 0 else None
        return out

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
        "holes": op_holes,
        "fillets": op_fillets,
        "design_intent": op_design_intent,
        "reverse_build": op_reverse_build, "replay": op_replay,
        "reuse": op_reuse,
        "library_match": op_library_match, "library_index": op_library_index,
        "library_query": op_library_query, "library_fetch": op_library_fetch,
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
        "projected_area": op_projected_area, "hydrostatics": op_hydrostatics,
        "tolerance_stack": op_tolerance_stack, "clearance": op_clearance,
        "thermal_expansion": op_thermal_expansion,
        "pressure_vessel": op_pressure_vessel,
        "section_modulus": op_section_modulus, "buckling": op_buckling,
        "list": op_list, "delete": op_delete, "export": op_export, "import_step": op_import_step,
    }
