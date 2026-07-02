"""Structural 3-D perception (the ``percept.*`` tool group).

The agent's *native* eyes. Humans perceive a model by rendering pixels; an AI
perceives it by reading exact structure. This module converts live B-rep
geometry into precise, stable, describable JSON the agent can reason over
without any image in the loop:

- ``percept.topology``   B-rep face-adjacency graph: every face typed
  (plane/cylinder/sphere/cone/torus/freeform) with area, centroid, normal or
  axis; every edge typed (line/circle/...) with length and **convexity**
  (convex / concave / smooth) determined by material sampling.
- ``percept.features``   engineering feature recognition from the topology:
  through-holes, blind holes, bosses, fillets -- the vocabulary of intent.
- ``percept.section``    planar cross-section -> closed polyline loops with
  exact vertex coordinates; reading a solid the way a CT scanner does.
- ``percept.relations``  pairwise spatial predicates between document objects:
  distance, contact, overlap, containment, relative direction.
- ``percept.scene``      whole-document digest: every object summarized with
  its bbox, primary dimensions, and the relation matrix.
- ``percept.describe``   canonical natural-language description synthesized
  from topology + features, one shape -> one stable paragraph.

This is the third channel of the perceive/verify loop: scalar metrics come
from ``solid.measure``/``measure.*``, optics from ``view.*``, and *structure*
from ``percept.*``. 不出於戶，以知天下.

Runs inside freecadcmd (headless).
"""

import math

import FreeCAD as App
import Part

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _v(v):
    return [_round(v.x), _round(v.y), _round(v.z)]


def _surface_info(face):
    s = face.Surface
    kind = type(s).__name__
    d = {"kind": kind.lower()}
    if kind == "Plane":
        d["normal"] = _v(face.normalAt(0, 0))
        d["position"] = _v(s.Position)
    elif kind == "Cylinder":
        d["radius"] = _round(s.Radius)
        d["axis"] = _v(s.Axis)
        d["center"] = _v(s.Center)
    elif kind == "Sphere":
        d["radius"] = _round(s.Radius)
        d["center"] = _v(s.Center)
    elif kind == "Cone":
        d["half_angle_deg"] = _round(math.degrees(s.SemiAngle))
        d["axis"] = _v(s.Axis)
        d["apex"] = _v(s.Apex)
    elif kind == "Toroid":
        d["major_radius"] = _round(s.MajorRadius)
        d["minor_radius"] = _round(s.MinorRadius)
        d["axis"] = _v(s.Axis)
        d["center"] = _v(s.Center)
    else:
        d["kind"] = "freeform:" + kind.lower()
    return d


def _curve_info(edge):
    # some OCC edges (e.g. seam/degenerate edges left by fillets) raise
    # "TypeError: undefined curve type" on .Curve access; report what we can.
    try:
        c = edge.Curve
    except Exception:
        return {"kind": "unknown", "length": _round(edge.Length)}
    kind = type(c).__name__
    d = {"kind": kind.lower(), "length": _round(edge.Length)}
    if kind == "Circle":
        d["radius"] = _round(c.Radius)
        d["center"] = _v(c.Center)
    return d


def _face_of_edge_map(shape):
    """edge index (1-based) -> list of face indices (1-based) sharing it."""
    emap = {}
    for fi, face in enumerate(shape.Faces, 1):
        for e in face.Edges:
            for ei, se in enumerate(shape.Edges, 1):
                if e.isSame(se):
                    emap.setdefault(ei, []).append(fi)
                    break
    return emap


def _edge_convexity(shape, edge, face_a, face_b):
    """convex / concave / smooth from the dihedral between the two faces.

    At the edge midpoint take each face's outward normal and its in-surface
    direction (from the edge toward the face interior, flattened onto the
    tangent plane). The edge is convex when each face's interior falls *below*
    the other face's tangent plane (d2.n1 < 0, like a box corner) and concave
    when it rises above it (like a pocket floor edge). Near-parallel normals:
    smooth (tangent-continuous, e.g. a fillet blend).
    """
    try:
        t = edge.FirstParameter + 0.5 * (edge.LastParameter - edge.FirstParameter)
        p = edge.valueAt(t)

        def normal_near(face, point):
            u, v = face.Surface.parameter(point)
            return face.normalAt(u, v)

        n1 = normal_near(face_a, p)
        n2 = normal_near(face_b, p)
        if n1.getAngle(n2) < 0.05:  # ~3 degrees: tangent-continuous
            return "smooth"

        tv = edge.tangentAt(t)
        eps = max(1e-6, shape.BoundBox.DiagonalLength * 1e-3)

        def into_face(face, n):
            # in-surface direction perpendicular to the edge; pick the sign
            # whose probe point actually lies on the face (a centroid heading
            # misleads on annular faces, whose centroid sits in the hole).
            d = n.cross(tv)
            if d.Length > 1e-9:
                d.normalize()
                for cand in (d, -d):
                    q = p + cand * eps
                    dist = face.distToShape(Part.Vertex(q))[0]
                    if dist < eps * 0.5:
                        return cand
            d = face.CenterOfMass - p
            d = d - n * d.dot(n)
            if d.Length < 1e-9:
                return None
            d.normalize()
            return d

        d1 = into_face(face_a, n1)
        d2 = into_face(face_b, n2)
        if d1 is None or d2 is None:
            return "unknown"
        s1 = d2.dot(n1)
        s2 = d1.dot(n2)
        if abs(s1) < 1e-6 and abs(s2) < 1e-6:
            return "smooth"
        return "concave" if (s1 + s2) > 0 else "convex"
    except Exception:
        return "unknown"


def _build_topology(shape):
    faces = []
    for fi, face in enumerate(shape.Faces, 1):
        d = {"id": "Face%d" % fi, "area": _round(face.Area),
             "centroid": _v(face.CenterOfMass),
             "wires": len(face.Wires), "edges": len(face.Edges)}
        d.update(_surface_info(face))
        faces.append(d)
    emap = _face_of_edge_map(shape)
    edges = []
    for ei, edge in enumerate(shape.Edges, 1):
        d = {"id": "Edge%d" % ei}
        d.update(_curve_info(edge))
        adj = emap.get(ei, [])
        d["faces"] = ["Face%d" % i for i in adj]
        if len(adj) == 2:
            d["convexity"] = _edge_convexity(
                shape, edge, shape.Faces[adj[0] - 1], shape.Faces[adj[1] - 1])
        edges.append(d)
    return {"faces": faces, "edges": edges,
            "counts": {"faces": len(faces), "edges": len(edges),
                       "vertices": len(shape.Vertexes),
                       "solids": len(shape.Solids),
                       "shells": len(shape.Shells)}}


def _recognize_features(shape, topo=None):
    """Holes, bosses, fillets from cylinder-face concavity + end probing."""
    topo = topo or _build_topology(shape)
    features = []
    smooth_faces = set()
    for ed in topo["edges"]:
        if ed.get("convexity") == "smooth":
            smooth_faces.update(ed["faces"])
    for fd in topo["faces"]:
        if fd["kind"] != "cylinder":
            continue
        fi = int(fd["id"][4:])
        face = shape.Faces[fi - 1]
        # concave cylindrical face (normal points at the axis) => hole
        u, v = face.ParameterRange[0], face.ParameterRange[2]
        p = face.valueAt(u, v)
        n = face.normalAt(u, v)
        axis_pt = V(*fd["center"])
        axis_dir = V(*fd["axis"])
        # vector from surface point to its projection on the axis
        w = p - axis_pt
        w_axis = axis_dir * w.dot(axis_dir)
        to_axis = (w_axis - w)
        if to_axis.Length > 1e-9:
            to_axis.normalize()
        concave = n.dot(to_axis) > 0
        # extent of this face along its own axis
        ts = [(vx.Point - axis_pt).dot(axis_dir) for vx in face.Vertexes]
        if ts:
            t_lo, t_hi = min(ts), max(ts)
        else:
            t_lo = t_hi = 0.0
        depth = t_hi - t_lo
        if concave:
            # probe on the hole axis just beyond each end: material there
            # means that end is capped (blind); both ends open => through.
            eps = max(depth * 0.02, shape.BoundBox.DiagonalLength * 1e-4)
            tol = shape.BoundBox.DiagonalLength * 1e-7
            lo_open = not shape.isInside(
                axis_pt + axis_dir * (t_lo - eps), tol, False)
            hi_open = not shape.isInside(
                axis_pt + axis_dir * (t_hi + eps), tol, False)
            if lo_open and hi_open:
                kind = "through_hole"
            elif lo_open or hi_open:
                kind = "blind_hole"
            else:
                kind = "internal_void"
            features.append({
                "type": kind, "face": fd["id"], "radius": fd["radius"],
                "depth": _round(depth), "axis": fd["axis"],
                "center": fd["center"]})
        else:
            # a fillet blends tangentially into its neighbours (smooth edges);
            # a boss meets them at sharp edges.
            kind = "fillet" if fd["id"] in smooth_faces else "boss"
            features.append({
                "type": kind, "face": fd["id"], "radius": fd["radius"],
                "axis": fd["axis"], "center": fd["center"],
                "height": _round(depth)})
    return features


def _detect_patterns(features):
    """Group repeated same-size features into circular / linear patterns.

    Repetition is design intent (a bolt circle, a rack of holes); reading it
    back collapses N features into one describable fact.
    """
    groups = {}
    for f in features:
        key = (f["type"], round(f["radius"], 4),
               tuple(round(abs(c), 4) for c in f.get("axis", [0, 0, 0])))
        groups.setdefault(key, []).append(f)
    patterns = []
    for (ftype, radius, _axis), members in groups.items():
        if len(members) < 3:
            continue
        centers = [V(*m["center"]) for m in members]
        centroid = V(0, 0, 0)
        for c in centers:
            centroid = centroid + c
        centroid = centroid * (1.0 / len(centers))
        dists = [(c - centroid).Length for c in centers]
        mean_d = sum(dists) / len(dists)
        if mean_d > 1e-6 and all(abs(d - mean_d) < 1e-3 + mean_d * 1e-3
                                 for d in dists):
            patterns.append({
                "type": "circular_pattern", "of": ftype, "count": len(members),
                "feature_radius": radius, "circle_radius": _round(mean_d),
                "center": _v(centroid),
                "members": [m["face"] for m in members]})
            continue
        # linear: all centers collinear with equal spacing
        d0 = centers[-1] - centers[0]
        if d0.Length > 1e-6:
            d0.normalize()
            offs = sorted((c - centers[0]).dot(d0) for c in centers)
            steps = [offs[i + 1] - offs[i] for i in range(len(offs) - 1)]
            coll = all(((c - centers[0]) - d0 * (c - centers[0]).dot(d0)).Length
                       < 1e-3 for c in centers)
            if coll and steps and all(abs(st - steps[0]) < 1e-3 for st in steps):
                patterns.append({
                    "type": "linear_pattern", "of": ftype,
                    "count": len(members), "feature_radius": radius,
                    "spacing": _round(steps[0]), "direction": _v(d0),
                    "members": [m["face"] for m in members]})
    return patterns


def _describe(name, shape, topo, features):
    bb = shape.BoundBox
    kinds = {}
    for f in topo["faces"]:
        k = f["kind"].split(":")[0]
        kinds[k] = kinds.get(k, 0) + 1
    conv = {}
    for e in topo["edges"]:
        c = e.get("convexity")
        if c:
            conv[c] = conv.get(c, 0) + 1
    parts = []
    parts.append("%s: %d solid(s), bbox %.4g x %.4g x %.4g mm, volume %.6g mm^3,"
                 " surface %.6g mm^2." % (
                     name, len(shape.Solids), bb.XLength, bb.YLength,
                     bb.ZLength, shape.Volume, shape.Area))
    parts.append("Topology: %d faces (%s), %d edges (%s), %d vertices." % (
        topo["counts"]["faces"],
        ", ".join("%d %s" % (n, k) for k, n in sorted(kinds.items())),
        topo["counts"]["edges"],
        ", ".join("%d %s" % (n, k) for k, n in sorted(conv.items())) or "n/a",
        topo["counts"]["vertices"]))
    if features:
        patterns = _detect_patterns(features)
        pattern_members = set()
        fparts = []
        for pt in patterns:
            pattern_members.update(pt["members"])
            if pt["type"] == "circular_pattern":
                fparts.append("%d x %s r=%.4g on a %.4g-radius circle at %s" % (
                    pt["count"], pt["of"], pt["feature_radius"],
                    pt["circle_radius"], pt["center"]))
            else:
                fparts.append("%d x %s r=%.4g spaced %.4g along %s" % (
                    pt["count"], pt["of"], pt["feature_radius"],
                    pt["spacing"], pt["direction"]))
        for ft in features:
            if ft["face"] in pattern_members:
                continue
            if "hole" in ft["type"] or ft["type"] == "internal_void":
                fparts.append("%s r=%.4g at %s along %s" % (
                    ft["type"], ft["radius"], ft["center"], ft["axis"]))
            else:
                fparts.append("%s r=%.4g" % (ft["type"], ft["radius"]))
        parts.append("Features: " + "; ".join(fparts) + ".")
    else:
        parts.append("Features: none recognized.")
    com = shape.Solids[0].CenterOfMass if len(shape.Solids) == 1 else None
    if com is not None:
        parts.append("Center of mass at (%.4g, %.4g, %.4g)." % (com.x, com.y, com.z))
    return " ".join(parts)


def register(state):

    def _get(name):
        obj = state.doc.getObject(name)
        if obj is None:
            raise ValueError("no such object %r" % name)
        shape = getattr(obj, "Shape", None)
        if shape is None or shape.isNull():
            raise ValueError("object %r has no shape" % name)
        return obj, shape

    def _req_name(a, op, key="object"):
        n = a.get(key)
        if not isinstance(n, str) or not n:
            raise ValueError("%s '%s' must be an object name string (got %r)"
                             % (op, key, n))
        return n

    def topology(a):
        name = _req_name(a, "percept.topology")
        _, shape = _get(name)
        topo = _build_topology(shape)
        topo["object"] = name
        return topo

    def features(a):
        name = _req_name(a, "percept.features")
        _, shape = _get(name)
        feats = _recognize_features(shape)
        return {"object": name, "features": feats, "count": len(feats),
                "patterns": _detect_patterns(feats)}

    def diff(a):
        na = _req_name(a, "percept.diff", "a")
        nb = _req_name(a, "percept.diff", "b")
        _, sa = _get(na)
        _, sb = _get(nb)
        added = sb.cut(sa)
        removed = sa.cut(sb)
        fa = _recognize_features(sa)
        fb = _recognize_features(sb)

        def sig(f):
            return (f["type"], round(f["radius"], 3),
                    tuple(round(x, 3) for x in f["center"]))

        sa_sigs = {sig(f) for f in fa}
        sb_sigs = {sig(f) for f in fb}
        ba, bb_ = sa.BoundBox, sb.BoundBox
        return {
            "a": na, "b": nb,
            "volume_delta": _round(sb.Volume - sa.Volume),
            "area_delta": _round(sb.Area - sa.Area),
            "material_added": _round(added.Volume if added.Solids else 0.0),
            "material_removed": _round(
                removed.Volume if removed.Solids else 0.0),
            "bbox_delta": [_round(bb_.XLength - ba.XLength),
                           _round(bb_.YLength - ba.YLength),
                           _round(bb_.ZLength - ba.ZLength)],
            "face_count_delta": len(sb.Faces) - len(sa.Faces),
            "features_gained": [f for f in fb if sig(f) not in sa_sigs],
            "features_lost": [f for f in fa if sig(f) not in sb_sigs],
            "identical": (abs(sb.Volume - sa.Volume) < 1e-9
                          and not (added.Solids or removed.Solids)),
        }

    def section(a):
        name = _req_name(a, "percept.section")
        _, shape = _get(name)
        normal = a.get("normal", [0, 0, 1])
        if not isinstance(normal, (list, tuple)) or len(normal) != 3:
            raise ValueError("percept.section 'normal' must be [x,y,z]")
        offset = a.get("offset")
        n = V(*[float(x) for x in normal])
        if n.Length < 1e-12:
            raise ValueError("percept.section 'normal' must be non-zero")
        if offset is None:
            c = shape.BoundBox.Center
            offset = c.dot(n) / n.Length
        wires = shape.slice(n, float(offset))
        loops = []
        for w in wires:
            pts = []
            for e in w.OrderedEdges if hasattr(w, "OrderedEdges") else w.Edges:
                k = max(2, int(math.ceil(e.Length / max(
                    shape.BoundBox.DiagonalLength * 0.02, 1e-6))) + 1)
                for i in range(k):
                    t = e.FirstParameter + (e.LastParameter - e.FirstParameter) \
                        * i / (k - 1)
                    pts.append(_v(e.valueAt(t)))
            loops.append({"closed": w.isClosed(), "length": _round(w.Length),
                          "points": pts})
        return {"object": name, "normal": [_round(x) for x in normal],
                "offset": _round(offset), "loops": loops,
                "loop_count": len(loops)}

    def relations(a):
        names = a.get("objects")
        if names is None:
            names = [o.Name for o in state.doc.Objects
                     if getattr(o, "Shape", None) is not None
                     and not o.Shape.isNull() and o.Shape.Solids]
        if not isinstance(names, list) or len(names) < 2:
            raise ValueError("percept.relations needs >= 2 shaped objects "
                             "(got %r)" % (names,))
        shapes = {}
        for n in names:
            _, shapes[n] = _get(n)
        rels = []
        for i, na in enumerate(names):
            for nb in names[i + 1:]:
                sa, sb = shapes[na], shapes[nb]
                dist = sa.distToShape(sb)[0]
                rel = {"a": na, "b": nb, "distance": _round(dist)}
                if dist < 1e-7:
                    common = sa.common(sb)
                    overlap = common.Volume if common.Solids else 0.0
                    if overlap > 1e-9:
                        if abs(overlap - sb.Volume) < 1e-6:
                            rel["relation"] = "a_contains_b"
                        elif abs(overlap - sa.Volume) < 1e-6:
                            rel["relation"] = "b_contains_a"
                        else:
                            rel["relation"] = "overlap"
                        rel["overlap_volume"] = _round(overlap)
                    else:
                        rel["relation"] = "contact"
                else:
                    rel["relation"] = "apart"
                    d = sb.BoundBox.Center - sa.BoundBox.Center
                    if d.Length > 1e-12:
                        d.normalize()
                        axes = [("+x", V(1, 0, 0)), ("-x", V(-1, 0, 0)),
                                ("+y", V(0, 1, 0)), ("-y", V(0, -1, 0)),
                                ("+z", V(0, 0, 1)), ("-z", V(0, 0, -1))]
                        rel["direction_b_from_a"] = max(
                            axes, key=lambda kv: d.dot(kv[1]))[0]
                rels.append(rel)
        return {"objects": names, "relations": rels}

    def scene(a):
        objs = []
        names = []
        for o in state.doc.Objects:
            shape = getattr(o, "Shape", None)
            if shape is None or shape.isNull():
                continue
            bb = shape.BoundBox
            d = {"name": o.Name, "label": getattr(o, "Label", ""),
                 "type": o.TypeId,
                 "bbox_min": [_round(bb.XMin), _round(bb.YMin), _round(bb.ZMin)],
                 "bbox_max": [_round(bb.XMax), _round(bb.YMax), _round(bb.ZMax)],
                 "faces": len(shape.Faces), "solids": len(shape.Solids)}
            if shape.Solids:
                d["volume"] = _round(shape.Volume)
                names.append(o.Name)
            objs.append(d)
        out = {"document": state.doc.Name, "objects": objs,
               "object_count": len(objs)}
        if len(names) >= 2 and a.get("relations", True):
            out["relations"] = relations({"objects": names})["relations"]
        return out

    def describe(a):
        name = _req_name(a, "percept.describe")
        _, shape = _get(name)
        topo = _build_topology(shape)
        feats = _recognize_features(shape, topo)
        return {"object": name,
                "description": _describe(name, shape, topo, feats),
                "feature_count": len(feats)}

    return {
        "percept.topology": topology,
        "percept.features": features,
        "percept.section": section,
        "percept.relations": relations,
        "percept.scene": scene,
        "percept.describe": describe,
        "percept.diff": diff,
    }
