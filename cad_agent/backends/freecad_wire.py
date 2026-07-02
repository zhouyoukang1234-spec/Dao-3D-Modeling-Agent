"""Wire / 2-D geometry utility operations (the ``wire.*`` tool group).

Wraps FreeCAD's ``DraftGeomUtils`` and ``Part`` wire primitives to give the agent
direct access to 2-D geometry construction, analysis, and transformation: build
wires from points, offset, fillet, find intersections, mirror, compute normals.
These are the building blocks for sketch-free profile authoring, tool-path
preparation, and parametric contour manipulation.

Runs inside freecadcmd (headless).
"""

import math

import FreeCAD as App
import Part

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _vec(v):
    """Convert a list/tuple/dict to App.Vector."""
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return V(float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)
    if isinstance(v, dict):
        return V(float(v.get("x", 0)), float(v.get("y", 0)), float(v.get("z", 0)))
    return v  # already a Vector or similar


def _vec_out(v):
    return [_round(v.x), _round(v.y), _round(v.z)]


def register(state):
    import DraftGeomUtils as DGU

    def make_wire(a):
        """Build a wire from a list of 2-D/3-D points. Optionally close it."""
        points = a.get("points")
        if not isinstance(points, list) or len(points) < 2:
            raise ValueError("wire.make 'points' must be a list of >= 2 points "
                             "(each [x,y] or [x,y,z])")
        pts = [_vec(p) for p in points]
        close = bool(a.get("close", False))
        edges = [Part.makeLine(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        if close and len(pts) >= 3:
            edges.append(Part.makeLine(pts[-1], pts[0]))
        wire = Part.Wire(edges)
        name = a.get("name", "Wire")
        obj = state.doc.addObject("Part::Feature", name)
        obj.Shape = wire
        state.doc.recompute()
        return {"name": obj.Name, "edges": len(wire.Edges),
                "closed": wire.isClosed(), "length": _round(wire.Length)}

    def offset(a):
        """Offset a wire inward or outward by a distance."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.offset 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.offset: no such object %r" % obj_name)
        wire = obj.Shape
        if not isinstance(wire, Part.Wire):
            wire = Part.Wire(wire.Edges)
        dist = float(a.get("distance", 10))
        normal = _vec(a.get("normal", [0, 0, 1]))
        ow = DGU.offsetWire(wire, normal, dist)
        if ow is None:
            raise ValueError("wire.offset: offsetWire returned None (degenerate?)")
        name = a.get("name", obj_name + "_offset")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = ow
        state.doc.recompute()
        return {"name": out.Name, "edges": len(ow.Edges),
                "closed": ow.isClosed(), "length": _round(ow.Length)}

    def fillet(a):
        """Fillet all corners of a wire with a given radius."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.fillet 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.fillet: no such object %r" % obj_name)
        wire = obj.Shape
        if not isinstance(wire, Part.Wire):
            wire = Part.Wire(wire.Edges)
        radius = float(a.get("radius", 5))
        fw = DGU.filletWire(wire, radius)
        if fw is None:
            raise ValueError("wire.fillet: filletWire returned None")
        name = a.get("name", obj_name + "_fillet")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = fw
        state.doc.recompute()
        return {"name": out.Name, "edges": len(fw.Edges),
                "closed": fw.isClosed(), "length": _round(fw.Length)}

    def normal(a):
        """Compute the normal of a planar wire."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.normal 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.normal: no such object %r" % obj_name)
        wire = obj.Shape
        if not isinstance(wire, Part.Wire):
            wire = Part.Wire(wire.Edges)
        n = DGU.getNormal(wire)
        if n is None:
            return {"normal": None, "planar": False}
        return {"normal": _vec_out(n), "planar": True}

    def intersect(a):
        """Find intersection points between two edges/wires."""
        name_a = a.get("a")
        name_b = a.get("b")
        if not isinstance(name_a, str) or not isinstance(name_b, str):
            raise ValueError("wire.intersect 'a' and 'b' must be object name strings")
        obj_a = state.doc.getObject(name_a)
        obj_b = state.doc.getObject(name_b)
        if obj_a is None:
            raise ValueError("wire.intersect: no such object %r" % name_a)
        if obj_b is None:
            raise ValueError("wire.intersect: no such object %r" % name_b)
        shape_a = obj_a.Shape
        shape_b = obj_b.Shape
        # Use first edge of each
        ea = shape_a.Edges[0] if shape_a.Edges else shape_a
        eb = shape_b.Edges[0] if shape_b.Edges else shape_b
        pts = DGU.findIntersection(ea, eb)
        return {"points": [_vec_out(p) for p in pts],
                "count": len(pts)}

    def mirror_wire(a):
        """Mirror a wire about a plane defined by a point and normal axis."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.mirror 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.mirror: no such object %r" % obj_name)
        point = _vec(a.get("point", [0, 0, 0]))
        axis = _vec(a.get("axis", [1, 0, 0]))
        mirrored = obj.Shape.mirror(point, axis)
        name = a.get("name", obj_name + "_mirror")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = mirrored
        state.doc.recompute()
        return {"name": out.Name, "edges": len(mirrored.Edges),
                "length": _round(mirrored.Length)}

    def info(a):
        """Wire analysis: length, closed, planar, edge count, bounding box."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.info 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.info: no such object %r" % obj_name)
        shape = obj.Shape
        wire = shape if isinstance(shape, Part.Wire) else Part.Wire(shape.Edges)
        n = DGU.getNormal(wire)
        bb = wire.BoundBox
        return {
            "name": obj.Name,
            "edges": len(wire.Edges),
            "length": _round(wire.Length),
            "closed": wire.isClosed(),
            "planar": n is not None,
            "normal": _vec_out(n) if n else None,
            "bbox": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)],
        }

    def extrude(a):
        """Extrude a wire into a solid along a direction vector."""
        obj_name = a.get("wire")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("wire.extrude 'wire' must be an object name string")
        obj = state.doc.getObject(obj_name)
        if obj is None:
            raise ValueError("wire.extrude: no such object %r" % obj_name)
        shape = obj.Shape
        wire = shape if isinstance(shape, Part.Wire) else Part.Wire(shape.Edges)
        direction = _vec(a.get("direction", [0, 0, 10]))
        if not wire.isClosed():
            raise ValueError("wire.extrude: wire must be closed to produce a solid")
        face = Part.Face(wire)
        solid = face.extrude(direction)
        name = a.get("name", obj_name + "_extruded")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = solid
        state.doc.recompute()
        return {"name": out.Name, "volume": _round(solid.Volume),
                "area": _round(solid.Area),
                "shape_type": solid.ShapeType}

    def circle_wire(a):
        """Create a circle wire at given center, radius, normal."""
        center = _vec(a.get("center", [0, 0, 0]))
        radius = float(a.get("radius", 10))
        normal_dir = _vec(a.get("normal", [0, 0, 1]))
        if radius <= 0:
            raise ValueError("wire.circle 'radius' must be positive (got %s)" % radius)
        circle = Part.Circle(center, normal_dir, radius)
        edge = circle.toShape()
        wire = Part.Wire([edge])
        name = a.get("name", "CircleWire")
        obj = state.doc.addObject("Part::Feature", name)
        obj.Shape = wire
        state.doc.recompute()
        return {"name": obj.Name, "radius": _round(radius),
                "length": _round(wire.Length),
                "circumference": _round(2 * math.pi * radius)}

    def arc_wire(a):
        """Create an arc wire from center, radius, start/end angles (degrees)."""
        center = _vec(a.get("center", [0, 0, 0]))
        radius = float(a.get("radius", 10))
        start_angle = float(a.get("start", 0))
        end_angle = float(a.get("end", 180))
        normal_dir = _vec(a.get("normal", [0, 0, 1]))
        if radius <= 0:
            raise ValueError("wire.arc 'radius' must be positive (got %s)" % radius)
        arc = Part.Circle(center, normal_dir, radius)
        edge = arc.toShape(math.radians(start_angle), math.radians(end_angle))
        wire = Part.Wire([edge])
        name = a.get("name", "ArcWire")
        obj = state.doc.addObject("Part::Feature", name)
        obj.Shape = wire
        state.doc.recompute()
        return {"name": obj.Name, "radius": _round(radius),
                "length": _round(wire.Length),
                "start": _round(start_angle), "end": _round(end_angle)}

    return {
        "wire.make": make_wire,
        "wire.offset": offset,
        "wire.fillet": fillet,
        "wire.normal": normal,
        "wire.intersect": intersect,
        "wire.mirror": mirror_wire,
        "wire.info": info,
        "wire.extrude": extrude,
        "wire.circle": circle_wire,
        "wire.arc": arc_wire,
    }
