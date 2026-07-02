"""Boolean / split / join compound operations (the ``bop.*`` tool group).

Wraps FreeCAD's ``BOPTools`` module for advanced boolean operations that go
beyond simple union/cut/intersect:

- **slice**: slice a shape with one or more cutting shapes, producing pieces
- **fragments**: boolean-fragment decomposition into disjoint volume cells
- **xor**: symmetric difference (parts unique to each shape)
- **connect**: fuse shapes keeping internal boundaries
- **embed**: embed one shape's faces into another
- **cutout**: boolean cutout (hollow-out)

These complement ``solid.fuse/cut/intersect`` (Part boolean ops) with the more
advanced BOPTools split/join pipeline used in multi-body, mold, and assembly
workflows.

Runs inside freecadcmd (headless).
"""

import FreeCAD as App

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _shape_summary(shape):
    return {
        "shape_type": shape.ShapeType,
        "solids": len(shape.Solids),
        "faces": len(shape.Faces),
        "edges": len(shape.Edges),
        "volume": _round(shape.Volume) if shape.Solids else None,
        "area": _round(shape.Area) if shape.Faces else None,
    }


def register(state):
    import BOPTools.SplitAPI as SplitAPI
    import BOPTools.JoinAPI as JoinAPI

    def _get_obj(name):
        obj = state.doc.getObject(name)
        if obj is None:
            raise ValueError("no such object %r" % name)
        return obj

    def slice_op(a):
        """Slice a shape with one or more cutting shapes."""
        base_name = a.get("base")
        tool_names = a.get("tools", [])
        if not isinstance(base_name, str) or not base_name:
            raise ValueError("bop.slice 'base' must be an object name string")
        if not isinstance(tool_names, list) or not tool_names:
            raise ValueError("bop.slice 'tools' must be a non-empty list of names")
        base_obj = _get_obj(base_name)
        tool_shapes = [_get_obj(n).Shape for n in tool_names]
        mode = a.get("mode", "Standard")
        result = SplitAPI.slice(base_obj.Shape, tool_shapes, mode)
        name = a.get("name", base_name + "_sliced")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = result
        state.doc.recompute()
        d = _shape_summary(result)
        d["name"] = out.Name
        return d

    def fragments(a):
        """Boolean fragment decomposition into disjoint volume cells."""
        shape_names = a.get("shapes", [])
        if not isinstance(shape_names, list) or len(shape_names) < 2:
            raise ValueError("bop.fragments 'shapes' must be a list of >= 2 names")
        shapes = [_get_obj(n).Shape for n in shape_names]
        mode = a.get("mode", "Standard")
        result = SplitAPI.booleanFragments(shapes, mode)
        name = a.get("name", "Fragments")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = result
        state.doc.recompute()
        d = _shape_summary(result)
        d["name"] = out.Name
        return d

    def xor_op(a):
        """Symmetric difference: parts unique to each shape."""
        shape_names = a.get("shapes", [])
        if not isinstance(shape_names, list) or len(shape_names) < 2:
            raise ValueError("bop.xor 'shapes' must be a list of >= 2 names")
        shapes = [_get_obj(n).Shape for n in shape_names]
        tolerance = float(a.get("tolerance", 0.0))
        result = SplitAPI.xor(shapes, tolerance)
        name = a.get("name", "XOR")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = result
        state.doc.recompute()
        d = _shape_summary(result)
        d["name"] = out.Name
        return d

    def connect(a):
        """Fuse shapes keeping internal boundaries."""
        shape_names = a.get("shapes", [])
        if not isinstance(shape_names, list) or len(shape_names) < 2:
            raise ValueError("bop.connect 'shapes' must be a list of >= 2 names")
        shapes = [_get_obj(n).Shape for n in shape_names]
        result = JoinAPI.connect(shapes)
        name = a.get("name", "Connected")
        out = state.doc.addObject("Part::Feature", name)
        out.Shape = result
        state.doc.recompute()
        d = _shape_summary(result)
        d["name"] = out.Name
        return d

    return {
        "bop.slice": slice_op,
        "bop.fragments": fragments,
        "bop.xor": xor_op,
        "bop.connect": connect,
    }
