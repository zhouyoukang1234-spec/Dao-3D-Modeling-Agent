"""Direct sub-element measurement operations (the ``measure.*`` tool group).

Wraps FreeCAD's ``Measure.Measurement`` class to expose precise geometric
measurement of individual faces, edges, vertices, and whole solids — area,
length, radius, angle, volume, center-of-mass, and inter-element distances.
These complement the existing ``analyze.*`` ops (which measure between named
solids) by reaching into sub-elements: "the area of Face3 on part X", "the
radius of Edge5", "the angle between Face1 and Face2".

Runs inside freecadcmd (headless).
"""

import FreeCAD as App

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _vec_out(v):
    if v is None:
        return None
    return [_round(v.x), _round(v.y), _round(v.z)]


def register(state):
    from Measure import Measurement

    def _get_obj(name):
        obj = state.doc.getObject(name)
        if obj is None:
            raise ValueError("no such object %r" % name)
        return obj

    def area(a):
        """Measure the area of a face sub-element (e.g. 'Face1') on an object."""
        obj_name = a.get("object")
        sub = a.get("sub", "")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("measure.area 'object' must be an object name string")
        _get_obj(obj_name)
        m = Measurement()
        m.addReference3D(obj_name, sub)
        val = m.area()
        m.clear()
        return {"object": obj_name, "sub": sub, "area": _round(val)}

    def length(a):
        """Measure the length of an edge sub-element (e.g. 'Edge1')."""
        obj_name = a.get("object")
        sub = a.get("sub", "")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("measure.length 'object' must be an object name string")
        _get_obj(obj_name)
        m = Measurement()
        m.addReference3D(obj_name, sub)
        val = m.length()
        m.clear()
        return {"object": obj_name, "sub": sub, "length": _round(val)}

    def radius(a):
        """Measure the radius of a circular edge or cylindrical face."""
        obj_name = a.get("object")
        sub = a.get("sub", "")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("measure.radius 'object' must be an object name string")
        _get_obj(obj_name)
        m = Measurement()
        m.addReference3D(obj_name, sub)
        val = m.radius()
        m.clear()
        return {"object": obj_name, "sub": sub, "radius": _round(val)}

    def volume(a):
        """Measure the volume of a whole solid."""
        obj_name = a.get("object")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("measure.volume 'object' must be an object name string")
        _get_obj(obj_name)
        m = Measurement()
        m.addReference3D(obj_name, "")
        val = m.volume()
        m.clear()
        return {"object": obj_name, "volume": _round(val)}

    def com(a):
        """Measure the center of mass of a solid."""
        obj_name = a.get("object")
        if not isinstance(obj_name, str) or not obj_name:
            raise ValueError("measure.com 'object' must be an object name string")
        _get_obj(obj_name)
        m = Measurement()
        m.addReference3D(obj_name, "")
        val = m.com()
        m.clear()
        return {"object": obj_name, "com": _vec_out(val)}

    def angle(a):
        """Measure the angle between two sub-elements (faces or edges)."""
        obj_a = a.get("a")
        sub_a = a.get("sub_a", "")
        obj_b = a.get("b")
        sub_b = a.get("sub_b", "")
        if not isinstance(obj_a, str) or not isinstance(obj_b, str):
            raise ValueError("measure.angle 'a' and 'b' must be object name strings")
        _get_obj(obj_a)
        _get_obj(obj_b)
        m = Measurement()
        m.addReference3D(obj_a, sub_a)
        m.addReference3D(obj_b, sub_b)
        val = m.angle()
        m.clear()
        return {"a": obj_a, "sub_a": sub_a,
                "b": obj_b, "sub_b": sub_b, "angle": _round(val)}

    def delta(a):
        """Measure the delta vector between two sub-elements."""
        obj_a = a.get("a")
        sub_a = a.get("sub_a", "")
        obj_b = a.get("b")
        sub_b = a.get("sub_b", "")
        if not isinstance(obj_a, str) or not isinstance(obj_b, str):
            raise ValueError("measure.delta 'a' and 'b' must be object name strings")
        _get_obj(obj_a)
        _get_obj(obj_b)
        m = Measurement()
        m.addReference3D(obj_a, sub_a)
        m.addReference3D(obj_b, sub_b)
        val = m.delta()
        m.clear()
        return {"a": obj_a, "b": obj_b,
                "delta": _vec_out(val),
                "distance": _round(val.Length) if val else None}

    def plane_distance(a):
        """Measure the distance between two parallel planar faces."""
        obj_a = a.get("a")
        sub_a = a.get("sub_a", "Face1")
        obj_b = a.get("b")
        sub_b = a.get("sub_b", "Face1")
        if not isinstance(obj_a, str) or not isinstance(obj_b, str):
            raise ValueError("measure.plane_distance 'a' and 'b' must be object name strings")
        _get_obj(obj_a)
        _get_obj(obj_b)
        m = Measurement()
        m.addReference3D(obj_a, sub_a)
        m.addReference3D(obj_b, sub_b)
        val = m.planePlaneDistance()
        m.clear()
        return {"a": obj_a, "sub_a": sub_a,
                "b": obj_b, "sub_b": sub_b,
                "distance": _round(val)}

    return {
        "measure.area": area,
        "measure.length": length,
        "measure.radius": radius,
        "measure.volume": volume,
        "measure.com": com,
        "measure.angle": angle,
        "measure.delta": delta,
        "measure.plane_distance": plane_distance,
    }
