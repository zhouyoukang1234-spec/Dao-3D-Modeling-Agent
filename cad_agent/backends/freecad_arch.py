"""BIM / Architecture operations (the ``bim.*`` tool group).

Wraps FreeCAD's ``Arch`` module to expose BIM authoring: walls, structures
(columns/beams), floors, buildings, sites, and the IFC-style spatial hierarchy.
Every element is a real ``Part::FeaturePython`` in the live document with a
proper ``Shape`` and BIM metadata (material, IFC type). Operations compose: a
wall from a wire, a floor from walls, a building from floors, a site from
buildings.

Runs inside freecadcmd (headless). The heavy Arch module is imported lazily on
first use.
"""

import FreeCAD as App

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _shape_summary(obj):
    d = {"name": obj.Name, "type": obj.TypeId, "label": getattr(obj, "Label", "")}
    shp = getattr(obj, "Shape", None)
    if shp is not None:
        try:
            d["volume"] = _round(shp.Volume)
            d["area"] = _round(shp.Area) if hasattr(shp, "Area") else None
            d["shape_type"] = shp.ShapeType
        except Exception:
            pass
    return d


def register(state):
    import Arch

    def wall(a):
        length = a.get("length", 4000)
        width = a.get("width", 200)
        height = a.get("height", 3000)
        name = a.get("name")
        base_name = a.get("base")
        if base_name:
            base_obj = state.doc.getObject(base_name)
            if base_obj is None:
                raise ValueError("bim.wall: no such base object %r" % base_name)
            w = Arch.makeWall(base_obj, width=width, height=height)
        else:
            w = Arch.makeWall(length=length, width=width, height=height)
        if name and isinstance(name, str):
            w.Label = name
        state.doc.recompute()
        return _shape_summary(w)

    def structure(a):
        length = a.get("length", 200)
        width = a.get("width", 200)
        height = a.get("height", 3000)
        name = a.get("name")
        s = Arch.makeStructure(length=length, width=width, height=height)
        if name and isinstance(name, str):
            s.Label = name
        state.doc.recompute()
        return _shape_summary(s)

    def floor(a):
        members = a.get("members", [])
        if not isinstance(members, list):
            raise ValueError("bim.floor 'members' must be a list of object names")
        objs = []
        for m in members:
            obj = state.doc.getObject(m)
            if obj is None:
                raise ValueError("bim.floor: no such object %r" % m)
            objs.append(obj)
        f = Arch.makeFloor(objs)
        name = a.get("name")
        if name and isinstance(name, str):
            f.Label = name
        state.doc.recompute()
        return {"name": f.Name, "type": f.TypeId, "label": f.Label,
                "members": len(objs)}

    def building(a):
        members = a.get("members", [])
        if not isinstance(members, list):
            raise ValueError("bim.building 'members' must be a list of object names")
        objs = []
        for m in members:
            obj = state.doc.getObject(m)
            if obj is None:
                raise ValueError("bim.building: no such object %r" % m)
            objs.append(obj)
        b = Arch.makeBuilding(objs)
        name = a.get("name")
        if name and isinstance(name, str):
            b.Label = name
        state.doc.recompute()
        return {"name": b.Name, "type": b.TypeId, "label": b.Label,
                "members": len(objs)}

    def site(a):
        members = a.get("members", [])
        if not isinstance(members, list):
            raise ValueError("bim.site 'members' must be a list of object names")
        objs = []
        for m in members:
            obj = state.doc.getObject(m)
            if obj is None:
                raise ValueError("bim.site: no such object %r" % m)
            objs.append(obj)
        s = Arch.makeSite(objs)
        name = a.get("name")
        if name and isinstance(name, str):
            s.Label = name
        state.doc.recompute()
        return {"name": s.Name, "type": s.TypeId, "label": s.Label,
                "members": len(objs)}

    def add_component(a):
        parent_name = a.get("parent")
        child_name = a.get("child")
        if not isinstance(parent_name, str) or not isinstance(child_name, str):
            raise ValueError("bim.add 'parent' and 'child' must be object name strings")
        parent = state.doc.getObject(parent_name)
        child = state.doc.getObject(child_name)
        if parent is None:
            raise ValueError("bim.add: no such parent %r" % parent_name)
        if child is None:
            raise ValueError("bim.add: no such child %r" % child_name)
        Arch.addComponents(child, parent)
        state.doc.recompute()
        return {"parent": parent.Name, "child": child.Name, "added": True}

    def tree(a):
        result = []
        for obj in state.doc.Objects:
            entry = {"name": obj.Name, "type": obj.TypeId,
                     "label": getattr(obj, "Label", "")}
            group = getattr(obj, "Group", None)
            if group is not None:
                entry["children"] = [c.Name for c in group]
            result.append(entry)
        return {"objects": result, "count": len(result)}

    return {
        "bim.wall": wall,
        "bim.structure": structure,
        "bim.floor": floor,
        "bim.building": building,
        "bim.site": site,
        "bim.add": add_component,
        "bim.tree": tree,
    }
