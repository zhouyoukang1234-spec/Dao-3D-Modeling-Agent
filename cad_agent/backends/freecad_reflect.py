"""Reflective universal dispatch -- the whole FreeCAD Python surface as one gate.

The curated ``solid.*`` / ``param.*`` / ... operators wrap the handful of calls a
recipe commonly needs. This module is their complement: instead of hand-wrapping
one more kernel call at a time, it exposes *every* FreeCAD-reachable Python
callable directly, so any method on any module or object -- ``Part.makeBox``,
``App.Vector``, a live ``Sketcher::SketchObject``'s ``addGeometry``, a raw
``Part.Shape``'s ``.common`` -- can be invoked over the same JSON-RPC pipe. The
static ``capability_map.json`` says *what exists*; this makes all of it *callable*.

The bridge is a marshalling layer. JSON in is decoded into live FreeCAD values
(``{"$vec": [x,y,z]}`` -> ``App.Vector``, ``{"$obj": "Name"}`` -> a document
object, ``{"$ref": id}`` -> a previously returned handle); the return is encoded
back out, with any non-JSON object (a ``Shape``, a geometry, a document object)
kept in a per-kernel handle table and returned as ``{"$ref": id, ...}`` plus a
small summary. Handles let calls chain: make a box -> get a ref -> call
``.cut`` on that ref with another ref. Reset clears the table.

Ops:

* ``reflect.roots``    -- the whitelisted root namespaces a target path resolves from.
* ``reflect.call``     -- call ``target`` (dotted path) or ``on``.``method`` with ``args`` / ``kwargs``.
* ``reflect.get``      -- read an attribute (``target`` path, or ``on`` + ``attr``).
* ``reflect.set``      -- write an attribute (``on`` + ``attr`` + ``value``) -- e.g. dial a live property.
* ``reflect.methods``  -- live public method / attribute surface of the resolved object.
* ``reflect.help``     -- signature + docstring of a resolved callable.
* ``reflect.free``     -- drop a handle (or all handles).

Everything routes through the same resident kernel and live document, so a
reflective call and a curated ``solid.*`` call operate on one shared model.
"""
import importlib
import inspect

import FreeCAD as App


# Encoded-value tags recognised on the way in (JSON -> live FreeCAD value).
_VEC = "$vec"
_ROT = "$rot"
_PLACEMENT = "$placement"
_OBJ = "$obj"
_REF = "$ref"


def register(state):
    # Per-kernel handle table: id -> live object that could not be JSON-encoded
    # (a Shape, a geometry, an App.Vector kept for chaining, ...). Closure-local
    # so a kernel __reset__ (which rebuilds handlers) starts fresh.
    handles = {}
    counter = {"n": 0}

    def _roots():
        # Root namespaces a dotted ``target`` resolves against. ``doc`` is the
        # live document so a target like ``doc.getObject`` reaches the model.
        roots = {"App": App, "FreeCAD": App, "doc": state.doc}
        for name in ("Part", "Sketcher", "PartDesign", "Mesh", "MeshPart",
                     "Draft", "TechDraw", "Fem", "Path", "Points", "Import",
                     "Spreadsheet", "Measure", "ReverseEngineering"):
            try:
                roots[name] = importlib.import_module(name)
            except Exception:
                continue
        return roots

    def _store(obj):
        counter["n"] += 1
        hid = counter["n"]
        handles[hid] = obj
        return hid

    def _shape_summary(shp):
        out = {"shapeType": getattr(shp, "ShapeType", None)}
        for attr, key in (("Volume", "volume"), ("Area", "area"),
                          ("Length", "length")):
            try:
                v = getattr(shp, attr)
                if isinstance(v, (int, float)):
                    out[key] = v
            except Exception:
                pass
        try:
            bb = shp.BoundBox
            out["bbox"] = [bb.XLength, bb.YLength, bb.ZLength]
        except Exception:
            pass
        for attr, key in (("Vertexes", "vertices"), ("Edges", "edges"),
                          ("Faces", "faces"), ("Solids", "solids")):
            try:
                out[key] = len(getattr(shp, attr))
            except Exception:
                pass
        return out

    def _encode(v):
        # Live FreeCAD value -> JSON. Non-encodable objects are kept in the
        # handle table and returned as a {"$ref": id, ...} descriptor.
        if v is None or isinstance(v, (bool, int, float, str)):
            return v
        if isinstance(v, (list, tuple)):
            return [_encode(x) for x in v]
        if isinstance(v, dict):
            return {str(k): _encode(x) for k, x in v.items()}
        # App.Vector -> {"$vec": [...]}
        if isinstance(v, App.Vector):
            return {_VEC: [v.x, v.y, v.z]}
        # a document object -> {"$obj": name}
        if hasattr(v, "TypeId") and hasattr(v, "Name") and hasattr(v, "Document"):
            return {_OBJ: v.Name, "type": v.TypeId, "label": getattr(v, "Label", None)}
        # a topological shape -> handle + geometric summary
        cls = type(v).__name__
        mod = type(v).__module__ or ""
        if mod.startswith("Part") and hasattr(v, "ShapeType"):
            hid = _store(v)
            d = {_REF: hid, "class": cls}
            d.update(_shape_summary(v))
            return d
        # anything else (a Placement, a geometry, a Rotation, ...) -> handle
        hid = _store(v)
        try:
            rep = repr(v)
        except Exception:
            rep = "<%s>" % cls
        return {_REF: hid, "class": cls, "repr": rep[:200]}

    def _decode(v):
        # JSON -> live FreeCAD value.
        if isinstance(v, list):
            return [_decode(x) for x in v]
        if not isinstance(v, dict):
            return v
        if _VEC in v:
            xyz = v[_VEC]
            return App.Vector(*(float(c) for c in xyz))
        if _ROT in v:
            r = v[_ROT]
            axis = App.Vector(*(float(c) for c in r.get("axis", [0, 0, 1])))
            return App.Rotation(axis, float(r.get("angle", 0.0)))
        if _PLACEMENT in v:
            p = v[_PLACEMENT]
            pos = App.Vector(*(float(c) for c in p.get("pos", [0, 0, 0])))
            rr = p.get("rot")
            if rr is not None:
                rot = _decode({_ROT: rr})
            else:
                rot = App.Rotation()
            return App.Placement(pos, rot)
        if _OBJ in v:
            name = v[_OBJ]
            obj = state.doc.getObject(name)
            if obj is None:
                raise ValueError("reflect: no document object named %r" % (name,))
            return obj
        if _REF in v:
            hid = v[_REF]
            if hid not in handles:
                raise ValueError("reflect: unknown handle %r (freed or never "
                                 "created)" % (hid,))
            return handles[hid]
        return {k: _decode(x) for k, x in v.items()}

    def _resolve_path(target):
        # Resolve a dotted path like "Part.makeBox" or "App.Vector" against the
        # root namespaces, returning (owner, attr_value).
        if not isinstance(target, str) or not target:
            raise ValueError("reflect: 'target' must be a non-empty dotted path "
                             "string (e.g. 'Part.makeBox')")
        parts = target.split(".")
        roots = _roots()
        head = parts[0]
        if head not in roots:
            raise ValueError("reflect: unknown root %r; known roots: %s"
                             % (head, ", ".join(sorted(roots))))
        obj = roots[head]
        for p in parts[1:]:
            try:
                obj = getattr(obj, p)
            except AttributeError:
                raise ValueError("reflect: %r has no attribute %r"
                                 % (target.rsplit(".", 1)[0], p))
        return obj

    def _base_and_name(a):
        # A target is either a dotted path ("target") or a bound method named by
        # "method" on an "on" object/ref. Returns (callable_or_value, label).
        if a.get("on") is not None:
            base = _decode(a["on"])
            method = a.get("method")
            if method is not None:
                if not isinstance(method, str) or not method:
                    raise ValueError("reflect: 'method' must be a method name")
                if not hasattr(base, method):
                    raise ValueError("reflect: object %s has no method %r"
                                     % (type(base).__name__, method))
                return getattr(base, method), method
            return base, type(base).__name__
        return _resolve_path(a.get("target")), a.get("target")

    # ---- ops ------------------------------------------------------------- #
    def roots(a):
        r = _roots()
        return {"roots": sorted(r),
                "modules": sorted(k for k, v in r.items()
                                  if inspect.ismodule(v))}

    def call(a):
        if a.get("on") is None and not a.get("target"):
            raise ValueError("reflect.call needs 'target' (dotted path) or 'on' "
                             "+ 'method'")
        fn, label = _base_and_name(a)
        if not callable(fn):
            raise ValueError("reflect.call: %r is not callable (it is %s); use "
                             "reflect.get to read it" % (label, type(fn).__name__))
        args = a.get("args") or []
        kwargs = a.get("kwargs") or {}
        if not isinstance(args, list):
            raise ValueError("reflect.call 'args' must be a list")
        if not isinstance(kwargs, dict):
            raise ValueError("reflect.call 'kwargs' must be a dict")
        dargs = [_decode(x) for x in args]
        dkwargs = {k: _decode(x) for k, x in kwargs.items()}
        result = fn(*dargs, **dkwargs)
        if a.get("recompute"):
            state.doc.recompute()
        return {"target": label, "result": _encode(result)}

    def get(a):
        val, label = _base_and_name(a) if a.get("on") is not None \
            else (_resolve_path(a.get("target")), a.get("target"))
        if a.get("on") is not None and a.get("attr"):
            base = _decode(a["on"])
            attr = a["attr"]
            if not hasattr(base, attr):
                raise ValueError("reflect.get: object %s has no attribute %r"
                                 % (type(base).__name__, attr))
            val = getattr(base, attr)
            label = attr
        return {"target": label, "value": _encode(val)}

    def set_(a):
        if a.get("on") is None:
            raise ValueError("reflect.set needs 'on' (the object/ref to mutate)")
        base = _decode(a["on"])
        attr = a.get("attr")
        if not isinstance(attr, str) or not attr:
            raise ValueError("reflect.set needs 'attr' (attribute name)")
        if "value" not in a:
            raise ValueError("reflect.set needs 'value'")
        if not hasattr(base, attr):
            raise ValueError("reflect.set: object %s has no attribute %r"
                             % (type(base).__name__, attr))
        setattr(base, attr, _decode(a["value"]))
        if a.get("recompute"):
            state.doc.recompute()
        return {"attr": attr, "value": _encode(getattr(base, attr))}

    def methods(a):
        if a.get("on") is not None:
            obj = _decode(a["on"])
            label = type(obj).__name__
        else:
            obj = _resolve_path(a.get("target"))
            label = a.get("target")
        names = [n for n in dir(obj) if not n.startswith("_")]
        callables, attrs = [], []
        for n in names:
            try:
                m = getattr(obj, n)
            except Exception:
                continue
            (callables if callable(m) else attrs).append(n)
        return {"target": label, "callables": sorted(callables),
                "attributes": sorted(attrs)}

    def help_(a):
        fn, label = _base_and_name(a)
        sig = None
        try:
            sig = str(inspect.signature(fn))
        except (TypeError, ValueError):
            sig = None
        doc = inspect.getdoc(fn)
        return {"target": label, "callable": callable(fn), "signature": sig,
                "doc": (doc or "")[:2000]}

    def free(a):
        if a.get("all"):
            n = len(handles)
            handles.clear()
            return {"freed": n}
        ref = a.get("ref")
        if ref is None:
            raise ValueError("reflect.free needs 'ref' (handle id) or all=true")
        existed = handles.pop(ref, None) is not None
        return {"ref": ref, "freed": bool(existed), "live": len(handles)}

    return {
        "reflect.roots": roots,
        "reflect.call": call,
        "reflect.get": get,
        "reflect.set": set_,
        "reflect.methods": methods,
        "reflect.help": help_,
        "reflect.free": free,
    }
