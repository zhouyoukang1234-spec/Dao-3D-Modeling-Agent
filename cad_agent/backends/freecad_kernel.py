"""Persistent live FreeCAD kernel — runs *inside* freecadcmd.

This is the low-level heart of the integration. Instead of spawning a fresh
``freecadcmd`` for every operation (stateless, slow, no live document), the host
launches this script once and keeps it alive. It owns a real, mutable
``FreeCAD`` document and executes operations against it in-process, exactly like
a human's interactive FreeCAD session — only driven over a JSON-RPC pipe.

Protocol (one JSON object per line):
    host -> kernel:  {"id": 1, "op": "box", "args": {...}}
    kernel -> host:  @@DAO@@{"id": 1, "ok": true, "data": {...}}

The ``@@DAO@@`` sentinel lets the host separate protocol frames from FreeCAD's
own banner / log chatter on stdout. The loop never dies on a bad op — errors are
returned as ``{"ok": false, "error": ...}`` frames.
"""
import json
import os
import sys
import traceback

SENTINEL = "@@DAO@@"

# Make sibling op modules importable when launched as a bare script by freecadcmd.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import FreeCAD as App  # noqa: E402  (provided by freecadcmd)


class KernelState:
    """Holds the live document and per-engine bookkeeping shared with op modules."""

    def __init__(self):
        self.app = App
        self.doc = App.newDocument("dao")
        # Direct-BREP objects (solid.*): name -> Part::Feature object name in doc.
        self.shapes = {}
        # Parametric bodies (param.*): logical name -> PartDesign::Body object name.
        self.bodies = {}
        # Named parameters: "Feature.param" -> (object_name, property_path).
        self.params = {}
        # Assembly bookkeeping (asm.*).
        self.assembly = None
        self.components = {}
        self.joints = {}
        self._undo = []

    def recompute(self):
        self.doc.recompute()

    def reset(self):
        try:
            App.closeDocument(self.doc.Name)
        except Exception:
            pass
        self.__init__()


def _doc_handlers(state):
    """Document-level ops (save/open/info) for persisting the live model."""
    def save(a):
        path = a.get("path")
        # saveAs needs a filesystem string; a non-string leaks a raw
        # 'TypeError: argument 1 must be str, ...'.
        if not isinstance(path, str) or not path:
            raise ValueError(
                "doc.save 'path' must be a non-empty file path string (got %r)"
                % (path,))
        state.doc.recompute()
        state.doc.saveAs(path)
        return {"path": path, "bytes": os.path.getsize(path) if os.path.exists(path) else 0,
                "objects": len(state.doc.Objects)}

    def info(a):
        return {"name": state.doc.Name, "objects": [
            {"name": o.Name, "label": o.Label, "type": o.TypeId} for o in state.doc.Objects]}

    return {"doc.save": save, "doc.info": info}


def _build_handlers(state):
    handlers = {}
    # Each op module exposes register(state) -> {op_name: callable(args)->dict}.
    import freecad_ops
    handlers.update(freecad_ops.register(state))
    handlers.update(_doc_handlers(state))
    try:
        import freecad_parametric
        handlers.update(freecad_parametric.register(state))
    except Exception as exc:  # parametric is optional; keep solid.* working
        sys.stderr.write("parametric load failed: %r\n" % (exc,))
    try:
        import freecad_assembly
        handlers.update(freecad_assembly.register(state))
    except Exception as exc:
        sys.stderr.write("assembly load failed: %r\n" % (exc,))
    try:
        import freecad_perceive
        handlers.update(freecad_perceive.register(state))
    except Exception as exc:
        sys.stderr.write("perceive load failed: %r\n" % (exc,))
    try:
        import freecad_advanced
        handlers.update(freecad_advanced.register(state))
    except Exception as exc:
        sys.stderr.write("advanced load failed: %r\n" % (exc,))
    try:
        import freecad_fem
        handlers.update(freecad_fem.register(state))
    except Exception as exc:  # FEM is optional (needs Fem + ccx); keep rest working
        sys.stderr.write("fem load failed: %r\n" % (exc,))
    try:
        import freecad_path
        handlers.update(freecad_path.register(state))
    except Exception as exc:  # CAM is optional (needs Path workbench); keep rest working
        sys.stderr.write("path load failed: %r\n" % (exc,))
    try:
        import freecad_surface
        handlers.update(freecad_surface.register(state))
    except Exception as exc:  # Surface/Draft/Points coverage is optional
        sys.stderr.write("surface load failed: %r\n" % (exc,))
    return handlers


def main():
    state = KernelState()
    handlers = _build_handlers(state)

    def emit(obj):
        sys.stdout.write(SENTINEL + json.dumps(obj, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    # Announce readiness + full op inventory so the host can register tools.
    emit({"id": 0, "ok": True, "data": {"ready": True, "ops": sorted(handlers.keys()),
                                         "freecad": ".".join(App.Version()[:3])}})

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception as exc:
            emit({"id": None, "ok": False, "error": "bad json: %s" % exc})
            continue
        rid = req.get("id")
        op = req.get("op")
        args = req.get("args") or {}
        if op == "__shutdown__":
            emit({"id": rid, "ok": True, "data": {"bye": True}})
            break
        if op == "__ops__":
            emit({"id": rid, "ok": True, "data": {"ops": sorted(handlers.keys())}})
            continue
        if op == "__reset__":
            state.reset()
            handlers = _build_handlers(state)
            emit({"id": rid, "ok": True, "data": {"reset": True}})
            continue
        fn = handlers.get(op)
        if fn is None:
            emit({"id": rid, "ok": False, "error": "unknown op: %s" % op})
            continue
        try:
            data = fn(args)
            if not isinstance(data, dict):
                data = {"value": data}
            emit({"id": rid, "ok": True, "data": data})
        except KeyError as exc:
            # A bare ``a["key"]`` access on a missing argument otherwise surfaces
            # as a cryptic ``KeyError: 'name'``. When the key is a plain argument
            # identifier, turn it into actionable guidance; descriptive KeyErrors
            # (e.g. ``_get`` raising "no such solid: X") are passed through as-is.
            key = exc.args[0] if exc.args else None
            if isinstance(key, str) and key and all(c.isalnum() or c == "_" for c in key):
                msg = "%s missing required argument '%s'" % (op, key)
            else:
                msg = str(key) if key is not None else str(exc)
            emit({"id": rid, "ok": False, "error": "ValueError: %s" % msg,
                  "trace": traceback.format_exc().splitlines()[-4:]})
        except Exception as exc:
            emit({"id": rid, "ok": False, "error": "%s: %s" % (type(exc).__name__, exc),
                  "trace": traceback.format_exc().splitlines()[-4:]})


# freecadcmd executes this file with __name__ set to the module basename
# (not "__main__"), so invoke main() unconditionally — this file is only ever
# run as the kernel entry script, never imported.
main()
