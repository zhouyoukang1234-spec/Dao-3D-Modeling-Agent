"""Persistent live FreeCAD **GUI** kernel — runs *inside* the full ``freecad``.

The headless kernel (``freecad_kernel.py``, driven by ``freecadcmd``) owns the
document/geometry layer. It cannot touch the GUI command layer: workbenches,
the menu/toolbar ``Command`` actions a human clicks, or the ``Selection``. Those
live only in ``FreeCADGui``, which ``freecadcmd`` never starts.

This script is the dual: launched once by the full ``freecad`` binary under the
Qt ``offscreen`` platform (no X server needed), it keeps the real application —
main window, workbenches, command registry, selection — alive and drives it over
the same ``@@DAO@@`` JSON-RPC line protocol as the headless kernel. So every GUI
command a user could click is invokable headlessly, plus commands users cannot
reach programmatically at all.

Concurrency: everything runs on a single (main) thread. Rather than return to
FreeCAD's own ``exec_`` loop, this script owns the main thread with a managed
loop that ``select()``s on the real stdin (fd 0) with a short timeout and pumps
the Qt/GUI event queue every iteration. A background reader thread was tried
first but wedged FreeCAD's command queue after the first command (a GIL/Qt
interaction), so the single-threaded design is deliberate. The loop never dies
on a bad op -- errors come back as ``{"ok": false, "error": ...}`` frames.
"""
import json
import os
import sys
import traceback

SENTINEL = "@@DAO@@"

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import FreeCAD as App  # noqa: E402  (provided by freecad)
import FreeCADGui as Gui  # noqa: E402
from PySide2 import QtCore  # noqa: E402


def _pump(cycles=25, msecs=10):
    """Drain the Qt event queue deterministically.

    FreeCAD GUI commands (e.g. Part primitives) finish their work in *deferred*
    events posted to the loop, not synchronously inside ``runCommand``. If the
    queue is not flushed, the next command can wedge waiting on that pending
    work. ``Gui.updateGui()`` alone proved timing-dependent, so we pump the
    ``QCoreApplication`` event queue a bounded number of times explicitly.
    """
    app = QtCore.QCoreApplication.instance()
    for _ in range(cycles):
        app.processEvents(QtCore.QEventLoop.AllEvents, msecs)


def _settle():
    """Nudge FreeCAD's internal Console/event machinery.

    FreeCAD's full GUI redirects ``sys.stderr`` through its ``Base::Console``
    handler, which internally posts queued signals and processes pending events.
    A single write+flush to the redirected stderr between ``Gui.runCommand``
    calls is the only reliable way to let the command manager settle; a raw
    ``os.write(2, ...)`` or ``Gui.updateGui()`` does **not** achieve the same
    effect (empirically proven: 5/5 with stderr, 0/5 without). See the session
    worklog for the full Heisenbug bisection.
    """
    sys.stderr.write(".")
    sys.stderr.flush()


_DBG = os.environ.get("DAO_GUI_DBG")


def _dbg(m):
    if _DBG:
        with open(_DBG, "a") as f:
            f.write(m + "\n")


def _emit(obj):
    # The full FreeCAD GUI redirects sys.stdout to its in-app console widget, so
    # protocol frames written through sys.stdout never reach the host pipe. Write
    # straight to file descriptor 1 (the real process stdout / pipe) instead.
    payload = (SENTINEL + json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
    os.write(1, payload)


class GuiState:
    """Holds the live GUI document shared with the op handlers."""

    def __init__(self):
        self.app = App
        self.gui = Gui
        self.doc = App.newDocument("dao_gui")
        _pump()

    def recompute(self):
        self.doc.recompute()
        _pump()

    def reset(self):
        try:
            App.closeDocument(self.doc.Name)
        except Exception:
            pass
        self.doc = App.newDocument("dao_gui")
        _pump()


def _obj_summary(o):
    d = {"name": o.Name, "type": o.TypeId, "label": getattr(o, "Label", None)}
    shp = getattr(o, "Shape", None)
    if shp is not None:
        try:
            d["volume"] = shp.Volume
            d["shape_type"] = shp.ShapeType
        except Exception:
            pass
    return d


def _build_handlers(state):
    def workbenches(a):
        return {"workbenches": sorted(Gui.listWorkbenches().keys())}

    def activate(a):
        name = a.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("gui.activate 'name' must be a workbench name string "
                             "(got %r)" % (name,))
        Gui.activateWorkbench(name)
        _pump()
        return {"active": Gui.activeWorkbench().name()}

    def active(a):
        return {"active": Gui.activeWorkbench().name()}

    def commands(a):
        names = list(Gui.Command.listAll())
        contains = a.get("contains")
        if contains:
            names = [n for n in names if contains.lower() in n.lower()]
        names.sort()
        return {"commands": names, "count": len(names)}

    def command_info(a):
        name = a.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("gui.command_info 'name' must be a command name "
                             "string (got %r)" % (name,))
        all_cmds = Gui.Command.listAll()
        if name not in all_cmds:
            raise ValueError("gui.command_info: no such GUI command %r" % (name,))
        info = Gui.Command.get(name).getInfo()
        return {"name": name, "info": info}

    def run(a):
        name = a.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError("gui.run 'name' must be a GUI command name string "
                             "(got %r)" % (name,))
        all_cmds = Gui.Command.listAll()
        if name not in all_cmds:
            raise ValueError("gui.run: no such GUI command %r (try gui.commands)"
                             % (name,))
        item = a.get("item", 0)
        before = {o.Name for o in state.doc.Objects}
        _settle()
        Gui.runCommand(name, item)
        _pump()
        state.doc.recompute()
        _pump()
        created = [_obj_summary(o) for o in state.doc.Objects
                   if o.Name not in before]
        return {"command": name, "created": created,
                "created_count": len(created),
                "objects": len(state.doc.Objects)}

    def select(a):
        names = a.get("names")
        if isinstance(names, str):
            names = [names]
        if not isinstance(names, list) or not names:
            raise ValueError("gui.select 'names' must be an object name (or list "
                             "of names) (got %r)" % (a.get("names"),))
        for n in names:
            obj = state.doc.getObject(n)
            if obj is None:
                raise ValueError("gui.select: no document object named %r" % (n,))
            Gui.Selection.addSelection(state.doc.Name, n)
        _pump()
        return {"selected": [o.Name for o in Gui.Selection.getSelection()]}

    def selection(a):
        return {"selected": [o.Name for o in Gui.Selection.getSelection()]}

    def clear_selection(a):
        Gui.Selection.clearSelection()
        _pump()
        return {"selected": []}

    def objects(a):
        objs = [_obj_summary(o) for o in state.doc.Objects]
        return {"objects": objs, "count": len(objs)}

    def recompute(a):
        _settle()
        _pump()
        state.doc.recompute()
        _pump()
        return {"objects": len(state.doc.Objects)}

    def save(a):
        path = a.get("path")
        if not isinstance(path, str) or not path:
            raise ValueError("gui.save 'path' must be a non-empty file path string "
                             "(got %r)" % (path,))
        _settle()
        _pump()
        state.doc.saveAs(path)
        _pump()
        return {"path": path,
                "bytes": os.path.getsize(path) if os.path.exists(path) else 0,
                "objects": len(state.doc.Objects)}

    return {
        "gui.workbenches": workbenches,
        "gui.activate": activate,
        "gui.active": active,
        "gui.commands": commands,
        "gui.command_info": command_info,
        "gui.run": run,
        "gui.select": select,
        "gui.selection": selection,
        "gui.clear_selection": clear_selection,
        "gui.objects": objects,
        "gui.recompute": recompute,
        "gui.save": save,
    }


def main():
    import select

    state = GuiState()
    handlers = _build_handlers(state)

    def handle(line):
        """Execute one request line; return False to stop the service loop."""
        nonlocal handlers
        try:
            req = json.loads(line)
        except Exception as exc:
            _emit({"id": None, "ok": False, "error": "bad json: %s" % exc})
            return True
        rid = req.get("id")
        op = req.get("op")
        args = req.get("args") or {}
        if op == "__shutdown__":
            _emit({"id": rid, "ok": True, "data": {"bye": True}})
            return False
        if op == "__ops__":
            _emit({"id": rid, "ok": True,
                   "data": {"ops": sorted(handlers.keys())}})
            return True
        if op == "__reset__":
            state.reset()
            handlers = _build_handlers(state)
            _emit({"id": rid, "ok": True, "data": {"reset": True}})
            return True
        fn = handlers.get(op)
        if fn is None:
            _emit({"id": rid, "ok": False, "error": "unknown op: %s" % op})
            return True
        try:
            data = fn(args)
            if not isinstance(data, dict):
                data = {"value": data}
            _emit({"id": rid, "ok": True, "data": data})
        except Exception as exc:
            _emit({"id": rid, "ok": False,
                   "error": "%s: %s" % (type(exc).__name__, exc),
                   "trace": traceback.format_exc().splitlines()[-4:]})
        return True

    # Announce readiness + op inventory before entering the service loop.
    _emit({"id": 0, "ok": True, "data": {"ready": True,
                                         "ops": sorted(handlers.keys()),
                                         "freecad": ".".join(App.Version()[:3])}})

    # Single-threaded managed event loop. A background reader thread doing a
    # blocking read on fd 0 concurrently with the main thread pumping Qt wedged
    # FreeCAD's command queue after the first command (a GIL/Qt interaction), so
    # everything runs on one thread: select() on the real stdin (fd 0) waits with
    # a short timeout, we pump the Qt/GUI event queue every iteration, and we
    # split raw bytes into request lines ourselves (the GUI wraps sys.stdin in a
    # non-iterable PythonStdin, so fd-level I/O is required).
    buf = b""
    running = True
    while running:
        # Pump generously *between* requests: FreeCAD finishes a command's work
        # in deferred events that only run when the loop is idle (between handler
        # calls), so a single processEvents here is not enough to let the prior
        # command settle before the next one starts.
        _pump(cycles=5, msecs=5)
        try:
            ready, _, _ = select.select([0], [], [], 0.02)
        except (OSError, ValueError):
            ready = []
        if not ready:
            continue
        try:
            chunk = os.read(0, 65536)
        except OSError:
            break
        if not chunk:  # stdin EOF -> shutdown
            break
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            if not handle(line):
                running = False
                break


main()
