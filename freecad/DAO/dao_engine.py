"""DAO engine — binds the agent's geometry ops to the *live* FreeCAD GUI document.

This is the heart of the in-application fusion. There is **no** separate kernel
process and **no** second document: the same ``cad_agent`` operation modules that
power the headless tool surface are imported in-process and pointed straight at
``App.ActiveDocument`` — the very document the human is editing in the GUI.

Consequences of that design choice:

* Everything the AI builds is an ordinary object in the user's document, shown
  live in FreeCAD's own 3D view and model tree.
* Every AI action runs inside a FreeCAD undo transaction, so the user can simply
  press Ctrl+Z — AI steps and manual steps share one history.
* The AI can reference objects the human made by hand (looked up by Label/Name),
  and the human can keep editing objects the AI made. One workspace, two hands.
"""
import importlib
import importlib.util
import os
import sys
import traceback

import FreeCAD as App

# --------------------------------------------------------------------------- #
# locate the repo so we can reuse cad_agent's op modules in-process
# --------------------------------------------------------------------------- #
_HERE = os.path.dirname(os.path.realpath(__file__))          # <repo>/freecad/DAO
_REPO = os.path.dirname(os.path.dirname(_HERE))              # <repo>
_BACKENDS = os.path.join(_REPO, "cad_agent", "backends")
_PLANNER_PY = os.path.join(_REPO, "cad_agent", "planner.py")
for _p in (_HERE, _BACKENDS):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_planner():
    spec = importlib.util.spec_from_file_location("dao_planner", _PLANNER_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["dao_planner"] = mod  # dataclasses resolves cls.__module__ here
    spec.loader.exec_module(mod)
    return mod.Planner


class GuiState:
    """Op-module state object, but bound to a live GUI document instead of a
    private kernel document. Mirrors ``KernelState``'s attribute surface so the
    existing ``register(state)`` functions work unchanged."""

    def __init__(self, doc):
        self.app = App
        self.doc = doc
        self.shapes = {}      # logical name -> Part::Feature object .Name
        self.bodies = {}      # logical name -> PartDesign::Body object .Name
        self.params = {}
        self.assembly = None
        self.components = {}
        self.joints = {}
        self._undo = []

    def recompute(self):
        self.doc.recompute()

    def sync_from_doc(self):
        """Make every object already in the document referenceable by the AI.

        Maps both Label and internal Name so a human can say "cut hole from
        Plate" about a part they drew by hand, and the planner's auto-names keep
        working too."""
        for o in self.doc.Objects:
            tid = getattr(o, "TypeId", "")
            if tid == "PartDesign::Body":
                self.bodies.setdefault(o.Label, o.Name)
                self.bodies.setdefault(o.Name, o.Name)
            elif o.isDerivedFrom("Part::Feature"):
                self.shapes.setdefault(o.Label, o.Name)
                self.shapes.setdefault(o.Name, o.Name)


def _build_handlers(state):
    """Register the full op surface against ``state`` (same set as the kernel)."""
    handlers = {}
    import freecad_ops
    # freecad_ops exposes bare BREP op names ("box", "cut", ...); they live in the
    # "solid.*" group (the param.*/asm.*/view.* modules already carry their prefix).
    for name, fn in freecad_ops.register(state).items():
        handlers["solid." + name] = fn
    for modname in ("freecad_parametric", "freecad_assembly",
                    "freecad_perceive", "freecad_advanced"):
        try:
            mod = importlib.import_module(modname)
            handlers.update(mod.register(state))
        except Exception as exc:  # optional groups must never break solid.*
            App.Console.PrintWarning("DAO: %s load failed: %r\n" % (modname, exc))

    # GUI-native perception (gui.*) — the agent's eyes on the live viewport.
    try:
        import dao_perceive
        handlers.update(dao_perceive.register(state))
    except Exception as exc:
        App.Console.PrintWarning("DAO: dao_perceive load failed: %r\n" % (exc,))

    # composite perception: one call → snapshot + scene + selection + errors.
    if "gui.scene" in handlers:
        def _perceive(a):
            out = {"scene": handlers["gui.scene"](a)}
            try:
                out["snapshot"] = handlers["gui.snapshot"](a)
            except Exception as exc:
                out["snapshot"] = {"error": repr(exc)}
            try:
                out["selection"] = handlers["gui.selection"](a)
            except Exception:
                out["selection"] = {"selected": [], "count": 0}
            return out
        handlers["gui.perceive"] = _perceive

    def _save(a):
        state.doc.recompute()
        state.doc.saveAs(a["path"])
        return {"path": a["path"], "objects": len(state.doc.Objects)}

    def _info(a):
        return {"name": state.doc.Name,
                "objects": [{"name": o.Name, "label": o.Label, "type": o.TypeId}
                            for o in state.doc.Objects]}
    handlers["doc.save"] = _save
    handlers["doc.info"] = _info
    return handlers


class DAOEngine:
    """Stateful bridge from chat text to live-document mutations."""

    def __init__(self):
        self.planner = _load_planner()()
        self._doc_name = None
        self.state = None
        self.handlers = {}

    # -- document binding --------------------------------------------------- #
    def _ensure_doc(self):
        doc = App.ActiveDocument or App.newDocument("DAO")
        if self.state is None or self._doc_name != doc.Name or \
                App.getDocument(doc.Name) is not self.state.doc:
            self.state = GuiState(doc)
            self.state.sync_from_doc()
            self.handlers = _build_handlers(self.state)
            self._doc_name = doc.Name
        else:
            self.state.sync_from_doc()
        return doc

    def ops(self):
        self._ensure_doc()
        return sorted(self.handlers)

    # -- perception --------------------------------------------------------- #
    def perceive(self, args=None):
        """Unified perception: structured scene + viewport snapshot + selection.

        Returns the composite ``gui.perceive`` payload, or a scene-only dict when
        running without a GUI (no viewport to capture)."""
        self._ensure_doc()
        fn = self.handlers.get("gui.perceive") or self.handlers.get("gui.scene")
        if fn is None:
            return {"error": "perception unavailable (no GUI)"}
        return fn(args or {})

    def snapshot(self, path=None, view=None):
        """Capture the live 3D viewport to ``path`` and return its location."""
        self._ensure_doc()
        fn = self.handlers.get("gui.snapshot")
        if fn is None:
            return {"error": "snapshot unavailable (no GUI)"}
        return fn({"path": path, "view": view})

    # -- execution ---------------------------------------------------------- #
    def run(self, text):
        """Plan ``text`` and execute it on the live document.

        Returns ``(note, results)`` where results is a list of per-step dicts:
        ``{"tool", "ok", "data"/"error"}``. The whole plan is one undo step."""
        doc = self._ensure_doc()
        plan = self.planner.plan(text)
        if plan.error:
            return plan.error, []
        results = []
        doc.openTransaction("DAO: %s" % text[:60])
        try:
            for step in plan.steps:
                tool = step["tool"]
                args = step.get("args", {})
                if tool in ("view.scene", "view.render"):
                    results.append({"tool": tool, "ok": True, "data": {"refreshed": True}})
                    continue
                if tool == "__reset__":
                    results.append(self._reset())
                    continue
                fn = self.handlers.get(tool)
                if fn is None:
                    results.append({"tool": tool, "ok": False, "error": "unknown op: %s" % tool})
                    continue
                try:
                    data = fn(args)
                    if not isinstance(data, dict):
                        data = {"value": data}
                    results.append({"tool": tool, "ok": True, "data": data})
                except Exception as exc:
                    results.append({"tool": tool, "ok": False,
                                    "error": "%s: %s" % (type(exc).__name__, exc),
                                    "trace": traceback.format_exc().splitlines()[-3:]})
        finally:
            doc.commitTransaction()
            doc.recompute()
        return plan.note, results

    # -- autonomous closed loop -------------------------------------------- #
    def clear(self):
        """Wipe all objects from the live document (one undo step) and reset the
        AI's name bookkeeping. Used between closed-loop iterations so each attempt
        builds from a clean slate in the *same* document the human is watching."""
        doc = self._ensure_doc()
        doc.openTransaction("DAO: clear")
        try:
            for _ in range(4):                      # a few passes for dependents
                objs = list(doc.Objects)
                if not objs:
                    break
                for o in objs:
                    try:
                        doc.removeObject(o.Name)
                    except Exception:
                        pass
        finally:
            doc.commitTransaction()
            doc.recompute()
        for d in (self.state.shapes, self.state.bodies,
                  self.state.components, self.state.joints):
            d.clear()
        self.state.assembly = None
        return {"cleared": True, "objects": len(doc.Objects)}

    def solve(self, goal_name, max_iters=10, on_iteration=None, **overrides):
        """Run the closed-loop agent on a named goal until its acceptance checks
        pass or the iteration budget is spent. Returns the full transcript."""
        self._ensure_doc()
        import dao_agent
        goal_cls = dao_agent.GOALS.get(goal_name)
        if goal_cls is None:
            return {"error": "unknown goal: %s" % goal_name,
                    "available": sorted(dao_agent.GOALS)}
        agent = dao_agent.ClosedLoopAgent(self, max_iters=max_iters)
        return agent.solve(goal_cls(**overrides), on_iteration=on_iteration)

    def _reset(self):
        """Clear AI bookkeeping and start a fresh document (manual work stays
        in the previous document, which is left open)."""
        doc = App.newDocument("DAO")
        self.state = GuiState(doc)
        self.handlers = _build_handlers(self.state)
        self._doc_name = doc.Name
        return {"tool": "__reset__", "ok": True, "data": {"document": doc.Name}}
