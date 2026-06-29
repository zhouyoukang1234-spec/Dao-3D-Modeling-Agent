"""CAM / manufacturing engine — the ``path.*`` tool group (FreeCAD Path).

The deepest end of the closed loop: turn a finished solid into real machine
instructions. FreeCAD 1.0's Path workbench (CalculiX's CAM sibling) builds a
``Path::Job`` over a stock solid, generates compensated tool paths for profile
and pocket operations against geometry-selected faces, and runs a real
post-processor (grbl, linuxcnc, ...) to emit G-code. This closes *design ->
manufacture*: the same parametric/BREP body the agent designed becomes a G-code
program with bounded, inspectable moves.

Geometry-driven like the rest: machined faces are picked by predicate (axis
extreme / normal / index), never by hard-coded face names.

Runs inside freecadcmd. ``register(state)`` returns ``{op_name: callable}``.
"""
import math
import os
import tempfile

import FreeCAD as App

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def _sel_num(d, key, default, label):
    """Coerce ``d[key]`` to float with a guided error -- a bare
    ``float(d[key])`` leaks 'could not convert string to float'."""
    if key not in d or d[key] is None:
        return float(default)
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float, str)):
        raise ValueError("%s must be a number (got %r)" % (label, v))
    try:
        return float(v)
    except (TypeError, ValueError):
        raise ValueError("%s must be a number (got %r)" % (label, v))


def _sel_vec(seq, label):
    """Coerce a 3-vector with a guided error instead of a raw
    'Expected sequence of size 3' / 'could not convert'."""
    if isinstance(seq, (str, bytes)) or not isinstance(seq, (list, tuple)) \
            or len(seq) != 3:
        raise ValueError(
            "%s must be a list of 3 numbers [x, y, z] (got %r)" % (label, seq))
    try:
        return V(float(seq[0]), float(seq[1]), float(seq[2]))
    except (TypeError, ValueError):
        raise ValueError(
            "%s components must all be numbers (got %r)" % (label, seq))


def _check_sel(sel):
    """A non-dict selector leaks 'str/int object has no attribute get' or
    'string indices must be integers'; demand a real dict up front."""
    if not isinstance(sel, dict):
        raise ValueError(
            "'select' must be a dict like {'index':[0]} / {'normal':[0,0,1]} / "
            "{'axis':'z','side':'max'} / {'diameter':d}; got %r" % (sel,))
    return sel


def register(state):
    import Path  # noqa: F401  (ensures the Path module initialises)
    import Path.Main.Job as PJob
    import Path.Op.Drilling as PDrilling
    import Path.Op.Pocket as PPocket
    import Path.Op.Profile as PProfile
    import Path.Post.Processor as Proc

    doc = state.doc
    if not hasattr(state, "cam"):
        state.cam = {}

    def _resolve(name):
        oname = state.shapes.get(name) or state.bodies.get(name)
        if not oname:
            raise KeyError("no such solid/body: %s" % name)
        obj = doc.getObject(oname)
        if obj is None or not getattr(obj, "Shape", None) or obj.Shape.isNull():
            raise ValueError("%s has no solid shape to machine" % name)
        return obj

    def _select_faces(shape, sel):
        """Pick face names by predicate (axis extreme / index / normal).

        ``normal`` and ``axis`` may be combined: filter to faces matching the
        ``normal`` first, then keep the ``side`` (min/max) extreme along ``axis``.
        This isolates e.g. a pocket *floor* (an upward +Z face that is NOT the
        topmost +Z face) which neither predicate can pick alone.
        """
        _check_sel(sel)
        faces = shape.Faces
        if "index" in sel:
            idx = sel["index"]
            idx = [idx] if isinstance(idx, (int, float)) and not isinstance(idx, bool) else idx
            if isinstance(idx, (str, bytes)) or not isinstance(idx, (list, tuple)):
                raise ValueError(
                    "'index' must be an int or list of ints (got %r)" % (idx,))
            nf = len(faces)
            out = []
            for i in idx:
                if isinstance(i, bool) or not isinstance(i, (int, float)) \
                        or float(i) != int(i):
                    raise ValueError(
                        "'index' values must be whole face numbers (got %r)"
                        % (idx,))
                fi = int(i)
                if fi < 1 or fi > nf:
                    raise ValueError(
                        "'index' face number %d out of range; the solid has %d "
                        "faces (1..%d)" % (fi, nf, nf))
                out.append("Face%d" % fi)
            return out

        cand = list(range(len(faces)))
        if "normal" in sel:
            d = _sel_vec(sel["normal"], "'normal'")
            d = d.normalize() if d.Length > 1e-9 else d
            min_dot = _sel_num(sel, "min_dot", 0.95, "'min_dot'")
            keep = []
            for i in cand:
                f = faces[i]
                u0, u1, v0, v1 = f.ParameterRange
                n = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
                if n.Length > 1e-9 and n.normalize().dot(d) >= min_dot:
                    keep.append(i)
            cand = keep

        if "axis" in sel:
            axkey = sel["axis"]
            if not isinstance(axkey, str) or axkey.lower() not in ("x", "y", "z"):
                raise ValueError(
                    "'axis' must be 'x', 'y' or 'z' (got %r)" % (axkey,))
            ax = {"x": 0, "y": 1, "z": 2}[axkey.lower()]
            side = str(sel.get("side", "max")).lower()
            vals = {i: faces[i].CenterOfMass[ax] for i in cand}
            if not vals:
                return []
            target = min(vals.values()) if side == "min" else max(vals.values())
            tol = _sel_num(sel, "tol", 1e-4, "'tol'")
            return ["Face%d" % (i + 1) for i, v in vals.items() if abs(v - target) <= tol]

        if "normal" in sel:
            return ["Face%d" % (i + 1) for i in cand]
        raise ValueError("unknown face selector: %r" % sel)

    def _top_face(shape):
        zmax = max(f.CenterOfMass.z for f in shape.Faces)
        for i, f in enumerate(shape.Faces):
            if abs(f.CenterOfMass.z - zmax) < 1e-6:
                return "Face%d" % (i + 1)
        return "Face1"

    def _select_holes(shape, sel):
        """Pick cylindrical hole faces for drilling.

        A hole is a cylindrical face whose axis is parallel to ``axis_dir``
        (default +Z). Optionally filter by ``diameter`` (mm). Returns the face
        names plus the distinct hole-center XY positions and the z-extent the
        bores span (so the drill depth can be bound from geometry).
        """
        _check_sel(sel)
        axis = _sel_vec(sel.get("axis_dir", (0, 0, 1)), "'axis_dir'")
        axis = axis.normalize() if axis.Length > 1e-9 else V(0, 0, 1)
        want_r = _sel_num(sel, "diameter", 0.0, "'diameter'") / 2.0 \
            if sel.get("diameter") else None
        rtol = _sel_num(sel, "tol", 0.5, "'tol'")
        names, centers, zmin, zmax = [], [], None, None
        for i, f in enumerate(shape.Faces):
            surf = f.Surface
            if surf.__class__.__name__ != "Cylinder":
                continue
            fa = V(*surf.Axis)
            if fa.Length < 1e-9 or abs(fa.normalize().dot(axis)) < 0.99:
                continue
            if want_r is not None and abs(surf.Radius - want_r) > rtol:
                continue
            names.append("Face%d" % (i + 1))
            c = surf.Center
            centers.append((_round(c.x, 3), _round(c.y, 3)))
            bb = f.BoundBox
            zmin = bb.ZMin if zmin is None else min(zmin, bb.ZMin)
            zmax = bb.ZMax if zmax is None else max(zmax, bb.ZMax)
        return names, sorted(set(centers)), zmin, zmax

    def _job():
        j = doc.getObject(state.cam.get("job", ""))
        if j is None:
            raise ValueError("call path.job first")
        return j

    def _path_bbox(op):
        xs, ys, zs = [], [], []
        for c in (op.Path.Commands if op.Path else []):
            p = c.Parameters
            if "X" in p:
                xs.append(p["X"])
            if "Y" in p:
                ys.append(p["Y"])
            if "Z" in p:
                zs.append(p["Z"])
        if not xs:
            return None
        return [_round(min(xs)), _round(min(ys), 4), _round(min(zs) if zs else 0.0),
                _round(max(xs)), _round(max(ys), 4), _round(max(zs) if zs else 0.0)]

    def job(a):
        """Create a Path Job over a solid (the stock) + a tool controller.

        args: target, tool_diameter(mm, optional), postprocessor=grbl
        """
        obj = _resolve(a["target"])
        # drop any prior job
        for o in list(doc.Objects):
            if o.isDerivedFrom("Path::FeatureCompoundPython") or "Path::" in o.TypeId:
                try:
                    doc.removeObject(o.Name)
                except Exception:
                    pass
        j = PJob.Create("Job", [obj])
        doc.recompute()
        tc = j.Tools.Group[0]
        dia = a.get("tool_diameter")
        if dia is not None:
            try:
                tc.Tool.Diameter = App.Units.Quantity("%s mm" % float(dia))
            except Exception:
                pass
        doc.recompute()
        state.cam = {"job": j.Name, "target": obj.Name, "tc": tc.Name,
                     "post": a.get("postprocessor", "grbl"), "ops": []}
        try:
            tdia = float(tc.Tool.Diameter.getValueAs("mm"))
        except Exception:
            tdia = None
        return {"job": j.Name, "stock": obj.Name, "tool": tc.Label,
                "tool_diameter_mm": _round(tdia) if tdia else None,
                "postprocessor": state.cam["post"]}

    def profile(a):
        """Profile (contour) the outline of selected faces.

        args: select(optional, default top face), side=Outside|Inside
        """
        j = _job()
        obj = doc.getObject(state.cam["target"])
        names = _select_faces(obj.Shape, a["select"]) if a.get("select") \
            else [_top_face(obj.Shape)]
        op = PProfile.Create("Profile")
        op.Base = [(obj, names)]
        op.ToolController = doc.getObject(state.cam["tc"])
        op.Side = a.get("side", "Outside")
        j.Proxy.addOperation(op)
        doc.recompute()
        n = len(op.Path.Commands) if op.Path else 0
        if n == 0:
            raise ValueError("profile produced no path (bad face selection?)")
        state.cam["ops"].append(op.Name)
        return {"op": "profile", "faces": names, "commands": n,
                "path_bbox": _path_bbox(op)}

    def pocket(a):
        """Pocket (clear material from) selected faces.

        Clears the selected face area from the stock top down to the face plane
        in ``step_down`` (mm) layers. Without explicit depths a flat selected
        face would give StartDepth==FinalDepth and emit an empty path, so the
        depths are bound from geometry: StartDepth = stock top, FinalDepth = the
        lowest selected face. Override with ``start_depth`` / ``final_depth`` /
        ``step_down``.

        args: select (faces defining the pocket area/floor),
              start_depth, final_depth, step_down (all optional, mm)
        """
        j = _job()
        obj = doc.getObject(state.cam["target"])
        names = _select_faces(obj.Shape, a["select"])
        if not names:
            raise ValueError("selector matched no faces: %r" % a["select"])
        floor_z = min(obj.Shape.getElement(n).CenterOfMass.z for n in names)
        op = PPocket.Create("Pocket")
        op.Base = [(obj, names)]
        op.ToolController = doc.getObject(state.cam["tc"])
        j.Proxy.addOperation(op)
        doc.recompute()
        try:
            top = j.Stock.Shape.BoundBox.ZMax
        except Exception:
            top = obj.Shape.BoundBox.ZMax
        start = _sel_num(a, "start_depth", top, "'start_depth'")
        final = _sel_num(a, "final_depth", floor_z, "'final_depth'")
        for prop, val in (("StartDepth", start), ("FinalDepth", final)):
            if prop in op.PropertiesList:
                try:
                    op.setExpression(prop, None)
                except Exception:
                    pass
                setattr(op, prop, val)
        step = a.get("step_down")
        if step and "StepDown" in op.PropertiesList:
            step = _sel_num(a, "step_down", 0.0, "'step_down'")
            try:
                op.setExpression("StepDown", None)
            except Exception:
                pass
            op.StepDown = step
        doc.recompute()
        n = len(op.Path.Commands) if op.Path else 0
        state.cam["ops"].append(op.Name)
        try:
            sd = float(op.StepDown.getValueAs("mm"))
        except Exception:
            sd = None
        passes = max(1, math.ceil((start - final) / sd)) if sd else None
        return {"op": "pocket", "faces": names, "commands": n,
                "start_depth": _round(start), "final_depth": _round(final),
                "step_down": _round(sd) if sd else None, "passes": passes,
                "path_bbox": _path_bbox(op)}

    def drill(a):
        """Drill the cylindrical holes of the target solid.

        Picks cylindrical bore faces (axis along ``select.axis_dir``, default
        +Z, optionally filtered by ``select.diameter``) and emits a drilling
        cycle at each hole center. Depths are bound from the bore geometry:
        StartDepth = bore top, FinalDepth = bore bottom (drill through), so a
        through hole drills the full thickness without manual depths. Enable
        peck drilling with ``peck`` (mm) for deep holes / chip clearing.

        args: select{axis_dir, diameter, tol} (optional), start_depth,
              final_depth, peck (all optional, mm)
        """
        j = _job()
        obj = doc.getObject(state.cam["target"])
        names, centers, zmin, zmax = _select_holes(obj.Shape, a.get("select", {}))
        if not names:
            raise ValueError("no cylindrical holes matched selector: %r" % a.get("select", {}))
        op = PDrilling.Create("Drilling")
        op.Base = [(obj, names)]
        op.ToolController = doc.getObject(state.cam["tc"])
        j.Proxy.addOperation(op)
        doc.recompute()
        start = _sel_num(a, "start_depth",
                         zmax if zmax is not None else obj.Shape.BoundBox.ZMax,
                         "'start_depth'")
        final = _sel_num(a, "final_depth",
                         zmin if zmin is not None else obj.Shape.BoundBox.ZMin,
                         "'final_depth'")
        for prop, val in (("StartDepth", start), ("FinalDepth", final)):
            if prop in op.PropertiesList:
                try:
                    op.setExpression(prop, None)
                except Exception:
                    pass
                setattr(op, prop, val)
        peck = a.get("peck")
        if peck and "PeckDepth" in op.PropertiesList:
            peck = _sel_num(a, "peck", 0.0, "'peck'")
            if "PeckEnabled" in op.PropertiesList:
                op.PeckEnabled = True
            op.PeckDepth = peck
        doc.recompute()
        n = len(op.Path.Commands) if op.Path else 0
        state.cam["ops"].append(op.Name)
        return {"op": "drill", "faces": names, "holes": len(centers),
                "centers": centers, "commands": n,
                "start_depth": _round(start), "final_depth": _round(final),
                "depth": _round(start - final), "peck": _round(peck) if peck else None,
                "path_bbox": _path_bbox(op)}

    def gcode(a):
        """Post-process the job to a G-code file and return move statistics.

        args: path(.nc/.gcode), postprocessor(optional, default job's)
        """
        j = _job()
        if not state.cam.get("ops"):
            raise ValueError("no operations to post; add path.profile/path.pocket first")
        post = a.get("postprocessor") or state.cam.get("post", "grbl")
        pp = Proc.PostProcessorFactory.get_post_processor(j, post)
        out = pp.export()
        txt = out if isinstance(out, str) else "\n".join(seg[1] for seg in out)
        path = a.get("path") or os.path.join(tempfile.mkdtemp(prefix="daocam_"), "out.nc")
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w") as fh:
            fh.write(txt)
        lines = txt.splitlines()
        g0 = sum(1 for ln in lines if ln.strip().startswith("G0"))
        g1 = sum(1 for ln in lines if ln.strip().startswith("G1"))
        return {"path": path, "postprocessor": post, "lines": len(lines),
                "rapids_g0": g0, "feeds_g1": g1, "chars": len(txt),
                "ops": list(state.cam["ops"])}

    return {
        "path.job": job,
        "path.profile": profile,
        "path.pocket": pocket,
        "path.drill": drill,
        "path.gcode": gcode,
    }
