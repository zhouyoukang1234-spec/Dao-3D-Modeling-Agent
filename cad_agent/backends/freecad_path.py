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
import os
import tempfile

import FreeCAD as App

V = App.Vector


def _round(x, n=4):
    return round(float(x), n)


def register(state):
    import Path  # noqa: F401  (ensures the Path module initialises)
    import Path.Main.Job as PJob
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
        """Pick face names by predicate (axis extreme / index / normal)."""
        faces = shape.Faces
        if "index" in sel:
            idx = sel["index"]
            idx = [idx] if isinstance(idx, (int, float)) else idx
            return ["Face%d" % int(i) for i in idx]
        if "axis" in sel:
            ax = {"x": 0, "y": 1, "z": 2}[sel["axis"].lower()]
            side = sel.get("side", "max").lower()
            vals = [f.CenterOfMass[ax] for f in faces]
            target = min(vals) if side == "min" else max(vals)
            tol = sel.get("tol", 1e-4)
            return ["Face%d" % (i + 1) for i, v in enumerate(vals)
                    if abs(v - target) <= tol]
        if "normal" in sel:
            d = V(*sel["normal"])
            d = d.normalize() if d.Length > 1e-9 else d
            out = []
            for i, f in enumerate(faces):
                u0, u1, v0, v1 = f.ParameterRange
                n = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
                if n.Length > 1e-9 and n.normalize().dot(d) >= sel.get("min_dot", 0.95):
                    out.append("Face%d" % (i + 1))
            return out
        raise ValueError("unknown face selector: %r" % sel)

    def _top_face(shape):
        zmax = max(f.CenterOfMass.z for f in shape.Faces)
        for i, f in enumerate(shape.Faces):
            if abs(f.CenterOfMass.z - zmax) < 1e-6:
                return "Face%d" % (i + 1)
        return "Face1"

    def _job():
        j = doc.getObject(state.cam.get("job", ""))
        if j is None:
            raise RuntimeError("call path.job first")
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
            raise RuntimeError("profile produced no path (bad face selection?)")
        state.cam["ops"].append(op.Name)
        return {"op": "profile", "faces": names, "commands": n,
                "path_bbox": _path_bbox(op)}

    def pocket(a):
        """Pocket (clear material from) selected faces.

        args: select (faces bounding the pocket floor)
        """
        j = _job()
        obj = doc.getObject(state.cam["target"])
        names = _select_faces(obj.Shape, a["select"])
        if not names:
            raise ValueError("selector matched no faces: %r" % a["select"])
        op = PPocket.Create("Pocket")
        op.Base = [(obj, names)]
        op.ToolController = doc.getObject(state.cam["tc"])
        j.Proxy.addOperation(op)
        doc.recompute()
        n = len(op.Path.Commands) if op.Path else 0
        state.cam["ops"].append(op.Name)
        return {"op": "pocket", "faces": names, "commands": n,
                "path_bbox": _path_bbox(op)}

    def gcode(a):
        """Post-process the job to a G-code file and return move statistics.

        args: path(.nc/.gcode), postprocessor(optional, default job's)
        """
        j = _job()
        if not state.cam.get("ops"):
            raise RuntimeError("no operations to post; add path.profile/path.pocket first")
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
        "path.gcode": gcode,
    }
