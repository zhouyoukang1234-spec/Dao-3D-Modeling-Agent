"""Structural FEM engine — the ``fem.*`` tool group (CalculiX + gmsh).

This is the deep-boundary group: instead of stopping at geometry, it drives
FreeCAD's *bundled physics solver*. FreeCAD 1.0 ships the CalculiX ``ccx``
finite-element solver and the ``gmsh`` mesher; this module wires the live
document's solids into a real ``Fem::FemAnalysis`` (material + boundary
conditions + load), meshes them, runs ccx headlessly, and reads back the
von-Mises stress / displacement field. That closes the loop the rest of the
agent needs to *self-correct on physics*, not just geometry: size a part until
``max von Mises < allowable`` (see ``fem.autosize``).

Everything is geometry-driven, never shape-hard-coded: faces for constraints and
loads are picked by predicate (extreme along an axis, by normal, or by index)
so the same ops drive a beam, a bracket, a lug or a housing unchanged.

Runs inside freecadcmd. ``register(state)`` returns ``{op_name: callable}``.
"""
import math
import os
import tempfile

import FreeCAD as App
import Part

V = App.Vector

# Minimal engineering material library. Mechanical: E in MPa, density kg/m^3,
# yield MPa. Thermal: alpha = linear expansion 1/K, k = conductivity W/m/K,
# cp = specific heat J/kg/K (used by the thermomech / thermal path).
MATERIALS = {
    "steel":    {"name": "Steel-Generic",    "E": 210000.0, "nu": 0.30, "rho": 7900.0, "yield": 250.0, "alpha": 1.2e-5,  "k": 43.0,  "cp": 500.0},
    "aluminum": {"name": "Aluminum-6061",    "E": 69000.0,  "nu": 0.33, "rho": 2700.0, "yield": 240.0, "alpha": 2.34e-5, "k": 167.0, "cp": 896.0},
    "titanium": {"name": "Titanium-Ti6Al4V", "E": 110000.0, "nu": 0.34, "rho": 4430.0, "yield": 880.0, "alpha": 8.6e-6,  "k": 6.7,   "cp": 523.0},
    "abs":      {"name": "ABS-Plastic",      "E": 2300.0,   "nu": 0.35, "rho": 1050.0, "yield": 40.0,  "alpha": 7.4e-5,  "k": 0.17,  "cp": 1300.0},
    "pla":      {"name": "PLA-Plastic",      "E": 3500.0,   "nu": 0.36, "rho": 1240.0, "yield": 50.0,  "alpha": 6.8e-5,  "k": 0.13,  "cp": 1800.0},
}


def _round(x, n=4):
    return round(float(x), n)


def _parse_buckling_factors(workdir):
    """Read the buckling load factors from CalculiX's .dat file.

    CalculiX writes, under a 'B U C K L I N G   F A C T O R   O U T P U T'
    banner, lines of '<mode no> <factor>'. Return the factors in mode order.
    """
    if not workdir or not os.path.isdir(workdir):
        return []
    dats = [f for f in os.listdir(workdir) if f.endswith(".dat")]
    if not dats:
        return []
    text = open(os.path.join(workdir, dats[0])).read()
    factors = []
    in_block = False
    for line in text.splitlines():
        squashed = line.replace(" ", "").upper()
        if "BUCKLINGFACTOROUTPUT" in squashed:
            in_block = True
            continue
        if in_block:
            parts = line.split()
            if len(parts) == 2:
                try:
                    factors.append((int(parts[0]), float(parts[1])))
                except ValueError:
                    if factors:
                        break
            elif factors and line.strip() and "FACTOR" not in line.upper() \
                    and "MODE" not in line.upper():
                break
    factors.sort(key=lambda t: t[0])
    return [f for _, f in factors]


def register(state):
    import ObjectsFem  # provided by freecadcmd
    doc = state.doc

    # Lazy per-document FEM bookkeeping kept on the kernel state.
    if not hasattr(state, "fem"):
        state.fem = {}

    def _resolve(name):
        """Logical name -> a doc object carrying a solid .Shape."""
        oname = state.shapes.get(name) or state.bodies.get(name)
        if not oname:
            raise KeyError("no such solid/body: %s" % name)
        obj = doc.getObject(oname)
        if obj is None:
            raise KeyError("object missing: %s" % name)
        if not getattr(obj, "Shape", None) or obj.Shape.isNull():
            raise ValueError("%s has no solid shape to analyse" % name)
        return obj

    def _faces_info(shape):
        info = []
        for i, f in enumerate(shape.Faces):
            c = f.CenterOfMass
            try:
                u0, u1, v0, v1 = f.ParameterRange
                n = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
            except Exception:
                n = V(0, 0, 0)
            info.append({"face": "Face%d" % (i + 1), "area": _round(f.Area),
                         "com": [_round(c.x), _round(c.y), _round(c.z)],
                         "normal": [_round(n.x, 3), _round(n.y, 3), _round(n.z, 3)]})
        return info

    def _select_faces(shape, sel):
        """Pick face names by predicate. sel is one of:
          {"axis":"x|y|z","side":"min|max"}  extreme face centre along an axis
          {"index": n | [n,...]}             1-based face index/indices
          {"normal":[x,y,z]}                 faces whose outward normal ~ dir
        """
        faces = shape.Faces
        if "index" in sel:
            idx = sel["index"]
            idx = [idx] if isinstance(idx, (int, float)) else idx
            return ["Face%d" % int(i) for i in idx]
        if "axis" in sel:
            ax = {"x": 0, "y": 1, "z": 2}[sel["axis"].lower()]
            side = sel.get("side", "min").lower()
            vals = [f.CenterOfMass[ax] for f in faces]
            target = min(vals) if side == "min" else max(vals)
            tol = sel.get("tol", 1e-4)
            return ["Face%d" % (i + 1) for i, v in enumerate(vals)
                    if abs(v - target) <= tol]
        if "cyl_radius" in sel:
            r = float(sel["cyl_radius"])
            tol = sel.get("tol", 1e-3)
            out = []
            for i, f in enumerate(faces):
                surf = f.Surface
                if isinstance(surf, Part.Cylinder) and abs(surf.Radius - r) <= tol:
                    out.append("Face%d" % (i + 1))
            return out
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

    def _clear(a=None):
        """Drop any existing analysis objects from the document."""
        for o in list(doc.Objects):
            if (o.isDerivedFrom("Fem::FemAnalysis")
                    or o.isDerivedFrom("Fem::FemSolverObject")
                    or o.isDerivedFrom("Fem::Constraint")
                    or o.isDerivedFrom("Fem::FemMeshObject")
                    or o.isDerivedFrom("Fem::FemResultObject")
                    or o.isDerivedFrom("App::MaterialObjectPython")
                    or "Fem" in o.TypeId):
                try:
                    doc.removeObject(o.Name)
                except Exception:
                    pass
        state.fem = {}
        doc.recompute()
        return {"cleared": True}

    def setup(a):
        """Create analysis + CalculiX solver + material + gmsh mesh on a solid.

        args: target, material(name|dict)=steel, mesh_size=auto, order=2
        returns mesh stats + per-face geometry (com/normal/area) so the caller
        can pick faces for fix/load by inspecting real geometry.
        """
        _clear()
        obj = _resolve(a["target"])
        shape = obj.Shape

        analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
        solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiX")
        solver.AnalysisType = "static"
        solver.GeometricalNonlinearity = "linear"
        solver.MatrixSolverType = "default"
        solver.IterationsControlParameterTimeUse = False
        analysis.addObject(solver)

        spec = a.get("material", "steel")
        if isinstance(spec, str):
            mp = MATERIALS.get(spec.lower(), MATERIALS["steel"])
        else:
            mp = {**MATERIALS["steel"], **spec}
        mat = ObjectsFem.makeMaterialSolid(doc, "Material")
        md = mat.Material
        md["Name"] = mp["name"]
        md["YoungsModulus"] = "%s MPa" % mp["E"]
        md["PoissonRatio"] = "%s" % mp["nu"]
        md["Density"] = "%s kg/m^3" % mp["rho"]
        # thermal properties so the same material drives the thermomech path
        if mp.get("alpha") is not None:
            md["ThermalExpansionCoefficient"] = "%s m/m/K" % mp["alpha"]
        if mp.get("k") is not None:
            md["ThermalConductivity"] = "%s W/m/K" % mp["k"]
        if mp.get("cp") is not None:
            md["SpecificHeat"] = "%s J/kg/K" % mp["cp"]
        mat.Material = md
        analysis.addObject(mat)

        bb = shape.BoundBox
        size = a.get("mesh_size") or max(1.0, bb.DiagonalLength / 18.0)
        mesh = ObjectsFem.makeMeshGmsh(doc, "FEMMesh")
        mesh.Shape = obj
        mesh.CharacteristicLengthMax = float(size)
        mesh.CharacteristicLengthMin = float(size) / 4.0
        mesh.ElementOrder = "2nd" if int(a.get("order", 2)) >= 2 else "1st"
        analysis.addObject(mesh)
        doc.recompute()

        from femmesh.gmshtools import GmshTools
        err = GmshTools(mesh).create_mesh()
        doc.recompute()
        fm = mesh.FemMesh
        if fm is None or fm.NodeCount == 0:
            raise RuntimeError("meshing produced no nodes (gmsh: %r)" % (err,))

        state.fem = {"analysis": analysis.Name, "solver": solver.Name,
                     "material": mat.Name, "mesh": mesh.Name, "target": obj.Name,
                     "yield": mp["yield"], "E": mp["E"], "alpha": mp.get("alpha"),
                     "constraints": []}
        return {"target": a["target"], "material": mp["name"], "yield_mpa": mp["yield"],
                "mesh_size": _round(size), "nodes": fm.NodeCount,
                "elements": fm.VolumeCount, "faces": _faces_info(shape)}

    def _need():
        if not state.fem.get("analysis"):
            raise RuntimeError("call fem.setup first")
        return (doc.getObject(state.fem["analysis"]), doc.getObject(state.fem["target"]))

    def fix(a):
        """Fully fix the faces matched by selector ``select`` (a dict)."""
        analysis, obj = _need()
        names = _select_faces(obj.Shape, a["select"])
        if not names:
            raise ValueError("selector matched no faces: %r" % a["select"])
        c = ObjectsFem.makeConstraintFixed(doc, "Fixed")
        c.References = [(obj, n) for n in names]
        analysis.addObject(c)
        state.fem["constraints"].append(c.Name)
        doc.recompute()
        return {"constraint": "fixed", "faces": names}

    def support(a):
        """Roller / symmetry support: zero only the selected displacement
        components on the matched faces (the others stay free).

        args: select, fix=["x"|"y"|"z", ...]  (axes whose displacement = 0)
        This is what pressure-vessel / symmetric models need -- a full fix would
        clamp radial expansion and corrupt the stress field; a roller doesn't.
        """
        analysis, obj = _need()
        names = _select_faces(obj.Shape, a["select"])
        if not names:
            raise ValueError("selector matched no faces: %r" % a["select"])
        axes = [str(x).lower() for x in a.get("fix", [])]
        if not axes:
            raise ValueError("fem.support needs fix=[axes] to zero")
        c = ObjectsFem.makeConstraintDisplacement(doc, "Support")
        c.References = [(obj, n) for n in names]
        for ax in ("x", "y", "z"):
            free = ax not in axes
            setattr(c, ax + "Free", free)
            if not free:
                setattr(c, ax + "Displacement", App.Units.Quantity("0 mm"))
        analysis.addObject(c)
        state.fem["constraints"].append(c.Name)
        doc.recompute()
        return {"constraint": "support", "faces": names, "zeroed": axes}

    def _direction_ref(shape, d):
        """Find a geometry reference whose orientation is parallel to ``d``, plus
        the ``Reversed`` flag needed so the effective force points along ``d``.

        FreeCAD's ConstraintForce takes a *Direction* reference (a planar face
        normal or a straight edge tangent) and a Reversed flag — DirectionVector
        is a read-only output computed from them. So we locate such a reference.
        Returns (subname, reversed) or (None, None) if nothing is parallel.
        """
        d = d.normalize()
        # prefer a planar face whose outward normal is parallel to d
        for i, f in enumerate(shape.Faces):
            try:
                if not isinstance(f.Surface, Part.Plane):
                    continue
                u0, u1, v0, v1 = f.ParameterRange
                n = f.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0).normalize()
            except Exception:
                continue
            dot = n.dot(d)
            if abs(dot) >= 0.999:
                return "Face%d" % (i + 1), bool(dot < 0)
        # fall back to a straight edge whose tangent is parallel to d
        for i, e in enumerate(shape.Edges):
            try:
                if not isinstance(e.Curve, Part.Line):
                    continue
                t = (e.Vertexes[1].Point - e.Vertexes[0].Point).normalize()
            except Exception:
                continue
            dot = t.dot(d)
            if abs(dot) >= 0.999:
                return "Edge%d" % (i + 1), bool(dot < 0)
        return None, None

    def load(a):
        """Apply a force (N, total) or pressure (MPa) to selected faces.

        args: select, kind=force|pressure, value, direction=[x,y,z] (force only)
        """
        analysis, obj = _need()
        names = _select_faces(obj.Shape, a["select"])
        if not names:
            raise ValueError("selector matched no faces: %r" % a["select"])
        kind = a.get("kind", "force").lower()
        if kind == "pressure":
            c = ObjectsFem.makeConstraintPressure(doc, "Pressure")
            c.References = [(obj, n) for n in names]
            # PropertyPressure stores base units (mN/mm^2); 1 MPa = 1 N/mm^2.
            c.Pressure = App.Units.Quantity("%s MPa" % float(a["value"]))
            c.Reversed = bool(a.get("reversed", False))
            analysis.addObject(c)
            out = {"constraint": "pressure", "faces": names, "mpa": float(a["value"])}
        else:
            c = ObjectsFem.makeConstraintForce(doc, "Force")
            c.References = [(obj, n) for n in names]
            # PropertyForce stores base units (mN); assign as a N quantity so the
            # solver sees Newtons, not milli-Newtons (a silent 1000x error).
            c.Force = App.Units.Quantity("%s N" % float(a["value"]))
            d = V(*(a.get("direction") or [0, 0, -1]))
            if d.Length < 1e-9:
                d = V(0, 0, -1)
            sub, _ = _direction_ref(obj.Shape, d)
            if sub is None:
                raise ValueError("no face/edge parallel to direction %r for the load" % list(d))
            c.Direction = (obj, [sub])
            c.Reversed = False
            analysis.addObject(c)
            doc.recompute()
            # FreeCAD derives DirectionVector from the reference with its own sign
            # convention; flip Reversed until the *effective* vector matches the
            # requested direction (ground-truth self-correction, not a guess).
            dv = c.DirectionVector
            if dv.dot(d.normalize()) < 0:
                c.Reversed = True
                doc.recompute()
                dv = c.DirectionVector
            out = {"constraint": "force", "faces": names, "newtons": float(a["value"]),
                   "direction_ref": sub, "reversed": bool(c.Reversed),
                   "effective_dir": [_round(dv.x, 3), _round(dv.y, 3), _round(dv.z, 3)]}
        state.fem["constraints"].append(c.Name)
        doc.recompute()
        return out

    def _run_ccx():
        """Write the input deck, run CalculiX headlessly, load results back.

        Returns the list of ``Fem::FemResultObject`` produced (one for static,
        N for an N-mode frequency run).
        """
        from femtools import ccxtools
        analysis = doc.getObject(state.fem["analysis"])
        solver = doc.getObject(state.fem["solver"])
        workdir = tempfile.mkdtemp(prefix="daofem_")
        os.makedirs(workdir, exist_ok=True)
        try:
            solver.WorkingDir = workdir
        except Exception:
            pass
        fea = ccxtools.FemToolsCcx(analysis=analysis, solver=solver)
        fea.update_objects()
        try:
            fea.setup_working_dir(workdir)
        except Exception:
            pass
        fea.purge_results()
        msg = fea.check_prerequisites()
        if msg and "Working directory" not in msg:
            raise RuntimeError("FEM prerequisites not met: %s" % msg)
        fea.write_inp_file()
        fea.ccx_run()
        fea.load_results()
        state.fem["workdir"] = workdir
        results = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")]
        if not results:
            raise RuntimeError("CalculiX produced no result object")
        return results

    def solve(a):
        """Run CalculiX static analysis; return max von-Mises + displacement + safety.

        args: allowable_mpa(optional, default=material yield)
        """
        solver = doc.getObject(state.fem["solver"])
        solver.AnalysisType = "static"
        result = _run_ccx()[0]
        state.fem["result"] = result.Name
        vm = list(result.vonMises or [])
        disp = list(result.DisplacementLengths or [])
        max_vm = max(vm) if vm else 0.0
        max_disp = max(disp) if disp else 0.0
        allow = float(a.get("allowable_mpa") or state.fem.get("yield") or 250.0)
        sf = (allow / max_vm) if max_vm > 1e-9 else float("inf")
        return {"max_von_mises_mpa": _round(max_vm), "max_disp_mm": _round(max_disp, 6),
                "allowable_mpa": _round(allow), "safety_factor": _round(sf, 3),
                "passed": bool(max_vm <= allow), "result_nodes": len(vm)}

    def temperature(a):
        """Apply a temperature boundary condition + reference temperature.

        args: value (applied temp, K), ref (reference/stress-free temp, K=0),
              select (faces to heat; default = every face -> uniform field)
        The thermal strain that drives the thermomech stress is alpha*(value-ref).
        """
        analysis, obj = _need()
        sel = a.get("select")
        names = _select_faces(obj.Shape, sel) if sel else \
            ["Face%d" % (i + 1) for i in range(len(obj.Shape.Faces))]
        if not names:
            raise ValueError("selector matched no faces: %r" % sel)
        ref = float(a.get("ref", 0.0))
        init = ObjectsFem.makeConstraintInitialTemperature(doc, "Tref")
        init.initialTemperature = App.Units.Quantity("%s K" % ref)
        analysis.addObject(init)
        ct = ObjectsFem.makeConstraintTemperature(doc, "Temp")
        ct.References = [(obj, n) for n in names]
        ct.Temperature = App.Units.Quantity("%s K" % float(a["value"]))
        analysis.addObject(ct)
        state.fem["constraints"] += [init.Name, ct.Name]
        state.fem["dT"] = float(a["value"]) - ref
        doc.recompute()
        return {"constraint": "temperature", "faces": names,
                "value_k": float(a["value"]), "ref_k": ref,
                "delta_k": float(a["value"]) - ref}

    def thermal(a):
        """Run a thermomechanical steady-state analysis (CalculiX *COUPLED
        TEMPERATURE-DISPLACEMENT, steady state): conducts heat to equilibrium and
        returns the resulting thermal-stress field + temperature range.

        Needs a temperature BC (fem.temperature) and enough displacement supports
        (fem.support/fem.fix) to react the thermal strain. args: allowable_mpa.
        """
        _need()
        solver = doc.getObject(state.fem["solver"])
        solver.AnalysisType = "thermomech"
        try:
            solver.ThermoMechSteadyState = True
        except Exception:
            pass
        result = _run_ccx()[0]
        state.fem["result"] = result.Name
        vm = list(result.vonMises or [])
        temp = list(getattr(result, "Temperature", []) or [])
        disp = list(result.DisplacementLengths or [])
        max_vm = max(vm) if vm else 0.0
        allow = float(a.get("allowable_mpa") or state.fem.get("yield") or 250.0)
        sf = (allow / max_vm) if max_vm > 1e-9 else float("inf")
        return {"max_von_mises_mpa": _round(max_vm),
                "max_disp_mm": _round(max(disp) if disp else 0.0, 6),
                "t_max_k": _round(max(temp) if temp else 0.0),
                "t_min_k": _round(min(temp) if temp else 0.0),
                "allowable_mpa": _round(allow), "safety_factor": _round(sf, 3),
                "passed": bool(max_vm <= allow), "result_nodes": len(vm)}

    def buckle(a):
        """Run a linear (Euler) buckling analysis (CalculiX *BUCKLE).

        Needs a fixed support (fem.fix) and a reference load (fem.load, the
        compressive load whose multiplier we seek). Returns the buckling load
        factors lambda_i in ascending order: the structure goes unstable when the
        applied load is scaled by lambda_1 (critical load = lambda_1 * applied).
        args: modes=1 (number of factors to extract).
        """
        _need()
        n = int(a.get("modes", 1))
        solver = doc.getObject(state.fem["solver"])
        solver.AnalysisType = "buckling"
        try:
            solver.BucklingFactors = n
        except Exception:
            pass
        _run_ccx()
        factors = _parse_buckling_factors(state.fem.get("workdir"))
        if not factors:
            raise RuntimeError("CalculiX produced no buckling factor output")
        return {"modes": len(factors), "factors": [_round(f, 4) for f in factors],
                "critical_factor": _round(factors[0], 4)}

    def spin(a):
        """Apply a centrifugal (rotational) body load about an axis (CalculiX
        *DLOAD CENTRIF): the inertial body force rho*omega^2*r of a spinning
        part. Pair with fem.support/fem.fix to react it, then fem.solve.

        args: rpm | hz (rotation speed), axis=[0,0,1] (spin axis direction),
              base=[0,0,0] (a point on the axis).
        A solid disk validates the closed form for the central stress:
        sigma_centre = (3+nu)/8 * rho * omega^2 * R^2.
        """
        analysis, obj = _need()
        if a.get("hz") is not None:
            hz = float(a["hz"])
        elif a.get("rpm") is not None:
            hz = float(a["rpm"]) / 60.0
        else:
            raise ValueError("fem.spin needs rpm or hz")
        axis = V(*(a.get("axis") or [0, 0, 1]))
        axis = axis.normalize() if axis.Length > 1e-9 else V(0, 0, 1)
        base = V(*(a.get("base") or [0, 0, 0]))
        span = max(1.0, obj.Shape.BoundBox.DiagonalLength)
        old = doc.getObject("SpinAxis")
        if old is not None:
            doc.removeObject("SpinAxis")
        line = doc.addObject("Part::Feature", "SpinAxis")
        line.Shape = Part.makeLine(base - axis * span, base + axis * span)
        cf = ObjectsFem.makeConstraintCentrif(doc, "Centrif")
        # RotationFrequency is a true frequency (Hz = rev/s); omega = 2*pi*f.
        cf.RotationFrequency = App.Units.Quantity("%s Hz" % hz)
        cf.RotationAxis = [(line, "Edge1")]
        analysis.addObject(cf)
        state.fem["constraints"].append(cf.Name)
        omega = 2.0 * math.pi * hz
        state.fem["omega"] = omega
        doc.recompute()
        return {"constraint": "centrif", "hz": _round(hz, 4),
                "rpm": _round(hz * 60.0, 2), "omega_rad_s": _round(omega, 4),
                "axis": [_round(axis.x, 3), _round(axis.y, 3), _round(axis.z, 3)],
                "base": [_round(base.x, 3), _round(base.y, 3), _round(base.z, 3)]}

    def modal(a):
        """Run a CalculiX eigenfrequency (modal) analysis.

        Needs constraints from fem.fix (loads are ignored by a frequency run).
        args: modes=6
        returns ascending natural frequencies in Hz.
        """
        _need()
        n = int(a.get("modes", 6))
        solver = doc.getObject(state.fem["solver"])
        solver.AnalysisType = "frequency"
        try:
            solver.EigenmodesCount = n
        except Exception:
            pass
        results = _run_ccx()
        freqs = sorted(float(getattr(o, "EigenmodeFrequency", 0.0) or 0.0)
                       for o in results)
        freqs = [f for f in freqs if f > 1e-6]
        return {"modes": len(freqs), "frequencies_hz": [_round(f, 2) for f in freqs]}

    def contour(a):
        """Render the last solved von-Mises stress field as a coloured point cloud.

        args: path(.png), view=iso|front|top|right, size=900
        Proves the result field is real data, not a scalar summary, and gives
        the agent an 'eye' on *where* the part is overstressed.
        """
        name = state.fem.get("result")
        result = doc.getObject(name) if name else None
        if result is None:
            for o in doc.Objects:
                if o.isDerivedFrom("Fem::FemResultObject") and list(o.vonMises or []):
                    result = o
        if result is None:
            raise RuntimeError("no solved result to contour; call fem.solve first")
        vm = list(result.vonMises or [])
        nn = list(result.NodeNumbers or [])
        nodes = result.Mesh.FemMesh.Nodes
        if not vm or not nn:
            raise RuntimeError("result carries no von-Mises field")
        pts = [(nodes[i].x, nodes[i].y, nodes[i].z) for i in nn]

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: F401
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        views = {"iso": (28, -60), "front": (0, -90), "top": (90, -90), "right": (0, 0)}
        elev, azim = views.get(a.get("view", "iso"), views["iso"])
        px = int(a.get("size", 900))
        fig = plt.figure(figsize=(px / 100.0, px / 100.0), dpi=100)
        ax = fig.add_subplot(111, projection="3d")
        sc = ax.scatter(xs, ys, zs, c=vm, cmap="jet", s=14, depthshade=False)
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_box_aspect((max(xs) - min(xs) or 1, max(ys) - min(ys) or 1,
                               max(zs) - min(zs) or 1))
        except Exception:
            pass
        cb = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
        cb.set_label("von Mises (MPa)")
        ax.set_title("%s  max=%.1f MPa" % (state.fem.get("target", ""), max(vm)))
        path = a["path"]
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        return {"path": path, "nodes": len(vm), "max_von_mises_mpa": _round(max(vm)),
                "bytes": os.path.getsize(path) if os.path.exists(path) else 0}

    return {
        "fem.setup": setup,
        "fem.fix": fix,
        "fem.support": support,
        "fem.load": load,
        "fem.temperature": temperature,
        "fem.solve": solve,
        "fem.thermal": thermal,
        "fem.buckle": buckle,
        "fem.spin": spin,
        "fem.modal": modal,
        "fem.contour": contour,
        "fem.clear": _clear,
    }
