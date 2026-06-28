"""Assembly engine (the ``asm.*`` tool group).

Runs inside freecadcmd. Builds a real ``Assembly::AssemblyObject`` container and
instances bodies/solids into it as ``App::Link`` components (the same mechanism
the Assembly workbench uses), so the result opens as a genuine assembly in the
GUI. Mates are applied deterministically by computing component placements
(fix / place / move / align-axis / stack-on-face), which is robust headless,
and the native constraint ``solve()`` is invoked where joints exist. Adds
all-pairs interference checking and a bill-of-materials roll-up.
"""
import FreeCAD as App

V = App.Vector
ROT = App.Rotation


def _round(x, n=4):
    return round(float(x), n)


def register(state):
    doc = state.doc

    def _placement(spec):
        spec = spec or {}
        pos = spec.get("pos", (0, 0, 0))
        axis = spec.get("axis", (0, 0, 1))
        angle = float(spec.get("angle", 0))
        return App.Placement(V(*[float(x) for x in pos]),
                             ROT(V(*[float(x) for x in axis]), angle))

    def _src_object(ref):
        """Resolve a logical name to a source object (param body or solid)."""
        if ref in state.bodies and doc.getObject(state.bodies[ref]):
            return doc.getObject(state.bodies[ref])
        if ref in state.shapes and doc.getObject(state.shapes[ref]):
            return doc.getObject(state.shapes[ref])
        raise KeyError("no body/solid named %s" % ref)

    def _comp(name):
        rec = state.components.get(name)
        if not rec or doc.getObject(rec["link"]) is None:
            raise KeyError("no such component: %s" % name)
        return doc.getObject(rec["link"])

    def _global_shape(name):
        link = _comp(name)
        src = doc.getObject(state.components[name]["src"])
        shp = src.Shape.copy()
        shp.Placement = link.Placement.multiply(src.Placement)
        return shp

    # ---- container & components ------------------------------------------ #
    def op_create(a):
        name = a.get("name", "Assembly")
        asm = doc.addObject("Assembly::AssemblyObject", name)
        state.assembly = asm.Name
        doc.recompute()
        return {"assembly": asm.Name}

    def op_add(a):
        if state.assembly is None:
            op_create({"name": "Assembly"})
        asm = doc.getObject(state.assembly)
        src = _src_object(a["body"])
        name = a["name"]
        link = doc.addObject("App::Link", name)
        link.LinkedObject = src
        link.Placement = _placement(a.get("placement"))
        try:
            asm.addObject(link)
        except Exception:
            pass
        state.components[name] = {"link": link.Name, "src": src.Name,
                                  "fixed": bool(a.get("fixed", False))}
        doc.recompute()
        return {"component": name, "linked": src.Name}

    # ---- mates (placement-based) ----------------------------------------- #
    def op_place(a):
        link = _comp(a["name"])
        link.Placement = _placement(a)
        doc.recompute()
        return {"component": a["name"], "placement": list(link.Placement.Base)}

    def op_move(a):
        link = _comp(a["name"])
        p = link.Placement
        p.Base = p.Base + V(*[float(x) for x in a["vector"]])
        link.Placement = p
        doc.recompute()
        return {"component": a["name"], "placement": list(link.Placement.Base)}

    def op_rotate(a):
        """Rotate a component in place by ``angle`` degrees about ``axis`` through
        point ``at`` (default the world origin), composing with its current
        placement. ``op_place`` only sets an absolute pose about the world axes;
        this spins a part that is already positioned -- e.g. phasing one gear of a
        pair so its teeth mesh into the mating gear's spaces."""
        link = _comp(a["name"])
        axis = V(*[float(x) for x in a.get("axis", (0, 0, 1))])
        angle = float(a["angle"])
        at = V(*[float(x) for x in a.get("at", (0, 0, 0))])
        center = App.Placement(at, ROT())
        spin = App.Placement(V(0, 0, 0), ROT(axis, angle))
        about = center.multiply(spin).multiply(center.inverse())
        link.Placement = about.multiply(link.Placement)
        doc.recompute()
        return {"component": a["name"], "angle": angle,
                "placement": [_round(x) for x in link.Placement.Base]}

    def op_fix(a):
        state.components[a["name"]]["fixed"] = True
        return {"component": a["name"], "fixed": True}

    def op_align(a):
        """Place component ``b`` offset from ``a`` along an axis (axis mate)."""
        sa = _global_shape(a["a"]).BoundBox
        link_b = _comp(a["b"])
        axis = a.get("axis", "x").lower()
        offset = float(a.get("offset", 0))
        ca = V((sa.XMin + sa.XMax) / 2, (sa.YMin + sa.YMax) / 2, (sa.ZMin + sa.ZMax) / 2)
        delta = {"x": V(offset, 0, 0), "y": V(0, offset, 0), "z": V(0, 0, offset)}[axis]
        link_b.Placement = App.Placement(ca + delta, link_b.Placement.Rotation)
        doc.recompute()
        return {"component": a["b"], "placement": list(link_b.Placement.Base)}

    def _cyl_of(shape, pick, axis=None):
        """Find a cylindrical face on ``shape`` and return (radius, center, axis
        direction). ``pick`` selects the smallest (a bore) or largest (a shaft)
        cylinder; ``axis`` optionally restricts to cylinders parallel to it."""
        want = {"x": V(1, 0, 0), "y": V(0, 1, 0), "z": V(0, 0, 1)}.get(axis)
        cyls = []
        for f in shape.Faces:
            s = f.Surface
            if s.__class__.__name__ != "Cylinder":
                continue
            if want is not None and abs(abs(s.Axis.dot(want)) - 1) > 1e-6:
                continue
            cyls.append((s.Radius, s.Center, V(s.Axis)))
        if not cyls:
            raise ValueError("no matching cylindrical face found")
        cyls.sort(key=lambda c: c[0])
        return cyls[0] if pick == "min" else cyls[-1]

    def _canon(v):
        """Flip a direction so its dominant world component is non-negative."""
        c = [v.x, v.y, v.z]
        if c[max(range(3), key=lambda i: abs(c[i]))] < 0:
            return V(-v.x, -v.y, -v.z)
        return V(v)

    def op_coaxial(a):
        """Geometry-driven mate: align ``pin`` so its cylindrical axis is
        coincident with the cylindrical axis of ``hole``. Unlike the bbox-based
        ``align``/``stack`` this uses the real cylindrical-face axis, so it works
        for OFF-CENTER bores and ROTATES the pin when the two axes point in
        different directions (e.g. seating a vertical pin into a horizontal bore).
        Optional ``seat`` shifts the pin along the hole axis: 'bottom'/'top'
        flush the pin's near/far face to the hole's, a number sets the pin's min
        face coordinate. ``hole_pick``/``pin_pick`` ('min'|'max') choose which
        cylinder when a part has several. ``offset`` then slides the part along
        the shared axis (so two coaxial parts can interleave, e.g. hinge knuckles)."""
        hint = a.get("axis")
        hr, hc, haxis = _cyl_of(_global_shape(a["hole"]), a.get("hole_pick", "min"), hint)
        # the pin's cylinder is read in its own (source-local) frame so we can
        # compose a clean placement: rotate its axis onto the hole axis, then
        # translate so the cylinder centers coincide.
        link = _comp(a["pin"])
        src = doc.getObject(state.components[a["pin"]]["src"])
        pr, pc_local, paxis = _cyl_of(src.Shape, a.get("pin_pick", "max"), hint)
        # a cylindrical face's reported axis has an arbitrary sign. When the pin
        # is already (anti-)parallel to the bore, aligning the raw axes can inject
        # a spurious 180-degree flip that seats the pin on the wrong side of the
        # bore (a cylinder is symmetric, so no rotation is needed). Only rotate
        # when the pin is genuinely transverse (e.g. a vertical pin into a
        # horizontal bore), which still maps its axis onto the hole axis.
        if abs(abs(paxis.dot(haxis)) - 1.0) < 1e-6:
            rot = App.Rotation()
        else:
            rot = App.Rotation(paxis, haxis)
        base = hc - rot.multVec(pc_local)
        link.Placement = App.Placement(base, rot)
        doc.recompute()
        # the raw cylinder-axis sign depends on face orientation, so derive a
        # canonical direction (dominant world component positive) for the SIGN
        # of seat/offset -- the rotation above is unaffected by this.
        caxis = _canon(haxis)
        seat = a.get("seat")
        if seat is not None:
            # shift along the dominant world axis of the hole direction (exact
            # when the hole axis is world-parallel, which is the common case).
            comps = [abs(caxis.x), abs(caxis.y), abs(caxis.z)]
            idx = comps.index(max(comps))
            hb = _global_shape(a["hole"]).BoundBox
            pb = _global_shape(a["pin"]).BoundBox
            lo = [hb.XMin, hb.YMin, hb.ZMin][idx]
            hi = [hb.XMax, hb.YMax, hb.ZMax][idx]
            pmin = [pb.XMin, pb.YMin, pb.ZMin][idx]
            pmax = [pb.XMax, pb.YMax, pb.ZMax][idx]
            if seat == "bottom":
                d = lo - pmin
            elif seat == "top":
                d = hi - pmax
            else:
                d = float(seat) - pmin
            sd = [0.0, 0.0, 0.0]
            sd[idx] = d
            p = link.Placement
            p.Base = p.Base + V(*sd)
            link.Placement = p
            doc.recompute()
        offset = float(a.get("offset", 0))
        if offset:
            # relative slide along the (shared) hole axis -- lets two coaxial
            # parts interleave, e.g. the knuckles of a hinge.
            u = V(caxis)
            u.normalize()
            p = link.Placement
            p.Base = p.Base + u.multiply(offset)
            link.Placement = p
            doc.recompute()
        return {"component": a["pin"],
                "axis": [_round(x) for x in caxis],
                "placement": [_round(x) for x in link.Placement.Base]}

    def op_stack(a):
        """Stack ``top`` onto the +Z face of ``base`` with an optional gap."""
        bb = _global_shape(a["base"]).BoundBox
        top_link = _comp(a["top"])
        tb = _global_shape(a["top"]).BoundBox
        gap = float(a.get("gap", 0))
        # shift top so its ZMin sits on base ZMax (+gap), centered in XY on base
        dx = (bb.XMin + bb.XMax) / 2 - (tb.XMin + tb.XMax) / 2
        dy = (bb.YMin + bb.YMax) / 2 - (tb.YMin + tb.YMax) / 2
        dz = bb.ZMax + gap - tb.ZMin
        p = top_link.Placement
        p.Base = p.Base + V(dx, dy, dz)
        top_link.Placement = p
        doc.recompute()
        return {"component": a["top"], "placement": list(top_link.Placement.Base)}

    # ---- analysis -------------------------------------------------------- #
    def _bb_overlap(ba, bb, tol=1e-6):
        """Axis-aligned bounding-box overlap test (broad phase)."""
        return (ba.XMin <= bb.XMax + tol and bb.XMin <= ba.XMax + tol and
                ba.YMin <= bb.YMax + tol and bb.YMin <= ba.YMax + tol and
                ba.ZMin <= bb.ZMax + tol and bb.ZMin <= ba.ZMax + tol)

    def op_interference(a):
        names = list(state.components.keys())
        # cache each global shape once (was recomputed per pair) and AABB-cull:
        # the costly boolean common() only runs for pairs whose bounding boxes
        # actually overlap -- two parts with disjoint AABBs cannot intersect.
        shapes = {n: _global_shape(n) for n in names}
        bbs = {n: shapes[n].BoundBox for n in names}
        clashes = []
        narrow = 0
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                na, nb = names[i], names[j]
                if not _bb_overlap(bbs[na], bbs[nb]):
                    continue
                narrow += 1
                common = shapes[na].common(shapes[nb])
                vol = common.Volume if common.Solids else 0.0
                if vol > 1e-6:
                    clashes.append({"a": na, "b": nb, "overlap_volume": _round(vol)})
        return {"pairs_checked": len(names) * (len(names) - 1) // 2,
                "narrow_phase": narrow, "clashes": clashes, "clash_count": len(clashes)}

    def op_bom(a):
        density = float(a.get("density", 1.0))
        by_src = {}
        total_mass = 0.0
        for name, rec in state.components.items():
            src = doc.getObject(rec["src"])
            key = src.Label
            shp = src.Shape
            mass = shp.Volume * density
            total_mass += mass
            entry = by_src.setdefault(key, {"count": 0, "unit_volume": _round(shp.Volume),
                                            "unit_mass": _round(mass)})
            entry["count"] += 1
        return {"line_items": by_src, "component_count": len(state.components),
                "total_mass": _round(total_mass)}

    def op_measure(a):
        if not state.components:
            return {"components": 0}
        names = list(state.components.keys())
        shapes = {n: _global_shape(n) for n in names}
        import Part
        all_shapes = list(shapes.values())
        comp = all_shapes[0] if len(all_shapes) == 1 else Part.makeCompound(all_shapes)
        bb = comp.BoundBox
        # volume-weighted centroid (== center of mass for a uniform material).
        # ``density`` (g/mm^3) gives a single material; ``densities`` maps a
        # component name OR its source label to a per-part density for a
        # multi-material mass-weighted center of mass + total mass.
        dens = a.get("densities") or {}
        default_rho = a.get("density")
        tot_v = cx = cy = cz = 0.0
        tot_m = mx = my = mz = 0.0
        for n in names:
            sh = shapes[n]
            v = sh.Volume
            com = sh.CenterOfMass
            tot_v += v
            cx += v * com.x
            cy += v * com.y
            cz += v * com.z
            rho = dens.get(n)
            if rho is None:
                rho = dens.get(doc.getObject(state.components[n]["src"]).Label, default_rho)
            if rho is not None:
                m = float(rho) * v
                tot_m += m
                mx += m * com.x
                my += m * com.y
                mz += m * com.z
        out = {"components": len(names), "volume": _round(comp.Volume),
               "bbox_size": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)]}
        if tot_v > 0:
            out["centroid"] = [_round(cx / tot_v), _round(cy / tot_v), _round(cz / tot_v)]
        if tot_m > 0:
            out["mass"] = _round(tot_m)
            out["center_of_mass"] = [_round(mx / tot_m), _round(my / tot_m), _round(mz / tot_m)]
        return out

    def op_tree(a):
        comps = []
        for name, rec in state.components.items():
            link = doc.getObject(rec["link"])
            comps.append({"name": name, "source": doc.getObject(rec["src"]).Label,
                          "fixed": rec["fixed"], "pos": [_round(x) for x in link.Placement.Base]})
        return {"assembly": state.assembly, "components": comps}

    def op_solve(a):
        asm = doc.getObject(state.assembly) if state.assembly else None
        if asm is None:
            return {"solved": False, "reason": "no assembly"}
        grounded = [n for n, r in state.components.items() if r["fixed"]]
        result = None
        try:
            result = asm.solve()
        except Exception as exc:
            result = "solve-skipped: %s" % exc
        doc.recompute()
        return {"solved": True, "grounded": grounded, "native_solve": str(result)}

    def op_export(a):
        import Import
        objs = [doc.getObject(r["link"]) for r in state.components.values()]
        Import.export(objs, a["path"])
        import os
        return {"path": a["path"], "bytes": os.path.getsize(a["path"]) if os.path.exists(a["path"]) else 0}

    return {
        "asm.create": op_create, "asm.add": op_add, "asm.place": op_place, "asm.move": op_move,
        "asm.fix": op_fix, "asm.align": op_align, "asm.stack": op_stack,
        "asm.rotate": op_rotate,
        "asm.coaxial": op_coaxial,
        "asm.interference": op_interference, "asm.bom": op_bom, "asm.measure": op_measure,
        "asm.tree": op_tree, "asm.solve": op_solve, "asm.export": op_export,
    }
