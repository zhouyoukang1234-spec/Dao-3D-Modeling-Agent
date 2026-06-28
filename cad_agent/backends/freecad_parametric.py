"""Parametric PartDesign engine (the ``param.*`` tool group).

Runs inside freecadcmd. Unlike the explicit-BREP ``solid.*`` ops, these build a
real, editable PartDesign **feature tree** — Body -> Sketch -> Pad/Pocket/
Revolution/Loft/Pipe -> dressups (Fillet/Chamfer) -> patterns. Sketches are
built fully-constrained and anchored to the body origin, and every dimension is
exposed as a named parameter so the agent can re-edit the design after the fact
(``param.set``) exactly like a human dragging a dimension. ``param.diagnose``
reports sketch constraint health (DoF / conflicts / redundancies) — the
closed-loop self-check.
"""
import math

import FreeCAD as App
import Part
import Sketcher

V = App.Vector
ROT = App.Rotation


def _round(x, n=4):
    return round(float(x), n)


def _gear_points(m, z, alpha_deg=20.0, n=6):
    """Outline points (x, y) of a standard involute spur gear: module ``m``,
    ``z`` teeth, ``alpha_deg`` pressure angle. Builds each tooth from sampled
    involute flanks between the base/root and addendum circles plus tip and root
    arcs, repeated around ``z`` teeth. Returns a closed CCW point loop."""
    alpha = math.radians(alpha_deg)
    rp = m * z / 2.0          # pitch radius
    rb = rp * math.cos(alpha)  # base radius
    ra = rp + m               # addendum (tip) radius
    rf = rp - 1.25 * m        # dedendum (root) radius
    inv = lambda t: math.tan(t) - t  # noqa: E731 (involute function)
    base_off = math.pi / (2 * z) + inv(alpha)  # half tooth angle datum
    rstart = max(rb, rf)

    def ang(r):
        ar = math.acos(max(-1.0, min(1.0, rb / r)))
        return base_off - inv(ar)

    a_root, a_tip = ang(rstart), ang(ra)
    pts = []
    add = lambda r, an: pts.append((r * math.cos(an), r * math.sin(an)))  # noqa: E731
    for i in range(z):
        phi = i * 2 * math.pi / z
        add(rf, phi - math.pi / z)
        if rf < rstart:
            add(rf, phi - a_root)
        for k in range(n + 1):                     # left flank, root -> tip
            r = rstart + (ra - rstart) * k / n
            add(r, phi - ang(r))
        for k in range(1, n):                      # tip arc
            add(ra, phi - a_tip + 2 * a_tip * k / n)
        for k in range(n + 1):                     # right flank, tip -> root
            r = ra - (ra - rstart) * k / n
            add(r, phi + ang(r))
        if rf < rstart:
            add(rf, phi + a_root)
        add(rf, phi + math.pi / z)
    # drop consecutive (and wrap-around) duplicate points -- each mid-gap point
    # is shared by neighbouring teeth and would otherwise yield a zero-length
    # segment that OCC rejects ("Both points are equal").
    dedup = []
    for p in pts:
        if not dedup or math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-7:
            dedup.append(p)
    if len(dedup) > 1 and math.hypot(dedup[0][0] - dedup[-1][0], dedup[0][1] - dedup[-1][1]) <= 1e-7:
        dedup.pop()
    return dedup, {"pitch_r": rp, "base_r": rb, "tip_r": ra, "root_r": rf}


def _axis_vec(d):
    """Map a signed axis label ('+Z'/'-X'/'Y'...) to a unit Vector."""
    s = str(d).strip().upper()
    sign = -1.0 if s.startswith("-") else 1.0
    ax = s.lstrip("+-")
    base = {"X": V(1, 0, 0), "Y": V(0, 1, 0), "Z": V(0, 0, 1)}.get(ax)
    if base is None:
        raise ValueError("bad axis label: %r" % (d,))
    return V(base.x * sign, base.y * sign, base.z * sign)


def _origin_plane(body, plane):
    """Return the body's origin datum plane object for 'XY'/'XZ'/'YZ'."""
    want = {"XY": "XY_Plane", "XZ": "XZ_Plane", "YZ": "YZ_Plane"}[plane.upper()]
    body.Origin  # ensure origin built
    for f in body.Origin.OriginFeatures:
        if f.Name.startswith(want) or want in f.Name or getattr(f, "Role", "") == want:
            return f
    # fallback by label
    for f in body.Origin.OriginFeatures:
        if want[:2] in f.Label:
            return f
    raise RuntimeError("origin plane not found: %s" % plane)


def _origin_axis(body, axis):
    """Return the body's origin datum line object for 'X'/'Y'/'Z'."""
    want = {"X": "X_Axis", "Y": "Y_Axis", "Z": "Z_Axis"}[axis.upper()]
    body.Origin  # ensure origin built
    for f in body.Origin.OriginFeatures:
        if getattr(f, "Role", "") == want or f.Name.startswith(want) or want in f.Name:
            return f
    for f in body.Origin.OriginFeatures:
        if want[0] + "_Axis" in f.Label:
            return f
    raise RuntimeError("origin axis not found: %s" % axis)


def _metrics(shape):
    bb = shape.BoundBox
    out = {"valid": bool(shape.isValid()), "volume": _round(shape.Volume),
           "area": _round(shape.Area), "faces": len(shape.Faces),
           "bbox_size": [_round(bb.XLength), _round(bb.YLength), _round(bb.ZLength)]}
    try:
        com = shape.CenterOfMass
        out["center_of_mass"] = [_round(com.x), _round(com.y), _round(com.z)]
    except Exception:
        pass
    return out


def register(state):
    doc = state.doc
    if not hasattr(state, "features"):
        state.features = {}  # (bodyname, logical_feature) -> real object Name

    def _reg_feature(bodyname, feat, obj):
        # Track the real (possibly suffixed) object name per body so later
        # references by the logical feature name resolve to the right body.
        state.features[(bodyname, feat)] = obj.Name

    def _body(name):
        oname = state.bodies.get(name)
        if not oname or doc.getObject(oname) is None:
            raise KeyError("no such body: %s" % name)
        return doc.getObject(oname)

    def _reg_param(feature, pname, obj, kind, ref, body=None):
        # Register the bare `feature.param` key (back-compat; last writer wins
        # across bodies) and, when the body is known, a fully-qualified
        # `body.feature.param` key so multi-body designs can disambiguate.
        entry = {"obj": obj.Name, "kind": kind, "ref": ref}
        state.params["%s.%s" % (feature, pname)] = entry
        if body:
            state.params["%s.%s.%s" % (body, feature, pname)] = entry

    def _profile_sketch(body, plane, profile, feature, sketch_name, params=True, bodyname=None,
                        offset=0.0):
        """Create a fully-constrained sketch on a body origin plane.

        Anchors geometry to the origin so DoF -> 0 (robust parametrics). When
        ``offset`` is non-zero the sketch plane is lifted that far along its
        normal (via AttachmentOffset) so features can stack at height instead of
        all starting at z=0. Returns the sketch; registers named dims when
        ``params``.
        """
        sk = body.newObject("Sketcher::SketchObject", sketch_name)
        sk.AttachmentSupport = [(_origin_plane(body, plane), "")]
        sk.MapMode = "FlatFace"
        if offset:
            sk.AttachmentOffset = App.Placement(V(0, 0, float(offset)), ROT())
        doc.recompute()

        if "circle" in profile:
            r = float(profile["circle"])
            at = profile.get("at", [0, 0])
            cx, cy = float(at[0]), float(at[1])
            gi = sk.addGeometry(Part.Circle(V(cx, cy, 0), V(0, 0, 1), r), False)
            if cx == 0 and cy == 0:
                sk.addConstraint(Sketcher.Constraint("Coincident", gi, 3, -1, 1))  # center->origin
            else:
                # locate the center off-origin and fully constrain it (DoF -> 0)
                px = sk.addConstraint(Sketcher.Constraint("DistanceX", -1, 1, gi, 3, cx))
                py = sk.addConstraint(Sketcher.Constraint("DistanceY", -1, 1, gi, 3, cy))
                sk.renameConstraint(px, "pos_x")
                sk.renameConstraint(py, "pos_y")
                if params:
                    _reg_param(feature, "pos_x", sk, "datum", "pos_x", bodyname)
                    _reg_param(feature, "pos_y", sk, "datum", "pos_y", bodyname)
            c = sk.addConstraint(Sketcher.Constraint("Radius", gi, r))
            sk.renameConstraint(c, "radius")
            if params:
                _reg_param(feature, "radius", sk, "datum", "radius", bodyname)

        elif "rect" in profile:
            w, h = [float(v) for v in profile["rect"]]
            x0, y0 = -w / 2.0, -h / 2.0
            pts = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]
            g = [sk.addGeometry(Part.LineSegment(V(*pts[i], 0), V(*pts[(i + 1) % 4], 0)), False)
                 for i in range(4)]
            for i in range(4):
                sk.addConstraint(Sketcher.Constraint("Coincident", g[i], 2, g[(i + 1) % 4], 1))
            sk.addConstraint(Sketcher.Constraint("Horizontal", g[0]))
            sk.addConstraint(Sketcher.Constraint("Horizontal", g[2]))
            sk.addConstraint(Sketcher.Constraint("Vertical", g[1]))
            sk.addConstraint(Sketcher.Constraint("Vertical", g[3]))
            # symmetry of two opposite corners about origin -> centered
            sk.addConstraint(Sketcher.Constraint("Symmetric", g[0], 1, g[2], 1, -1, 1))
            cw = sk.addConstraint(Sketcher.Constraint("DistanceX", g[0], 1, g[0], 2, w))
            ch = sk.addConstraint(Sketcher.Constraint("DistanceY", g[1], 1, g[1], 2, h))
            sk.renameConstraint(cw, "width")
            sk.renameConstraint(ch, "height")
            if params:
                _reg_param(feature, "width", sk, "datum", "width", bodyname)
                _reg_param(feature, "height", sk, "datum", "height", bodyname)

        elif "grid" in profile:
            # a rectangular nx-by-ny array of equal circles in ONE sketch.
            # PartDesign LinearPattern is single-direction and nesting two
            # patterns / MultiTransform is unreliable headless, so a fully
            # constrained sketch grid is the robust way to a hole array.
            gd = profile["grid"]
            r = float(gd["circle"])
            nx, ny = int(gd.get("nx", 2)), int(gd.get("ny", 2))
            dx, dy = float(gd.get("dx", 10)), float(gd.get("dy", 10))
            at = gd.get("at", [-(nx - 1) * dx / 2.0, -(ny - 1) * dy / 2.0])
            x0, y0 = float(at[0]), float(at[1])
            for j in range(ny):
                for i in range(nx):
                    cx, cy = x0 + i * dx, y0 + j * dy
                    gi = sk.addGeometry(Part.Circle(V(cx, cy, 0), V(0, 0, 1), r), False)
                    sk.addConstraint(Sketcher.Constraint("DistanceX", -1, 1, gi, 3, cx))
                    sk.addConstraint(Sketcher.Constraint("DistanceY", -1, 1, gi, 3, cy))
                    sk.addConstraint(Sketcher.Constraint("Radius", gi, r))

        elif "polygon" in profile:
            pts = [V(float(p[0]), float(p[1]), 0) for p in profile["polygon"]]
            n = len(pts)
            g = [sk.addGeometry(Part.LineSegment(pts[i], pts[(i + 1) % n]), False) for i in range(n)]
            for i in range(n):
                sk.addConstraint(Sketcher.Constraint("Coincident", g[i], 2, g[(i + 1) % n], 1))
            # freeform polygon: intentionally left under-constrained (diagnose reports honestly)

        elif "gear" in profile:
            # involute spur-gear outline as a closed wire of line segments.
            # built from the parametric tooth geometry (module/teeth/pressure
            # angle) rather than hand digitized points. ``internal: true`` yields
            # a ring (internal) gear: an outer rim circle with an inward-toothed
            # bore, padded into an annulus.
            gp = profile["gear"]
            m_ = float(gp["module"])
            z_ = int(gp["teeth"])
            raw, _info = _gear_points(m_, z_, float(gp.get("pressure_angle", 20.0)),
                                      int(gp.get("samples", 6)))
            # optional rigid rotation of the whole outline about the gear axis
            # (degrees). Used to build helical gears: lofting cross-sections
            # rotated by an increasing angle produces a twisted (helical) tooth.
            rot_deg = float(gp.get("rotate", 0.0))
            if rot_deg:
                ca, sa = math.cos(math.radians(rot_deg)), math.sin(math.radians(rot_deg))
                raw = [(x * ca - y * sa, x * sa + y * ca) for x, y in raw]
            # A real gear has hundreds of flank samples (z*~23 points). Adding a
            # per-vertex Coincident constraint makes the Sketcher solver cost
            # super-linear and a z>=36 gear blows past the RPC timeout. The
            # consecutive segments already share *exact* endpoint coordinates,
            # so the wire closes geometrically and Pad consumes it fine without
            # any constraints (same as the grid profile). Keep it constraint-free.
            if gp.get("internal"):
                # Reflect each flank point radially about the pitch circle
                # (r -> 2*rp - r): an external tooth (tip outward) becomes an
                # internal tooth (tip inward), i.e. the toothed bore of a ring
                # gear. An outer rim circle closes the annulus so Pad makes a
                # ring with a geared hole. This is the standard quick internal-
                # gear approximation; meshing is validated by interference.
                rp_ = _info["pitch_r"]
                inner = []
                for x, y in raw:
                    r = math.hypot(x, y)
                    if r < 1e-9:
                        continue
                    rr = (2.0 * rp_ - r) / r
                    inner.append(V(x * rr, y * rr, 0))
                rim = rp_ + 1.25 * m_ + float(gp.get("rim", 3.0))
                sk.addGeometry(Part.Circle(V(0, 0, 0), V(0, 0, 1), rim), False)
                n = len(inner)
                for i in range(n):
                    sk.addGeometry(Part.LineSegment(inner[i], inner[(i + 1) % n]), False)
            else:
                pts = [V(x, y, 0) for x, y in raw]
                n = len(pts)
                for i in range(n):
                    sk.addGeometry(Part.LineSegment(pts[i], pts[(i + 1) % n]), False)

        elif "rack" in profile:
            # involute rack: a straight gear (infinite pitch radius). Standard
            # rack teeth are trapezoids with straight flanks inclined at the
            # pressure angle. Pitch line at y=0; teeth up (addendum m), roots
            # down (dedendum 1.25 m); a solid base below closes the strip.
            rp_ = profile["rack"]
            m_ = float(rp_["module"])
            n_ = int(rp_["teeth"])
            pa = math.radians(float(rp_.get("pressure_angle", 20.0)))
            p_ = math.pi * m_                       # circular pitch
            ha, hf = m_, 1.25 * m_
            back = float(rp_.get("back", 2.0 * m_))
            tip_h = p_ / 4.0 - ha * math.tan(pa)    # half tip thickness
            root_h = p_ / 4.0 + hf * math.tan(pa)   # half thickness at root line
            top = []
            for k in range(n_):
                cx = k * p_
                top.append((cx - root_h, -hf))
                top.append((cx - tip_h, ha))
                top.append((cx + tip_h, ha))
                top.append((cx + root_h, -hf))
            x_min = top[0][0]
            x_max = top[-1][0]
            y_base = -hf - back
            pts = [V(x, y, 0) for x, y in top]
            pts.append(V(x_max, y_base, 0))
            pts.append(V(x_min, y_base, 0))
            n = len(pts)
            for i in range(n):
                sk.addGeometry(Part.LineSegment(pts[i], pts[(i + 1) % n]), False)

        else:
            raise ValueError("unknown profile: %r" % (profile,))

        doc.recompute()
        return sk

    # ---- body / sketch ---------------------------------------------------- #
    def op_body(a):
        name = a["name"]
        b = doc.addObject("PartDesign::Body", name)
        state.bodies[name] = b.Name
        doc.recompute()
        return {"body": name}

    def op_sketch(a):
        body = _body(a["body"])
        feat = a.get("feature", a.get("name", "Sketch"))
        sk = _profile_sketch(body, a.get("plane", "XY"), a["profile"], feat,
                             a.get("name", feat + "_sk"), bodyname=a["body"],
                             offset=float(a.get("offset", 0)))
        return {"sketch": sk.Name, "dof": sk.DoF, "fully_constrained": bool(sk.FullyConstrained)}

    # ---- additive / subtractive features --------------------------------- #
    def _feature(a, kind):
        body = _body(a["body"])
        feat = a.get("feature", kind.title())
        sk = _profile_sketch(body, a.get("plane", "XY"), a["profile"], feat, feat + "_sk",
                             bodyname=a["body"], offset=float(a.get("offset", 0)))
        sk.Visibility = False
        f = body.newObject("PartDesign::%s" % kind, feat)
        _reg_feature(a["body"], feat, f)
        f.Profile = sk
        if kind in ("Pad", "Pocket"):
            f.Length = float(a.get("length", 10))
            if a.get("through"):
                f.Type = "ThroughAll"
            if a.get("midplane"):
                f.Midplane = True
            # A Pad from XY grows +Z into free space; a Pocket from that same
            # plane defaults to -Z (away from a +Z body) and removes nothing.
            # Default pockets to cut toward the material unless told otherwise.
            reversed_default = (kind == "Pocket")
            f.Reversed = bool(a.get("reversed", reversed_default))
            _reg_param(feat, "length", f, "prop", "Length", a["body"])
        elif kind in ("Revolution", "Groove"):
            f.Angle = float(a.get("angle", 360))
            # revolve about sketch vertical axis by default
            f.ReferenceAxis = (sk, ["V_Axis"])
            _reg_param(feat, "angle", f, "prop", "Angle", a["body"])
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("%s produced invalid shape" % kind)
        return {"feature": feat, **_metrics(body.Tip.Shape), "dof": sk.DoF}

    def op_pad(a):
        return _feature(a, "Pad")

    def op_pocket(a):
        return _feature(a, "Pocket")

    def op_revolve(a):
        return _feature(a, "Revolution")

    def op_groove(a):
        # subtractive revolve: cuts a revolved profile (e.g. an O-ring groove)
        return _feature(a, "Groove")

    def op_loft(a):
        body = _body(a["body"])
        feat = a.get("feature", "Loft")
        sketches = []
        for i, sec in enumerate(a["sections"]):
            sk = _profile_sketch(body, sec.get("plane", "XY"), sec["profile"],
                                 "%s_s%d" % (feat, i), "%s_s%d_sk" % (feat, i),
                                 params=(i == 0), bodyname=a["body"])
            # offset section along plane normal. With MapMode=FlatFace the
            # sketch Placement is driven by the attachment and recompute would
            # overwrite a manual Placement, so move it via AttachmentOffset
            # (expressed in the plane's local frame, +Z = plane normal).
            off = float(sec.get("offset", 0))
            if off:
                sk.AttachmentOffset = App.Placement(V(0, 0, off), ROT())
            sk.Visibility = False
            sketches.append(sk)
        doc.recompute()
        f = body.newObject("PartDesign::AdditiveLoft", feat)
        _reg_feature(a["body"], feat, f)
        f.Profile = sketches[0]
        f.Sections = sketches[1:]
        if a.get("ruled"):
            f.Ruled = True
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("loft produced invalid shape")
        return {"feature": feat, **_metrics(body.Tip.Shape)}

    def op_helical(a):
        """Helical involute gear: loft involute cross-sections that rotate by an
        increasing twist along the axis. The pitch-radius helix angle ``beta``
        (deg) sets the total tooth twist over the face width
        ``twist = W*tan(beta)/rp``; ``hand`` ("right"/"left") sets its sign.
        Falls back to a spur gear when ``beta`` is 0."""
        m_ = float(a["module"])
        z_ = int(a["teeth"])
        w_ = float(a.get("length", 10))
        beta = float(a.get("helix_angle", 0.0))
        nsec = max(2, int(a.get("sections", 5)))
        pa = float(a.get("pressure_angle", 20.0))
        rp = m_ * z_ / 2.0
        twist = math.degrees(w_ * math.tan(math.radians(beta)) / rp) if beta else 0.0
        if str(a.get("hand", "right")).lower().startswith("l"):
            twist = -twist
        sections = []
        for k in range(nsec):
            frac = k / float(nsec - 1)
            sections.append({"profile": {"gear": {"module": m_, "teeth": z_,
                                                  "pressure_angle": pa,
                                                  "rotate": twist * frac}},
                             "offset": w_ * frac})
        res = op_loft({"body": a["body"], "feature": a.get("feature", "Helical"),
                       "sections": sections})
        res["helix_angle"] = beta
        res["twist_deg"] = twist
        return res

    def op_bevel(a):
        """Bevel involute gear (intersecting axes): a tapered gear whose teeth
        shrink toward the pitch-cone apex. Lofts involute sections whose module
        scales with cone distance ``cd/Ro`` and which step up the axis by the
        axial height. ``pitch_cone_angle`` (deg, default 45 => a miter gear) and
        ``face_width`` set the cone. Back pitch radius R=m*z/2, cone distance
        Ro=R/sin(gamma), axial height H=face_width*cos(gamma)."""
        m_ = float(a["module"])
        z_ = int(a["teeth"])
        gamma = math.radians(float(a.get("pitch_cone_angle", 45.0)))
        b_ = float(a.get("face_width", 10.0))
        nsec = max(2, int(a.get("sections", 6)))
        pa = float(a.get("pressure_angle", 20.0))
        r_ = m_ * z_ / 2.0
        ro = r_ / math.sin(gamma)
        h_ = b_ * math.cos(gamma)
        sections = []
        for k in range(nsec):
            frac = k / float(nsec - 1)
            scale = (ro - frac * b_) / ro
            sections.append({"profile": {"gear": {"module": m_ * scale, "teeth": z_,
                                                  "pressure_angle": pa}},
                             "offset": frac * h_})
        res = op_loft({"body": a["body"], "feature": a.get("feature", "Bevel"),
                       "sections": sections})
        res.update({"pitch_radius": r_, "cone_distance": ro, "axial_height": h_,
                    "pitch_cone_angle": math.degrees(gamma)})
        return res

    def op_sweep(a):
        body = _body(a["body"])
        feat = a.get("feature", "Pipe")
        pathspec = a["path"]

        if "helix" in pathspec:
            # helical sweep (coil spring / thread). The helix starts at
            # (R,0,0) with tangent ~ +Y, so the profile must sit on the XZ
            # plane centered at the start radius for a perpendicular sweep.
            hx = pathspec["helix"]
            radius = float(hx["radius"])
            # lift the helix base so threads sit on a shank. NOTE for cut=True:
            # the helix must overrun BOTH ends of the target solid (start/end on
            # a free FLAT end face, not mid-surface) or OCC yields an invalid
            # sliver -- so pass z below the base and height past the top. The
            # tool exiting the end face leaves a near-zero-volume helical lip, so
            # the cut solid's volume/turns are exact but its bbox may be slightly
            # inflated at the run-out (matches how a real thread runs off an end).
            z0 = float(hx.get("z", 0))
            helix = doc.addObject("Part::Helix", feat + "_helix")
            helix.Pitch = float(hx["pitch"])
            helix.Height = float(hx["height"])
            helix.Radius = radius
            helix.Angle = float(hx.get("angle", 0))  # 0 = cylindrical, >0 = conical
            if z0:
                helix.Placement = App.Placement(V(0, 0, z0), ROT())
            helix.Visibility = False  # helper spine: keep out of bbox/renders
            doc.recompute()
            prof_profile = dict(a["profile"])
            if "circle" in prof_profile:
                prof_profile.setdefault("at", [radius, z0])
            prof = _profile_sketch(body, "XZ", prof_profile, feat, feat + "_prof",
                                   bodyname=a["body"])
            prof.Visibility = False
            # additive coil by default; cut=True turns it into a thread groove
            kind = "SubtractivePipe" if a.get("cut") else "AdditivePipe"
            f = body.newObject("PartDesign::%s" % kind, feat)
            _reg_feature(a["body"], feat, f)
            f.Profile = prof
            f.Spine = (helix, [])
            doc.recompute()
            if f.Shape.isNull() or not f.Shape.isValid():
                raise RuntimeError("helical sweep produced invalid shape")
            return {"feature": feat,
                    "turns": round(float(helix.Height) / float(helix.Pitch), 3),
                    **_metrics(body.Tip.Shape)}

        prof = _profile_sketch(body, a.get("plane", "XY"), a["profile"], feat, feat + "_prof",
                               bodyname=a["body"])
        prof.Visibility = False
        # path sketch: a polyline in the given plane
        psk = body.newObject("Sketcher::SketchObject", feat + "_path")
        psk.AttachmentSupport = [(_origin_plane(body, pathspec.get("plane", "XZ")), "")]
        psk.MapMode = "FlatFace"
        doc.recompute()
        pts = [V(float(p[0]), float(p[1]), 0) for p in pathspec["points"]]
        for i in range(len(pts) - 1):
            gi = psk.addGeometry(Part.LineSegment(pts[i], pts[i + 1]), False)
            if i > 0:
                psk.addConstraint(Sketcher.Constraint("Coincident", gi - 1, 2, gi, 1))
        psk.Visibility = False
        doc.recompute()
        f = body.newObject("PartDesign::AdditivePipe", feat)
        _reg_feature(a["body"], feat, f)
        f.Profile = prof
        f.Spine = (psk, ["Edge%d" % i for i in range(1, len(pts))])
        # sharp 90-degree polyline corners make a plain sweep degenerate on the
        # 2nd+ leg; rounding the transition keeps the full swept volume valid.
        if "Transition" in f.PropertiesList:
            try:
                f.Transition = "Round corner"
            except Exception:
                pass
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("sweep produced invalid shape")
        return {"feature": feat, **_metrics(body.Tip.Shape)}

    # ---- dressups --------------------------------------------------------- #
    def _dressup(a, kind):
        body = _body(a["body"])
        feat = a.get("feature", kind)
        tip = body.Tip
        edges = a.get("edges")
        if edges is None:
            refs = ["Edge%d" % (i + 1) for i in range(len(tip.Shape.Edges))]
        else:
            refs = ["Edge%d" % (i + 1) for i in edges]
        f = body.newObject("PartDesign::%s" % kind, feat)
        _reg_feature(a["body"], feat, f)
        f.Base = (tip, refs)
        if kind == "Fillet":
            f.Radius = float(a["radius"])
            _reg_param(feat, "radius", f, "prop", "Radius", a["body"])
        else:
            f.Size = float(a["size"])
            _reg_param(feat, "size", f, "prop", "Size", a["body"])
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("%s produced invalid shape" % kind)
        return {"feature": feat, **_metrics(body.Tip.Shape)}

    def op_fillet(a):
        return _dressup(a, "Fillet")

    def op_chamfer(a):
        return _dressup(a, "Chamfer")

    def op_shell(a):
        """Hollow the body to a thin wall (PartDesign Thickness). ``open`` names
        the face direction(s) to remove so the result is an open shell (default
        +Z, the top); pass explicit ``faces`` indices to override. ``thickness``
        is the wall (inward by default; ``outward=True`` grows it instead)."""
        body = _body(a["body"])
        feat = a.get("feature", "Shell")
        tip = body.Tip
        refs = a.get("faces")
        if refs is None:
            dirs = a.get("open", "+Z")
            dirs = [dirs] if isinstance(dirs, str) else dirs
            wants = [_axis_vec(d) for d in dirs]
            sel = []
            for i, fc in enumerate(tip.Shape.Faces):
                if fc.Surface.__class__.__name__ != "Plane":
                    continue
                nrm = fc.normalAt(0, 0)
                if any(abs(nrm.dot(w) - 1) < 1e-3 for w in wants):
                    sel.append(i)
            if not sel:
                raise ValueError("no planar face found opening %r" % (dirs,))
            refs = ["Face%d" % (i + 1) for i in sel]
        else:
            refs = ["Face%d" % (i + 1) for i in refs]
        f = body.newObject("PartDesign::Thickness", feat)
        _reg_feature(a["body"], feat, f)
        f.Base = (tip, refs)
        f.Value = float(a["thickness"])
        f.Reversed = not bool(a.get("outward", False))  # default: wall grows inward
        f.Mode = 0   # Skin
        f.Join = 0   # Arc
        doc.recompute()
        _reg_param(feat, "value", f, "prop", "Value", a["body"])
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("shell produced invalid shape")
        return {"feature": feat, "opened_faces": refs, **_metrics(body.Tip.Shape)}

    # ---- transformed feature patterns ------------------------------------ #
    def _originals(body, a):
        """Resolve the feature(s) to replicate; default to the body tip."""
        names = a.get("originals")
        if not names:
            return [body.Tip]
        bodyname = a["body"]
        objs = []
        for n in names:
            # resolve within THIS body first: feature names collide across
            # bodies (FreeCAD suffixes duplicates, e.g. Hole/Hole001), so a
            # bare doc.getObject(n) can grab another body's feature. The
            # per-body feature registry maps the logical name to the real one.
            real = state.features.get((bodyname, n))
            o = doc.getObject(real) if real else None
            if o is None:
                o = next((x for x in body.Group if x.Name == n or x.Label == n), None)
            if o is None:
                o = doc.getObject(n)  # last resort: global lookup
            if o is None:
                raise KeyError("no such feature to pattern: %s" % n)
            # PartDesign rejects another transform (mirror/pattern) as an
            # original -- it silently yields an invalid shape. Fail clearly and
            # point at the robust route (a grid profile builds a 2D array in one
            # sketch; chaining transforms for full rectangular symmetry is not
            # supported by the kernel).
            if o.TypeId.startswith("PartDesign::") and o.TypeId.split("::")[1] in (
                    "Mirrored", "LinearPattern", "PolarPattern", "MultiTransform"):
                raise ValueError(
                    "cannot use transform feature %r as a pattern/mirror original; "
                    "PartDesign does not support chaining transforms -- use a grid "
                    "profile for rectangular arrays instead" % n)
            objs.append(o)
        return objs

    def op_pattern_polar(a):
        body = _body(a["body"])
        feat = a.get("feature", "PolarPattern")
        originals = _originals(body, a)
        f = body.newObject("PartDesign::PolarPattern", feat)
        _reg_feature(a["body"], feat, f)
        f.Originals = originals
        f.Axis = (_origin_axis(body, a.get("axis", "Z")), [""])
        # angle=360 (default) is a true full circle: FreeCAD lays out exactly
        # `count` copies at 360/count spacing (the original at 0 included, no
        # last==first overlap). Don't override BaseFeature; instead advance the
        # body Tip so the pattern joins the solid chain (else it stays inert).
        f.Angle = float(a.get("angle", 360))
        f.Occurrences = int(a.get("count", 4))
        for o in originals:
            o.Visibility = False
        body.Tip = f
        _reg_param(feat, "occurrences", f, "prop", "Occurrences", a["body"])
        _reg_param(feat, "angle", f, "prop", "Angle", a["body"])
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("polar pattern produced invalid shape")
        return {"feature": feat, "occurrences": int(f.Occurrences), **_metrics(body.Tip.Shape)}

    def op_pattern_linear(a):
        body = _body(a["body"])
        feat = a.get("feature", "LinearPattern")
        originals = _originals(body, a)
        f = body.newObject("PartDesign::LinearPattern", feat)
        _reg_feature(a["body"], feat, f)
        f.Originals = originals
        f.Direction = (_origin_axis(body, a.get("axis", "X")), [""])
        f.Reversed = bool(a.get("reversed", False))
        # Length is the total span across all `count` copies (endpoints inclusive).
        f.Length = float(a.get("length", 50))
        f.Occurrences = int(a.get("count", 3))
        for o in originals:
            o.Visibility = False
        body.Tip = f
        _reg_param(feat, "occurrences", f, "prop", "Occurrences", a["body"])
        _reg_param(feat, "length", f, "prop", "Length", a["body"])
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("linear pattern produced invalid shape")
        return {"feature": feat, "occurrences": int(f.Occurrences), **_metrics(body.Tip.Shape)}

    def op_pattern_mirror(a):
        body = _body(a["body"])
        feat = a.get("feature", "Mirrored")
        originals = _originals(body, a)
        f = body.newObject("PartDesign::Mirrored", feat)
        _reg_feature(a["body"], feat, f)
        f.Originals = originals
        # mirror across an origin datum plane (default XY); like the other
        # transforms the body Tip must advance or the feature stays inert.
        f.MirrorPlane = (_origin_plane(body, a.get("plane", "XY")), [""])
        for o in originals:
            o.Visibility = False
        body.Tip = f
        doc.recompute()
        if f.Shape.isNull() or not f.Shape.isValid():
            raise RuntimeError("mirror pattern produced invalid shape")
        return {"feature": feat, **_metrics(body.Tip.Shape)}

    # ---- parameters & re-edit -------------------------------------------- #
    def op_params(a):
        out = {}
        for k, v in state.params.items():
            obj = doc.getObject(v["obj"])
            if obj is None:
                continue
            if v["kind"] == "prop":
                val = getattr(obj, v["ref"])
                out[k] = float(getattr(val, "Value", val))
            else:
                # datum: read current constraint value
                try:
                    out[k] = float(obj.getDatum(v["ref"]))
                except Exception:
                    out[k] = None
        return {"params": out}

    def op_set(a):
        key = a["param"]
        if key not in state.params:
            raise KeyError("no such param: %s (have %s)" % (key, list(state.params)))
        v = state.params[key]
        obj = doc.getObject(v["obj"])
        value = float(a["value"])
        if v["kind"] == "prop":
            # integer properties (e.g. pattern Occurrences) reject floats
            cur = getattr(obj, v["ref"])
            if isinstance(cur, int) and not isinstance(cur, bool):
                setattr(obj, v["ref"], int(round(value)))
            else:
                setattr(obj, v["ref"], value)
        else:
            obj.setDatum(v["ref"], value)
        doc.recompute()
        # report tip metrics of the owning body if discoverable
        return {"param": key, "value": value, "recomputed": True}

    # ---- diagnostics & tree ---------------------------------------------- #
    def op_diagnose(a):
        # optionally scope to a single body -- otherwise every sketch in the
        # document (e.g. other bodies' profiles) is folded into all_healthy,
        # which made a per-body check report unrelated bodies' DoF.
        scope = None
        if a.get("body"):
            scope = _body(a["body"])
        sketches = []
        total_dof = 0
        all_ok = True
        for o in doc.Objects:
            if o.TypeId == "Sketcher::SketchObject":
                if scope is not None and o.getParentGeoFeatureGroup() is not scope:
                    continue
                dof = int(o.DoF)
                total_dof += dof
                conflict = list(o.ConflictingConstraints)
                redun = list(o.RedundantConstraints)
                malformed = list(getattr(o, "MalformedConstraints", []))
                healthy = (dof == 0 and not conflict and not redun and not malformed)
                all_ok = all_ok and healthy
                sketches.append({
                    "sketch": o.Name, "dof": dof,
                    "fully_constrained": bool(o.FullyConstrained),
                    "conflicting": conflict, "redundant": redun, "malformed": malformed,
                    "healthy": healthy})
        return {"sketches": sketches, "total_dof": total_dof, "all_healthy": all_ok}

    def op_tree(a):
        body = _body(a["body"])
        feats = []
        for o in body.Group:
            feats.append({"name": o.Name, "type": o.TypeId.split("::")[-1]})
        return {"body": a["body"], "tip": body.Tip.Name if body.Tip else None, "features": feats}

    def op_measure(a):
        body = _body(a["body"])
        return _metrics(body.Tip.Shape)

    return {
        "param.body": op_body, "param.sketch": op_sketch,
        "param.pad": op_pad, "param.pocket": op_pocket, "param.revolve": op_revolve,
        "param.groove": op_groove,
        "param.loft": op_loft, "param.sweep": op_sweep,
        "param.helical": op_helical, "param.bevel": op_bevel,
        "param.fillet": op_fillet, "param.chamfer": op_chamfer, "param.shell": op_shell,
        "param.pattern_polar": op_pattern_polar, "param.pattern_linear": op_pattern_linear,
        "param.mirror": op_pattern_mirror,
        "param.params": op_params, "param.set": op_set,
        "param.diagnose": op_diagnose, "param.tree": op_tree, "param.measure": op_measure,
    }
