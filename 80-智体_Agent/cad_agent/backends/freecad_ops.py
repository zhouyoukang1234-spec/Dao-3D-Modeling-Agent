#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
freecad_ops.py — FreeCAD 纯几何本源 (运行于 FreeCAD 自带 python 内)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 万法归一 — 把 "用 Part 内核造形" 的纯几何抽出为无副作用模块,
让两条路同源复用, 一字不二:

    · 无头子进程内核 freecad_kernel.py  (经 stdin/stdout 行式 RPC, 形状以 BREP 串往返)
    · GUI 在世后端     freecad_live.py    (在 App.ActiveDocument 活文档上就地造形)

本模块只依赖 FreeCAD/Part, 不依赖 cad_agent 任何模块, 亦无任何 I/O 或全局状态:
    build(op, args, shapes) → Part.Shape         纯几何 (输入形状以 Part.Shape 传入)
    pack(shape, deflection) → {brep, mesh, metrics}
    measure / export                              度量 / 落盘
    op(op_name, args)                             内核门面 (BREP 串 ⇄ 形状, 供子进程用)
"""
import math

import FreeCAD as App  # noqa: E402  (仅 FreeCAD python 内可用)
import Part  # noqa: E402


# ── BREP 字符串 ⇄ Part.Shape ───────────────────────────────────────────────
def load(brep):
    s = Part.Shape()
    s.importBrepFromString(brep)
    return s


def dump(shape):
    return shape.exportBrepToString()


def vec(a):
    return App.Vector(float(a[0]), float(a[1]), float(a[2]))


def _section_wire(sec):
    """把一个 loft 截面描述转成位于 z=sec['z'] 平面上的【闭合 Wire】.
    支持 circle(r,segments) / rect(w,h) / points([[x,y],...]); center 平移 (默认原点)."""
    z = float(sec.get("z", 0.0))
    cx, cy = (sec.get("center", [0.0, 0.0]) + [0.0, 0.0])[:2]
    if "circle" in sec:
        r = float(sec["circle"])
        n = int(sec.get("segments", 48))
        pts2 = [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    elif "rect" in sec:
        w, h = float(sec["rect"][0]) / 2.0, float(sec["rect"][1]) / 2.0
        pts2 = [(-w, -h), (w, -h), (w, h), (-w, h)]
    elif "points" in sec:
        pts2 = [(float(p[0]), float(p[1])) for p in sec["points"]]
    else:
        raise ValueError("loft 截面须含 circle/rect/points 之一")
    vs = [vec([cx + x, cy + y, z]) for (x, y) in pts2]
    vs.append(vs[0])
    return Part.makePolygon(vs)


def _path_wire(points, bend_radius=0.0):
    """把转折控制点列构造为扫掠路径 Wire (直线段 + 拐角相切圆弧). 返回 (wire, 起点切向).
    bend_radius<=0 或仅 2 点: 退化为折线 (尖角); >0: 各内拐角以该半径切弧圆滑成 G1 路径.
    切弧法: 拐角 B 处, 沿两腿各退让 setback=R/tan(φ/2) 得切点 T1/T2 (φ=内夹角),
    弧心在角平分线上距 B 为 R/sin(φ/2). 此乃真实管路布线, 避免 BSpline 插值过冲自交."""
    pts = [vec(p) for p in points]
    if len(pts) < 2:
        raise ValueError("sweep 路径至少 2 点")
    t0 = pts[1].sub(pts[0])
    R = float(bend_radius or 0.0)
    if R <= 0 or len(pts) == 2:
        return Part.Wire([Part.makeLine(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]), t0
    edges = []
    cur = pts[0]
    for i in range(1, len(pts) - 1):
        A, B, C = pts[i - 1], pts[i], pts[i + 1]
        uA = A.sub(B); uC = C.sub(B)
        la, lc = uA.Length, uC.Length
        if la < 1e-9 or lc < 1e-9:
            continue
        uA = App.Vector(uA.x / la, uA.y / la, uA.z / la)
        uC = App.Vector(uC.x / lc, uC.y / lc, uC.z / lc)
        cosphi = max(-1.0, min(1.0, uA.dot(uC)))
        phi = math.acos(cosphi)
        if phi < 1e-6 or phi > math.pi - 1e-6:
            continue  # 共线: 不需切弧
        setback = min(R / math.tan(phi / 2.0), la * 0.999, lc * 0.999)
        T1 = B.add(uA.multiply(setback))
        T2 = B.add(uC.multiply(setback))
        bis = uA.add(uC)
        bl = bis.Length or 1.0
        bis = App.Vector(bis.x / bl, bis.y / bl, bis.z / bl)
        center = B.add(bis.multiply(R / math.sin(phi / 2.0)))
        toB = B.sub(center)
        tl = toB.Length or 1.0
        arc_mid = center.add(App.Vector(toB.x / tl, toB.y / tl, toB.z / tl).multiply(R))
        if cur.distanceToPoint(T1) > 1e-7:           # 相邻切弧相接时直段长 0, 跳过退化线
            edges.append(Part.makeLine(cur, T1))
        edges.append(Part.Arc(T1, arc_mid, T2).toShape())
        cur = T2
    if cur.distanceToPoint(pts[-1]) > 1e-7:
        edges.append(Part.makeLine(cur, pts[-1]))
    return Part.Wire(edges), t0


def solidify(shape):
    """把布尔/特征结果归一为实体: 单实体取之, 多实体合为 Compound, 否则原样."""
    try:
        solids = shape.Solids
    except Exception:
        return shape
    if len(solids) == 1:
        return solids[0]
    if len(solids) > 1:
        return Part.makeCompound(solids)
    return shape


def _tight_bbox(shape):
    """轴对齐【紧】包围盒. shape.BoundBox 对 BSpline 面 (放样/布尔后) 取自控制极点,
    会松弛外扩 (实测放样挖孔后 z 多出 ±2mm); optimalBoundingBox(useTriangulation=True)
    据三角网算得紧致真实范围. 不可用时回退 BoundBox."""
    try:
        return shape.optimalBoundingBox(True)
    except Exception:
        return shape.BoundBox


def metrics(shape):
    bb = _tight_bbox(shape)
    closed = bool(shape.isClosed())
    try:
        com = shape.CenterOfMass
    except Exception:
        com = App.Vector((bb.XMin + bb.XMax) / 2.0,
                         (bb.YMin + bb.YMax) / 2.0,
                         (bb.ZMin + bb.ZMax) / 2.0)
    return {
        "volume": round(float(shape.Volume), 6),
        "area": round(float(shape.Area), 6),
        "closed": closed,
        "solids": len(shape.Solids),
        "faces": len(shape.Faces),
        "edges": len(shape.Edges),
        "bbox_min": [round(bb.XMin, 6), round(bb.YMin, 6), round(bb.ZMin, 6)],
        "bbox_max": [round(bb.XMax, 6), round(bb.YMax, 6), round(bb.ZMax, 6)],
        "extents": [round(bb.XLength, 6), round(bb.YLength, 6), round(bb.ZLength, 6)],
        "centroid": [round(com.x, 6), round(com.y, 6), round(com.z, 6)],
    }


def tess(shape, deflection):
    verts, facets = shape.tessellate(float(deflection))
    V = [[round(v.x, 6), round(v.y, 6), round(v.z, 6)] for v in verts]
    F = [[int(a), int(b), int(c)] for (a, b, c) in facets]
    return {"vertices": V, "faces": F}


def pack(shape, deflection=0.4):
    """把 Part.Shape 打包为引擎无关三元组: 权威 BREP 串 + 网格镜像 + 精确度量."""
    return {"brep": dump(shape), "mesh": tess(shape, deflection), "metrics": metrics(shape)}


# ── 纯几何造形: 输入形状以 Part.Shape (而非 BREP 串) 传入, 无副作用 ──────────
def build(op, a, shapes=None):
    """纯几何动作 → 返回新的 Part.Shape. shapes: {名: Part.Shape} 已加载输入."""
    shapes = shapes or {}

    if op == "box":
        s = Part.makeBox(float(a["x"]), float(a["y"]), float(a["z"]))
        if a.get("center"):
            c = a["center"]
            s.translate(vec([c[0] - a["x"] / 2.0, c[1] - a["y"] / 2.0, c[2] - a["z"] / 2.0]))
        return s

    if op == "cylinder":
        s = Part.makeCylinder(float(a["radius"]), float(a["height"]))
        if a.get("center"):
            c = a["center"]
            s.translate(vec([c[0], c[1], c[2] - a["height"] / 2.0]))
        return s

    if op == "sphere":
        s = Part.makeSphere(float(a["radius"]))
        if a.get("center"):
            s.translate(vec(a["center"]))
        return s

    if op == "cone":
        s = Part.makeCone(float(a["radius1"]), float(a["radius2"]), float(a["height"]))
        if a.get("center"):
            c = a["center"]
            s.translate(vec([c[0], c[1], c[2] - a["height"] / 2.0]))
        return s

    if op == "torus":
        s = Part.makeTorus(float(a["radius1"]), float(a["radius2"]))
        if a.get("center"):
            s.translate(vec(a["center"]))
        return s

    if op == "extrude":
        # 任意 2D 闭合多边形轮廓 (XY 平面, 点列 [[x,y],...]) 沿 +Z 拉伸 height 成棱柱.
        # center 给定时按"棱柱包围盒中心居于 center"平移 (与 box 同义).
        pts = a["points"]
        if len(pts) < 3:
            raise ValueError("extrude 轮廓至少 3 点")
        h = float(a["height"])
        vs = [vec([float(p[0]), float(p[1]), 0.0]) for p in pts]
        vs.append(vs[0])  # 闭合
        face = Part.Face(Part.makePolygon(vs))
        s = solidify(face.extrude(App.Vector(0, 0, h)))
        if a.get("center"):
            c = a["center"]
            bb = s.BoundBox
            s = s.copy()
            s.translate(vec([c[0] - (bb.XMin + bb.XMax) / 2.0,
                             c[1] - (bb.YMin + bb.YMax) / 2.0,
                             c[2] - (bb.ZMin + bb.ZMax) / 2.0]))
        return s

    if op == "revolve":
        # 2D 轮廓 (XZ 平面, 点列 [[x,z],...]; x=半径方向) 绕 axis (默认 Z) 旋转 angle° 成回转体.
        pts = a["points"]
        if len(pts) < 3:
            raise ValueError("revolve 轮廓至少 3 点")
        ang = float(a.get("angle", 360.0))
        vs = [vec([float(p[0]), 0.0, float(p[1])]) for p in pts]
        vs.append(vs[0])
        face = Part.Face(Part.makePolygon(vs))
        s = solidify(face.revolve(vec(a.get("base", [0, 0, 0])),
                                  vec(a.get("axis", [0, 0, 1])), ang))
        if a.get("center"):
            c = a["center"]
            bb = s.BoundBox
            s = s.copy()
            s.translate(vec([c[0] - (bb.XMin + bb.XMax) / 2.0,
                             c[1] - (bb.YMin + bb.YMax) / 2.0,
                             c[2] - (bb.ZMin + bb.ZMax) / 2.0]))
        return s

    if op == "pattern_polar":
        # 把 x 绕 axis 阵列 count 份 (full 360 时均布 step=angle/count, 否则 step=angle/(count-1)),
        # 融为一体返回 (不相交则为 Compound, 可直接作钻孔刀具或多体特征).
        src = shapes["x"]
        n = int(a["count"])
        if n < 1:
            raise ValueError("pattern count 须 ≥1")
        ang = float(a.get("angle", 360.0))
        full = abs(ang - 360.0) < 1e-9
        step = ang / n if full else (ang / (n - 1) if n > 1 else 0.0)
        axis = vec(a.get("axis", [0, 0, 1]))
        center = vec(a.get("center", [0, 0, 0]))
        r = None
        for i in range(n):
            c = src.copy()
            c.rotate(center, axis, step * i)
            r = c if r is None else r.fuse(c)
        return solidify(r.removeSplitter())

    if op == "pattern_linear":
        src = shapes["x"]
        n = int(a["count"])
        if n < 1:
            raise ValueError("pattern count 须 ≥1")
        dx, dy, dz = float(a["dx"]), float(a["dy"]), float(a["dz"])
        r = None
        for i in range(n):
            c = src.copy()
            c.translate(vec([dx * i, dy * i, dz * i]))
            r = c if r is None else r.fuse(c)
        return solidify(r.removeSplitter())

    if op == "loft":
        # 放样: 给一串截面 (各自所在 z 高度), 顺次蒙皮成实体. 真实"过渡接头"(方转圆等).
        #   sections: [{...}, ...] 每段二选一形状键 + z:
        #     {"circle": r, "z": z, "segments": n(默认 48), "center": [x,y](默认[0,0])}
        #     {"rect": [w, h], "z": z, "center": [x,y]}
        #     {"points": [[x,y],...], "z": z}   (任意闭合多边形)
        #   ruled: True=直纹(段间直线过渡)/False=光滑(默认).
        secs = a["sections"]
        if len(secs) < 2:
            raise ValueError("loft 至少 2 个截面")
        wires = [_section_wire(sec) for sec in secs]
        r = solidify(Part.makeLoft(wires, True, bool(a.get("ruled", False))))
        if not r.isClosed() or len(r.Solids) < 1:
            raise RuntimeError("loft 结果非封闭实体 (检查截面朝向/点序是否一致)")
        return r

    if op == "sweep":
        # 沿路径扫掠圆截面成管/杆 (真实管路布线: 直段 + 拐角圆弧过渡).
        # path: 转折控制点列 (≥2); bend_radius>0 时各内拐角以该半径切弧圆滑 → 精确 直线+相切弧
        # 路径 (而非 BSpline 插值: 插值在直↔弧过渡处会过冲自交, 致扫掠体退化). profile_radius: 圆截面半径.
        path_wire, t0 = _path_wire(a["path"], float(a.get("bend_radius", 0.0)))
        r = float(a["profile_radius"])
        circ = Part.Wire(Part.makeCircle(r, vec(a["path"][0]), t0))
        shp = path_wire.makePipeShell([circ], True, bool(a.get("frenet", False)))
        r2 = solidify(shp)
        if not r2.isClosed() or len(r2.Solids) < 1:
            raise RuntimeError("sweep 结果非封闭实体 (路径过弯/截面过大致自交?)")
        return r2

    if op == "helix":
        # 螺旋扫掠成线圈/弹簧/螺纹体: 沿螺旋线 (pitch 节距, height 总高, radius 螺旋半径)
        # 扫圆截面 (wire_radius 丝径). left_handed 左旋. 压缩弹簧/拉簧本体即此.
        helix = Part.makeHelix(float(a["pitch"]), float(a["height"]), float(a["radius"]),
                               0.0, bool(a.get("left_handed", False)))
        e0 = helix.Edges[0]
        p0 = e0.valueAt(e0.FirstParameter)
        t0 = e0.tangentAt(e0.FirstParameter)
        circ = Part.Wire(Part.makeCircle(float(a["wire_radius"]), p0, t0))
        coil = solidify(helix.makePipeShell([circ], True, True))
        if not coil.isClosed() or len(coil.Solids) < 1:
            raise RuntimeError("helix 扫掠结果非封闭实体 (wire_radius 过大/节距过小致相邻圈自交?)")
        return coil

    if op == "boolean":
        A = shapes["a"]
        B = shapes["b"]
        kind = str(a["op"]).lower()
        if kind in ("union", "fuse"):
            r = A.fuse(B)
        elif kind in ("difference", "cut"):
            r = A.cut(B)
        elif kind in ("intersection", "common"):
            r = A.common(B)
        else:
            raise ValueError("op 须为 union/difference/intersection")
        r = r.removeSplitter()  # 融合共面, 得干净 BREP
        if len(r.Faces) == 0:
            raise RuntimeError("布尔结果为空 (检查两体是否相交/包含)")
        return solidify(r)

    if op == "translate":
        s = shapes["x"].copy()
        s.translate(vec([a["dx"], a["dy"], a["dz"]]))
        return s

    if op == "rotate":
        s = shapes["x"].copy()
        s.rotate(vec(a.get("center", [0, 0, 0])), vec(a.get("axis", [0, 0, 1])), float(a["angle_deg"]))
        return s

    if op == "fillet":
        return _edge_feature(shapes["x"], "fillet", float(a["radius"]),
                             select=str(a.get("edges", "auto")),
                             near=a.get("near"), within=a.get("within"))

    if op == "chamfer":
        return _edge_feature(shapes["x"], "chamfer", float(a["distance"]),
                             select=str(a.get("edges", "auto")),
                             near=a.get("near"), within=a.get("within"))

    if op == "shell":
        # 抽壳 (薄壁挖空): 把实体掏空, 留壁厚 thickness; 指定一个面作开口 (掏开的口).
        #   open_near=[x,y,z]: 取离该点最近的面作开口 (精确锁面)
        #   open_dir=[ax,ay,az]: 取外法线与该方向一致、且最靠该向边界的平面作开口 (如顶面取 [0,0,1])
        #   皆不给: 全封闭中空壳 (无开口, 内有空腔).
        # OCC makeThickness: offset 取负 = 向内掏空, 壁厚 = |offset|.
        return _shell(shapes["x"], float(a["thickness"]),
                      open_near=a.get("open_near"), open_dir=a.get("open_dir"))

    raise ValueError("未知几何动作: " + str(op))


def _shell(shape, thickness, open_near=None, open_dir=None, tol=1e-3):
    faces = list(shape.Faces)
    rm = []
    if open_near is not None:
        vtx = Part.Vertex(vec(open_near))
        rm = [min(faces, key=lambda f: f.distToShape(vtx)[0])]
    elif open_dir is not None:
        d = vec(open_dir)
        L = d.Length or 1.0
        d = App.Vector(d.x / L, d.y / L, d.z / L)
        scored = []
        for f in faces:
            if not isinstance(f.Surface, Part.Plane):
                continue
            try:
                u, v = f.Surface.parameter(f.CenterOfMass)
                n = f.normalAt(u, v)
            except Exception:
                continue
            if n.dot(d) > 0.95:
                scored.append((f.CenterOfMass.dot(d), f))
        if not scored:
            raise RuntimeError("shell: 沿 open_dir 未找到朝向一致的平面 (无法定位开口面)")
        rm = [max(scored, key=lambda kv: kv[0])[1]]
    try:
        r = shape.makeThickness(rm, -abs(float(thickness)), tol)
    except Exception as e:
        raise RuntimeError("shell makeThickness 失败 (壁厚过大/几何过尖?): %s" % e)
    r = solidify(r)
    if not r.isClosed() or len(r.Solids) < 1:
        raise RuntimeError("shell 结果非封闭实体")
    if r.Volume >= shape.Volume:
        raise RuntimeError("shell 未掏空 (体积未减小, 检查 thickness/开口面)")
    return r


# ── 棱边特征 (倒圆/倒角): 健壮化 ──────────────────────────────────────────────
# 真实零件上对"全部棱边"一次性 makeChamfer/makeFillet 极易触发 OCC StdFail_NotDone
# (孔的圆柱棱、特征交线尤甚). 故:
#   ① 默认只取"两侧皆平面"的硬直棱 (工程上正应倒的棱, 跳过孔口圆棱);
#   ② 整批失败则贪心累加 —— 始终对【原形】施加"已选棱列表", 故棱引用恒定、无需在演化形上
#      重新匹配 (后者会错配并产出破面/多体); 每加一棱都校验 (闭合 ∧ 单实体 ∧ 体积合理),
#      不合格即弃该棱. 如此决不因个别坏棱而全盘失败, 亦决不产出破损实体.
def _edge_feature(shape, kind, val, select="auto", near=None, within=None):
    def _mk(shp, elist):
        return shp.makeFillet(val, elist) if kind == "fillet" else shp.makeChamfer(val, elist)

    def _valid(shp):
        r = solidify(shp)
        if not r.isClosed() or len(r.Solids) != 1:
            return None
        # 倒角/倒圆只去料或微增(倒圆凸边), 体积不应突变 (>5% 视为自交破损)
        if abs(r.Volume - shape.Volume) > 0.05 * shape.Volume:
            return None
        return r

    def _planar_adj(e):
        try:
            fs = shape.ancestorsOfType(e, Part.Face)
            return len(fs) >= 2 and all(isinstance(f.Surface, Part.Plane) for f in fs)
        except Exception:
            return False

    all_edges = shape.Edges
    if near is not None and within is not None:
        # 按"棱到给定点的最近距离 ≤ within"定向选棱 (点取在目标棱上即可精确锁定,
        # 可区分同心圆等 COM 重合的棱). 此时不再叠加 planar 限制.
        vtx = Part.Vertex(vec(near))
        cand = [e for e in all_edges if e.distToShape(vtx)[0] <= float(within)]
    elif select == "all":
        cand = list(all_edges)
    elif select == "straight":
        cand = [e for e in all_edges if isinstance(e.Curve, Part.Line)]
    else:  # auto / planar: 只取两侧皆平面的硬棱
        cand = [e for e in all_edges if _planar_adj(e)] or list(all_edges)

    # ① 整批快路 (校验通过即用)
    try:
        r = _valid(_mk(shape, cand))
        if r is not None:
            return r
    except Exception:
        pass

    # ② 贪心累加 (恒对原形施加, 棱引用稳定; 每步校验)
    kept = []
    for e in cand:
        trial = kept + [e]
        try:
            if _valid(_mk(shape, trial)) is not None:
                kept = trial
        except Exception:
            pass
    if not kept:
        raise RuntimeError(
            "%s 失败: 该形上无可安全处理的棱 (val=%s)" % (kind, val))
    return _valid(_mk(shape, kept))


def measure(shape, other=None):
    out = {"metrics": metrics(shape)}
    if other is not None:
        out["min_distance"] = round(float(shape.distToShape(other)[0]), 6)
    return out


def export(shape, path):
    low = path.lower()
    if low.endswith((".step", ".stp")):
        shape.exportStep(path)
    elif low.endswith((".iges", ".igs")):
        shape.exportIges(path)
    elif low.endswith(".stl"):
        shape.exportStl(path)
    elif low.endswith(".brep"):
        shape.exportBrep(path)
    else:
        raise ValueError("不支持的导出格式: " + path)
    import os
    return {"path": path, "bytes": os.path.getsize(path)}


# ── 内核门面: BREP 串 ⇄ 形状 (供无头子进程 freecad_kernel.py 用) ────────────
def op(op_name, a):
    """无头内核动作分发: 输入形状以 BREP 串经 a['shapes'] 传入, 结果序列化返回."""
    if op_name == "ping":
        return {"freecad": ".".join(App.Version()[:3])}

    sh = {k: load(v) for k, v in a.get("shapes", {}).items()}
    defl = float(a.get("deflection", 0.4))

    if op_name == "measure":
        return measure(sh["x"], sh.get("y"))

    if op_name == "tessellate":
        return {"mesh": tess(sh["x"], defl), "metrics": metrics(sh["x"])}

    if op_name == "export":
        return export(sh["x"], a["path"])

    shape = build(op_name, a, sh)
    return pack(shape, defl)
