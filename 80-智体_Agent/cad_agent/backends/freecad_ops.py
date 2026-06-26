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


def metrics(shape):
    bb = shape.BoundBox
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
        s = shapes["x"]
        r = s.makeFillet(float(a["radius"]), s.Edges)
        return solidify(r)

    if op == "chamfer":
        s = shapes["x"]
        r = s.makeChamfer(float(a["distance"]), s.Edges)
        return solidify(r)

    raise ValueError("未知几何动作: " + str(op))


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
