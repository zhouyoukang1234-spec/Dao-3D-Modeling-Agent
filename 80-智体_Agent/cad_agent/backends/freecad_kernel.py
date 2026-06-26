#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
freecad_kernel.py — FreeCAD 无头几何内核 (运行于 FreeCAD 自带 python 内)
═══════════════════════════════════════════════════════════════════════════════
弱者道之用 — 不把 agent 塞进 FreeCAD, 而是把 FreeCAD 降格为一个 *纯函数式* 的
BREP 几何内核服务: 经 stdin/stdout 以 JSON 行收发, 每次调用

    请求:  {"op": <动作>, "args": {... , "shapes": {名: brep字符串}}}
    应答:  __FCR__ {"ok": true, "data": {"brep": <新形状>, "mesh": {...}, "metrics": {...}}}

内核 *不持有状态*: 输入形状以 BREP 字符串随调用传入, 输出形状以 BREP 字符串返回.
于是上层 Workspace (引擎无关的具名对象表) 全权拥有状态 —— 快照/撤销/对比天然成立,
此即 "万法归一" 在 BREP 引擎上的落地: FreeCAD 只是一只可随时替换的 "手".

由 freecad_backend.py 以子进程方式拉起; 本文件不依赖 cad_agent 任何模块,
以便被复制到纯 ASCII 临时路径后用 freecadcmd 执行 (规避中文路径 argv 乱码).
"""
import sys
import json

import FreeCAD as App  # noqa: E402  (仅 FreeCAD python 内可用)
import Part  # noqa: E402

RESP = "__FCR__ "  # 应答行哨兵; 反扫此前缀即可越过 FreeCAD 启动横幅与杂讯


# ── BREP 字符串 ⇄ Part.Shape ───────────────────────────────────────────────
def _load(brep):
    s = Part.Shape()
    s.importBrepFromString(brep)
    return s


def _dump(shape):
    return shape.exportBrepToString()


def _vec(a):
    return App.Vector(float(a[0]), float(a[1]), float(a[2]))


def _solidify(shape):
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


def _metrics(shape):
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


def _tess(shape, deflection):
    verts, facets = shape.tessellate(float(deflection))
    V = [[round(v.x, 6), round(v.y, 6), round(v.z, 6)] for v in verts]
    F = [[int(a), int(b), int(c)] for (a, b, c) in facets]
    return {"vertices": V, "faces": F}


def _result(shape, deflection):
    return {"brep": _dump(shape), "mesh": _tess(shape, deflection), "metrics": _metrics(shape)}


# ── 动作 ────────────────────────────────────────────────────────────────────
def _op(op, a):
    sh = a.get("shapes", {})  # {名: brep}
    defl = float(a.get("deflection", 0.4))

    if op == "ping":
        return {"freecad": ".".join(App.Version()[:3])}

    if op == "box":
        s = Part.makeBox(float(a["x"]), float(a["y"]), float(a["z"]))
        if a.get("center"):
            c = a["center"]
            s.translate(_vec([c[0] - a["x"] / 2.0, c[1] - a["y"] / 2.0, c[2] - a["z"] / 2.0]))
        return _result(s, defl)

    if op == "cylinder":
        s = Part.makeCylinder(float(a["radius"]), float(a["height"]))
        if a.get("center"):
            c = a["center"]
            s.translate(_vec([c[0], c[1], c[2] - a["height"] / 2.0]))
        return _result(s, defl)

    if op == "sphere":
        s = Part.makeSphere(float(a["radius"]))
        if a.get("center"):
            s.translate(_vec(a["center"]))
        return _result(s, defl)

    if op == "cone":
        s = Part.makeCone(float(a["radius1"]), float(a["radius2"]), float(a["height"]))
        if a.get("center"):
            c = a["center"]
            s.translate(_vec([c[0], c[1], c[2] - a["height"] / 2.0]))
        return _result(s, defl)

    if op == "torus":
        s = Part.makeTorus(float(a["radius1"]), float(a["radius2"]))
        if a.get("center"):
            s.translate(_vec(a["center"]))
        return _result(s, defl)

    if op == "boolean":
        A = _load(sh["a"]); B = _load(sh["b"])
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
        return _result(_solidify(r), defl)

    if op == "translate":
        s = _load(sh["x"])
        s.translate(_vec([a["dx"], a["dy"], a["dz"]]))
        return _result(s, defl)

    if op == "rotate":
        s = _load(sh["x"])
        s.rotate(_vec(a.get("center", [0, 0, 0])), _vec(a.get("axis", [0, 0, 1])), float(a["angle_deg"]))
        return _result(s, defl)

    if op == "fillet":
        s = _load(sh["x"])
        edges = s.Edges
        r = s.makeFillet(float(a["radius"]), edges)
        return _result(_solidify(r), defl)

    if op == "chamfer":
        s = _load(sh["x"])
        edges = s.Edges
        r = s.makeChamfer(float(a["distance"]), edges)
        return _result(_solidify(r), defl)

    if op == "measure":
        s = _load(sh["x"])
        out = {"metrics": _metrics(s)}
        if "y" in sh:
            other = _load(sh["y"])
            d = s.distToShape(other)[0]
            out["min_distance"] = round(float(d), 6)
        return out

    if op == "tessellate":
        s = _load(sh["x"])
        return {"mesh": _tess(s, defl), "metrics": _metrics(s)}

    if op == "export":
        s = _load(sh["x"])
        path = a["path"]
        s.exportBrep(path) if path.lower().endswith(".brep") else None
        low = path.lower()
        if low.endswith((".step", ".stp")):
            s.exportStep(path)
        elif low.endswith((".iges", ".igs")):
            s.exportIges(path)
        elif low.endswith(".stl"):
            s.exportStl(path)
        elif low.endswith(".brep"):
            pass  # 已导出
        else:
            raise ValueError("不支持的导出格式: " + path)
        import os
        return {"path": path, "bytes": os.path.getsize(path)}

    raise ValueError("未知内核动作: " + str(op))


def main():
    sys.stdout.write(RESP + json.dumps({"ok": True, "data": {"boot": True}}) + "\n")
    sys.stdout.flush()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            if req.get("op") == "shutdown":
                sys.stdout.write(RESP + json.dumps({"ok": True, "data": {"bye": True}}) + "\n")
                sys.stdout.flush()
                return
            data = _op(req["op"], req.get("args", {}))
            resp = {"ok": True, "data": data}
        except Exception as e:  # noqa: BLE001 — 边界归一
            import traceback
            resp = {"ok": False, "error": "%s: %s" % (type(e).__name__, e),
                    "trace": traceback.format_exc(limit=3)}
        sys.stdout.write(RESP + json.dumps(resp) + "\n")
        sys.stdout.flush()


# freecadcmd 执行本文件时 __name__ 未必为 "__main__"; 故直接调用.
main()
