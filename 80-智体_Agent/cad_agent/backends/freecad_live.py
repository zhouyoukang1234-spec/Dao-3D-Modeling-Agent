#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
freecad_live.py — FreeCAD 在世后端 · 智体就地操作"活文档"
═══════════════════════════════════════════════════════════════════════════════
道法自然 — 如 Cursor 之于 VS Code: 不另起进程, 而是 *住进* FreeCAD 自己的 python,
在当前打开的活文档 (App.ActiveDocument) 上就地造形. 每个工作区对象一一对应一个
Part::Feature 文档对象, 故所建之物即时显现于模型树与三维视图 —— AI 之"手"直接
落在用户眼前的工程里.

与无头子进程后端 freecad_backend 注册 *同名同义* 的 solid.* 工具, 纯几何同源自
freecad_ops (一字不二). 于是同一套 perceive→act→verify 会话逻辑既能驱动无头内核,
也能驱动 GUI 活文档 —— 仅"手"所在之处不同.

    · 权威 BREP 串 + 精确度量 → Workspace 对象 meta (引擎无关地全权拥有状态)
    · 三角剖分网格            → Workspace 对象主体 (供 perception "看见")
    · 活文档 Part::Feature    → GUI 模型树 / 三维视图即时可见 (meta['doc_obj'] 记其内名)
"""
from __future__ import annotations

import re
from typing import Any, Dict, Optional

import numpy as np

import FreeCAD as App  # noqa: E402  (仅 FreeCAD python 内可用)

from . import freecad_ops as ops
from .. import perception
from ..tools import ToolParam, ToolRegistry, Workspace

__all__ = ["register_freecad_live_tools", "LiveBridge"]


def _ident(name: str) -> str:
    """把对象名净化为 FreeCAD 合法内名 (字母数字下划线, 首字符非数字)."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not s or s[0].isdigit():
        s = "O_" + s
    return s


class LiveBridge:
    """掌管一个活文档: 把 Part.Shape 落为 Part::Feature 对象, 镜像进 Workspace."""

    def __init__(self, doc_name: str = "DaoAgent", deflection: float = 0.4) -> None:
        self.doc_name = doc_name
        self.deflection = float(deflection)
        self._doc = None
        self._owned: set = set()  # 本桥所建文档对象 Name 集 (reconcile 仅触及它们)

    # ── 活文档 ────────────────────────────────────────────────────────────
    @property
    def doc(self):
        if self._doc is None or self._doc not in App.listDocuments().values():
            self._doc = App.ActiveDocument or self._new_doc()
        return self._doc

    def _new_doc(self):
        # GUI 在场时以 hidden 建档: 文档照常存活并供智体就地造形, 但不自动开 3D 视图,
        # 从根上规避无 GPU VM 上 llvmpipe 视口的持续重绘风暴 (形之可见改由感知层 numpy 软栅).
        try:
            import FreeCADGui  # noqa: F401  (仅判定 GUI 是否在场)
            if App.GuiUp:
                return App.newDocument(self.doc_name, hidden=True)
        except Exception:
            pass
        return App.newDocument(self.doc_name)

    def _obj_of(self, ws: Workspace, name: str):
        """取该工作区对象对应的活文档对象 (无则建 Part::Feature)."""
        meta = ws.get(name)["meta"] if ws.has(name) else {}
        internal = meta.get("doc_obj")
        obj = self.doc.getObject(internal) if internal else None
        if obj is None:
            obj = self.doc.addObject("Part::Feature", _ident(name))
            obj.Label = name
            self._owned.add(obj.Name)
        return obj

    def put(self, ws: Workspace, name: str, shape, extra_meta: Optional[Dict[str, Any]] = None) -> str:
        """把 Part.Shape 落到活文档对象, 并镜像 (BREP/度量/网格) 进 Workspace."""
        obj = self._obj_of(ws, name)
        obj.Shape = shape
        self.doc.recompute()
        packed = ops.pack(shape, self.deflection)
        V = np.asarray(packed["mesh"]["vertices"], float).reshape(-1, 3)
        F = np.asarray(packed["mesh"]["faces"], int).reshape(-1, 3)
        meta = {"engine": "freecad-live", "brep": packed["brep"],
                "metrics": packed["metrics"], "doc_obj": obj.Name}
        if extra_meta:
            meta.update(extra_meta)
        ws.put(name, V, F, meta)
        return name

    def remove(self, ws: Workspace, name: str) -> None:
        if ws.has(name):
            internal = ws.get(name)["meta"].get("doc_obj")
            if internal and self.doc.getObject(internal) is not None:
                self.doc.removeObject(internal)
                self._owned.discard(internal)
        self.doc.recompute()

    def reconcile(self, ws: Workspace) -> Dict[str, Any]:
        """把活文档对齐到 Workspace 当前态 (供 undo/redo 后调用):
        删去 Workspace 已无的智体对象, 为残缺者按 BREP 重建形状. 仅触及本桥所建对象."""
        live_internal = {ws.get(n)["meta"].get("doc_obj") for n in ws.names()}
        removed, rebuilt = [], []
        # 删除孤儿: 本桥所建、但 Workspace 当前态已不指向的文档对象
        for internal in list(self._owned):
            obj = self.doc.getObject(internal)
            if obj is None:
                self._owned.discard(internal)
            elif internal not in live_internal:
                removed.append(obj.Label)
                self.doc.removeObject(internal)
                self._owned.discard(internal)
        # 重建缺失: Workspace 有但文档无其对象 (例如 redo 后), 按权威 BREP 复形
        for nm in ws.names():
            internal = ws.get(nm)["meta"].get("doc_obj")
            if not internal or self.doc.getObject(internal) is None:
                brep = ws.get(nm)["meta"].get("brep")
                if brep:
                    obj = self.doc.addObject("Part::Feature", _ident(nm))
                    obj.Label = nm
                    obj.Shape = ops.load(brep)
                    ws.get(nm)["meta"]["doc_obj"] = obj.Name
                    self._owned.add(obj.Name)
                    rebuilt.append(nm)
        self.doc.recompute()
        return {"removed": removed, "rebuilt": rebuilt}

    def shape_of(self, ws: Workspace, name: str):
        """取工作区对象的 Part.Shape (优先活文档对象, 回退 BREP 串)."""
        o = ws.get(name)
        internal = o["meta"].get("doc_obj")
        obj = self.doc.getObject(internal) if internal else None
        if obj is not None and not obj.Shape.isNull():
            return obj.Shape
        brep = o["meta"].get("brep")
        if not brep:
            raise ValueError(f"对象 '{name}' 无 BREP/活文档形状, 不能用 solid.* 工具")
        return ops.load(brep)


def _summary(ws: Workspace, name: str) -> Dict[str, Any]:
    o = ws.get(name)
    m = o["meta"].get("metrics", {})
    return {
        "name": name, "volume": m.get("volume"), "area": m.get("area"),
        "closed": m.get("closed"), "solids": m.get("solids"), "extents": m.get("extents"),
        "n_vertices": int(len(o["vertices"])), "n_faces": int(len(o["faces"])),
    }


# ═══════════════════════════════════════════════════════════════════════════
# 工具处理函数 (闭包捕获 bridge)
# ═══════════════════════════════════════════════════════════════════════════
def _make_handlers(B: LiveBridge):

    def _prim(op, prefix):
        def h(ws: Workspace, a: Dict[str, Any]) -> Dict[str, Any]:
            args = {k: v for k, v in a.items() if k != "name"}
            shape = ops.build(op, args)
            name = a.get("name") or ws.fresh_name(prefix)
            B.put(ws, name, shape, {"primitive": op})
            return _summary(ws, name)
        return h

    def h_scene_list(ws, a):
        return {"count": len(ws), "objects": [_summary(ws, n) for n in ws.names()]}

    def h_scene_clear(ws, a):
        n = len(ws)
        for nm in ws.names():
            B.remove(ws, nm)
            ws.delete(nm)
        return {"cleared": n}

    def h_boolean(ws, a):
        shape = ops.build("boolean", {"op": a["op"]},
                          {"a": B.shape_of(ws, a["a"]), "b": B.shape_of(ws, a["b"])})
        name = a.get("result") or ws.fresh_name(str(a["op"])[:3] + "_")
        if a.get("consume"):
            for k in (a["a"], a["b"]):
                if ws.has(k) and k != name:
                    B.remove(ws, k)
                    ws.delete(k)
        B.put(ws, name, shape, {"op": a["op"], "a": a["a"], "b": a["b"]})
        return {"op": a["op"], **_summary(ws, name)}

    def h_translate(ws, a):
        shape = ops.build("translate", {"dx": a["dx"], "dy": a["dy"], "dz": a["dz"]},
                          {"x": B.shape_of(ws, a["name"])})
        B.put(ws, a["name"], shape)
        return _summary(ws, a["name"])

    def h_rotate(ws, a):
        shape = ops.build("rotate", {"angle_deg": a["angle_deg"], "axis": a.get("axis", [0, 0, 1]),
                                     "center": a.get("center", [0, 0, 0])},
                          {"x": B.shape_of(ws, a["name"])})
        B.put(ws, a["name"], shape)
        return _summary(ws, a["name"])

    def h_fillet(ws, a):
        shape = ops.build("fillet", {"radius": a["radius"]}, {"x": B.shape_of(ws, a["name"])})
        B.put(ws, a["name"], shape, {"fillet": a["radius"]})
        return _summary(ws, a["name"])

    def h_chamfer(ws, a):
        shape = ops.build("chamfer", {"distance": a["distance"]}, {"x": B.shape_of(ws, a["name"])})
        B.put(ws, a["name"], shape, {"chamfer": a["distance"]})
        return _summary(ws, a["name"])

    def h_delete(ws, a):
        B.remove(ws, a["name"])
        ws.delete(a["name"])
        return {"deleted": a["name"], "remaining": ws.names()}

    def h_rename(ws, a):
        ws.rename(a["name"], a["new_name"])
        o = ws.get(a["new_name"])
        internal = o["meta"].get("doc_obj")
        obj = B.doc.getObject(internal) if internal else None
        if obj is not None:
            obj.Label = a["new_name"]
        return {"renamed_to": a["new_name"]}

    def h_measure(ws, a):
        other = B.shape_of(ws, a["to"]) if a.get("to") else None
        res = ops.measure(B.shape_of(ws, a["name"]), other)
        out = {"name": a["name"], **res["metrics"]}
        out["watertight"] = out.get("closed")  # 与 mesh 后端同义键, 供引擎无关 verify 复用
        if "min_distance" in res:
            out["min_distance_to"] = {"other": a["to"], "distance": res["min_distance"]}
        return out

    def h_export(ws, a):
        from pathlib import Path
        p = Path(a["path"])
        p.parent.mkdir(parents=True, exist_ok=True)
        res = ops.export(B.shape_of(ws, a["name"]), str(p))
        return {"name": a["name"], **res}

    def h_perceive(ws, a):
        o = ws.get(a["name"])
        m = perception.Mesh(o["vertices"], o["faces"], a["name"])
        r = perception.perceive(m, resolution=int(a.get("resolution", 192)),
                                out_dir=a.get("out_dir"), save_png=bool(a.get("save_png", False)))
        bm = o["meta"].get("metrics", {})
        rep = dict(r["report"])
        rep["brep_volume"] = bm.get("volume")
        rep["brep_area"] = bm.get("area")
        rep["brep_closed"] = bm.get("closed")
        return {"name": a["name"], "summary": r["summary"], "report": rep, "renders": r["renders"]}

    return locals()


# ═══════════════════════════════════════════════════════════════════════════
# 注册 (与 freecad_backend 同名同义; 仅"手"落在活文档而非子进程)
# ═══════════════════════════════════════════════════════════════════════════
def register_freecad_live_tools(reg: ToolRegistry, bridge: Optional[LiveBridge] = None) -> ToolRegistry:
    B = bridge or LiveBridge()
    H = _make_handlers(B)
    P = ToolParam
    reg.freecad_bridge = B

    reg.add("scene.list", "列出活文档内所有 BREP 实体对象及其度量概要.",
            H["h_scene_list"], [], category="scene")
    reg.add("scene.clear", "清空活文档所有智体对象.",
            H["h_scene_clear"], [], category="scene", mutates=True)

    reg.add("solid.box", "在活文档创建长方体 (x/y/z 边长, 可选 center 中心点).",
            H["_prim"]("box", "box"), [
                P("x", "number", "X 边长"), P("y", "number", "Y 边长"), P("z", "number", "Z 边长"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.cylinder", "在活文档创建圆柱 (radius/height, 轴向 Z, center 为中心).",
            H["_prim"]("cylinder", "cyl"), [
                P("radius", "number", "半径"), P("height", "number", "高"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.sphere", "在活文档创建球 (radius; center 为球心).",
            H["_prim"]("sphere", "sph"), [
                P("radius", "number", "半径"),
                P("center", "array", "球心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.cone", "在活文档创建圆台/圆锥 (radius1 底/ radius2 顶/ height 高).",
            H["_prim"]("cone", "cone"), [
                P("radius1", "number", "底半径"), P("radius2", "number", "顶半径 (0=锥)"),
                P("height", "number", "高"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.torus", "在活文档创建圆环 (radius1 主半径/ radius2 管半径).",
            H["_prim"]("torus", "tor"), [
                P("radius1", "number", "主半径"), P("radius2", "number", "管半径"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.boolean",
            "BREP 布尔: op∈{union,difference,intersection}; a-b 两对象; "
            "可选 result 命名、consume 是否消耗输入. 结果自动 removeSplitter 清理.",
            H["h_boolean"], [
                P("op", "string", "union/difference/intersection"),
                P("a", "string", "对象 A"), P("b", "string", "对象 B"),
                P("result", "string", "结果名", False, None),
                P("consume", "boolean", "完成后删除 A、B", False, False),
            ], category="boolean", mutates=True)

    reg.add("solid.translate", "平移实体.",
            H["h_translate"], [
                P("name", "string", "对象名"),
                P("dx", "number", "X 位移"), P("dy", "number", "Y 位移"), P("dz", "number", "Z 位移"),
            ], category="transform", mutates=True)

    reg.add("solid.rotate", "绕轴旋转实体 (角度制).",
            H["h_rotate"], [
                P("name", "string", "对象名"), P("angle_deg", "number", "角度 (度)"),
                P("axis", "array", "轴向 [x,y,z]", False, [0, 0, 1]),
                P("center", "array", "旋转中心", False, [0, 0, 0]),
            ], category="transform", mutates=True)

    reg.add("solid.fillet", "对实体所有棱边倒圆角 (radius).",
            H["h_fillet"], [
                P("name", "string", "对象名"), P("radius", "number", "圆角半径"),
            ], category="feature", mutates=True)

    reg.add("solid.chamfer", "对实体所有棱边倒角 (distance).",
            H["h_chamfer"], [
                P("name", "string", "对象名"), P("distance", "number", "倒角距离"),
            ], category="feature", mutates=True)

    reg.add("solid.delete", "删除对象 (连同活文档对象).",
            H["h_delete"], [P("name", "string", "对象名")], category="object", mutates=True)

    reg.add("solid.rename", "重命名对象.",
            H["h_rename"], [P("name", "string", "原名"), P("new_name", "string", "新名")],
            category="object", mutates=True)

    reg.add("solid.measure",
            "BREP 精确度量: 体积/面积/水密/质心/包围盒/实体数; 可选 to 求到另一对象最小间距.",
            H["h_measure"], [
                P("name", "string", "对象名"),
                P("to", "string", "另一对象名 (求最小间距)", False, None),
            ], category="measure")

    reg.add("solid.export", "导出实体为 STEP/IGES/STL/BREP (按扩展名定格式).",
            H["h_export"], [
                P("name", "string", "对象名"), P("path", "string", "输出路径 (含扩展名)"),
            ], category="io")

    reg.add("solid.perceive",
            "感知实体: 多视角渲染 + 结构报告 (含 BREP 精确体积/面积/水密) + 自然语言摘要.",
            H["h_perceive"], [
                P("name", "string", "对象名"),
                P("resolution", "integer", "渲染分辨率", False, 192),
                P("out_dir", "string", "PNG 输出目录", False, None),
                P("save_png", "boolean", "是否落盘 PNG", False, False),
            ], category="perceive")

    return reg
