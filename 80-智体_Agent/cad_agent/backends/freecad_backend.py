#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
freecad_backend.py — FreeCAD 引擎后端 · 把成熟 BREP 内核注入统一工具协议
═══════════════════════════════════════════════════════════════════════════════
道法自然 — 如 Cursor 之于 VS Code: 不另起炉灶, 而是站在成熟的 FreeCAD 之上演化.
本后端把 FreeCAD 降为一只可替换的 "手": 经 freecad_kernel.py 子进程 (运行于
FreeCAD 自带 python) 以 JSON 行驱动真实 BREP 实体建模, 再把结果

    · 权威 BREP 字符串      → 存入 Workspace 对象 meta['brep'] (引擎无关地全权拥有状态)
    · 精确度量 (体积/面积/水密/质心) → meta['metrics'] (BREP 精确, 非网格近似)
    · 三角剖分网格 (verts/faces)     → Workspace 对象主体 (供 perception "看见")

于是上层 perception / session / mcp_server 一字不改即可驱动 FreeCAD:
同一套 perceive→act→verify, 仅把引擎从 mesh 换成 FreeCAD —— 此即引擎无关之证.

工具族 (与 mesh 后端同义, 命名空间 solid.* 标识 "BREP 实体引擎"):
    scene.list / scene.clear
    solid.box / cylinder / sphere / cone / torus    参数化实体图元
    solid.boolean                                    布尔 (union/difference/intersection)
    solid.translate / rotate                         刚体变换
    solid.fillet / chamfer                           倒圆 / 倒角 (BREP 特有, 网格难为)
    solid.duplicate / delete / rename                对象管理
    solid.measure                                    BREP 精确度量 (+ 最小间距)
    solid.export                                     导出 STEP/IGES/STL/BREP
    solid.perceive                                   感知 (复用 perception)
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from .. import perception
from ..tools import ToolParam, ToolRegistry, Workspace

__all__ = ["FreeCADKernel", "register_freecad_tools", "find_freecadcmd"]

_RESP = "__FCR__ "
_KERNEL_SRC = Path(__file__).resolve().parent / "freecad_kernel.py"


# ═══════════════════════════════════════════════════════════════════════════
# 定位 freecadcmd
# ═══════════════════════════════════════════════════════════════════════════
def find_freecadcmd() -> Optional[str]:
    """按 环境变量 → 常见安装位置 → PATH 顺序定位 freecadcmd(.exe)."""
    env = os.environ.get("FREECADCMD") or os.environ.get("FREECAD_CMD")
    if env and Path(env).exists():
        return env
    pats = [
        r"C:\Program Files\FreeCAD*\bin\freecadcmd.exe",
        r"C:\Program Files\FreeCAD*\bin\FreeCADCmd.exe",
        "/usr/bin/freecadcmd",
        "/usr/local/bin/freecadcmd",
        "/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd",
    ]
    for p in pats:
        hits = sorted(glob.glob(p))
        if hits:
            return hits[-1]
    return shutil.which("freecadcmd") or shutil.which("FreeCADCmd")


# ═══════════════════════════════════════════════════════════════════════════
# 内核子进程管理 (线程安全的单请求-单应答)
# ═══════════════════════════════════════════════════════════════════════════
class FreeCADKernel:
    """以子进程拉起 freecadcmd 运行 freecad_kernel.py, 经 stdin/stdout 行式 RPC.

    无状态内核: 每次 call 把所需输入形状 (BREP 字符串) 随参数传入, 取回新形状.
    """

    def __init__(self, freecadcmd: Optional[str] = None) -> None:
        self.exe = freecadcmd or find_freecadcmd()
        if not self.exe:
            raise RuntimeError(
                "未找到 freecadcmd; 请安装 FreeCAD 或设置环境变量 FREECADCMD 指向 freecadcmd(.exe)")
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._script_path = self._stage_kernel()

    @staticmethod
    def _stage_kernel() -> str:
        """把内核脚本复制到纯 ASCII 临时路径, 规避中文路径经 argv 传入 freecadcmd 的乱码."""
        dst_dir = Path(tempfile.gettempdir()) / "dao_fc_kernel"
        dst_dir.mkdir(parents=True, exist_ok=True)
        dst = dst_dir / "freecad_kernel.py"
        shutil.copyfile(_KERNEL_SRC, dst)
        return str(dst)

    def _spawn(self) -> None:
        self._proc = subprocess.Popen(
            [self.exe, self._script_path],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, encoding="utf-8", bufsize=1,
        )
        # 排空启动横幅, 直到内核 boot 标记, 使后续 请求↔应答 一一对齐
        assert self._proc.stdout
        while True:
            line = self._proc.stdout.readline()
            if line == "":
                raise RuntimeError("FreeCAD 内核启动失败 (未见 boot 标记)")
            if line.startswith(_RESP):
                if json.loads(line[len(_RESP):]).get("data", {}).get("boot"):
                    break

    def start(self) -> "FreeCADKernel":
        """显式启动并握手 (越过 FreeCAD 启动横幅)."""
        self.call("ping")
        return self

    def call(self, op: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._spawn()
            assert self._proc and self._proc.stdin and self._proc.stdout
            self._proc.stdin.write(json.dumps({"op": op, "args": args or {}}) + "\n")
            self._proc.stdin.flush()
            while True:
                line = self._proc.stdout.readline()
                if line == "":
                    raise RuntimeError(f"FreeCAD 内核进程意外退出 (op={op})")
                if line.startswith(_RESP):
                    resp = json.loads(line[len(_RESP):])
                    break
            if not resp.get("ok"):
                raise RuntimeError(resp.get("error", "FreeCAD 内核未知错误"))
            return resp.get("data", {})

    def close(self) -> None:
        with self._lock:
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.stdin.write(json.dumps({"op": "shutdown"}) + "\n")
                    self._proc.stdin.flush()
                    self._proc.wait(timeout=5)
                except Exception:  # noqa: BLE001
                    self._proc.kill()
            self._proc = None


# ═══════════════════════════════════════════════════════════════════════════
# Workspace 存取: BREP 权威 + 网格镜像
# ═══════════════════════════════════════════════════════════════════════════
def _store(ws: Workspace, name: str, res: Dict[str, Any],
           extra_meta: Optional[Dict[str, Any]] = None) -> str:
    mesh = res["mesh"]
    V = np.asarray(mesh["vertices"], float).reshape(-1, 3)
    F = np.asarray(mesh["faces"], int).reshape(-1, 3)
    meta = {"engine": "freecad", "brep": res["brep"], "metrics": res["metrics"]}
    if extra_meta:
        meta.update(extra_meta)
    ws.put(name, V, F, meta)
    return name


def _brep(ws: Workspace, name: str) -> str:
    o = ws.get(name)
    b = o["meta"].get("brep")
    if not b:
        raise ValueError(f"对象 '{name}' 非 FreeCAD/BREP 对象 (无 brep), 不能用 solid.* 工具")
    return b


def _summary(ws: Workspace, name: str) -> Dict[str, Any]:
    o = ws.get(name)
    m = o["meta"].get("metrics", {})
    return {
        "name": name,
        "volume": m.get("volume"),
        "area": m.get("area"),
        "closed": m.get("closed"),
        "solids": m.get("solids"),
        "extents": m.get("extents"),
        "n_vertices": int(len(o["vertices"])),
        "n_faces": int(len(o["faces"])),
    }


# ═══════════════════════════════════════════════════════════════════════════
# 工具处理函数 (闭包捕获 kernel)
# ═══════════════════════════════════════════════════════════════════════════
def _make_handlers(K: FreeCADKernel):

    def _prim(op, prefix):
        def h(ws: Workspace, a: Dict[str, Any]) -> Dict[str, Any]:
            args = {k: v for k, v in a.items() if k != "name"}
            args["deflection"] = a.get("deflection", 0.4)
            res = K.call(op, args)
            name = a.get("name") or ws.fresh_name(prefix)
            _store(ws, name, res, {"primitive": op})
            return _summary(ws, name)
        return h

    def h_scene_list(ws, a):
        return {"count": len(ws), "objects": [_summary(ws, n) for n in ws.names()]}

    def h_scene_clear(ws, a):
        n = len(ws)
        for nm in ws.names():
            ws.delete(nm)
        return {"cleared": n}

    def h_boolean(ws, a):
        res = K.call("boolean", {
            "op": a["op"], "deflection": a.get("deflection", 0.4),
            "shapes": {"a": _brep(ws, a["a"]), "b": _brep(ws, a["b"])},
        })
        name = a.get("result") or ws.fresh_name(str(a["op"])[:3] + "_")
        _store(ws, name, res, {"op": a["op"], "a": a["a"], "b": a["b"]})
        if a.get("consume"):
            for k in (a["a"], a["b"]):
                if ws.has(k) and k != name:
                    ws.delete(k)
        return {"op": a["op"], **_summary(ws, name)}

    def h_translate(ws, a):
        res = K.call("translate", {
            "dx": a["dx"], "dy": a["dy"], "dz": a["dz"],
            "deflection": a.get("deflection", 0.4), "shapes": {"x": _brep(ws, a["name"])}})
        _store(ws, a["name"], res)
        return _summary(ws, a["name"])

    def h_rotate(ws, a):
        res = K.call("rotate", {
            "angle_deg": a["angle_deg"], "axis": a.get("axis", [0, 0, 1]),
            "center": a.get("center", [0, 0, 0]), "deflection": a.get("deflection", 0.4),
            "shapes": {"x": _brep(ws, a["name"])}})
        _store(ws, a["name"], res)
        return _summary(ws, a["name"])

    def h_fillet(ws, a):
        res = K.call("fillet", {"radius": a["radius"], "deflection": a.get("deflection", 0.4),
                                "shapes": {"x": _brep(ws, a["name"])}})
        _store(ws, a["name"], res, {"fillet": a["radius"]})
        return _summary(ws, a["name"])

    def h_chamfer(ws, a):
        res = K.call("chamfer", {"distance": a["distance"], "deflection": a.get("deflection", 0.4),
                                 "shapes": {"x": _brep(ws, a["name"])}})
        _store(ws, a["name"], res, {"chamfer": a["distance"]})
        return _summary(ws, a["name"])

    def h_duplicate(ws, a):
        o = ws.get(a["name"])
        name = a.get("new_name") or ws.fresh_name(a["name"] + "_copy")
        ws.put(name, o["vertices"].copy(), o["faces"].copy(), dict(o["meta"]))
        return _summary(ws, name)

    def h_delete(ws, a):
        ws.delete(a["name"])
        return {"deleted": a["name"], "remaining": ws.names()}

    def h_rename(ws, a):
        ws.rename(a["name"], a["new_name"])
        return {"renamed_to": a["new_name"]}

    def h_measure(ws, a):
        shapes = {"x": _brep(ws, a["name"])}
        if a.get("to"):
            shapes["y"] = _brep(ws, a["to"])
        res = K.call("measure", {"shapes": shapes})
        out = {"name": a["name"], **res["metrics"]}
        out["watertight"] = out.get("closed")  # 与 mesh 后端同义键, 供引擎无关的 verify 复用
        if "min_distance" in res:
            out["min_distance_to"] = {"other": a["to"], "distance": res["min_distance"]}
        return out

    def h_export(ws, a):
        p = Path(a["path"])
        p.parent.mkdir(parents=True, exist_ok=True)
        res = K.call("export", {"path": str(p), "shapes": {"x": _brep(ws, a["name"])}})
        return {"name": a["name"], **res}

    def h_perceive(ws, a):
        o = ws.get(a["name"])
        m = perception.Mesh(o["vertices"], o["faces"], a["name"])
        r = perception.perceive(m, resolution=int(a.get("resolution", 192)),
                                out_dir=a.get("out_dir"), save_png=bool(a.get("save_png", False)))
        # 用 BREP 精确度量覆盖网格近似的体积/水密
        bm = o["meta"].get("metrics", {})
        rep = dict(r["report"])
        rep["brep_volume"] = bm.get("volume")
        rep["brep_area"] = bm.get("area")
        rep["brep_closed"] = bm.get("closed")
        return {"name": a["name"], "summary": r["summary"], "report": rep, "renders": r["renders"]}

    return locals()


# ═══════════════════════════════════════════════════════════════════════════
# 注册
# ═══════════════════════════════════════════════════════════════════════════
def register_freecad_tools(reg: ToolRegistry, kernel: Optional[FreeCADKernel] = None) -> ToolRegistry:
    """把 FreeCAD 后端的全部工具注入给定 registry; 复用其内核(或新建)."""
    K = kernel or FreeCADKernel()
    H = _make_handlers(K)
    P = ToolParam
    reg.freecad_kernel = K  # 便于 mcp_server/会话结束时 close()

    reg.add("scene.list", "列出工作区内所有 BREP 实体对象及其度量概要.",
            H["h_scene_list"], [], category="scene")
    reg.add("scene.clear", "清空工作区所有对象.",
            H["h_scene_clear"], [], category="scene", mutates=True)

    reg.add("solid.box", "创建长方体实体 (x/y/z 边长, 可选 center 中心点).",
            H["_prim"]("box", "box"), [
                P("x", "number", "X 边长"), P("y", "number", "Y 边长"), P("z", "number", "Z 边长"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.cylinder", "创建圆柱实体 (radius/height, 轴向 Z, center 为中心).",
            H["_prim"]("cylinder", "cyl"), [
                P("radius", "number", "半径"), P("height", "number", "高"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.sphere", "创建球实体 (radius; center 为球心).",
            H["_prim"]("sphere", "sph"), [
                P("radius", "number", "半径"),
                P("center", "array", "球心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.cone", "创建圆台/圆锥实体 (radius1 底/ radius2 顶/ height 高).",
            H["_prim"]("cone", "cone"), [
                P("radius1", "number", "底半径"), P("radius2", "number", "顶半径 (0=锥)"),
                P("height", "number", "高"),
                P("center", "array", "中心 [x,y,z]", False, None),
                P("name", "string", "对象名", False, None),
            ], category="primitive", mutates=True)

    reg.add("solid.torus", "创建圆环实体 (radius1 主半径/ radius2 管半径).",
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

    reg.add("solid.duplicate", "复制对象.",
            H["h_duplicate"], [
                P("name", "string", "对象名"), P("new_name", "string", "副本名", False, None),
            ], category="object", mutates=True)

    reg.add("solid.delete", "删除对象.",
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
