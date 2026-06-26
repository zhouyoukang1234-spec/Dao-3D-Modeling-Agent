#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dao_interpreter.py — 对话→工具调用 解释器 (DaoAgent 的"语言层")
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 它不自造语义, 而是 *顺着* ToolRegistry 已有的工具契约把一行文本解析成
一次工具调用 —— 故新增后端工具即自动可被对话驱动, 无须改解释器 (无为而无不为).

这是确定性的精简指令文法 (离线即用). 真正的自然语言→工具序列可由 LLM 驱动器经
mcp_server 接管 —— 二者产出同一种 (tool, args) 调用, 故底层会话逻辑一字不变.

文法 (大小写不敏感, 别名见 ALIASES):
    box x=60 y=40 z=6 name=base        # key=value
    box 60 40 6 base                   # 位置参数 (按工具必填参数次序)
    cyl r=3 h=20 at=15,20,3 name=h1    # at/center 用逗号分隔→数组
    union base rib -> flange consume   # 布尔: 位置 a b, ->结果名, consume 标志
    diff flange h1 -> flange consume
    move base dz=6                     # 平移
    rotate base angle=45 axis=0,0,1
    fillet base r=2 ; chamfer base d=1
    measure flange ; perceive flange ; export flange path=C:\\x\\f.step
    list ; clear ; delete h1 ; rename a b
    undo ; redo ; help
"""
from __future__ import annotations

import re
import shlex
from typing import Any, Dict, List, Optional, Tuple

# 友好动词 → (工具全名, 预置参数)
ALIASES: Dict[str, Tuple[str, Dict[str, Any]]] = {
    "box": ("solid.box", {}),
    "cube": ("solid.box", {}),
    "cyl": ("solid.cylinder", {}), "cylinder": ("solid.cylinder", {}),
    "sphere": ("solid.sphere", {}), "ball": ("solid.sphere", {}),
    "cone": ("solid.cone", {}),
    "torus": ("solid.torus", {}), "ring": ("solid.torus", {}),
    "union": ("solid.boolean", {"op": "union"}), "add": ("solid.boolean", {"op": "union"}),
    "fuse": ("solid.boolean", {"op": "union"}),
    "diff": ("solid.boolean", {"op": "difference"}), "difference": ("solid.boolean", {"op": "difference"}),
    "cut": ("solid.boolean", {"op": "difference"}), "sub": ("solid.boolean", {"op": "difference"}),
    "intersect": ("solid.boolean", {"op": "intersection"}),
    "intersection": ("solid.boolean", {"op": "intersection"}), "common": ("solid.boolean", {"op": "intersection"}),
    "move": ("solid.translate", {}), "translate": ("solid.translate", {}),
    "rotate": ("solid.rotate", {}), "rot": ("solid.rotate", {}),
    "fillet": ("solid.fillet", {}), "round": ("solid.fillet", {}),
    "chamfer": ("solid.chamfer", {}), "bevel": ("solid.chamfer", {}),
    "measure": ("solid.measure", {}), "meas": ("solid.measure", {}),
    "perceive": ("solid.perceive", {}), "see": ("solid.perceive", {}), "look": ("solid.perceive", {}),
    "export": ("solid.export", {}), "save": ("solid.export", {}),
    "delete": ("solid.delete", {}), "del": ("solid.delete", {}), "rm": ("solid.delete", {}),
    "rename": ("solid.rename", {}),
    "list": ("scene.list", {}), "ls": ("scene.list", {}),
    "clear": ("scene.clear", {}), "reset": ("scene.clear", {}),
}

# 友好键 → 工具参数名
KEY_ALIASES = {
    "r": "radius", "rad": "radius", "h": "height", "ht": "height",
    "r1": "radius1", "r2": "radius2", "d": "distance", "dist": "distance",
    "at": "center", "pos": "center", "c": "center", "ctr": "center",
    "angle": "angle_deg", "deg": "angle_deg", "res": "resolution",
    "to": "to", "n": "name",
}

META_VERBS = {"undo", "redo", "help", "?"}


class ParseError(ValueError):
    pass


def _coerce(raw: str, ptype: str) -> Any:
    """按工具参数类型把字符串值强转."""
    if ptype == "array":
        return [float(x) for x in raw.split(",") if x != ""]
    if ptype == "number":
        return float(raw)
    if ptype == "integer":
        return int(float(raw))
    if ptype == "boolean":
        return raw.lower() in ("1", "true", "yes", "on", "t")
    return raw  # string


def parse(line: str, registry) -> Dict[str, Any]:
    """把一行解析为 {kind, ...}. kind ∈ {tool, meta, empty}.
       tool: {kind:'tool', tool, args}; meta: {kind:'meta', verb}."""
    line = (line or "").strip()
    if not line:
        return {"kind": "empty"}
    toks = shlex.split(line)
    verb = toks[0].lower()

    if verb in META_VERBS:
        return {"kind": "meta", "verb": "help" if verb == "?" else verb}

    # 允许直接写工具全名 (solid.box ...)
    if verb in ALIASES:
        tool_name, preset = ALIASES[verb]
        args: Dict[str, Any] = dict(preset)
    elif registry.has(verb):
        tool_name, args = verb, {}
    else:
        raise ParseError(f"未知指令 '{verb}'. 输入 help 查看可用指令.")

    spec = registry.get(verb if verb in registry_names(registry) else tool_name)
    params = {p.name: p for p in spec.params}
    required = [p.name for p in spec.params if p.required]

    positionals: List[str] = []
    consume_flag = False
    for t in toks[1:]:
        if t == "consume" or t == "--consume":
            consume_flag = True
            continue
        if t == "->" :
            continue
        m = re.match(r"^->(.+)$", t)
        if m:  # ->name  → result/name
            _assign_result(tool_name, args, m.group(1))
            continue
        if "=" in t:
            k, v = t.split("=", 1)
            k = KEY_ALIASES.get(k.lower(), k.lower())
            if k not in params:
                raise ParseError(f"'{verb}' 无参数 '{k}'. 该工具参数: {', '.join(params)}")
            args[k] = _coerce(v, params[k].type)
        else:
            positionals.append(t)

    if consume_flag and "consume" in params:
        args["consume"] = True

    # 处理 -> 之后另起的 token (shlex 已拆), 以及布尔的 result
    # 位置参数: 依次填入尚未给定的必填参数
    remaining = [r for r in required if r not in args]
    for val, pname in zip(positionals, remaining):
        args[pname] = _coerce(val, params[pname].type)
    extra = positionals[len(remaining):]
    if extra:
        # 余下位置参数视作可选 name (基本图元) 之类
        if "name" in params and "name" not in args:
            args["name"] = extra[0]
        elif "result" in params and "result" not in args:
            args["result"] = extra[0]

    # 便利: 平移三分量缺省补 0 (move base dz=6 即只沿 Z 移)
    if tool_name == "solid.translate":
        for k in ("dx", "dy", "dz"):
            args.setdefault(k, 0.0)

    missing = [r for r in required if r not in args]
    if missing:
        raise ParseError(f"'{verb}' 缺少必填参数: {', '.join(missing)}")
    return {"kind": "tool", "tool": tool_name, "args": args}


def _assign_result(tool_name: str, args: Dict[str, Any], val: str) -> None:
    if tool_name == "solid.boolean":
        args["result"] = val
    else:
        args["name"] = val


def registry_names(registry) -> set:
    try:
        return set(registry.names())
    except Exception:
        return set()


def help_text(registry) -> str:
    lines = ["DaoAgent 指令 (道法自然 · 顺工具契约而行):",
             "  造形  box/cube · cyl · sphere · cone · torus   例: box 60 40 6 base | cyl r=3 h=20 at=15,20,3 name=h1",
             "  布尔  union/diff/intersect a b -> 结果 [consume] 例: diff flange h1 -> flange consume",
             "  变换  move 名 dz=6 · rotate 名 angle=45 axis=0,0,1",
             "  特征  fillet 名 r=2 · chamfer 名 d=1",
             "  度量  measure 名 [to=另一名]   感知 perceive/see 名",
             "  场景  list · clear · delete 名 · rename 旧 新 · export 名 path=...",
             "  会话  undo · redo · help",
             "已装载工具: " + ", ".join(sorted(registry_names(registry)))]
    return "\n".join(lines)
