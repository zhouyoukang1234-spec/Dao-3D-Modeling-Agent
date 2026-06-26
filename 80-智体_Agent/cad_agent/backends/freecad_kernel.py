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

纯几何尽在同源的 freecad_ops 模块 (无头/在世两路共用, 一字不二). 本文件只管
stdin/stdout 行协议. 由 freecad_backend.py 以子进程方式拉起, 与 freecad_ops.py
一并暂存到纯 ASCII 临时路径后用 freecadcmd 执行 (规避中文路径 argv 乱码).
"""
import sys
import json

import freecad_ops as ops  # 同目录暂存; 脚本所在目录在 sys.path[0]

RESP = "__FCR__ "  # 应答行哨兵; 反扫此前缀即可越过 FreeCAD 启动横幅与杂讯


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
            data = ops.op(req["op"], req.get("args", {}))
            resp = {"ok": True, "data": data}
        except Exception as e:  # noqa: BLE001 — 边界归一
            import traceback
            resp = {"ok": False, "error": "%s: %s" % (type(e).__name__, e),
                    "trace": traceback.format_exc(limit=3)}
        sys.stdout.write(RESP + json.dumps(resp) + "\n")
        sys.stdout.flush()


# freecadcmd 执行本文件时 __name__ 未必为 "__main__"; 故直接调用.
main()
