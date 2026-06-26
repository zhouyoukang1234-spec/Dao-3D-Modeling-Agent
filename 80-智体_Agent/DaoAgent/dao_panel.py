#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dao_panel.py — DaoAgent 对话停靠面板 (FreeCAD 之"框", 如 Cursor 之聊天侧栏)
═══════════════════════════════════════════════════════════════════════════════
道法自然 · 在成熟的 FreeCAD GUI 上挂一层"对话管理系统"——用户在框里说一句, 智体即
在 *当前活文档* 上 perceive→act→verify, 所建之物即时现于模型树与三维视图.

本质与"AI 写代码"同构: 把一行意图解析成一次工具调用 (dao_interpreter), 交由引擎无关
的 AgentSession 执行 (session), 落到 FreeCAD 活文档 (freecad_live). 底层形式从"改代码"
变为"改模型", 范式不变.
"""
from __future__ import annotations

import html
import os
import tempfile
import traceback
from typing import Any, Dict, List, Tuple

from PySide2 import QtCore, QtGui, QtWidgets

import FreeCAD as App
import FreeCADGui as Gui

import dao_interpreter as di


# ═══════════════════════════════════════════════════════════════════════════
# 控制器: 把对话行接到智体会话 (活文档后端)
# ═══════════════════════════════════════════════════════════════════════════
class DaoController:
    def __init__(self) -> None:
        from cad_agent.tools import ToolRegistry
        from cad_agent.backends.freecad_live import register_freecad_live_tools, LiveBridge
        from cad_agent.session import AgentSession

        self.bridge = LiveBridge(doc_name="DaoAgent")
        self.registry = ToolRegistry()
        register_freecad_live_tools(self.registry, bridge=self.bridge)
        self.session = AgentSession(name="dao", registry=self.registry)
        self._png_dir = os.path.join(tempfile.gettempdir(), "dao_agent_views")
        os.makedirs(self._png_dir, exist_ok=True)
        # 触发活文档建立, 随即闭其 3D 视图 (无 GPU VM 上 llvmpipe 视口会持续重绘风暴)
        _ = self.bridge.doc
        self._close_gl_views()

    # —— 对外: 处理一行, 返回 (消息列表, 可选预览PNG路径) ——
    def handle(self, line: str) -> Tuple[List[Tuple[str, str]], str]:
        msgs: List[Tuple[str, str]] = []
        png = ""
        try:
            parsed = di.parse(line, self.registry)
        except di.ParseError as e:
            return [("error", str(e))], png

        kind = parsed["kind"]
        if kind == "empty":
            return msgs, png
        if kind == "meta":
            return self._meta(parsed["verb"])

        tool, args = parsed["tool"], parsed["args"]
        res = self.session.act(tool, args)
        if not res.ok:
            return [("error", f"{tool} 失败: {res.error}")], png

        msgs.append(("agent", self._format(tool, res.data)))
        spec = self.registry.get(tool)

        if tool == "solid.perceive":
            png = self._save_iso(args["name"])
        if getattr(spec, "mutates", False):
            self._refresh_view()
        return msgs, png

    # —— 元动作 ——
    def _meta(self, verb: str) -> Tuple[List[Tuple[str, str]], str]:
        if verb == "help":
            return [("info", di.help_text(self.registry))], ""
        if verb == "undo":
            ok = self.session.undo()
            if not ok:
                return [("info", "无可撤销的步骤.")], ""
            rec = self.bridge.reconcile(self.session.workspace)
            self._refresh_view()
            tail = (f" (移除 {', '.join(rec['removed'])})" if rec["removed"] else "")
            return [("info", "已撤销." + tail)], ""
        if verb == "redo":
            return [("info", "redo 暂未启用 (当前仅支持 undo).")], ""
        return [("error", f"未知元动作 {verb}")], ""

    # —— 把工具结果格式化为可读文本 ——
    def _format(self, tool: str, data: Dict[str, Any]) -> str:
        if tool == "scene.list":
            if not data.get("objects"):
                return "活文档为空."
            rows = [f"  · {o['name']}: V={o.get('volume')} 水密={o.get('closed')} 尺寸={o.get('extents')}"
                    for o in data["objects"]]
            return f"活文档 {data['count']} 个对象:\n" + "\n".join(rows)
        if tool == "scene.clear":
            return f"已清空 {data.get('cleared', 0)} 个对象."
        if tool == "solid.measure":
            return ("度量 {name}: 体积={volume} 面积={area} 水密={closed} "
                    "实体={solids} 面={faces} 棱={edges}\n  包围盒={extents} 质心={centroid}"
                    ).format(**{k: data.get(k) for k in
                                ("name", "volume", "area", "closed", "solids", "faces", "edges",
                                 "extents", "centroid")})
        if tool == "solid.perceive":
            r = data.get("report", {})
            return (f"感知 {data['name']}:\n  {data.get('summary', '')}\n"
                    f"  BREP 精确: 体积={r.get('brep_volume')} 面积={r.get('brep_area')} "
                    f"水密={r.get('brep_closed')}")
        if tool == "solid.export":
            return f"已导出 {data.get('name')} → {data.get('path')} ({data.get('format', '')})"
        if tool in ("solid.delete",):
            return f"已删除 {data.get('deleted')}; 余: {data.get('remaining')}"
        if tool == "solid.rename":
            return f"已重命名为 {data.get('renamed_to')}"
        # 通用图元/布尔/变换/特征: 概要
        name = data.get("name") or data.get("result") or "?"
        bits = [f"{name}"]
        if data.get("op"):
            bits.append(f"[{data['op']}]")
        if data.get("volume") is not None:
            bits.append(f"V={data['volume']}")
        if data.get("extents"):
            bits.append(f"尺寸={data['extents']}")
        if data.get("closed") is not None:
            bits.append(f"水密={data['closed']}")
        return "✔ " + " ".join(bits)

    # —— 落地等轴测预览 PNG (智体"看见") ——
    def _save_iso(self, name: str) -> str:
        try:
            r = self.session.perceive(name, save_png=True, out_dir=self._png_dir, resolution=240)
            return r.data.get("renders", {}).get("iso", {}).get("png", "")
        except Exception:
            return ""

    # —— 变更后: 重算文档并闭其 3D 视图 (智体之"眼"改用 numpy 软栅, 见 perception) ——
    def _refresh_view(self) -> None:
        try:
            App.ActiveDocument.recompute()
        except Exception:
            pass
        self._close_gl_views()

    # —— 关闭所有 3D 视图与起始页, 规避无 GPU VM 上 llvmpipe 的持续重绘风暴 ——
    # 文档与模型树保留 (Qt 控件, 不经 OpenGL); 形之可见改由感知层 numpy 渲染.
    @staticmethod
    def _close_gl_views() -> None:
        try:
            mw = Gui.getMainWindow()
            if mw is None:
                return
            for mdi in mw.findChildren(QtWidgets.QMdiArea):
                mdi.closeAllSubWindows()
                for sub in list(mdi.subWindowList()):
                    sub.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
# 停靠面板 UI
# ═══════════════════════════════════════════════════════════════════════════
_ROLE_COLOR = {"you": "#1565c0", "agent": "#1b5e20", "error": "#b71c1c",
               "info": "#5d4037", "sys": "#455a64"}
_ROLE_LABEL = {"you": "你", "agent": "智体", "error": "✗", "info": "ℹ", "sys": "·"}


class DaoDock(QtWidgets.QDockWidget):
    _instance = None

    def __init__(self, parent=None) -> None:
        super().__init__("道 · DaoAgent", parent)
        self.setObjectName("DaoAgentDock")
        self.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)
        self._ctl = None  # 延迟构造 (首次发送时建活文档)

        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        self.log = QtWidgets.QTextBrowser()
        self.log.setOpenExternalLinks(False)
        self.log.setStyleSheet("font-size:12px;")
        lay.addWidget(self.log, 3)

        self.preview = QtWidgets.QLabel()
        self.preview.setAlignment(QtCore.Qt.AlignCenter)
        self.preview.setMinimumHeight(180)
        self.preview.setStyleSheet("background:#f5f5f8;border:1px solid #ddd;")
        self.preview.setText("智体之眼 · 感知预览")
        lay.addWidget(self.preview, 2)

        row = QtWidgets.QHBoxLayout()
        self.input = QtWidgets.QLineEdit()
        self.input.setPlaceholderText("说一句 (例: box 60 40 6 base | diff flange h1 -> flange consume | help)")
        self.input.returnPressed.connect(self._send)
        send = QtWidgets.QPushButton("发送")
        send.clicked.connect(self._send)
        row.addWidget(self.input, 1)
        row.addWidget(send)
        lay.addLayout(row)

        quick = QtWidgets.QHBoxLayout()
        for label, cmd in [("法兰示例", "__demo__"), ("列表", "list"),
                           ("撤销", "undo"), ("清空", "clear"), ("帮助", "help")]:
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, c=cmd: self._quick(c))
            quick.addWidget(b)
        lay.addLayout(quick)

        self.setWidget(w)
        self._post("sys", "道法自然 · 无为而无不为. 输入 help 查看指令; 或点「法兰示例」一键演示.")

    # —— 控制器懒构造 ——
    def controller(self) -> DaoController:
        if self._ctl is None:
            self._post("sys", "正在装载智体 (活文档后端) …")
            self._ctl = DaoController()
            self._post("sys", "智体就绪. 工具: " + ", ".join(sorted(self._ctl.registry.names())))
        return self._ctl

    # —— 发送 ——
    def _send(self) -> None:
        line = self.input.text().strip()
        if not line:
            return
        self.input.clear()
        self._post("you", line)
        self._dispatch(line)

    def _quick(self, cmd: str) -> None:
        if cmd == "__demo__":
            self._run_demo()
            return
        self._post("you", cmd)
        self._dispatch(cmd)

    def _dispatch(self, line: str) -> None:
        try:
            msgs, png = self.controller().handle(line)
        except Exception as e:
            self._post("error", f"{type(e).__name__}: {e}")
            App.Console.PrintError(traceback.format_exc())
            return
        for role, text in msgs:
            self._post(role, text)
        if png and os.path.exists(png):
            self._show_png(png)

    # —— 一键法兰演示 ——
    def _run_demo(self) -> None:
        plan = [
            "box 60 40 6 base",
            "box 60 6 30 rib at=30,3,15",
            "union base rib -> flange consume",
            "cyl r=3 h=20 at=15,20,3 name=h1",
            "cyl r=3 h=20 at=45,20,3 name=h2",
            "diff flange h1 -> flange consume",
            "diff flange h2 -> flange consume",
            "perceive flange",
        ]
        self._post("sys", "▶ 法兰示例: 底板 ∪ 立筋 − 2 孔 → 感知")
        for line in plan:
            self._post("you", line)
            self._dispatch(line)
            QtWidgets.QApplication.processEvents()

    # —— 渲染消息 ——
    def _post(self, role: str, text: str) -> None:
        color = _ROLE_COLOR.get(role, "#333")
        tag = _ROLE_LABEL.get(role, role)
        safe = html.escape(text).replace("\n", "<br>")
        self.log.append(
            f'<div style="margin:2px 0"><b style="color:{color}">{tag}</b> '
            f'<span style="color:{color}">{safe}</span></div>')
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _show_png(self, path: str) -> None:
        pix = QtGui.QPixmap(path)
        if pix.isNull():
            return
        self.preview.setPixmap(pix.scaled(self.preview.width(), self.preview.height(),
                                          QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    # —— 单例获取/显示 ——
    @classmethod
    def show_dock(cls) -> "DaoDock":
        mw = Gui.getMainWindow()
        if cls._instance is None:
            cls._instance = DaoDock(mw)
            mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, cls._instance)
        cls._instance.show()
        cls._instance.raise_()
        return cls._instance
