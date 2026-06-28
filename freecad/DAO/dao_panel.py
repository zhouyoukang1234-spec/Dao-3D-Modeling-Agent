"""DAO AI dock panel — the human/AI conversation surface, living inside FreeCAD.

A dockable Qt widget added to FreeCAD's own main window so it persists across
every workbench. The human types intent (or clicks a quick chip); the engine
turns it into real operations on the live document, which appear instantly in
FreeCAD's native 3D view and tree. The human keeps full manual control at all
times — this panel only ever *adds* to the same shared document and undo stack.
"""
import json

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtCore, QtWidgets

import dao_agent
from dao_engine import DAOEngine

DOCK_NAME = "DAO_AI_Panel"

_QUICK = [
    "box 40x30x10 name plate",
    "cylinder r=6 h=40 name hole",
    "cut hole from plate",
    "fillet plate radius 2",
    "polar pattern lug count 6",
    "measure plate",
    "perceive",
    "assembly demo",
    "solve press_fit",
    "solve safe_fillet",
    "solve bolt_circle",
    "solve bearing_block",
    "solve l_bracket",
    "solve pin_joint",
    "solve gear_pair",
    "solve hinge",
    "list objects",
    "reset",
]

# A self-contained multi-part assembly, driven entirely by direct tool calls —
# proof the agent can compose complex builds (parts + container + links + BOM).
_ASM_DEMO = json.dumps([
    {"tool": "solid.box", "args": {"name": "base", "length": 80, "width": 80, "height": 8}},
    {"tool": "solid.cylinder", "args": {"name": "post", "radius": 6, "height": 50}},
    {"tool": "solid.cylinder", "args": {"name": "cap", "radius": 12, "height": 6}},
    {"tool": "asm.create", "args": {"name": "Rig"}},
    {"tool": "asm.add", "args": {"name": "plate", "body": "base", "fixed": True}},
    {"tool": "asm.add", "args": {"name": "col", "body": "post", "placement": {"pos": [40, 40, 8]}}},
    {"tool": "asm.add", "args": {"name": "top", "body": "cap", "placement": {"pos": [40, 40, 58]}}},
    {"tool": "asm.bom", "args": {}},
])


class DAOPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(DAOPanel, self).__init__(parent)
        self.engine = DAOEngine()
        self._build_ui()
        self._say("dao", "道法自然。我已接入当前 FreeCAD 文档。"
                          "用中文/英文描述你的意图，或点下方快捷指令；"
                          "你手动建的对象我也能引用，AI 的每步都可 Ctrl+Z 撤销。")

    # -- ui ----------------------------------------------------------------- #
    def _build_ui(self):
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet(
            "QTextEdit{background:#10141b;color:#d7dde6;border:1px solid #2a3340;"
            "font-family:Consolas,monospace;font-size:12px;}")
        lay.addWidget(self.log, 1)

        chips = QtWidgets.QGridLayout()
        chips.setSpacing(4)
        for i, c in enumerate(_QUICK):
            b = QtWidgets.QPushButton(c)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.setStyleSheet(
                "QPushButton{background:#1b2430;color:#cfe2ff;border:1px solid #2f3d4f;"
                "border-radius:10px;padding:3px 8px;font-size:11px;}"
                "QPushButton:hover{background:#243246;}")
            b.clicked.connect(lambda _=False, t=c: self._run(t))
            chips.addWidget(b, i // 2, i % 2)
        lay.addLayout(chips)

        row = QtWidgets.QHBoxLayout()
        self.input = QtWidgets.QLineEdit()
        self.input.setPlaceholderText("例如：box 20x10x5 / cut hole from plate / fillet it radius 2")
        self.input.setStyleSheet(
            "QLineEdit{background:#0d1117;color:#e6edf3;border:1px solid #2f3d4f;"
            "border-radius:6px;padding:6px;}")
        self.input.returnPressed.connect(self._send)
        send = QtWidgets.QPushButton("发送")
        send.setStyleSheet(
            "QPushButton{background:#2563eb;color:white;border:none;border-radius:6px;"
            "padding:6px 14px;font-weight:bold;}QPushButton:hover{background:#1d4ed8;}")
        send.clicked.connect(self._send)
        row.addWidget(self.input, 1)
        row.addWidget(send)
        lay.addLayout(row)

    # -- chat --------------------------------------------------------------- #
    def _say(self, who, text):
        color = {"you": "#7dd3fc", "dao": "#a7f3d0", "err": "#fca5a5",
                 "sys": "#94a3b8"}.get(who, "#d7dde6")
        label = {"you": "你", "dao": "DAO", "err": "错误", "sys": "·"}.get(who, who)
        self.log.append(
            '<span style="color:%s"><b>%s</b> &nbsp;%s</span>' % (color, label, text))
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _send(self):
        text = self.input.text().strip()
        if text:
            self.input.clear()
            self._run(text)

    def _run(self, text):
        self._say("you", text)
        low = text.strip().lower()
        if low in ("ops", "tools", "能力", "工具"):
            self._say("dao", "%d tools: %s" % (len(self.engine.ops()),
                                               ", ".join(self.engine.ops())))
            return
        if low.startswith(("solve ", "目标 ", "自主 ")):
            self._solve(low.split(None, 1)[1].strip())
            return
        intent = dao_agent.resolve_goal_intent(text)
        if intent is not None:
            name, overrides = intent
            if overrides:
                self._say("dao", "识别到目标意图：%s，参数 %s" % (name, _fmt_params(overrides)))
            else:
                self._say("dao", "识别到目标意图：%s" % name)
            self._solve(name, **overrides)
            return
        if low in ("assembly demo", "装配演示", "asm demo"):
            text = _ASM_DEMO
        had_objects = bool(App.ActiveDocument and App.ActiveDocument.Objects)
        try:
            note, results = self.engine.run(text)
        except Exception as exc:
            self._say("err", "engine: %s" % exc)
            return
        if not results:
            self._say("err", note)
            return
        if note:
            self._say("dao", note)
        for r in results:
            if not r.get("ok"):
                self._say("err", "%s ✗ %s" % (r["tool"], r.get("error")))
                continue
            data = r.get("data", {})
            self._say("sys", "%s → %s" % (r["tool"], _fmt(data)))
            self._maybe_show_perception(r["tool"], data)
        self._refresh_view(fit=not had_objects)

    def _solve(self, goal, **overrides):
        """Run the autonomous closed loop on a goal, narrating each iteration so the
        human watches the model self-correct in the live 3D view."""
        self._say("dao", "自主闭环求解目标 <b>%s</b>：建模 → 感知 → 验证 → 自纠 → 循环" % goal)
        first = {"v": True}

        def on_iter(step):
            verdict = "✓ 通过" if step["passed"] else ("✗ " + ", ".join(step["failed"]))
            self._say("sys", "iter %d  %s  → %s"
                      % (step["iter"], _fmt_params(step["params"]), verdict))
            for c in step["checks"]:
                if not c["ok"]:
                    self._say("sys", "&nbsp;&nbsp;· %s: %s" % (c["name"], _fmt_check(c)))
            self._refresh_view(fit=first["v"])
            first["v"] = False

        try:
            res = self.engine.solve(goal, on_iteration=on_iter, **overrides)
        except Exception as exc:
            self._say("err", "solve: %s" % exc)
            return
        if res.get("error"):
            self._say("err", "%s（可用：%s）" % (res["error"], ", ".join(res.get("available", []))))
            return
        tag = "达成" if res["solved"] else "未达成（预算用尽）"
        self._say("dao", "目标 <b>%s</b> %s，共 %d 次迭代；最终参数 %s"
                  % (goal, tag, res["iterations"], _fmt_params(res["final_params"])))
        try:
            per = self.engine.perceive({})
            self._maybe_show_perception("gui.perceive", per)
        except Exception:
            pass

    def _maybe_show_perception(self, tool, data):
        """Render scene summary + embed the captured viewport image in the log."""
        scene = data.get("scene") if isinstance(data, dict) else None
        if scene:
            self._say("dao", _scene_summary(scene))
            sel = data.get("selection") or {}
            if sel.get("count"):
                self._say("dao", "human selection: %s" % _fmt_selection(sel))
        snap = data.get("snapshot") if isinstance(data, dict) else None
        path = (snap or {}).get("path") if isinstance(snap, dict) else None
        if not path and tool == "gui.snapshot":
            path = data.get("path")
        if path:
            url = QtCore.QUrl.fromLocalFile(path).toString()
            self.log.append(
                '<img src="%s" width="320" '
                'style="border:1px solid #2a3340;margin:4px 0;">' % url)
            self.log.verticalScrollBar().setValue(
                self.log.verticalScrollBar().maximum())

    def _refresh_view(self, fit=False):
        doc = App.ActiveDocument
        if doc:
            doc.recompute()
        Gui.updateGui()
        # Only fit on the first object so we never hijack the human's camera.
        if fit:
            try:
                Gui.SendMsgToActiveView("ViewFit")
            except Exception:
                pass


_PARAM_KEYS = ("pin_r", "shaft_r", "bore_r", "hole_r", "radius", "bcr", "n")


def _fmt_params(p):
    if not isinstance(p, dict):
        return str(p)
    bits = ["%s=%s" % (k, p[k]) for k in _PARAM_KEYS if k in p]
    return ", ".join(dict.fromkeys(bits)) or ", ".join(
        "%s=%s" % (k, v) for k, v in list(p.items())[:3])


def _fmt_check(c):
    out = c.get("name", "")
    if "measured" in c:
        out = "measured=%s" % (c["measured"],)
    if "target" in c:
        out += " target=%s" % (c["target"],)
    if "detail" in c and c["detail"]:
        out += " %s" % (c["detail"],)
    return out


def _fmt(data):
    if not isinstance(data, dict):
        return str(data)
    keep = ("volume", "area", "faces", "edges", "value", "document", "count",
            "objects", "interfering", "mass", "assembly", "component", "linked",
            "solved", "grounded", "line_items", "component_count", "path",
            "problems", "placement", "view")
    bits = []
    for k in keep:
        if k in data:
            v = data[k]
            if isinstance(v, (list, dict)):
                v = "[%d]" % len(v)
            bits.append("%s=%s" % (k, v))
    return ", ".join(bits) if bits else "ok"


def _scene_summary(scene):
    n = scene.get("count", 0)
    bb = scene.get("bbox")
    span = ("span=%s" % bb["dims"]) if bb else ""
    errs = scene.get("errors") or []
    head = "scene: %d object(s) %s" % (n, span)
    if errs:
        head += " · errors: %s" % ", ".join(errs)
    parts = []
    for o in scene.get("objects", [])[:6]:
        d = o.get("bbox", {}).get("dims") if o.get("bbox") else None
        vol = o.get("volume")
        tag = o.get("label") or o.get("name")
        seg = tag
        if d:
            seg += " %gx%gx%g" % tuple(d)
        if vol is not None:
            seg += " V=%g" % vol
        if o.get("visible") is False:
            seg += " (hidden)"
        parts.append(seg)
    if parts:
        head += "<br>&nbsp;&nbsp;" + "<br>&nbsp;&nbsp;".join(parts)
    return head


def _fmt_selection(sel):
    out = []
    for s in sel.get("selected", []):
        sub = ("/" + ",".join(s["subs"])) if s.get("subs") else ""
        out.append("%s%s" % (s.get("label") or s.get("object"), sub))
    return "; ".join(out)


def ensure_panel():
    """Create the dock once and show it; re-show if already created."""
    mw = Gui.getMainWindow()
    if mw is None:
        return None
    existing = mw.findChild(QtWidgets.QDockWidget, DOCK_NAME)
    if existing is not None:
        existing.show()
        existing.raise_()
        return existing
    dock = QtWidgets.QDockWidget("DAO · AI 工作台", mw)
    dock.setObjectName(DOCK_NAME)
    dock.setWidget(DAOPanel(dock))
    dock.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)
    mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
    dock.show()
    dock.raise_()
    return dock
