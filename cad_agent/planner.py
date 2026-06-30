"""Natural-language -> tool-call planner for the web workspace.

Turns a human chat line into an ordered list of ``{"tool", "args"}`` steps that
the server executes against the live FreeCAD kernel. It is intentionally
*engine-agnostic and FreeCAD-free* (pure text -> plan) so it can be unit-tested
in CI without FreeCAD installed.

Three input styles are accepted, in priority order:

1. **Raw plan** — a JSON object/array of ``{"tool", "args"}`` steps, executed
   verbatim (power users / other agents).
2. **Direct tool call** — ``solid.box {"name": "x", "length": 10, ...}``.
3. **Natural language** — ``box 20x10x5``, ``cut hole from plate``,
   ``fillet part radius 2``, ``polar pattern lug count 6`` ... (this module).

An optional LLM hook (:func:`llm_plan`) can supersede the deterministic grammar
when an API key is configured; the grammar remains the always-available default
so the workspace works with zero external dependencies.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Plan:
    steps: List[Dict[str, Any]] = field(default_factory=list)
    note: str = ""
    error: Optional[str] = None

    def add(self, tool: str, **args: Any) -> "Plan":
        self.steps.append({"tool": tool, "args": args})
        return self


_NUM = r"[-+]?\d*\.?\d+"


def _nums(text: str) -> List[float]:
    return [float(x) for x in re.findall(_NUM, text)]


def _kv(text: str, *keys: str) -> Optional[float]:
    """Find ``key = value`` / ``key value`` / ``key:value`` for any alias."""
    for k in keys:
        m = re.search(r"\b%s\s*[=:]?\s*(%s)" % (k, _NUM), text)
        if m:
            return float(m.group(1))
    return None


class Planner:
    """Stateful planner: auto-names objects and resolves 'it' to the last one."""

    def __init__(self) -> None:
        self._n = 0
        self.last_name: Optional[str] = None

    def _name(self, prefix: str, text: str) -> str:
        m = re.search(r"\b(?:name|call(?:ed)?|as)\s+([A-Za-z_]\w*)", text)
        if m:
            name = m.group(1)
        else:
            self._n += 1
            name = "%s%d" % (prefix, self._n)
        self.last_name = name
        return name

    def _ref(self, token: Optional[str]) -> Optional[str]:
        if token in (None, "it", "this", "that", "current"):
            return self.last_name
        return token

    # ------------------------------------------------------------------ #
    def plan(self, text: str) -> Plan:
        raw = text.strip()
        if not raw:
            return Plan(error="empty input")

        # (1) raw JSON plan
        if raw[0] in "[{":
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError as exc:
                return Plan(error="invalid JSON plan: %s" % exc)
            steps = obj if isinstance(obj, list) else [obj]
            norm = []
            for s in steps:
                if "tool" not in s:
                    return Plan(error="each step needs a 'tool' field")
                norm.append({"tool": s["tool"], "args": s.get("args", {})})
            return Plan(steps=norm, note="raw plan (%d step(s))" % len(norm))

        # (2) direct tool call: "<group.op> {json}"
        m = re.match(r"^([a-z_]+\.[a-z_]+)\s+(\{.*\})\s*$", raw, re.S)
        if m:
            try:
                args = json.loads(m.group(2))
            except json.JSONDecodeError as exc:
                return Plan(error="invalid args JSON: %s" % exc)
            return Plan(steps=[{"tool": m.group(1), "args": args}],
                        note="direct call %s" % m.group(1))

        # (3) natural language
        return self._nl(raw)

    # ------------------------------------------------------------------ #
    def _nl(self, raw: str) -> Plan:
        t = raw.lower()

        # housekeeping
        if re.search(r"\b(reset|clear|new document|start over|clean)\b", t):
            self._n = 0
            self.last_name = None
            return Plan(steps=[{"tool": "__reset__", "args": {}}], note="reset the document")
        if re.search(r"\b(list|what.*objects|show objects|tree)\b", t):
            return Plan(steps=[{"tool": "solid.list", "args": {}}], note="list solids")
        if re.search(r"\b(look|perceive|observe|snapshot|capture|inspect scene|see)\b", t) \
                or re.search(r"观察|感知|看一?看|快照", raw):
            return Plan(steps=[{"tool": "gui.perceive", "args": {}}],
                        note="perceive the live scene")
        if re.search(r"\brender|screenshot|picture|image\b", t):
            return Plan(steps=[{"tool": "view.scene", "args": {}}], note="refresh view")

        # search-before-model: an explicit retrieval intent reaches for the
        # world's libraries instead of building from primitives -- the agent's
        # orchestration policy made concrete (when to fetch vs when to model).
        rp = self._resource(t, raw)
        if rp:
            return rp

        # delete
        m = re.search(r"\b(?:delete|remove|drop)\s+([A-Za-z_]\w*)", t)
        if m:
            return Plan(steps=[{"tool": "solid.delete", "args": {"name": m.group(1)}}],
                        note="delete %s" % m.group(1))

        # save / export
        m = re.search(r"\b(?:save|export)\b.*?([A-Za-z]:\\[^\s]+|/[^\s]+|[\w.-]+\.\w+)", raw)
        if re.search(r"\bsave\b", t) and m:
            return Plan(steps=[{"tool": "doc.save", "args": {"path": m.group(1)}}],
                        note="save document")
        if re.search(r"\bexport\b", t) and m:
            return Plan(steps=[{"tool": "solid.export", "args": {"path": m.group(1)}}],
                        note="export geometry")

        # measure / inspect
        m = re.search(r"\b(?:measure|inspect|properties of|how big is)\s+([A-Za-z_]\w*)?", t)
        if m:
            name = self._ref(m.group(1))
            if name:
                tool = "solid.inspect" if "inspect" in t else "solid.measure"
                return Plan(steps=[{"tool": tool, "args": {"name": name}}],
                            note="measure %s" % name)

        # booleans: "cut B from A" / "subtract", "union A and B", "intersect"
        bp = self._boolean(t)
        if bp:
            return bp

        # transforms
        tp = self._transform(t)
        if tp:
            return tp

        # fillet / chamfer
        dp = self._dressup(t)
        if dp:
            return dp

        # patterns
        pp = self._pattern(t)
        if pp:
            return pp

        # primitives (may emit several)
        prim = self._primitives(t)
        if prim.steps:
            return prim

        return Plan(error=(
            "I couldn't parse that. Try e.g. 'box 20x10x5', 'cylinder r=5 h=20', "
            "'cut hole from plate', 'fillet it radius 2', 'polar pattern lug count 6', "
            "or a direct call like solid.box {\"name\":\"b\",\"length\":10,\"width\":10,\"height\":10}."))

    # ------------------------------------------------------------------ #
    # the searchable libraries the planner knows how to name-route to.
    _PLATFORMS = {
        "printables": "printables", "prusa": "printables",
        "sketchfab": "sketchfab", "thingiverse": "thingiverse",
        "grabcad": "grabcad", "nasa": "nasa", "github": "github",
    }
    # words that signal "this is a retrieval, not a modelling, request".
    _FETCH_STRONG = (r"\b(?:search|download|fetch|browse)\b", r"搜索|下载|检索|查找模型")
    _FETCH_WEAK = (r"\b(?:find|look\s+for|locate|grab|get\s+me)\b",
                   r"查找|找一?个|来一?个|需要|想要|要一?个")
    _RES_HINT = (r"\b(?:online|on\s+the\s+web|internet|librar(?:y|ies)|repositor(?:y|ies)|"
                 r"catalog|printables|sketchfab|thingiverse|grabcad|nasa|github|"
                 r"existing|ready[- ]made|off[- ]the[- ]shelf|standard\s+part)\b",
                 r"网上|在线|资源库|模型库|现成|标准件")

    def _resource(self, t: str, raw: str) -> Optional[Plan]:
        strong = any(re.search(p, x) for p, x in zip(self._FETCH_STRONG, (t, raw)))
        weak = any(re.search(p, x) for p, x in zip(self._FETCH_WEAK, (t, raw)))
        hint = any(re.search(p, x) for p, x in zip(self._RES_HINT, (t, raw)))
        # strong verbs (search/download) route on their own; ambiguous verbs
        # (find/get) only route when a library hint confirms the intent, so
        # "find the volume" keeps measuring rather than searching the web.
        if not (strong or (weak and hint)):
            return None

        # which platform(s)? a named library narrows the search; else default.
        # plain substring (no \b): \b never fires between a CJK char and Latin,
        # so "在printables上" must still detect the platform.
        platforms = sorted({self._PLATFORMS[k] for k in self._PLATFORMS if k in t})

        # an explicit "download <platform> id <x>" pulls a concrete model.
        dm = re.search(r"\b(?:download|fetch|get)\b.*?\bid\s*[=:]?\s*([\w./-]+)", t)
        if dm and re.search(r"\b(?:download|fetch)\b", t):
            plat = platforms[0] if platforms else "printables"
            return Plan(steps=[{"tool": "resource.download",
                                "args": {"platform": plat, "id": dm.group(1)}}],
                        note="download %s/%s" % (plat, dm.group(1)))

        # otherwise it is a search. Distil the query: drop the retrieval verbs,
        # the library hints and filler so only the *thing wanted* remains.
        query = raw
        strip = [
            # compound phrases first, before single-word filler eats their guts
            # (e.g. the "the" inside "off-the-shelf").
            r"\b(?:off[- ]the[- ]shelf|ready[- ]made|standard\s+part)\b",
            r"\b(?:can\s+you|could\s+you|i\s+(?:want|need|would\s+like)\s+to)\b",
            r"\bon\s+the\s+(?:web|internet)\b",
            r"\b(?:on|from|in)\s+(?:printables|sketchfab|thingiverse|grabcad|nasa|github)\b",
            r"\b(?:search|download|fetch|browse|find|look\s+for|locate|grab|get\s+me|get)\b",
            r"\b(?:please|existing|online|internet|web)\b",
            r"\b(?:librar(?:y|ies)|repositor(?:y|ies)|catalog)\b",
            r"\b(?:for|me|a|an|the|some|any)\b",
            r"\b(?:3d\s+)?(?:model|models|part|parts|stl|step|file|files)\b",
            r"我|你|的|在|上|搜索|下载|检索|查找|找一?个|来一?个|需要|想要|要一?个|"
            r"一个|一只|网上|在线|资源库|模型库|现成|标准件|模型|文件",
            # platform names last & boundary-free: \b never fires inside CJK
            # context like "在printables上", so strip them plainly.
            r"(?i)printables|sketchfab|thingiverse|grabcad|nasa|github",
        ]
        for pat in strip:
            query = re.sub(pat, " ", query, flags=re.I)
        query = re.sub(r"\s+", " ", query).strip(" ,.;:")
        if not query:
            query = raw.strip()
        args: Dict[str, Any] = {"query": query}
        if platforms:
            args["platforms"] = platforms
        return Plan(steps=[{"tool": "resource.search", "args": args}],
                    note="search libraries for %r%s"
                    % (query, " on %s" % ", ".join(platforms) if platforms else ""))

    # ------------------------------------------------------------------ #
    def _primitives(self, t: str) -> Plan:
        p = Plan()
        # box: "box 20x10x5" or "box length 20 width 10 height 5"
        if re.search(r"\b(box|cube|block|plate|盒|方块)\b", t):
            dims = re.search(r"(%s)\s*[x×*]\s*(%s)\s*[x×*]\s*(%s)" % (_NUM, _NUM, _NUM), t)
            if dims:
                length, width, height = (float(dims.group(i)) for i in (1, 2, 3))
            else:
                length = _kv(t, "length", "l") or 10
                width = _kv(t, "width", "w") or 10
                height = _kv(t, "height", "h", "thickness", "thick") or 10
            name = self._name("box", t)
            p.add("solid.box", name=name, length=length, width=width, height=height)
            p.note = "box %gx%gx%g -> %s" % (length, width, height, name)
            return p
        # cylinder
        if re.search(r"\b(cylinder|cyl|rod|pin|圆柱)\b", t):
            r = _kv(t, "radius", "r") or (_kv(t, "diameter", "dia", "d") or 0) / 2 or 5
            h = _kv(t, "height", "h", "length", "l") or 10
            name = self._name("cyl", t)
            p.add("solid.cylinder", name=name, radius=r, height=h)
            p.note = "cylinder r=%g h=%g -> %s" % (r, h, name)
            return p
        # sphere / ball
        if re.search(r"\b(sphere|ball|球)\b", t):
            r = _kv(t, "radius", "r") or (_nums(t)[0] if _nums(t) else 5)
            name = self._name("sph", t)
            p.add("solid.sphere", name=name, radius=r)
            p.note = "sphere r=%g -> %s" % (r, name)
            return p
        # cone
        if re.search(r"\b(cone|taper|锥)\b", t):
            n = _nums(t)
            r1 = _kv(t, "radius1", "r1", "bottom") or (n[0] if len(n) > 0 else 8)
            r2 = _kv(t, "radius2", "r2", "top") or (n[1] if len(n) > 1 else 0)
            h = _kv(t, "height", "h") or (n[2] if len(n) > 2 else 12)
            name = self._name("cone", t)
            p.add("solid.cone", name=name, radius1=r1, radius2=r2, height=h)
            p.note = "cone r1=%g r2=%g h=%g -> %s" % (r1, r2, h, name)
            return p
        # torus / ring
        if re.search(r"\b(torus|ring|donut|环)\b", t):
            n = _nums(t)
            r1 = _kv(t, "radius1", "r1", "major") or (n[0] if len(n) > 0 else 10)
            r2 = _kv(t, "radius2", "r2", "minor") or (n[1] if len(n) > 1 else 3)
            name = self._name("torus", t)
            p.add("solid.torus", name=name, radius1=r1, radius2=r2)
            p.note = "torus R=%g r=%g -> %s" % (r1, r2, name)
            return p
        return p

    def _boolean(self, t: str) -> Optional[Plan]:
        # "cut/subtract X from Y" -> Y - X
        m = re.search(r"\b(?:cut|subtract|remove)\s+([A-Za-z_]\w*)\s+(?:from|out of)\s+([A-Za-z_]\w*)", t)
        if m:
            tool_b, base = m.group(1), m.group(2)
            return Plan(steps=[{"tool": "solid.cut", "args": {"a": base, "b": tool_b, "out": base}}],
                        note="cut %s from %s" % (tool_b, base))
        m = re.search(r"\b(?:union|fuse|join|combine|merge|add)\s+([A-Za-z_]\w*)\s+(?:and|with|to|\+)\s+([A-Za-z_]\w*)", t)
        if m:
            a, b = m.group(1), m.group(2)
            return Plan(steps=[{"tool": "solid.union", "args": {"a": a, "b": b, "out": a}}],
                        note="union %s + %s" % (a, b))
        m = re.search(r"\b(?:intersect|common|overlap)\s+([A-Za-z_]\w*)\s+(?:and|with|\&)\s+([A-Za-z_]\w*)", t)
        if m:
            a, b = m.group(1), m.group(2)
            return Plan(steps=[{"tool": "solid.common", "args": {"a": a, "b": b, "out": a}}],
                        note="intersect %s & %s" % (a, b))
        m = re.search(r"\b(?:interference|clash|collision)\b.*?([A-Za-z_]\w*)\s+(?:and|with|vs)\s+([A-Za-z_]\w*)", t)
        if m:
            return Plan(steps=[{"tool": "solid.interference",
                                "args": {"a": m.group(1), "b": m.group(2)}}],
                        note="interference %s vs %s" % (m.group(1), m.group(2)))
        return None

    def _transform(self, t: str) -> Optional[Plan]:
        m = re.search(r"\b(?:move|translate|shift)\s+([A-Za-z_]\w*)?\s*(?:by)?\s*"
                      r"(%s)[,\s]+(%s)[,\s]+(%s)" % (_NUM, _NUM, _NUM), t)
        if m:
            name = self._ref(m.group(1))
            vec = [float(m.group(i)) for i in (2, 3, 4)]
            return Plan(steps=[{"tool": "solid.translate", "args": {"name": name, "vector": vec}}],
                        note="move %s by %s" % (name, vec))
        m = re.search(r"\brotate\s+([A-Za-z_]\w*)?\s*(?:by)?\s*(%s)\s*(?:deg|degrees|°)?\s*"
                      r"(?:about|around|on)?\s*([xyz])?" % _NUM, t)
        if m:
            name = self._ref(m.group(1))
            angle = float(m.group(2))
            axis = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}.get(m.group(3) or "z")
            return Plan(steps=[{"tool": "solid.rotate",
                                "args": {"name": name, "axis": axis, "angle": angle}}],
                        note="rotate %s %g° about %s" % (name, angle, m.group(3) or "z"))
        m = re.search(r"\bmirror\s+([A-Za-z_]\w*)?\s*(?:about|across|over)?\s*([xyz])?\s*(?:plane|axis)?", t)
        if m and "mirror" in t:
            name = self._ref(m.group(1))
            normal = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}.get(m.group(2) or "x")
            return Plan(steps=[{"tool": "solid.mirror", "args": {"name": name, "normal": normal}}],
                        note="mirror %s" % name)
        return None

    def _dressup(self, t: str) -> Optional[Plan]:
        m = re.search(r"\bfillet\s+([A-Za-z_]\w*)?", t)
        if m and "fillet" in t:
            name = self._ref(m.group(1))
            r = _kv(t, "radius", "r") or (_nums(t)[-1] if _nums(t) else 1)
            return Plan(steps=[{"tool": "solid.fillet", "args": {"name": name, "radius": r}}],
                        note="fillet %s r=%g" % (name, r))
        m = re.search(r"\bchamfer\s+([A-Za-z_]\w*)?", t)
        if m and "chamfer" in t:
            name = self._ref(m.group(1))
            s = _kv(t, "size", "distance", "d") or (_nums(t)[-1] if _nums(t) else 1)
            return Plan(steps=[{"tool": "solid.chamfer", "args": {"name": name, "size": s}}],
                        note="chamfer %s size=%g" % (name, s))
        return None

    def _pattern(self, t: str) -> Optional[Plan]:
        if "polar" in t or "circular" in t or "radial" in t:
            m = re.search(r"\b(?:pattern|array)\s+([A-Za-z_]\w*)", t) or \
                re.search(r"\b([A-Za-z_]\w*)\b", t)
            name = self._ref(m.group(1) if m else None)
            count = int(_kv(t, "count", "number", "n", "copies") or 6)
            angle = _kv(t, "angle") or 360
            return Plan(steps=[{"tool": "solid.pattern_polar",
                                "args": {"name": name, "count": count, "angle": angle}}],
                        note="polar pattern %s x%d" % (name, count))
        if "pattern" in t or "array" in t or "linear" in t:
            m = re.search(r"\b(?:pattern|array)\s+([A-Za-z_]\w*)", t)
            name = self._ref(m.group(1) if m else None)
            count = int(_kv(t, "count", "number", "n", "copies") or 3)
            step = re.search(r"step\s+(%s)[,\s]+(%s)[,\s]+(%s)" % (_NUM, _NUM, _NUM), t)
            vec = [float(step.group(i)) for i in (1, 2, 3)] if step else [10, 0, 0]
            return Plan(steps=[{"tool": "solid.pattern_linear",
                                "args": {"name": name, "count": count, "step": vec}}],
                        note="linear pattern %s x%d" % (name, count))
        return None


# ---------------------------------------------------------------------------- #
# Optional LLM hook (used only when an API key is configured)
# ---------------------------------------------------------------------------- #
def llm_available() -> bool:
    import os
    return bool(os.environ.get("OPENAI_API_KEY") or os.environ.get("ANTHROPIC_API_KEY"))
