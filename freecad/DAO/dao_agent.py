"""DAO closed-loop agent — perceive · act · verify · self-correct · repeat.

This is the part that makes the fusion *autonomous* rather than a one-shot
command runner. Given a declarative **goal** (target geometry + measurable
acceptance checks), the agent:

    1. builds the geometry on the live document (one undo transaction),
    2. perceives + measures the result through the same op surface,
    3. verifies invariants (no error state, watertight solids) and the goal's
       own acceptance checks,
    4. if anything fails, *remediates* — it adjusts the design variables based on
       the measured error — and loops,

until every check passes or the iteration budget is spent. Each goal therefore
runs a real engineering loop (measure → correct → re-measure), not a script.

The goals here are deliberately ones where the naive first attempt is *wrong*
(an interfering press-fit, an over-large fillet that OCC rejects, a bolt circle
whose holes breach the rim) so the self-correction is genuine and observable.
"""
import json
import math
import re


# --------------------------------------------------------------------------- #
# generic invariants — true for any valid solid model
# --------------------------------------------------------------------------- #
def invariants(doc):
    checks = []
    errs = []
    for o in doc.Objects:
        try:
            st = list(o.State)
        except Exception:
            st = []
        if any(s in ("Error", "Invalid") for s in st):
            errs.append(o.Name)
    checks.append({"name": "no_error_state", "ok": not errs, "detail": errs})

    bad = []
    for o in doc.Objects:
        sh = getattr(o, "Shape", None)
        if sh is not None and not sh.isNull() and sh.Solids:
            try:
                if not sh.isValid() or not sh.isClosed():
                    bad.append(o.Name)
            except Exception:
                bad.append(o.Name)
    checks.append({"name": "watertight_solids", "ok": not bad, "detail": bad})
    return checks


# --------------------------------------------------------------------------- #
# goal base
# --------------------------------------------------------------------------- #
class Bisect:
    """Reusable bisection optimiser for a single continuous design variable.

    Far better than a fixed step: it brackets the variable and halves the
    interval each iteration, so it converges in ~log2(range/tol) steps regardless
    of how far off the first guess is. Goals mix this in and call ``_bis_init``
    once, then ``_bis_step(measured, target, increasing)`` from ``remediate``.
    """

    def _bis_init(self, key, lo, hi):
        self._bk, self._blo, self._bhi = key, float(lo), float(hi)

    def _bis_step(self, measured, target, increasing, tol=1e-3):
        cur = self.params[self._bk]
        # narrow the bracket using the sign of the error and the monotonicity
        if (measured < target) == increasing:
            self._blo = cur            # need a larger variable
        else:
            self._bhi = cur            # need a smaller variable
        nxt = round((self._blo + self._bhi) / 2.0, 4)
        if abs(nxt - cur) < tol:
            return False
        self.params[self._bk] = nxt
        return True


class Goal:
    name = "goal"
    description = ""

    def __init__(self, **overrides):
        self.params = dict(self.defaults())
        self.params.update(overrides)

    def defaults(self):
        return {}

    def plan(self):
        """Return a list of {tool,args} steps building the goal from params."""
        raise NotImplementedError

    def evaluate(self, eng, build_results):
        """Return a list of acceptance checks for the current build."""
        raise NotImplementedError

    def remediate(self, checks):
        """Mutate self.params to fix failing checks. Return True if changed."""
        return False

    # helpers shared by goals -------------------------------------------------
    @staticmethod
    def _measure(eng, name):
        try:
            return eng.handlers["solid.measure"]({"name": name})
        except Exception as exc:
            return {"error": repr(exc)}

    @staticmethod
    def _interference(eng, a, b):
        try:
            return eng.handlers["solid.interference"]({"a": a, "b": b})
        except Exception as exc:
            return {"error": repr(exc)}


# --------------------------------------------------------------------------- #
# Goal 1: a controlled press-fit (clearance must land in a target band)
# --------------------------------------------------------------------------- #
class PressFit(Goal, Bisect):
    name = "press_fit"
    description = "Pin in a bored plate; radial clearance must land in [lo, hi]."

    def defaults(self):
        return {"hole_r": 8.0, "pin_r": 8.6, "lo": 0.2, "hi": 0.6,
                "plate": 60.0, "thick": 12.0, "pin_h": 30.0, "cx": 30.0, "cy": 30.0}

    def __init__(self, **overrides):
        super(PressFit, self).__init__(**overrides)
        # clearance = hole_r - pin_r, so it *decreases* as pin_r grows.
        self._bis_init("pin_r", self.params["hole_r"] - self.params["hi"] - 0.2,
                       self.params["pin_r"])

    def plan(self):
        p = self.params
        return [
            {"tool": "solid.box", "args": {"name": "plate", "length": p["plate"],
                                           "width": p["plate"], "height": p["thick"]}},
            {"tool": "solid.cylinder", "args": {"name": "bore", "radius": p["hole_r"],
                                                "height": p["thick"] + 10}},
            {"tool": "solid.translate", "args": {"name": "bore",
                                                 "vector": [p["cx"], p["cy"], -5]}},
            {"tool": "solid.cut", "args": {"a": "plate", "b": "bore", "out": "plate"}},
            {"tool": "solid.cylinder", "args": {"name": "pin", "radius": p["pin_r"],
                                                "height": p["pin_h"]}},
            {"tool": "solid.translate", "args": {"name": "pin",
                                                 "vector": [p["cx"], p["cy"], 0]}},
        ]

    def evaluate(self, eng, build_results):
        p = self.params
        inter = self._interference(eng, "plate", "pin")
        clearance = p["hole_r"] - p["pin_r"]
        checks = [
            {"name": "no_interference", "ok": not inter.get("interfering", True),
             "detail": inter},
            {"name": "clearance_in_band",
             "ok": (p["lo"] <= clearance <= p["hi"]),
             "measured": round(clearance, 4), "target": [p["lo"], p["hi"]]},
        ]
        return checks

    def remediate(self, checks):
        p = self.params
        clearance = p["hole_r"] - p["pin_r"]
        target = (p["lo"] + p["hi"]) / 2.0
        # clearance decreases as pin_r increases → increasing=False
        return self._bis_step(clearance, target, increasing=False)


# --------------------------------------------------------------------------- #
# Goal 2: largest fillet OCC will accept on a thin box (over-large at first)
# --------------------------------------------------------------------------- #
class SafeFillet(Goal):
    name = "safe_fillet"
    description = "Fillet a thin box; back off radius until OCC yields a valid solid."

    def defaults(self):
        return {"L": 40.0, "W": 20.0, "H": 10.0, "radius": 8.0, "min_radius": 0.5}

    def plan(self):
        p = self.params
        return [
            {"tool": "solid.box", "args": {"name": "blk", "length": p["L"],
                                           "width": p["W"], "height": p["H"]}},
            {"tool": "solid.fillet", "args": {"name": "blk", "radius": p["radius"]}},
        ]

    def evaluate(self, eng, build_results):
        p = self.params
        build_ok = all(r.get("ok") for r in build_results)
        m = self._measure(eng, "blk")
        valid = bool(m.get("valid")) and bool(m.get("closed", True))
        checks = [
            {"name": "fillet_built", "ok": build_ok,
             "detail": [r for r in build_results if not r.get("ok")]},
            {"name": "valid_solid", "ok": build_ok and valid,
             "measured": {"radius": p["radius"], "volume": m.get("volume")}},
        ]
        return checks

    def remediate(self, checks):
        p = self.params
        if p["radius"] <= p["min_radius"]:
            return False
        p["radius"] = round(max(p["min_radius"], p["radius"] - 1.5), 4)
        return True


# --------------------------------------------------------------------------- #
# Goal 3: bolt circle whose holes must stay interior (clean topology)
# --------------------------------------------------------------------------- #
class BoltCircle(Goal):
    name = "bolt_circle"
    description = "N holes on a bolt circle; must stay interior → exactly N+3 faces."

    def defaults(self):
        return {"flange_r": 40.0, "thick": 8.0, "n": 6, "hole_r": 4.0, "bcr": 38.0}

    def plan(self):
        p = self.params
        n, bcr = int(p["n"]), p["bcr"]
        steps = [
            {"tool": "solid.cylinder", "args": {"name": "flange", "radius": p["flange_r"],
                                                "height": p["thick"]}},
        ]
        for i in range(n):
            ang = 2 * math.pi * i / n
            x = round(bcr * math.cos(ang), 4)
            y = round(bcr * math.sin(ang), 4)
            steps.append({"tool": "solid.cylinder",
                          "args": {"name": "h%d" % i, "radius": p["hole_r"],
                                   "height": p["thick"] + 10}})
            steps.append({"tool": "solid.translate",
                          "args": {"name": "h%d" % i, "vector": [x, y, -5]}})
            steps.append({"tool": "solid.cut",
                          "args": {"a": "flange", "b": "h%d" % i, "out": "flange"}})
        return steps

    def evaluate(self, eng, build_results):
        p = self.params
        m = self._measure(eng, "flange")
        faces = m.get("faces")
        expected = int(p["n"]) + 3            # top + bottom + outer wall + N holes
        checks = [
            {"name": "build_ok", "ok": all(r.get("ok") for r in build_results)},
            {"name": "holes_interior_topology", "ok": (faces == expected),
             "measured": faces, "target": expected},
            {"name": "geometric_margin",
             "ok": (p["bcr"] + p["hole_r"] <= p["flange_r"] - 2.0),
             "measured": round(p["bcr"] + p["hole_r"], 4),
             "target": round(p["flange_r"] - 2.0, 4)},
        ]
        return checks

    def remediate(self, checks):
        p = self.params
        if p["bcr"] <= p["hole_r"] + 2:
            return False
        p["bcr"] = round(p["bcr"] - 2.0, 4)
        return True


# --------------------------------------------------------------------------- #
# Goal 4: a bearing block — multi-part, zero-interference running fit
# --------------------------------------------------------------------------- #
class BearingBlock(Goal, Bisect):
    name = "bearing_block"
    description = ("Shaft in a bored block with 4 interior mounting holes; running "
                   "clearance must land in [lo, hi] with zero interference.")

    def defaults(self):
        return {"bx": 60.0, "by": 40.0, "bz": 40.0, "bore_r": 12.0,
                "shaft_r": 12.4, "shaft_h": 70.0, "lo": 0.3, "hi": 0.8,
                "mh_r": 3.0, "inset": 8.0}

    def __init__(self, **overrides):
        super(BearingBlock, self).__init__(**overrides)
        # clearance = bore_r - shaft_r, decreases as shaft_r grows.
        self._bis_init("shaft_r", self.params["bore_r"] - self.params["hi"] - 0.2,
                       self.params["shaft_r"])

    def plan(self):
        p = self.params
        cx, cy = p["bx"] / 2.0, p["by"] / 2.0
        steps = [
            {"tool": "solid.box", "args": {"name": "block", "length": p["bx"],
                                           "width": p["by"], "height": p["bz"]}},
            {"tool": "solid.cylinder", "args": {"name": "bore", "radius": p["bore_r"],
                                                "height": p["bz"] + 20}},
            {"tool": "solid.translate", "args": {"name": "bore",
                                                 "vector": [cx, cy, -10]}},
            {"tool": "solid.cut", "args": {"a": "block", "b": "bore", "out": "block"}},
        ]
        ins = p["inset"]
        corners = [(ins, ins), (p["bx"] - ins, ins),
                   (ins, p["by"] - ins), (p["bx"] - ins, p["by"] - ins)]
        for i, (x, y) in enumerate(corners):
            steps.append({"tool": "solid.cylinder",
                          "args": {"name": "m%d" % i, "radius": p["mh_r"],
                                   "height": p["bz"] + 20}})
            steps.append({"tool": "solid.translate",
                          "args": {"name": "m%d" % i, "vector": [x, y, -10]}})
            steps.append({"tool": "solid.cut",
                          "args": {"a": "block", "b": "m%d" % i, "out": "block"}})
        steps += [
            {"tool": "solid.cylinder", "args": {"name": "shaft", "radius": p["shaft_r"],
                                                "height": p["shaft_h"]}},
            {"tool": "solid.translate", "args": {"name": "shaft",
                                                 "vector": [cx, cy, -15]}},
        ]
        return steps

    def evaluate(self, eng, build_results):
        p = self.params
        inter = self._interference(eng, "block", "shaft")
        clearance = p["bore_r"] - p["shaft_r"]
        m = self._measure(eng, "block")
        faces = m.get("faces")
        expected = 6 + 1 + 4                   # box + bore wall + 4 mounting walls
        return [
            {"name": "build_ok", "ok": all(r.get("ok") for r in build_results)},
            {"name": "no_interference", "ok": not inter.get("interfering", True),
             "detail": inter},
            {"name": "running_clearance",
             "ok": (p["lo"] <= clearance <= p["hi"]),
             "measured": round(clearance, 4), "target": [p["lo"], p["hi"]]},
            {"name": "mounting_holes_interior", "ok": (faces == expected),
             "measured": faces, "target": expected},
        ]

    def remediate(self, checks):
        p = self.params
        clearance = p["bore_r"] - p["shaft_r"]
        target = (p["lo"] + p["hi"]) / 2.0
        return self._bis_step(clearance, target, increasing=False)


# --------------------------------------------------------------------------- #
# Goal 5: an L-bracket — fuse two plates, fillet the joint (back off), drill
# --------------------------------------------------------------------------- #
class LBracket(Goal):
    name = "l_bracket"
    description = ("Fuse a base + upright into an L, fillet the inner joint as "
                   "large as OCC allows, drill interior bolt holes.")

    def defaults(self):
        return {"base_l": 60.0, "base_w": 40.0, "t": 8.0, "up_h": 50.0,
                "radius": 9.0, "min_radius": 1.0, "hole_r": 3.5, "inset": 10.0}

    def plan(self):
        p = self.params
        steps = [
            {"tool": "solid.box", "args": {"name": "base", "length": p["base_l"],
                                           "width": p["base_w"], "height": p["t"]}},
            {"tool": "solid.box", "args": {"name": "up", "length": p["t"],
                                           "width": p["base_w"], "height": p["up_h"]}},
            {"tool": "solid.union", "args": {"a": "base", "b": "up", "out": "bracket"}},
            {"tool": "solid.fillet", "args": {"name": "bracket", "radius": p["radius"]}},
        ]
        ins = p["inset"]
        for i, (x, y) in enumerate([(p["base_l"] - ins, ins),
                                    (p["base_l"] - ins, p["base_w"] - ins)]):
            steps.append({"tool": "solid.cylinder",
                          "args": {"name": "b%d" % i, "radius": p["hole_r"],
                                   "height": p["t"] + 20}})
            steps.append({"tool": "solid.translate",
                          "args": {"name": "b%d" % i, "vector": [x, y, -10]}})
            steps.append({"tool": "solid.cut",
                          "args": {"a": "bracket", "b": "b%d" % i, "out": "bracket"}})
        return steps

    def evaluate(self, eng, build_results):
        p = self.params
        build_ok = all(r.get("ok") for r in build_results)
        m = self._measure(eng, "bracket")
        valid = bool(m.get("valid")) and bool(m.get("closed", True))
        return [
            {"name": "build_ok", "ok": build_ok,
             "detail": [r for r in build_results if not r.get("ok")]},
            {"name": "valid_watertight_bracket", "ok": build_ok and valid,
             "measured": {"radius": p["radius"], "volume": m.get("volume"),
                          "faces": m.get("faces")}},
        ]

    def remediate(self, checks):
        p = self.params
        if p["radius"] <= p["min_radius"]:
            return False
        p["radius"] = round(max(p["min_radius"], p["radius"] - 1.5), 4)
        return True


# --------------------------------------------------------------------------- #
# Goal 6: an assembly mate — pin seated in a bracket bore via asm.* (coaxial
# mate + native solve + clash detection), tuned to a clearance band.
# --------------------------------------------------------------------------- #
class PinJoint(Goal, Bisect):
    name = "pin_joint"
    description = ("Seat a pin into a bracket bore with asm.coaxial; the grounded "
                   "assembly must be clash-free with clearance in [lo, hi].")

    def defaults(self):
        return {"bx": 50.0, "by": 30.0, "bz": 24.0, "bore_r": 8.0,
                "pin_r": 8.3, "pin_h": 50.0, "lo": 0.2, "hi": 0.5}

    def __init__(self, **overrides):
        super(PinJoint, self).__init__(**overrides)
        self._bis_init("pin_r", self.params["bore_r"] - self.params["hi"] - 0.2,
                       self.params["pin_r"])

    def plan(self):
        p = self.params
        cx, cy = p["bx"] / 2.0, p["by"] / 2.0
        return [
            {"tool": "solid.box", "args": {"name": "bracket", "length": p["bx"],
                                           "width": p["by"], "height": p["bz"]}},
            {"tool": "solid.cylinder", "args": {"name": "bore", "radius": p["bore_r"],
                                                "height": p["bz"] + 20}},
            {"tool": "solid.translate", "args": {"name": "bore",
                                                 "vector": [cx, cy, -10]}},
            {"tool": "solid.cut", "args": {"a": "bracket", "b": "bore", "out": "bracket"}},
            {"tool": "solid.cylinder", "args": {"name": "pin", "radius": p["pin_r"],
                                                "height": p["pin_h"]}},
            {"tool": "asm.create", "args": {"name": "Joint"}},
            {"tool": "asm.add", "args": {"name": "brk", "body": "bracket", "fixed": True}},
            {"tool": "asm.add", "args": {"name": "pin", "body": "pin"}},
            {"tool": "asm.coaxial", "args": {"hole": "brk", "pin": "pin", "seat": "bottom"}},
        ]

    def _handler(self, eng, tool, args):
        try:
            return eng.handlers[tool](args)
        except Exception as exc:
            return {"error": repr(exc)}

    def evaluate(self, eng, build_results):
        p = self.params
        clash = self._handler(eng, "asm.interference", {})
        solve = self._handler(eng, "asm.solve", {})
        clearance = p["bore_r"] - p["pin_r"]
        grounded = solve.get("grounded") or []
        return [
            {"name": "build_ok", "ok": all(r.get("ok") for r in build_results)},
            {"name": "no_clash", "ok": (clash.get("clash_count") == 0),
             "measured": clash.get("clash_count"), "detail": clash.get("clashes")},
            {"name": "clearance_in_band",
             "ok": (p["lo"] <= clearance <= p["hi"]),
             "measured": round(clearance, 4), "target": [p["lo"], p["hi"]]},
            {"name": "bracket_grounded", "ok": ("brk" in grounded),
             "detail": {"grounded": grounded}},
        ]

    def remediate(self, checks):
        p = self.params
        clearance = p["bore_r"] - p["pin_r"]
        target = (p["lo"] + p["hi"]) / 2.0
        return self._bis_step(clearance, target, increasing=False)


# --------------------------------------------------------------------------- #
# Goal 7: a meshing gear pair — center distance must equal the pitch sum so the
# pitch circles are tangent (true mesh) with no interference.
# --------------------------------------------------------------------------- #
class GearPair(Goal, Bisect):
    name = "gear_pair"
    description = ("Two gear blanks whose centre distance must equal m*(z1+z2)/2 "
                   "so the pitch circles mesh tangentially without interfering.")

    def defaults(self):
        return {"m": 2.0, "z1": 20, "z2": 30, "h": 12.0,
                "center": 58.0, "tol": 0.4}

    def _pitch(self):
        p = self.params
        r1 = p["m"] * p["z1"] / 2.0
        r2 = p["m"] * p["z2"] / 2.0
        return r1, r2

    def __init__(self, **overrides):
        super(GearPair, self).__init__(**overrides)
        r1, r2 = self._pitch()
        self._bis_init("center", r1 + r2, self.params["center"])

    def plan(self):
        p = self.params
        r1, r2 = self._pitch()
        return [
            {"tool": "solid.cylinder", "args": {"name": "g1", "radius": r1,
                                                "height": p["h"]}},
            {"tool": "solid.cylinder", "args": {"name": "g2", "radius": r2,
                                                "height": p["h"]}},
            {"tool": "solid.translate", "args": {"name": "g2",
                                                 "vector": [p["center"], 0, 0]}},
        ]

    def evaluate(self, eng, build_results):
        p = self.params
        r1, r2 = self._pitch()
        pitch_sum = r1 + r2
        inter = self._interference(eng, "g1", "g2")
        gap = round(p["center"] - pitch_sum, 4)
        return [
            {"name": "build_ok", "ok": all(r.get("ok") for r in build_results)},
            {"name": "no_interference", "ok": not inter.get("interfering", True),
             "detail": inter},
            {"name": "pitch_circles_mesh",
             "ok": (0.0 <= gap <= p["tol"]),
             "measured": gap, "target": [0.0, p["tol"]],
             "detail": {"center": p["center"], "pitch_sum": round(pitch_sum, 4)}},
        ]

    def remediate(self, checks):
        r1, r2 = self._pitch()
        target = r1 + r2 + self.params["tol"] / 2.0
        return self._bis_step(self.params["center"], target, increasing=True)


# --------------------------------------------------------------------------- #
# Goal 8: a hinge — two knuckles + a pin, all coaxial; the grounded 3-component
# assembly must be clash-free with the pin clearance in band.
# --------------------------------------------------------------------------- #
class Hinge(Goal, Bisect):
    name = "hinge"
    description = ("Two knuckles and a pin seated coaxially; the grounded "
                   "3-component assembly must be clash-free, clearance in band.")

    def defaults(self):
        return {"kw": 16.0, "kd": 24.0, "kh": 18.0, "bore_r": 5.0,
                "pin_r": 5.3, "pin_h": 60.0, "lo": 0.15, "hi": 0.4}

    def __init__(self, **overrides):
        super(Hinge, self).__init__(**overrides)
        self._bis_init("pin_r", self.params["bore_r"] - self.params["hi"] - 0.2,
                       self.params["pin_r"])

    def _knuckle(self, name, z):
        p = self.params
        # a block with a horizontal (Y-axis) bore, so the pin runs along Y.
        return [
            {"tool": "solid.box", "args": {"name": name, "length": p["kw"],
                                           "width": p["kd"], "height": p["kh"]}},
            {"tool": "solid.translate", "args": {"name": name, "vector": [0, 0, z]}},
            {"tool": "solid.cylinder", "args": {"name": name + "_b", "radius": p["bore_r"],
                                                "height": p["kd"] + 20}},
            {"tool": "solid.rotate", "args": {"name": name + "_b", "axis": [1, 0, 0],
                                              "angle": -90, "center": [0, 0, 0]}},
            {"tool": "solid.translate", "args": {"name": name + "_b",
                                                 "vector": [p["kw"] / 2.0, -10,
                                                            z + p["kh"] / 2.0]}},
            {"tool": "solid.cut", "args": {"a": name, "b": name + "_b", "out": name}},
        ]

    def plan(self):
        p = self.params
        steps = self._knuckle("low", 0.0) + self._knuckle("high", p["kh"])
        steps += [
            {"tool": "solid.cylinder", "args": {"name": "pin", "radius": p["pin_r"],
                                                "height": p["pin_h"]}},
            {"tool": "asm.create", "args": {"name": "Hinge"}},
            {"tool": "asm.add", "args": {"name": "k_low", "body": "low", "fixed": True}},
            {"tool": "asm.add", "args": {"name": "k_high", "body": "high"}},
            {"tool": "asm.add", "args": {"name": "pin", "body": "pin"}},
            {"tool": "asm.coaxial", "args": {"hole": "k_low", "pin": "pin", "seat": "bottom"}},
        ]
        return steps

    def _handler(self, eng, tool, args):
        try:
            return eng.handlers[tool](args)
        except Exception as exc:
            return {"error": repr(exc)}

    def evaluate(self, eng, build_results):
        p = self.params
        clash = self._handler(eng, "asm.interference", {})
        solve = self._handler(eng, "asm.solve", {})
        clearance = p["bore_r"] - p["pin_r"]
        grounded = solve.get("grounded") or []
        tree = self._handler(eng, "asm.tree", {})
        comps = tree.get("components") or []
        return [
            {"name": "build_ok", "ok": all(r.get("ok") for r in build_results)},
            {"name": "three_components", "ok": (len(comps) == 3),
             "measured": len(comps)},
            {"name": "no_clash", "ok": (clash.get("clash_count") == 0),
             "measured": clash.get("clash_count"), "detail": clash.get("clashes")},
            {"name": "clearance_in_band",
             "ok": (p["lo"] <= clearance <= p["hi"]),
             "measured": round(clearance, 4), "target": [p["lo"], p["hi"]]},
            {"name": "knuckle_grounded", "ok": ("k_low" in grounded),
             "detail": {"grounded": grounded}},
        ]

    def remediate(self, checks):
        p = self.params
        clearance = p["bore_r"] - p["pin_r"]
        target = (p["lo"] + p["hi"]) / 2.0
        return self._bis_step(clearance, target, increasing=False)


GOALS = {g.name: g for g in
         (PressFit, SafeFillet, BoltCircle, BearingBlock, LBracket, PinJoint,
          GearPair, Hinge)}


# --------------------------------------------------------------------------- #
# natural-language intent -> goal (so free text triggers the closed loop)
# --------------------------------------------------------------------------- #
# each entry: goal name -> list of trigger substrings (lowercased, EN + zh)
GOAL_INTENTS = {
    "press_fit":     ["press fit", "press-fit", "pressfit", "压配", "过盈", "压入"],
    "safe_fillet":   ["safe fillet", "safe-fillet", "圆角退让", "安全圆角", "退让圆角"],
    "bolt_circle":   ["bolt circle", "bolt-circle", "螺栓圆", "螺栓阵列", "法兰孔", "分度圆孔"],
    "bearing_block": ["bearing block", "bearing", "轴承座", "轴承块", "轴承"],
    "l_bracket":     ["l bracket", "l-bracket", "l 支架", "l支架", "角支架", "l 型", "l型"],
    "pin_joint":     ["pin joint", "pin-joint", "销轴", "插销", "销连接", "pin in"],
    "gear_pair":     ["gear pair", "gear-pair", "齿轮副", "啮合齿轮", "齿轮啮合", "齿轮", "啮合"],
    "hinge":         ["hinge", "铰链", "合页"],
}


def _gear_overrides(text):
    """Parse 'M2 20/30', 'm=2 z 20 30', '模数2 齿数20 30' -> gear params. The
    module token is consumed first so its digit can't be misread as a tooth."""
    ov = {}
    rest = text
    mm = re.search(r"\bm\s*=?\s*(\d+\.?\d*)", rest) or re.search(r"模数\s*(\d+\.?\d*)", rest)
    if mm:
        ov["m"] = float(mm.group(1))
        rest = rest[:mm.start()] + " " + rest[mm.end():]   # drop it before teeth
    teeth = re.search(r"(\d+)\s*[/x×]\s*(\d+)", rest)
    if teeth:
        ov["z1"], ov["z2"] = int(teeth.group(1)), int(teeth.group(2))
    else:
        ints = re.findall(r"\d+", rest)
        if len(ints) >= 2:
            ov["z1"], ov["z2"] = int(ints[0]), int(ints[1])
    if "m" in ov or "z1" in ov:
        m = ov.get("m", 2.0)
        z1, z2 = ov.get("z1", 20), ov.get("z2", 30)
        ov["center"] = round(m * (z1 + z2) / 2.0 + 8.0, 3)   # start off (too far)
    return ov


def _band(text):
    """A clearance/tolerance band 'A-B' (also '~', '到', '至') -> (lo, hi)."""
    m = re.search(r"(?:clearance|tolerance|fit|间隙|公差|配合)\D{0,6}"
                  r"(\d+\.?\d*)\s*(?:-|~|to|到|至)\s*(\d+\.?\d*)", text)
    if not m:
        m = re.search(r"(\d+\.?\d*)\s*(?:-|~|到|至)\s*(\d+\.?\d*)\s*(?:clearance|间隙|公差)",
                      text)
    if m:
        lo, hi = float(m.group(1)), float(m.group(2))
        return (lo, hi) if lo <= hi else (hi, lo)
    return None


def _after(text, words):
    """First number following any of ``words`` (a labelled dimension)."""
    for w in words:
        m = re.search(w + r"\D{0,6}(\d+\.?\d*)", text)
        if m:
            return float(m.group(1))
    return None


def _count(text):
    m = re.search(r"(\d+)\s*(?:holes?|bolts?|孔|个孔|螺栓)", text)
    return int(m.group(1)) if m else None


def _generic_overrides(name, text):
    """Pull goal-relevant dimensions out of free text (EN + zh). Only keys the
    goal actually declares are returned, so unknown numbers are ignored."""
    if name == "gear_pair":
        return _gear_overrides(text)
    ov = {}
    keys = set(GOALS[name]().defaults())

    def put(k, v):
        if v is not None and k in keys:
            ov[k] = v

    band = _band(text)
    if band:
        put("lo", band[0])
        put("hi", band[1])
    put("n", _count(text))
    put("hole_r", _after(text, ["hole", "孔"]))
    put("bore_r", _after(text, ["bore", "孔", "内径"]))
    put("pin_r", _after(text, ["pin", "销", "shaft", "轴"]))
    put("shaft_r", _after(text, ["shaft", "轴"]))
    put("radius", _after(text, ["fillet", "圆角", "radius", "半径"]))
    put("flange_r", _after(text, ["flange", "法兰"]))
    # recompute a deliberately-off start so the loop still has to correct
    if name == "bolt_circle" and ("flange_r" in ov or "n" in ov):
        ov.setdefault("bcr", round(GOALS[name](**ov).params["flange_r"] - 2.0, 3))
    return ov


def resolve_goal_intent(text):
    """Map free-text intent to (goal_name, overrides) or None. Lets a human just
    say 'make a meshing M2 20/30 gear pair' or 'press fit hole 8 clearance 0.2-0.6'
    and have the closed loop fire with the right parameters."""
    low = (text or "").lower()
    best = None                              # (pos, -len, name)
    for name, keys in GOAL_INTENTS.items():
        for k in keys:
            pos = low.find(k)
            if pos < 0:
                continue
            cand = (pos, -len(k), name)      # earliest match wins, then longest key
            if best is None or cand < best:
                best = cand
    if best is None:
        return None
    name = best[2]
    return name, _generic_overrides(name, low)


# --------------------------------------------------------------------------- #
# the loop
# --------------------------------------------------------------------------- #
class ClosedLoopAgent:
    def __init__(self, engine, max_iters=10):
        self.eng = engine
        self.max_iters = max_iters

    def solve(self, goal, on_iteration=None):
        transcript = []
        solved = False
        verdict = {"passed": False, "checks": []}
        for i in range(1, self.max_iters + 1):
            self.eng.clear()
            note, build = self.eng.run(json.dumps(goal.plan()))
            doc = self.eng.state.doc
            doc.recompute()
            checks = invariants(doc) + goal.evaluate(self.eng, build)
            verdict = {"passed": all(c["ok"] for c in checks), "checks": checks}
            step = {"iter": i, "params": dict(goal.params),
                    "passed": verdict["passed"],
                    "failed": [c["name"] for c in checks if not c["ok"]],
                    "checks": checks}
            transcript.append(step)
            if on_iteration:
                on_iteration(step)
            if verdict["passed"]:
                solved = True
                break
            if not goal.remediate([c for c in checks if not c["ok"]]):
                break
        return {"goal": goal.name, "description": goal.description,
                "solved": solved, "iterations": len(transcript),
                "final_params": dict(goal.params), "transcript": transcript}
