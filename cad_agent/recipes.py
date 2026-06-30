"""Parametric recipe library -- distilled, reusable orchestration wisdom.

The smoke *capstones* proved that a multi-step part (``smoke_project``) and a
multi-component assembly (``smoke_assembly_project``) can be driven through the
tool registry. Those were one-offs with fixed numbers. This module promotes
that experience into the **system itself**: each recipe is a pure, engine-
agnostic function that turns parameters into an ordered list of ``{"tool",
"args"}`` steps -- the same dicts the planner emits and :class:`ToolRegistry`
executes. So the "how to build a bolted stack / a flanged bracket" knowledge is
no longer trapped in a test; it is a parameterised, broadly-adaptive building
block any session (now or later) can compose at any size.

A recipe never touches a kernel. It only *describes* work, returning a
:class:`Recipe` (steps + a ``meta`` block of closed-form expectations a caller
can verify against). :meth:`cad_agent.session.AgentSession.make` is the thin
execution path that runs the steps on whatever backend is wired in.

Design rules (mirroring the backend op discipline):
* validate every parameter up front with a guided ``ValueError`` (no bare
  ``TypeError`` leaking from arithmetic on a string);
* emit only stable, already-registered tool names (``solid.*`` / ``asm.*``);
* keep names ``prefix``-able so a recipe can be instanced more than once.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List


@dataclass
class Recipe:
    """A generated plan: ordered tool steps plus closed-form expectations."""
    name: str
    steps: List[Dict[str, Any]]
    meta: Dict[str, Any] = field(default_factory=dict)


Step = Dict[str, Any]


def _pos(x: float, key: str) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise ValueError("%s must be a number (got %r)" % (key, x))
    if x <= 0:
        raise ValueError("%s must be positive (got %g)" % (key, x))
    return float(x)


def _int_pos(n: Any, key: str) -> int:
    if isinstance(n, bool) or not isinstance(n, int):
        raise ValueError("%s must be an integer (got %r)" % (key, n))
    if n < 1:
        raise ValueError("%s must be >= 1 (got %d)" % (key, n))
    return n


# --------------------------------------------------------------------------- #
# recipe: flanged bracket (one multi-step PART) -- distilled from smoke_project
# --------------------------------------------------------------------------- #
def flanged_bracket(length: float = 80.0, width: float = 50.0, height: float = 10.0,
                    boss_r: float = 8.0, boss_h: float = 14.0, bore_r: float = 4.0,
                    hole_r: float = 3.0, hole_inset: float = 10.0,
                    prefix: str = "") -> Recipe:
    """A base plate with a central raised boss bored through, plus four corner
    mounting holes -- the canonical mounting bracket, at any dimensions.

    ``boss_h`` is the boss height ABOVE the plate (the modelled cylinder spans
    plate + boss so it fuses cleanly). ``bore_r`` < ``boss_r`` and ``hole_r``
    leave material; the corner holes inset ``hole_inset`` from each plate edge.
    """
    L, W, H = _pos(length, "length"), _pos(width, "width"), _pos(height, "height")
    br, bh = _pos(boss_r, "boss_r"), _pos(boss_h, "boss_h")
    cr = _pos(bore_r, "bore_r")
    hr, hi = _pos(hole_r, "hole_r"), _pos(hole_inset, "hole_inset")
    if cr >= br:
        raise ValueError("bore_r (%g) must be smaller than boss_r (%g)" % (cr, br))
    if 2 * hi >= min(L, W):
        raise ValueError(
            "hole_inset (%g) too large for plate %gx%g" % (hi, L, W))
    cx, cy = L / 2.0, W / 2.0
    boss_total = H + bh
    p = prefix
    base, boss = p + "plate", p + "boss"
    steps: List[Step] = [
        {"tool": "solid.box", "args": {"name": base, "length": L, "width": W, "height": H}},
        {"tool": "solid.cylinder", "args": {"name": p + "boss_raw", "radius": br,
                                            "height": boss_total, "pos": [cx, cy, 0]}},
        {"tool": "solid.union", "args": {"a": base, "b": p + "boss_raw", "out": boss}},
        {"tool": "solid.cylinder", "args": {"name": p + "bore", "radius": cr,
                                            "height": boss_total + 10, "pos": [cx, cy, -5]}},
        {"tool": "solid.cut", "args": {"a": boss, "b": p + "bore", "out": boss}},
    ]
    corners = [(hi, hi), (L - hi, hi), (hi, W - hi), (L - hi, W - hi)]
    for i, (hx, hy) in enumerate(corners, 1):
        hole = "%shole%d" % (p, i)
        steps.append({"tool": "solid.cylinder",
                      "args": {"name": hole, "radius": hr, "height": H + 10, "pos": [hx, hy, -5]}})
        steps.append({"tool": "solid.cut", "args": {"a": boss, "b": hole, "out": boss}})

    volume = (L * W * H + math.pi * br * br * bh
              - math.pi * cr * cr * boss_total
              - 4 * math.pi * hr * hr * H)
    return Recipe(name="flanged_bracket", steps=steps,
                  meta={"part": boss, "volume": round(volume, 4),
                        "bbox_size": [L, W, boss_total]})


# --------------------------------------------------------------------------- #
# recipe: bolted spacer stack (a multi-component ASSEMBLY) -- distilled and
# generalised from smoke_assembly_project (any number of spacers, any sizes)
# --------------------------------------------------------------------------- #
def bolted_stack(n_spacers: int = 3, plate_size: float = 60.0, plate_h: float = 10.0,
                 spacer_r: float = 20.0, spacer_h: float = 8.0, bore_r: float = 6.0,
                 bolt_r: float = 5.0, nut_r: float = 9.0, nut_h: float = 8.0,
                 prefix: str = "") -> Recipe:
    """A bored base plate, ``n_spacers`` identical washers stacked up the centre,
    a bolt seated coaxially through them and a nut on top -- a real assembly at
    any spacer count / size.

    ``bolt_r`` < ``bore_r`` gives a clearance fit (no interference). The bolt is
    sized to run flush from the plate bottom to the nut top.
    """
    n = _int_pos(n_spacers, "n_spacers")
    ps, ph = _pos(plate_size, "plate_size"), _pos(plate_h, "plate_h")
    sr, sh = _pos(spacer_r, "spacer_r"), _pos(spacer_h, "spacer_h")
    cr, rr = _pos(bore_r, "bore_r"), _pos(bolt_r, "bolt_r")
    nr, nh = _pos(nut_r, "nut_r"), _pos(nut_h, "nut_h")
    if rr >= cr:
        raise ValueError(
            "bolt_r (%g) must be smaller than bore_r (%g) for a clearance fit"
            % (rr, cr))
    if cr >= sr or cr >= nr:
        raise ValueError("bore_r (%g) must be smaller than spacer_r and nut_r" % cr)
    if 2 * sr > ps:
        raise ValueError("spacer dia (%g) exceeds plate size (%g)" % (2 * sr, ps))

    cx = cy = ps / 2.0
    stack_top = ph + n * sh                 # top face of the last washer
    bolt_h = stack_top + nh                 # flush plate-bottom .. nut-top
    p = prefix
    base, spacer, bolt, nut = p + "base", p + "spacer", p + "bolt", p + "nut"

    steps: List[Step] = [
        # ---- model the four distinct source parts ---- #
        {"tool": "solid.box", "args": {"name": p + "base_blank",
                                       "length": ps, "width": ps, "height": ph}},
        {"tool": "solid.cylinder", "args": {"name": p + "base_bore", "radius": cr,
                                            "height": ph + 20, "pos": [cx, cy, -10]}},
        {"tool": "solid.cut", "args": {"a": p + "base_blank", "b": p + "base_bore", "out": base}},
        {"tool": "solid.cylinder", "args": {"name": p + "sp_out", "radius": sr, "height": sh}},
        {"tool": "solid.cylinder", "args": {"name": p + "sp_bore", "radius": cr,
                                            "height": sh + 16, "pos": [0, 0, -8]}},
        {"tool": "solid.cut", "args": {"a": p + "sp_out", "b": p + "sp_bore", "out": spacer}},
        {"tool": "solid.cylinder", "args": {"name": bolt, "radius": rr, "height": bolt_h}},
        {"tool": "solid.cylinder", "args": {"name": p + "nut_out", "radius": nr, "height": nh}},
        {"tool": "solid.cylinder", "args": {"name": p + "nut_bore", "radius": cr,
                                            "height": nh + 16, "pos": [0, 0, -8]}},
        {"tool": "solid.cut", "args": {"a": p + "nut_out", "b": p + "nut_bore", "out": nut}},
        # ---- instance + mate ---- #
        {"tool": "asm.create", "args": {"name": p + "Stack"}},
        {"tool": "asm.add", "args": {"body": base, "name": p + "Base", "fixed": True}},
    ]
    insts = ["%sS%d" % (p, i) for i in range(1, n + 1)]
    for inst in insts:
        steps.append({"tool": "asm.add", "args": {"body": spacer, "name": inst}})
    steps.append({"tool": "asm.add", "args": {"body": bolt, "name": p + "Bolt"}})
    steps.append({"tool": "asm.add", "args": {"body": nut, "name": p + "Nut"}})

    # stack the washers up the +Z faces, base -> S1 -> S2 -> ...
    below = p + "Base"
    for inst in insts:
        steps.append({"tool": "asm.stack", "args": {"base": below, "top": inst}})
        below = inst
    steps.append({"tool": "asm.coaxial", "args": {"hole": p + "Base", "pin": p + "Bolt",
                                                  "seat": "bottom"}})
    steps.append({"tool": "asm.coaxial", "args": {"hole": p + "Base", "pin": p + "Nut",
                                                  "seat": stack_top}})

    v_plate = ps * ps * ph - math.pi * cr * cr * ph
    v_wash = math.pi * sr * sr * sh - math.pi * cr * cr * sh
    v_bolt = math.pi * rr * rr * bolt_h
    v_nut = math.pi * nr * nr * nh - math.pi * cr * cr * nh
    return Recipe(name="bolted_stack", steps=steps,
                  meta={"assembly": p + "Stack", "component_count": n + 3,
                        "spacer_count": n, "axis": [cx, cy],
                        "bbox_size": [ps, ps, bolt_h],
                        "total_volume": round(v_plate + n * v_wash + v_bolt + v_nut, 4),
                        "unit_volume": {"plate": round(v_plate, 4), "spacer": round(v_wash, 4),
                                        "bolt": round(v_bolt, 4), "nut": round(v_nut, 4)}})


RECIPES: Dict[str, Callable[..., Recipe]] = {
    "flanged_bracket": flanged_bracket,
    "bolted_stack": bolted_stack,
}


def generate(name: str, **params: Any) -> Recipe:
    """Look a recipe up by name and expand it with ``params`` -- a guided error
    names the available recipes rather than leaking a bare ``KeyError``."""
    fn = RECIPES.get(name)
    if fn is None:
        raise ValueError("unknown recipe %r (available: %s)"
                         % (name, ", ".join(sorted(RECIPES))))
    return fn(**params)
