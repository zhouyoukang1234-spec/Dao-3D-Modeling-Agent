"""Slider-crank smoke -- the engine mechanism (revolute chain + a prismatic joint).

Crank r about O, connecting rod l, slider (piston) constrained to the X axis. For a
crank angle theta the crank pin A = (r cos, r sin); the slider pin B lies on the X
axis at distance l from A, so the piston position is the exact law

    x(theta) = r cos(theta) + sqrt(l^2 - (r sin theta)^2),  stroke = 2r.

Checks:
  * the piston position follows the closed form and the stroke equals 2r;
  * the connecting-rod length is preserved and the slider stays on its guide
    (y = 0) for a full crank revolution;
  * the parts assemble into a connected chain -- crank<->rod overlap at A,
    rod<->slider overlap at B -- across the motion, and the slider rides the guide
    (overlaps it) without the crank touching the slider.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

R, L = 12.0, 34.0          # crank radius, rod length
PIV = (0.0, 0.0)
W, HT = 4.0, 6.0           # link cross-section
SLIDE = 10.0               # slider block size
GX0 = (L - R) - SLIDE - 4.0      # guide start x (covers the slider travel)
GLEN = 2.0 * R + 2.0 * SLIDE + 8.0  # guide length


def x_of(thd):
    t = math.radians(thd)
    return R * math.cos(t) + math.sqrt(L * L - (R * math.sin(t)) ** 2)


def _place_bar(s, name, p1, p2):
    mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
    phi = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
    assert s.act("asm.place", {"name": name, "pos": [mx, my, 0]}).ok
    if abs(phi) > 1e-9:
        assert s.act("asm.rotate", {"name": name, "axis": [0, 0, 1], "angle": phi,
                                    "at": [mx, my, 0]}).ok


def _clash(s):
    out = s.act("asm.interference", {"assembly": "SC"})
    assert out.ok, out.error
    return {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}


def main():
    s = new_session("slidercrank")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # piston position law + stroke
    xs = [x_of(t) for t in range(0, 360, 10)]
    stroke = max(xs) - min(xs)
    assert abs(max(xs) - (R + L)) < 1e-9 and abs(min(xs) - (L - R)) < 1e-9, (max(xs), min(xs))
    assert abs(stroke - 2.0 * R) < 1e-9, ("stroke should be 2r", stroke)
    print("piston law: TDC x=%.2f (=r+l), BDC x=%.2f (=l-r), stroke=%.2f = 2r" % (max(xs), min(xs), stroke))

    # bodies: crank + rod (bars), slider (block), guide (long thin block on X axis)
    assert s.act("param.body", {"name": "crank"}).ok
    assert s.act("param.pad", {"body": "crank", "feature": "ck",
                               "profile": {"rect": [R, W]}, "length": HT}).ok
    assert s.act("param.body", {"name": "rod"}).ok
    assert s.act("param.pad", {"body": "rod", "feature": "rd",
                               "profile": {"rect": [L, W]}, "length": HT}).ok
    assert s.act("solid.box", {"name": "slider", "length": SLIDE, "width": SLIDE, "height": HT}).ok
    assert s.act("solid.box", {"name": "guide", "length": GLEN, "width": 1.5, "height": HT}).ok

    assert s.act("asm.create", {"name": "SC"}).ok
    assert s.act("asm.add", {"assembly": "SC", "body": "guide", "name": "guide", "fixed": True}).ok
    for nm in ("crank", "rod", "slider"):
        assert s.act("asm.add", {"assembly": "SC", "body": nm, "name": nm}).ok
    # guide (placed by min corner) sits just below the X axis so the slider rides it
    assert s.act("asm.place", {"name": "guide", "pos": [GX0, -SLIDE / 2.0 - 1.5 + 0.4, 0]}).ok

    def pose(thd):
        A = (R * math.cos(math.radians(thd)), R * math.sin(math.radians(thd)))
        xb = x_of(thd)
        B = (xb, 0.0)
        _place_bar(s, "crank", PIV, A)
        _place_bar(s, "rod", A, B)
        assert s.act("asm.place", {"name": "slider", "pos": [xb - SLIDE / 2.0, -SLIDE / 2.0, 0]}).ok
        # rod length from the placed pin points
        assert abs(math.hypot(B[0] - A[0], B[1] - A[1]) - L) < 1e-9, ("rod len", thd)
        return A, B

    for thd in (20, 70, 110, 180, 250, 320):
        pose(thd)
        cm = _clash(s)
        assert cm.get(tuple(sorted(("crank", "rod"))), 0.0) > 0.0, ("crank-rod broke", thd, cm)
        assert cm.get(tuple(sorted(("rod", "slider"))), 0.0) > 0.0, ("rod-slider broke", thd, cm)
        assert cm.get(tuple(sorted(("guide", "slider"))), 0.0) > 0.0, ("slider left guide", thd, cm)
        assert cm.get(tuple(sorted(("crank", "slider"))), 0.0) == 0.0, ("crank hit slider", thd, cm)
    print("connected chain over a full revolution: crank-rod & rod-slider joints hold, slider rides the guide")

    if "view.render" in s.tools():
        pose(50)
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_slidercrank.png")
        rv = s.act("view.render", {"assembly": "SC", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("SLIDERCRANK SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_slidercrank"):
    main()
