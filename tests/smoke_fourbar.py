"""Four-bar linkage smoke -- a closed kinematic chain (the loop-closure equation).

Ground O2--O4 (length L1), crank L2 at O2, coupler L3, rocker L4 at O4. For a crank
angle theta the moving pivot A = O2 + L2*(cos,sin); the coupler/rocker joint B is
the intersection of circle(A, L3) and circle(O4, L4) -- the loop-closure equation.
With Grashof s+l <= p+q and the crank shortest, the crank fully rotates
(crank-rocker), so B exists for every theta.

Checks:
  * Grashof classifies this as a crank-rocker;
  * for a full crank revolution the loop closes: the solved B keeps |B-A|=L3 and
    |B-O4|=L4 exactly, and the rocker angle stays within its swing;
  * the four links assemble into a connected chain -- each pair of links sharing a
    pivot physically overlaps there, while the opposite (non-adjacent) links do
    not -- and this holds as the crank is driven round.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

L1, L2, L3, L4 = 40.0, 15.0, 35.0, 30.0   # ground, crank, coupler, rocker
O2 = (0.0, 0.0)
O4 = (L1, 0.0)
W, HT = 4.0, 6.0                            # link cross-section


def solve_B(A):
    ax, ay = A
    dx, dy = O4[0] - ax, O4[1] - ay
    d = math.hypot(dx, dy)
    assert abs(L3 - L4) <= d <= L3 + L4, ("no loop closure", d)
    a = (L3 * L3 - L4 * L4 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, L3 * L3 - a * a))
    px, py = ax + a * dx / d, ay + a * dy / d
    return (px - h * dy / d, py + h * dx / d)     # upper branch


def _place_bar(s, name, p1, p2):
    mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
    phi = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
    assert s.act("asm.place", {"name": name, "pos": [mx, my, 0]}).ok
    if abs(phi) > 1e-9:
        assert s.act("asm.rotate", {"name": name, "axis": [0, 0, 1], "angle": phi,
                                    "at": [mx, my, 0]}).ok


def _clash(s):
    out = s.act("asm.interference", {"assembly": "FB"})
    assert out.ok, out.error
    return {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}


def main():
    s = new_session("fourbar")
    print("FreeCAD", s.registry.kernel.freecad_version)

    lens = sorted([L1, L2, L3, L4])
    grashof = lens[0] + lens[3] <= lens[1] + lens[2]
    assert grashof and min(L1, L2, L3, L4) == L2, "expected a Grashof crank-rocker"
    print("Grashof crank-rocker: s+l=%.0f <= p+q=%.0f, crank (L2=%.0f) is shortest"
          % (lens[0] + lens[3], lens[1] + lens[2], L2))

    # loop closes for a full crank revolution
    rocker_angles = []
    for thd in range(0, 360, 10):
        th = math.radians(thd)
        A = (L2 * math.cos(th), L2 * math.sin(th))
        B = solve_B(A)
        assert abs(math.hypot(B[0] - A[0], B[1] - A[1]) - L3) < 1e-9, ("coupler len", thd)
        assert abs(math.hypot(B[0] - O4[0], B[1] - O4[1]) - L4) < 1e-9, ("rocker len", thd)
        rocker_angles.append(math.degrees(math.atan2(B[1] - O4[1], B[0] - O4[0])))
    print("full crank revolution: loop closes every 10 deg; rocker swings %.1f..%.1f deg"
          % (min(rocker_angles), max(rocker_angles)))

    # build the four links as bars and assemble the chain
    bars = {"ground": L1, "crank": L2, "coupler": L3, "rocker": L4}
    for nm, ln in bars.items():
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"rect": [ln, W]}, "length": HT}).ok
    assert s.act("asm.create", {"name": "FB"}).ok
    assert s.act("asm.add", {"assembly": "FB", "body": "ground", "name": "ground", "fixed": True}).ok
    for nm in ("crank", "coupler", "rocker"):
        assert s.act("asm.add", {"assembly": "FB", "body": nm, "name": nm}).ok

    def pose(thd):
        th = math.radians(thd)
        A = (L2 * math.cos(th), L2 * math.sin(th))
        B = solve_B(A)
        _place_bar(s, "ground", O2, O4)
        _place_bar(s, "crank", O2, A)
        _place_bar(s, "coupler", A, B)
        _place_bar(s, "rocker", O4, B)
        return A, B

    # adjacency: which link pairs share a pivot (must overlap) vs not (must not)
    adjacent = [("crank", "ground"), ("crank", "coupler"),
                ("coupler", "rocker"), ("ground", "rocker")]
    for thd in (30, 90, 150, 210, 300):
        pose(thd)
        cm = _clash(s)
        for pair in adjacent:
            key = tuple(sorted(pair))
            assert cm.get(key, 0.0) > 0.0, ("chain broken at joint %s (theta=%d)" % (pair, thd), cm)
        # the non-adjacent pair (crank<->rocker) must not touch
        assert cm.get(tuple(sorted(("crank", "rocker"))), 0.0) == 0.0, \
            ("crank and rocker should not collide", thd, cm)
    print("connected chain: all 4 joints overlap, crank & rocker never collide, across the sweep")

    if "view.render" in s.tools():
        pose(60)
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_fourbar.png")
        rv = s.act("view.render", {"assembly": "FB", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("FOURBAR SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_fourbar"):
    main()
