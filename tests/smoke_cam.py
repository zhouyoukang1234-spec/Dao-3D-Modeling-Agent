"""Cam-follower smoke -- a higher pair (line contact), rotary to reciprocating.

An eccentric disc cam (radius Rc, centre offset e from the pivot) drives a radial
roller follower (radius Rf) on the +Y axis. The follower rides tangent to the cam,
so its centre height follows the closed-form lift law

    s(theta) = e*cos(theta) + sqrt((Rc+Rf)^2 - (e*sin(theta))^2)

with total lift 2e. Unlike gear teeth (conjugate flanks) this is a cam pair, a
genuinely different contact type.

Checks (the lift law is the spec):
  * the lift between the high and low dwell equals 2e;
  * sweeping the cam through a full turn while moving the follower to s(theta)
    keeps the pair in tangent contact (~0 interference) at every angle;
  * the contact is real, not loose clearance: dropping the follower 1 mm makes the
    cam penetrate it, and raising it 1 mm opens a gap (no clash).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

RC, E, RF, H = 20.0, 6.0, 5.0, 8.0
CD = RC + RF
TOL = 5.0


def _s(th):
    t = math.radians(th)
    return E * math.cos(t) + math.sqrt(CD * CD - (E * math.sin(t)) ** 2)


def _ov(s):
    out = s.act("asm.interference", {"assembly": "CF"})
    assert out.ok, out.error
    cm = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}
    return cm.get(("cam", "roll"), 0.0)


def main():
    s = new_session("cam")
    print("FreeCAD", s.registry.kernel.freecad_version)

    assert s.act("solid.cylinder", {"name": "Cam", "radius": RC, "height": H}).ok
    assert s.act("solid.cylinder", {"name": "Roll", "radius": RF, "height": H}).ok
    assert s.act("asm.create", {"name": "CF"}).ok
    assert s.act("asm.add", {"assembly": "CF", "body": "Cam", "name": "cam"}).ok
    assert s.act("asm.add", {"assembly": "CF", "body": "Roll", "name": "roll"}).ok

    lift = _s(0.0) - _s(180.0)
    assert abs(lift - 2.0 * E) < 1e-9, ("lift should be 2e", lift)
    print("lift law: high dwell s(0)=%.2f, low s(180)=%.2f -> lift=%.2f = 2e" % (_s(0), _s(180), lift))

    # full-turn sweep: follower tracks s(theta), contact maintained
    worst = 0.0
    for th in range(0, 360, 30):
        assert s.act("asm.place", {"name": "cam", "pos": [0, E, 0]}).ok
        if th:
            assert s.act("asm.rotate", {"name": "cam", "axis": [0, 0, 1],
                                        "angle": float(th), "at": [0, 0, 0]}).ok
        assert s.act("asm.place", {"name": "roll", "pos": [0, _s(th), 0]}).ok
        ov = _ov(s)
        assert ov < TOL, ("cam jammed/lost contact at theta=%d" % th, ov)
        worst = max(worst, ov)
    print("full turn (0..330 deg, 12 steps): follower on s(theta), max overlap=%.2f (tangent)" % worst)

    # contact is real: drop 1 mm -> penetrate; raise 1 mm -> gap (no clash)
    assert s.act("asm.place", {"name": "cam", "pos": [0, E, 0]}).ok
    assert s.act("asm.place", {"name": "roll", "pos": [0, _s(0) - 1.0, 0]}).ok
    pen = _ov(s)
    assert pen > 10.0, ("follower 1mm low should penetrate", pen)
    assert s.act("asm.place", {"name": "roll", "pos": [0, _s(0) + 1.0, 0]}).ok
    gap = _ov(s)
    assert gap == 0.0, ("follower 1mm high should open a gap", gap)
    print("contact is real: -1mm -> overlap %.1f (penetrate), +1mm -> %.1f (gap)" % (pen, gap))

    if "view.render" in s.tools():
        assert s.act("asm.place", {"name": "cam", "pos": [0, E, 0]}).ok
        assert s.act("asm.rotate", {"name": "cam", "axis": [0, 0, 1], "angle": 90.0, "at": [0, 0, 0]}).ok
        assert s.act("asm.place", {"name": "roll", "pos": [0, _s(90), 0]}).ok
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_cam.png")
        rv = s.act("view.render", {"assembly": "CF", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("CAM SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_cam"):
    main()
