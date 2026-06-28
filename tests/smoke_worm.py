"""Worm drive (crossed-axis, high single-stage reduction) -- the last classic
member of the gear family (after spur / helical / internal / rack / bevel).

A worm is a screw: a helical thread of axial pitch px = pi*m and lead
L = starts*px wound on a pitch cylinder of diameter dw. It meshes at 90 deg with
a worm wheel of N teeth (pitch diameter dg = m*N) at centre distance
C = (dw + dg)/2. The defining properties, all asserted from closed form:

  * geometry: the swept thread makes exactly height/L turns;
  * reduction: one worm revolution advances the wheel by `starts` teeth, so the
    wheel turns L/Rg = 2*pi*starts/N per worm rev -> ratio i = N/starts;
  * self-locking: the lead angle lambda = atan(starts*m/dw); a single-start worm
    (small lambda) is self-locking (lambda < friction angle), a multi-start worm
    (large lambda) is back-drivable -- exactly the worm-drive design rule;
  * crossed-axis mesh: built at C the worm and (simplified straight-tooth) wheel
    are tangent at the pitch point -- pushing the wheel in jams hard, pulling it
    out opens a clean gap, bracketing C = (dw+dg)/2 as the true mesh distance.

The wheel is a standard involute spur gear (not a throated worm wheel); that
approximation is declared honestly and only its pitch-tangency / kinematics are
asserted, never a perfect conjugate line contact.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

M = 2.0
H = 30.0
MU = 0.15                       # worm-gear friction coefficient
FRICTION_ANGLE = math.degrees(math.atan(MU))   # self-locking threshold ~8.53 deg


def build_worm(s, name, dw, starts):
    lead = starts * math.pi * M
    rw = dw / 2.0
    thread_r = M
    assert s.act("param.body", {"name": name}).ok
    assert s.act("param.pad", {"body": name, "feature": "core",
                               "profile": {"circle": rw - M}, "length": H}).ok
    sw = s.act("param.sweep", {"body": name, "feature": "thread",
                               "profile": {"circle": thread_r},
                               "path": {"helix": {"radius": rw - thread_r / 2.0,
                                                  "pitch": lead, "height": H, "z": 0}}})
    assert sw.ok, sw.error
    return sw.data["turns"], lead


def lead_angle(dw, starts):
    return math.degrees(math.atan(starts * M / dw))


def main():
    print("Worm drive (crossed-axis, single-stage reduction)  m=%.0f" % M)
    s = new_session("worm")
    print("FreeCAD", s.registry.kernel.freecad_version)

    DW, STARTS, N = 24.0, 1, 30
    turns, lead = build_worm(s, "W", DW, STARTS)
    rw, rg = DW / 2.0, M * N / 2.0
    C = rw + rg

    # 1) thread geometry: height / lead turns
    assert abs(turns - H / lead) < 1e-2, ("worm turns", turns, H / lead)
    print("  worm: dw=%.0f starts=%d  thread turns=%.3f (=H/lead=%.3f)" % (DW, STARTS, turns, H / lead))

    # 2) reduction ratio from the lead: wheel turns L/Rg per worm rev = 2*pi*starts/N
    wheel_rad_per_rev = lead / rg
    assert abs(wheel_rad_per_rev - 2.0 * math.pi * STARTS / N) < 1e-9, wheel_rad_per_rev
    ratio = N / STARTS
    assert abs((2.0 * math.pi / wheel_rad_per_rev) - ratio) < 1e-9
    print("  reduction: 1 worm rev -> wheel %.4f rad = %.2f deg (1 tooth=%.2f deg) -> i = N/starts = %.0f:1"
          % (wheel_rad_per_rev, math.degrees(wheel_rad_per_rev), 360.0 / N, ratio))

    # 3) self-locking rule: single-start locks, 4-start back-drives
    lam1 = lead_angle(DW, 1)
    lam4 = lead_angle(DW, 4)
    assert lam1 < FRICTION_ANGLE, ("single-start should self-lock", lam1, FRICTION_ANGLE)
    assert lam4 > FRICTION_ANGLE, ("4-start should back-drive", lam4, FRICTION_ANGLE)
    print("  self-locking: lead angle 1-start=%.2f deg < friction %.2f deg (locks); 4-start=%.2f deg > (back-drives)"
          % (lam1, FRICTION_ANGLE, lam4))

    # 4) crossed-axis mesh tangency at C = (dw+dg)/2
    assert s.act("param.body", {"name": "G"}).ok
    assert s.act("param.pad", {"body": "G", "feature": "Gf",
                               "profile": {"gear": {"module": M, "teeth": N}}, "length": 8.0}).ok
    assert s.act("asm.create", {"name": "WG"}).ok
    assert s.act("asm.add", {"assembly": "WG", "body": "W", "name": "worm", "fixed": True}).ok
    assert s.act("asm.add", {"assembly": "WG", "body": "G", "name": "wheel"}).ok

    def overlap(dC):
        ctr = [C + dC, 0, H / 2.0]
        assert s.act("asm.place", {"name": "wheel", "pos": ctr}).ok
        assert s.act("asm.rotate", {"name": "wheel", "axis": [1, 0, 0], "angle": 90.0, "at": ctr}).ok
        out = s.act("asm.interference", {"assembly": "WG"})
        assert out.ok, out.error
        return sum(c["overlap_volume"] for c in out.data["clashes"])

    jam, mesh, gap = overlap(-1.5), overlap(0.0), overlap(1.5)
    assert mesh < 30.0, ("worm-wheel should be tangent at C", mesh)
    assert jam > 500.0, ("pushing wheel in should jam", jam)
    assert gap < 1.0, ("pulling wheel out should open a gap", gap)
    assert jam > mesh > gap
    print("  crossed-axis mesh at C=%.0f: in -1.5 jams %.0f, tangent %.1f, out +1.5 gap %.2f" % (C, jam, mesh, gap))

    if "view.render" in s.tools():
        overlap(0.0)
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_worm.png")
        rv = s.act("view.render", {"assembly": "WG", "view": "iso", "path": o})
        if rv.ok:
            print("  contour -> %s (%d bytes)" % (o, rv.data["bytes"]))

    s.registry.kernel.shutdown()
    print("WORM SMOKE OK")


if __name__ in ("__main__", "smoke_worm"):
    main()
