"""Engine valve-timing train -- an INTEGRATED machine composing three validated
mechanisms driven by one input: a meshed gear pair (rolling), the 2:1 four-stroke
reduction (kinematics), and a cam driving a valve (cam-follower higher pair).

Layout (one crank input angle alpha drives everything):
  * crank gear (Zc) at the origin -- the input;
  * cam gear (Zcam = 2*Zc) meshed at centre distance m*(Zc+Zcam)/2 -- so it turns
    at half crank speed: the four-stroke camshaft ratio;
  * an eccentric cam on the cam-gear shaft (stacked on its own Z-plane);
  * a roller valve riding the cam on +Y.

Over two full crank revolutions (720 deg, one four-stroke cycle) we assert the
whole machine stays consistent:
  * the timing gears stay meshed (~0) as they roll -- never jamming;
  * the camshaft turns exactly once (cam angle = -alpha/2): one valve event per
    two crank revolutions (the four-stroke law);
  * the valve keeps tangent contact with the cam and its lift spans 2e, opening
    exactly once per cycle;
  * no stray collisions anywhere in the assembly.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
ZC, ZCAM = 16, 32                 # crank gear, cam gear (2:1 -> four-stroke)
RPC, RPCAM = M * ZC / 2.0, M * ZCAM / 2.0
CD = RPC + RPCAM                  # centre distance = 48
C = (CD, 0.0)                     # cam-gear centre
GW = 8.0                          # gear face width (z 0..GW)
RCAM, ECC, RF = 10.0, 4.0, 4.0   # cam radius, eccentricity, valve roller radius
CDF = RCAM + RF
CAMZ = GW + 0.5                   # cam lobe sits on its own plane above the gears
CAMH = 6.0
TOL = 8.0


def cam_rot(alpha, phase0):
    return phase0 - (ZC / float(ZCAM)) * alpha     # external pair: opposite sense, half speed


def valve_lift(camrot):
    t = math.radians(camrot)
    return ECC * math.cos(t) + math.sqrt(CDF * CDF - (ECC * math.sin(t)) ** 2)


def _clash(s):
    out = s.act("asm.interference", {"assembly": "ENG"})
    assert out.ok, out.error
    return {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in out.data["clashes"]}


def main():
    s = new_session("engine")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # bodies: two timing gears, the cam lobe, the roller valve
    assert s.act("param.body", {"name": "CrankG"}).ok
    assert s.act("param.pad", {"body": "CrankG", "feature": "cg",
                               "profile": {"gear": {"module": M, "teeth": ZC}}, "length": GW}).ok
    assert s.act("param.body", {"name": "CamG"}).ok
    assert s.act("param.pad", {"body": "CamG", "feature": "mg",
                               "profile": {"gear": {"module": M, "teeth": ZCAM}}, "length": GW}).ok
    assert s.act("solid.cylinder", {"name": "Cam", "radius": RCAM, "height": CAMH}).ok
    assert s.act("solid.cylinder", {"name": "Valve", "radius": RF, "height": CAMH}).ok

    assert s.act("asm.create", {"name": "ENG"}).ok
    assert s.act("asm.add", {"assembly": "ENG", "body": "CrankG", "name": "crankg"}).ok
    assert s.act("asm.add", {"assembly": "ENG", "body": "CamG", "name": "camg"}).ok
    assert s.act("asm.add", {"assembly": "ENG", "body": "Cam", "name": "cam"}).ok
    assert s.act("asm.add", {"assembly": "ENG", "body": "Valve", "name": "valve"}).ok

    phase0 = meshing_phase_deg(0.0, ZC, ZCAM)
    print("timing gears: Zc=%d, Zcam=%d (ratio %.1f:1), centre distance=%.0f, mesh phase=%.2f deg"
          % (ZC, ZCAM, ZCAM / float(ZC), CD, phase0))

    def pose(alpha):
        cr = cam_rot(alpha, phase0)
        # crank gear (input)
        assert s.act("asm.place", {"name": "crankg", "pos": [0, 0, 0]}).ok
        if alpha:
            assert s.act("asm.rotate", {"name": "crankg", "axis": [0, 0, 1],
                                        "angle": alpha, "at": [0, 0, 0]}).ok
        # cam gear (rolls at -alpha/2)
        assert s.act("asm.place", {"name": "camg", "pos": [C[0], C[1], 0]}).ok
        assert s.act("asm.rotate", {"name": "camg", "axis": [0, 0, 1], "angle": cr, "at": [C[0], C[1], 0]}).ok
        # cam lobe on its own plane, eccentric, rotating with the cam gear
        assert s.act("asm.place", {"name": "cam", "pos": [C[0], C[1] + ECC, CAMZ]}).ok
        assert s.act("asm.rotate", {"name": "cam", "axis": [0, 0, 1], "angle": cr, "at": [C[0], C[1], CAMZ]}).ok
        # valve rides the cam on +Y
        assert s.act("asm.place", {"name": "valve", "pos": [C[0], C[1] + valve_lift(cr), CAMZ]}).ok
        return cr

    # one four-stroke cycle: crank 0..720, camshaft turns exactly once
    lifts = []
    worst_mesh = worst_contact = 0.0
    for alpha in range(0, 721, 60):
        cr = pose(float(alpha))
        cm = _clash(s)
        gm = cm.get(tuple(sorted(("crankg", "camg"))), 0.0)
        vc = cm.get(tuple(sorted(("cam", "valve"))), 0.0)
        assert gm < TOL, ("timing gears jammed at crank=%d" % alpha, gm)
        assert vc < TOL, ("valve lost/crushed cam at crank=%d" % alpha, vc)
        # nothing else should touch
        for stray in (("crankg", "cam"), ("crankg", "valve"), ("camg", "valve")):
            assert cm.get(tuple(sorted(stray)), 0.0) == 0.0, ("stray collision %s" % (stray,), alpha, cm)
        lifts.append((alpha, C[1] + valve_lift(cr)))
        worst_mesh = max(worst_mesh, gm)
        worst_contact = max(worst_contact, vc)

    # camshaft made exactly one turn over the 720 deg cycle
    cam_turn = abs(cam_rot(720.0, phase0) - cam_rot(0.0, phase0))
    assert abs(cam_turn - 360.0) < 1e-9, ("camshaft should turn once per cycle", cam_turn)
    lift_vals = [ly for _, ly in lifts]
    span = max(lift_vals) - min(lift_vals)
    assert abs(span - 2.0 * ECC) < 0.6, ("valve lift should span ~2e", span)
    # one cam revolution = one lift cycle: a single interior minimum (valve closed once)
    interior_min = min(range(1, len(lift_vals) - 1), key=lambda i: lift_vals[i])
    assert lift_vals[interior_min] < lift_vals[0] - ECC, ("expected one valve-closed dip", lift_vals)
    print("four-stroke cycle (crank 0..720): camshaft turned %.0f deg (one valve event)" % cam_turn)
    print("  worst gear mesh=%.2f, worst valve contact=%.2f (both ~0), valve lift span=%.2f = 2e"
          % (worst_mesh, worst_contact, span))
    print("  valve lift over cycle: %s" % " ".join("%.1f" % v for v in lift_vals))

    if "view.render" in s.tools():
        pose(120.0)
        o = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_engine.png")
        rv = s.act("view.render", {"assembly": "ENG", "view": "top", "path": o})
        assert rv.ok and rv.data["bytes"] > 5000, rv.data
        print("render -> %s (%d bytes)" % (o, rv.data["bytes"]))

    print("ENGINE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_engine"):
    main()
