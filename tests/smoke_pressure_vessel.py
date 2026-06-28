"""Thick-walled pressure vessel FEM vs the Lamé closed form -- validating the
*pressure* load path and the new roller/symmetry support (fem.support).

A quarter of a thick ring (inner radius a, outer b, internal pressure p) is meshed
with gmsh; the two cut planes get symmetry rollers (zero only the in-plane normal
displacement, not a full clamp), the base gets an axial roller, and the bore is
pressurised. CalculiX's peak von Mises sits at the inner wall, where Lamé gives

    sigma_theta(a) = p (b^2 + a^2) / (b^2 - a^2),  sigma_r(a) = -p,  sigma_z ~ 0
    vM = sqrt(sigma_theta^2 + sigma_r^2 - sigma_theta*sigma_r)

This is the exact analogue of the cantilever-beam check, on the pressure path:

  * FEM peak vM matches the Lamé inner-wall vM within 5%;
  * it scales linearly with pressure;
  * a thicker wall lowers the stress, and still matches its own Lamé value;
  * the roller BC is *necessary*: fully clamping the symmetry planes instead
    corrupts the field (it fights the radial growth), so that result must NOT
    match Lamé -- proving fem.support is doing real work, not decoration.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

NU = 0.30
YIELD = 600.0


def lame_vm(a, b, p):
    sth = p * (b * b + a * a) / (b * b - a * a)
    sr = -p
    return math.sqrt(sth * sth + sr * sr - sth * sr)


def build_quarter(s, a, b, h):
    assert s.act("solid.cylinder", {"name": "outer", "radius": b, "height": h}).ok
    assert s.act("solid.cylinder", {"name": "inner", "radius": a, "height": h}).ok
    assert s.act("solid.cut", {"a": "outer", "b": "inner", "out": "ring"}).ok
    assert s.act("solid.box", {"name": "q", "length": b + 1, "width": b + 1,
                               "height": h + 2, "pos": [0, 0, -1]}).ok
    assert s.act("solid.common", {"a": "ring", "b": "q", "out": "quarter"}).ok


def run_vessel(tag, a, b, h, p, clamp=False, render=None):
    s = new_session("pv_" + tag)
    build_quarter(s, a, b, h)
    st = s.act("fem.setup", {"target": "quarter",
                             "material": {"name": "Steel", "E": 210000.0, "nu": NU, "yield": YIELD},
                             "order": 2, "mesh_size": 2.5})
    assert st.ok, st.error
    if clamp:
        # WRONG on purpose: a full clamp on the symmetry planes also locks
        # tangential/axial motion and fights the radial growth.
        assert s.act("fem.fix", {"select": {"normal": [0, -1, 0], "min_dot": 0.9}}).ok
        assert s.act("fem.fix", {"select": {"normal": [-1, 0, 0], "min_dot": 0.9}}).ok
    else:
        assert s.act("fem.support", {"select": {"normal": [0, -1, 0], "min_dot": 0.9}, "fix": ["y"]}).ok
        assert s.act("fem.support", {"select": {"normal": [-1, 0, 0], "min_dot": 0.9}, "fix": ["x"]}).ok
    assert s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]}).ok
    pl = s.act("fem.load", {"select": {"cyl_radius": a}, "kind": "pressure", "value": p})
    assert pl.ok and pl.data["faces"], pl.error
    sol = s.act("fem.solve", {"allowable_mpa": YIELD})
    assert sol.ok, sol.error
    vm = sol.data["max_von_mises_mpa"]
    ref = lame_vm(a, b, p)
    print("  %-9s a=%.0f b=%.0f p=%.0f  nodes=%d  FEM vM=%.2f  Lame=%.2f  ratio=%.3f%s"
          % (tag, a, b, p, st.data["nodes"], vm, ref, vm / ref, "  [CLAMPED]" if clamp else ""))
    if render and "fem.contour" in s.tools():
        rv = s.act("fem.contour", {"path": render, "view": "top"})
        if rv.ok:
            print("  contour -> %s (%d bytes)" % (render, rv.data["bytes"]))
    s.registry.kernel.shutdown()
    return vm, ref


def main():
    print("Thick-wall pressure vessel FEM vs Lame")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)

    vm1, ref1 = run_vessel("base", 20.0, 40.0, 4.0, 50.0,
                           render=os.path.join(out, "smoke_pressure_vessel.png"))
    vm2, ref2 = run_vessel("double_p", 20.0, 40.0, 4.0, 100.0)
    vm3, ref3 = run_vessel("thickwall", 20.0, 60.0, 4.0, 50.0)
    vmc, _ = run_vessel("clamped", 20.0, 40.0, 4.0, 50.0, clamp=True)

    # 1) the roller model matches Lame at the inner wall
    assert abs(vm1 / ref1 - 1.0) < 0.05, ("base vs Lame", vm1, ref1)
    assert abs(vm3 / ref3 - 1.0) < 0.05, ("thickwall vs Lame", vm3, ref3)

    # 2) linear in pressure
    assert abs(vm2 / vm1 - 2.0) < 0.03, ("not linear in p", vm1, vm2)

    # 3) Lame physics: thicker wall -> lower stress
    assert vm3 < vm1, ("thicker wall should reduce stress", vm3, vm1)
    assert ref3 < ref1

    # 4) the roller is necessary: a full clamp must NOT reproduce Lame
    assert abs(vmc / ref1 - 1.0) > 0.10, ("clamp should corrupt the field", vmc, ref1)

    print("inner-wall vM matches Lame: base %.1f%%, thickwall %.1f%% off"
          % (abs(vm1 / ref1 - 1) * 100, abs(vm3 / ref3 - 1) * 100))
    print("linear in p: 100/50 ratio = %.3f (=2)" % (vm2 / vm1))
    print("thicker wall lowers stress: %.1f -> %.1f MPa (Lame %.1f -> %.1f)"
          % (vm1, vm3, ref1, ref3))
    print("roller required: full clamp gives %.1f vs Lame %.1f (%.0f%% off -> wrong BC)"
          % (vmc, ref1, abs(vmc / ref1 - 1) * 100))
    print("PRESSURE-VESSEL SMOKE OK")


if __name__ in ("__main__", "smoke_pressure_vessel"):
    main()
