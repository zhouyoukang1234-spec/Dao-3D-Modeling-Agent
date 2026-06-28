"""Centrifugal (rotational body load) FEM vs the spinning-disk closed form.

A solid disk spun about its axis carries no surface load at all -- the stress
comes purely from the inertial body force rho*omega^2*r (CalculiX *DLOAD
CENTRIF). For a solid disk of radius R the stress peaks at the centre, where it
is equibiaxial (sigma_r = sigma_theta), with the classical value

    sigma_centre = (3 + nu)/8 * rho * omega^2 * R^2

(so the centre von Mises equals that same number, since sigma_r = sigma_theta
=> vM = sigma at the centre). A quarter model with symmetry rollers on its two
cut planes plus a base z-roller represents the full disk exactly while removing
all rigid-body modes without restraining the radial growth.

The suite checks:
  * the FEM centre stress matches (3+nu)/8 rho omega^2 R^2 within 5%;
  * it scales as omega^2 (doubling rpm quadruples the stress);
  * it scales as R^2 (a bigger disk is proportionally more stressed);
  * the peak is at the centre -- i.e. this is a genuine body-load field, a
    failure mode distinct from any applied surface load.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

E, NU = 210000.0, 0.30
RHO_KGM3 = 7900.0
RHO_TMM3 = RHO_KGM3 * 1e-12          # kg/m^3 -> tonne/mm^3 (MPa-mm-N-t units)
H = 10.0
STEEL = {"name": "Steel", "E": E, "nu": NU, "rho": RHO_KGM3, "yield": 250.0}


def centre_stress(R, rpm):
    omega = 2.0 * math.pi * rpm / 60.0
    return (3.0 + NU) / 8.0 * RHO_TMM3 * omega ** 2 * R ** 2


def run_disk(tag, R, rpm):
    s = new_session("spin_" + tag)
    # quarter disk in the +x +y quadrant: cylinder INTERSECT a corner box
    assert s.act("solid.cylinder", {"name": "full", "radius": R, "height": H}).ok
    assert s.act("solid.box", {"name": "corner", "length": R + 1.0,
                               "width": R + 1.0, "height": H + 2.0,
                               "pos": [0, 0, -1]}).ok
    assert s.act("solid.common", {"a": "full", "b": "corner", "out": "disk"}).ok
    st = s.act("fem.setup", {"target": "disk", "material": STEEL,
                             "order": 2, "mesh_size": R / 16.0})
    assert st.ok, st.error
    # symmetry rollers: x=0 cut plane -> zero x, y=0 cut plane -> zero y,
    # base z=0 -> zero z. Together: exact full-disk model, no rigid modes.
    assert s.act("fem.support", {"select": {"normal": [-1, 0, 0]}, "fix": ["x"]}).ok
    assert s.act("fem.support", {"select": {"normal": [0, -1, 0]}, "fix": ["y"]}).ok
    assert s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]}).ok
    sp = s.act("fem.spin", {"rpm": rpm, "axis": [0, 0, 1]})
    assert sp.ok, sp.error
    sv = s.act("fem.solve", {})
    assert sv.ok, sv.error
    fem_vm = sv.data["max_von_mises_mpa"]
    cf = centre_stress(R, rpm)
    print("  %-6s R=%.0f rpm=%.0f nodes=%d  FEM peak vM=%.4f  closed=%.4f  ratio=%.4f"
          % (tag, R, rpm, st.data["nodes"], fem_vm, cf, fem_vm / cf))
    s.registry.kernel.shutdown()
    return fem_vm, cf


def main():
    print("Spinning-disk centrifugal FEM vs (3+nu)/8 rho omega^2 R^2")
    f1, c1 = run_disk("base", 100.0, 3000.0)
    f2, c2 = run_disk("fast", 100.0, 6000.0)
    f3, c3 = run_disk("big", 150.0, 3000.0)

    # 1) matches the closed form within 5% (peak == centre for a solid disk)
    assert abs(f1 / c1 - 1.0) < 0.05, ("base vs closed form", f1, c1)
    assert abs(f2 / c2 - 1.0) < 0.05, ("fast vs closed form", f2, c2)
    assert abs(f3 / c3 - 1.0) < 0.05, ("big vs closed form", f3, c3)

    # 2) sigma ~ omega^2 : doubling rpm quadruples the stress
    assert abs((f2 / f1) - (6000.0 / 3000.0) ** 2) < 0.05, ("not omega^2", f1, f2)

    # 3) sigma ~ R^2 : 1.5x radius -> 2.25x stress
    assert abs((f3 / f1) - (150.0 / 100.0) ** 2) < 0.05, ("not R^2", f1, f3)

    print("FEM matches closed form within: base %.2f%%, fast %.2f%%, big %.2f%%"
          % (abs(f1 / c1 - 1) * 100, abs(f2 / c2 - 1) * 100, abs(f3 / c3 - 1) * 100))
    print("sigma ~ omega^2: f(6000)/f(3000) = %.3f (=2^2=4.000)" % (f2 / f1))
    print("sigma ~ R^2: f(150)/f(100) = %.3f (=1.5^2=2.250)" % (f3 / f1))
    print("ROTORDISK SMOKE OK")


if __name__ in ("__main__", "smoke_rotordisk"):
    main()
