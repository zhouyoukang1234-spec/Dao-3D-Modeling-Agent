"""Gear-tooth bending FEM vs the Lewis equation -- close the geometry<->simulation
loop on a real machine element (the way the cantilever beam closed at 0.4%).

A gear tooth is, in Lewis's idealization, a cantilever beam built into the rim and
loaded tangentially at the tip; the root bending stress is

    sigma = Ft / (b * m * Y)            (Lewis)

with Y the dimensionless form factor set purely by the tooth's proportions. We
model the tooth as exactly that beam -- a trapezoidal cantilever (root thickness
t_root tapering to t_tip over height h) built in at the root -- mesh it with gmsh,
fix the root, push the tip tangentially with CalculiX, and read max von Mises.

Because the FEM is the ground truth and Lewis is the (approximate) beam model, we
assert the things that must hold for the simulation to be physically right rather
than chasing a single magic number:

  * the FEM root stress matches the closed-form cantilever fibre stress
    Sxx = 6*Ft*h/(b*t_root^2) within 20% (the geometry<->FEM<->analytical loop);
  * linearity: doubling Ft doubles the stress (linear elasticity);
  * the back-figured form factor Y = Ft/(b*m*sigma) lands in the documented
    involute range;
  * Lewis physics: a slender (tall, thin-root) tooth is weaker -- higher root
    stress and a smaller Y -- than a stubby tooth, exactly as the formula says;
  * a von Mises contour shows the peak sitting at the loaded root (rendered).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

M = 4.0          # module
B = 12.0         # face width
MAT = {"E": 210000.0, "nu": 0.30, "yield": 250.0}


def tooth_polygon(t_root, t_tip, h):
    """Trapezoidal tooth cross-section in X (tangential) - Y (radial), CCW.
    Root edge sits on y=0 (built in), tip edge at y=h."""
    return [
        [-t_root / 2.0, 0.0], [t_root / 2.0, 0.0],
        [t_tip / 2.0, h], [-t_tip / 2.0, h],
    ]


def run_tooth(tag, t_root, t_tip, h, Ft, render=None):
    """Build one tooth, FEM it, return (max_von_mises, nominal_beam_stress)."""
    s = new_session("gearfem_" + tag)
    body = "tooth_" + tag
    assert s.act("solid.extrude", {"name": body,
                                   "profile": {"polygon": tooth_polygon(t_root, t_tip, h)},
                                   "dir": [0, 0, B]}).ok

    st = s.act("fem.setup", {"target": body, "material": {"name": "Steel", **MAT},
                             "order": 2, "mesh_size": 2.0})
    assert st.ok, st.error
    # build in the root (outward normal -Y), push the tip tangentially (+X)
    assert s.act("fem.fix", {"select": {"normal": [0, -1, 0], "min_dot": 0.9}}).ok
    ld = s.act("fem.load", {"select": {"normal": [0, 1, 0], "min_dot": 0.9},
                            "kind": "force", "value": Ft, "direction": [1, 0, 0]})
    assert ld.ok, ld.error
    assert abs(ld.data["effective_dir"][0] - 1.0) < 1e-3, ("load not tangential", ld.data)
    sol = s.act("fem.solve", {"allowable_mpa": MAT["yield"]})
    assert sol.ok, sol.error
    vm = sol.data["max_von_mises_mpa"]
    nominal = 6.0 * Ft * h / (B * t_root * t_root)   # plain cantilever root fibre stress
    print("  %-8s Ft=%4.0f N  nodes=%d  max_vM=%.2f MPa  beam=%.2f MPa  SCF=%.2f"
          % (tag, Ft, st.data["nodes"], vm, nominal, vm / nominal))
    if render and "fem.contour" in s.tools():
        rv = s.act("fem.contour", {"path": render, "view": "top"})
        if rv.ok:
            print("  contour -> %s (%d bytes)" % (render, rv.data["bytes"]))
    s.registry.kernel.shutdown()
    return vm, nominal


def main():
    print("Gear-tooth bending FEM vs Lewis  (module m=%.0f, face b=%.0f)" % (M, B))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)

    # stubby tooth (thick root, short) -- the strong one
    vm_s, nom_s = run_tooth("stubby", t_root=10.0, t_tip=6.0, h=7.0, Ft=400.0,
                            render=os.path.join(out, "smoke_gear_fem.png"))
    # same tooth, double load -> linear elasticity
    vm_s2, _ = run_tooth("stubby2x", t_root=10.0, t_tip=6.0, h=7.0, Ft=800.0)
    # slender tooth (thin root, tall) -- the weak one
    vm_l, nom_l = run_tooth("slender", t_root=7.0, t_tip=3.5, h=11.0, Ft=400.0)

    # 1) linear elasticity: 2x load -> 2x stress
    assert abs(vm_s2 / vm_s - 2.0) < 0.03, ("not linear", vm_s, vm_s2)

    # 2) FEM root stress matches the closed-form cantilever beam within 20%
    r_s, r_l = vm_s / nom_s, vm_l / nom_l
    assert abs(r_s - 1.0) < 0.20, ("stubby FEM vs beam", vm_s, nom_s, r_s)
    assert abs(r_l - 1.0) < 0.20, ("slender FEM vs beam", vm_l, nom_l, r_l)

    # 3) back-figured Lewis form factor in the documented involute band
    def lewis_Y(vm, Ft):
        return Ft / (B * M * vm)
    Y_s, Y_l = lewis_Y(vm_s, 400.0), lewis_Y(vm_l, 400.0)
    assert 0.10 <= Y_s <= 0.60, ("stubby Y out of range", Y_s)
    assert 0.10 <= Y_l <= 0.60, ("slender Y out of range", Y_l)

    # 4) Lewis physics: the slender tooth is weaker -- more stress, smaller Y
    assert vm_l > vm_s, ("slender tooth should be more stressed", vm_l, vm_s)
    assert Y_l < Y_s, ("slender tooth should have smaller form factor", Y_l, Y_s)

    print("FEM vs closed-form beam: stubby %.2f/%.2f=%.0f%%, slender %.2f/%.2f=%.0f%% (<20%% off)"
          % (vm_s, nom_s, abs(r_s - 1) * 100, vm_l, nom_l, abs(r_l - 1) * 100))
    print("linearity: 800N/400N stress ratio = %.3f (=2)" % (vm_s2 / vm_s))
    print("Lewis form factor Y: stubby=%.3f, slender=%.3f (involute range)"
          % (Y_s, Y_l))
    print("Lewis physics confirmed: slender tooth weaker (vM %.1f > %.1f, Y %.3f < %.3f)"
          % (vm_l, vm_s, Y_l, Y_s))
    print("GEAR-FEM SMOKE OK")


if __name__ in ("__main__", "smoke_gear_fem"):
    main()
