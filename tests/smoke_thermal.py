"""Thermomechanical (steady-state) FEM vs the closed-form thermal stress.

A prismatic bar is given a uniform temperature rise dT above its stress-free
reference. If it can expand freely it develops *no* stress; if its axial growth
is blocked it develops the classic uniaxial thermal stress

    sigma = E * alpha * dT   (compressive),   von Mises = E * alpha * dT.

The bar is supported as a statically-determinate frame: the min-x / min-y /
min-z faces get a roller each (only the normal component zeroed), so the body
is free of rigid-body motion yet free to expand outward from the origin corner.
Blocking the max-z face axially as well is what creates the stress. The suite:

  * free expansion -> essentially zero stress (the BC, not the heat, makes stress);
  * axially blocked -> max von Mises matches E*alpha*dT within 3%;
  * it is linear in dT (double the rise -> double the stress);
  * the temperature field actually equilibrates to the imposed dT everywhere.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

E = 210000.0
ALPHA = 1.2e-5      # steel, matches the fem material library
NU = 0.30


def run_bar(tag, dT, block_axial, render=None):
    s = new_session("therm_" + tag)
    assert s.act("solid.box", {"name": "bar", "length": 10, "width": 10,
                               "height": 100}).ok
    st = s.act("fem.setup", {"target": "bar",
                             "material": {"name": "Steel", "E": E, "nu": NU,
                                          "yield": 600.0, "alpha": ALPHA},
                             "order": 2, "mesh_size": 4.0})
    assert st.ok, st.error
    # statically-determinate rollers: free thermal expansion from the origin corner
    assert s.act("fem.support", {"select": {"axis": "x", "side": "min"}, "fix": ["x"]}).ok
    assert s.act("fem.support", {"select": {"axis": "y", "side": "min"}, "fix": ["y"]}).ok
    assert s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]}).ok
    if block_axial:
        assert s.act("fem.support", {"select": {"axis": "z", "side": "max"}, "fix": ["z"]}).ok
    assert s.act("fem.temperature", {"value": dT, "ref": 0.0}).ok
    sol = s.act("fem.thermal", {"allowable_mpa": 600.0})
    assert sol.ok, sol.error
    vm = sol.data["max_von_mises_mpa"]
    ref = E * ALPHA * dT
    print("  %-9s dT=%.0f block=%s nodes=%d  FEM vM=%.2f  E*alpha*dT=%.2f  T=[%.0f,%.0f]"
          % (tag, dT, block_axial, st.data["nodes"], vm, ref,
             sol.data["t_min_k"], sol.data["t_max_k"]))
    if render and "fem.contour" in s.tools():
        rv = s.act("fem.contour", {"path": render, "view": "iso"})
        if rv.ok:
            print("  contour -> %s (%d bytes)" % (render, rv.data["bytes"]))
    s.registry.kernel.shutdown()
    return vm, ref, sol.data


def main():
    print("Thermomechanical bar FEM vs E*alpha*dT")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)

    vm_free, _, _ = run_bar("free", 100.0, False)
    vm1, ref1, d1 = run_bar("blocked", 100.0, True,
                            render=os.path.join(out, "smoke_thermal.png"))
    vm2, ref2, _ = run_bar("double_dT", 200.0, True)

    # 1) free expansion develops no meaningful stress
    assert vm_free < 5.0, ("free expansion should be ~0 stress", vm_free)

    # 2) blocked axial growth -> sigma = E*alpha*dT within 3%
    assert abs(vm1 / ref1 - 1.0) < 0.03, ("blocked vs closed form", vm1, ref1)
    assert abs(vm2 / ref2 - 1.0) < 0.03, ("double_dT vs closed form", vm2, ref2)

    # 3) linear in dT
    assert abs(vm2 / vm1 - 2.0) < 0.03, ("not linear in dT", vm1, vm2)

    # 4) the constraint, not the heat, makes the stress
    assert vm1 > 50.0 * vm_free, ("blocking must dominate free", vm1, vm_free)

    # 5) the temperature field reached the imposed rise
    assert abs(d1["t_max_k"] - 100.0) < 1e-3 and abs(d1["t_min_k"] - 100.0) < 1e-3, \
        ("temperature did not equilibrate", d1)

    print("free expansion vM=%.2f (~0); blocked matches E*alpha*dT: %.1f%%, %.1f%% off"
          % (vm_free, abs(vm1 / ref1 - 1) * 100, abs(vm2 / ref2 - 1) * 100))
    print("linear in dT: 200/100 ratio = %.3f (=2)" % (vm2 / vm1))
    print("constraint makes the stress: blocked %.1f >> free %.2f MPa" % (vm1, vm_free))
    print("THERMAL SMOKE OK")


if __name__ in ("__main__", "smoke_thermal"):
    main()
