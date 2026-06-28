"""Linear (Euler) buckling FEM vs the closed-form column buckling load.

A slender square column is built in at its base and pushed axially at its free
top; CalculiX's *BUCKLE step returns the load factor lambda such that the
structure goes unstable at lambda*P_applied. For a fixed-free (cantilever)
column the Euler critical load is

    Pcr = pi^2 * E * I / (K L)^2,   K = 2 (fixed-free),   I = b^4 / 12

so with a 1000 N reference load the factor must equal Pcr/1000. The suite checks:

  * the FEM factor matches Euler (K=2) within 5%;
  * it scales as 1/L^2 (a longer column buckles at a proportionally lower load);
  * it scales as I ~ b^4 (a fatter column is dramatically stiffer);
  * the critical load is far above the *stress* limit for this slender column --
    i.e. it would buckle long before it yields, which is *why* a buckling check
    is a distinct, necessary failure mode, not a corollary of the stress check.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

E = 210000.0
P_REF = 1000.0


def euler_factor(L, b, K=2.0):
    second_moment = b ** 4 / 12.0
    return (math.pi ** 2 * E * second_moment / (K * L) ** 2) / P_REF


def run_column(tag, L, b):
    s = new_session("buck_" + tag)
    assert s.act("solid.box", {"name": "col", "length": b, "width": b,
                               "height": L}).ok
    st = s.act("fem.setup", {"target": "col",
                             "material": {"name": "Steel", "E": E, "nu": 0.30, "yield": 250.0},
                             "order": 2, "mesh_size": b / 2.0})
    assert st.ok, st.error
    assert s.act("fem.fix", {"select": {"axis": "z", "side": "min"}}).ok
    ld = s.act("fem.load", {"select": {"axis": "z", "side": "max"},
                            "kind": "force", "value": P_REF, "direction": [0, 0, -1]})
    assert ld.ok, ld.error
    bk = s.act("fem.buckle", {"modes": 1})
    assert bk.ok, bk.error
    factor = bk.data["critical_factor"]
    euler = euler_factor(L, b)
    print("  %-9s L=%.0f b=%.0f nodes=%d  FEM factor=%.3f  Euler(K=2)=%.3f  ratio=%.3f  Pcr=%.0f N"
          % (tag, L, b, st.data["nodes"], factor, euler, factor / euler, factor * P_REF))
    s.registry.kernel.shutdown()
    return factor, euler


def main():
    print("Euler column buckling FEM vs pi^2 EI/(KL)^2")
    f1, e1 = run_column("base", 200.0, 10.0)
    f2, e2 = run_column("long", 300.0, 10.0)
    f3, e3 = run_column("fat", 200.0, 15.0)

    # 1) matches Euler (fixed-free, K=2) within 5%
    assert abs(f1 / e1 - 1.0) < 0.05, ("base vs Euler", f1, e1)
    assert abs(f2 / e2 - 1.0) < 0.05, ("long vs Euler", f2, e2)
    assert abs(f3 / e3 - 1.0) < 0.05, ("fat vs Euler", f3, e3)

    # 2) Pcr ~ 1/L^2 : the 1.5x-longer column buckles at (200/300)^2 of the load
    assert abs((f2 / f1) - (200.0 / 300.0) ** 2) < 0.05, ("not 1/L^2", f1, f2)

    # 3) Pcr ~ I ~ b^4 : 1.5x thicker -> (15/10)^4 = 5.06x stiffer
    assert abs((f3 / f1) - (15.0 / 10.0) ** 4) < 0.10, ("not b^4", f1, f3)

    # 4) buckling is the governing mode: Pcr is well below the load that would
    #    yield this slender column (sigma=Pcr/A vs 250 MPa) -> distinct check.
    sigma_at_buckle = f1 * P_REF / (10.0 * 10.0)
    assert sigma_at_buckle < 250.0, ("column should buckle before yield", sigma_at_buckle)

    print("FEM matches Euler within: base %.1f%%, long %.1f%%, fat %.1f%%"
          % (abs(f1 / e1 - 1) * 100, abs(f2 / e2 - 1) * 100, abs(f3 / e3 - 1) * 100))
    print("Pcr ~ 1/L^2: f(300)/f(200) = %.3f (=(2/3)^2=%.3f)"
          % (f2 / f1, (200.0 / 300.0) ** 2))
    print("Pcr ~ b^4: f(15)/f(10) = %.3f (=(3/2)^4=%.3f)"
          % (f3 / f1, (15.0 / 10.0) ** 4))
    print("buckling governs: sigma@Pcr = %.1f MPa < 250 MPa yield (slender column)"
          % sigma_at_buckle)
    print("BUCKLING SMOKE OK")


if __name__ in ("__main__", "smoke_buckling"):
    main()
