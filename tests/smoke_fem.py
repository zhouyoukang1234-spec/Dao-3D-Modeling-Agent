"""FEM smoke + physics-driven closed loop (needs FreeCAD with Fem + ccx).

1. Solve a cantilever beam under a tip load and check the FEM max von-Mises
   stress against the analytic bending solution sigma = 6 F L / (b H^2) — proof
   the solver path is physically correct, not a rubber stamp.
2. Run an autonomous *design -> simulate -> self-correct* loop: bisect the beam
   height H until the simulated max stress meets an allowable target. This is
   the closed loop driven by real physics instead of geometry heuristics.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _solve_beam(s, L, b, H, force, material="steel", allow=None):
    """(re)build a cantilever beam and return the FEM solve result dict."""
    s.act("solid.box", {"name": "beam", "length": L, "width": b, "height": H})
    r = s.act("fem.setup", {"target": "beam", "material": material})
    assert r.ok, r.error
    nodes = r.data["nodes"]
    r = s.act("fem.fix", {"select": {"axis": "x", "side": "min"}})
    assert r.ok, r.error
    r = s.act("fem.load", {"select": {"axis": "x", "side": "max"},
                           "kind": "force", "value": force, "direction": [0, 0, -1]})
    assert r.ok, r.error
    args = {} if allow is None else {"allowable_mpa": allow}
    r = s.act("fem.solve", args, )
    assert r.ok, r.error
    r.data["nodes"] = nodes
    return r.data


def main():
    s = new_session("fem")
    print("FreeCAD", s.registry.kernel.freecad_version)
    femtools = [t for t in s.tools() if t.startswith("fem.")]
    print("fem ops:", sorted(femtools))
    assert "fem.solve" in femtools, "fem.* tools not registered — Fem/ccx missing?"

    # --- 1. validate against the analytic cantilever bending stress ----------
    L, b, H, F = 100.0, 10.0, 10.0, 1000.0
    d = _solve_beam(s, L, b, H, F)
    analytic = 6.0 * F * L / (b * H * H)   # MPa, max fibre bending stress
    fem_vm = d["max_von_mises_mpa"]
    rel = abs(fem_vm - analytic) / analytic
    print("beam %dx%dx%d F=%gN nodes=%d: FEM max von Mises=%.1f MPa  analytic=%.1f MPa  rel=%.1f%%  disp=%.4f mm"
          % (L, b, H, F, d["nodes"], fem_vm, analytic, 100 * rel, d["max_disp_mm"]))
    # quadratic-element FEM matches slender-beam theory closely; allow for mesh
    # discretisation + clamped-edge stress concentration.
    assert 0.8 * analytic <= fem_vm <= 1.6 * analytic, (fem_vm, analytic)
    # tip deflection delta = F L^3 / (3 E Iyy),  Iyy = b H^3 / 12
    Iyy = b * H ** 3 / 12.0
    disp_analytic = F * L ** 3 / (3.0 * 210000.0 * Iyy)
    assert abs(d["max_disp_mm"] - disp_analytic) / disp_analytic < 0.25, (d["max_disp_mm"], disp_analytic)

    # --- 1b. genericity: same fem.* ops drive a complex L-bracket body --------
    assert s.act("param.body", {"name": "BR"}).ok
    Lprof = [[0, 0], [40, 0], [40, 8], [8, 8], [8, 40], [0, 40]]
    assert s.act("param.pad", {"body": "BR", "feature": "LWall",
                               "profile": {"polygon": Lprof}, "length": 30}).ok
    r = s.act("fem.setup", {"target": "BR", "material": "aluminum"})
    assert r.ok, r.error
    assert s.act("fem.fix", {"select": {"axis": "z", "side": "min"}}).ok
    ld = s.act("fem.load", {"select": {"axis": "y", "side": "max"},
                            "kind": "force", "value": 500, "direction": [0, -1, 0]})
    assert ld.ok, ld.error
    # the load must actually point where asked (self-corrected against FreeCAD's
    # own DirectionVector sign convention)
    assert ld.data["effective_dir"][1] < -0.99, ld.data
    rb = s.act("fem.solve", {"allowable_mpa": 240})
    assert rb.ok, rb.error
    print("L-bracket (Al, %d nodes): max vM=%.2f MPa  SF=%.1f  passed=%s"
          % (r.data["nodes"], rb.data["max_von_mises_mpa"], rb.data["safety_factor"], rb.data["passed"]))
    assert rb.data["max_von_mises_mpa"] > 0.5 and rb.data["passed"]

    # --- 1c. stress-cloud contour render (the agent's eye on the field) -------
    _solve_beam(s, L, b, 10.0, F)  # fresh static result on the beam
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_beam_stress.png")
    rc = s.act("fem.contour", {"path": out, "view": "iso"})
    assert rc.ok, rc.error
    assert rc.data["bytes"] > 5000 and rc.data["nodes"] > 100, rc.data
    print("contour render: %d nodes, %d bytes -> %s" % (rc.data["nodes"], rc.data["bytes"], out))

    # --- 1d. modal analysis vs analytic cantilever 1st natural frequency ------
    s.act("solid.box", {"name": "mbeam", "length": L, "width": b, "height": H})
    assert s.act("fem.setup", {"target": "mbeam", "material": "steel"}).ok
    assert s.act("fem.fix", {"select": {"axis": "x", "side": "min"}}).ok
    rm = s.act("fem.modal", {"modes": 5})
    assert rm.ok, rm.error
    f1 = rm.data["frequencies_hz"][0]
    # Euler-Bernoulli: f1 = (1.875^2 / 2pi) * sqrt(E I / (rho A L^4))  [SI]
    E = 210e9
    Im = (b * 1e-3) * (H * 1e-3) ** 3 / 12.0
    A = (b * 1e-3) * (H * 1e-3)
    rho = 7900.0
    Lm = L * 1e-3
    f1_analytic = (1.875 ** 2) / (2 * 3.141592653589793 * Lm ** 2) * (E * Im / (rho * A)) ** 0.5
    print("modal: f1=%.1f Hz  analytic=%.1f Hz  modes=%s" % (f1, f1_analytic, rm.data["frequencies_hz"]))
    assert 0.7 * f1_analytic <= f1 <= 1.5 * f1_analytic, (f1, f1_analytic)

    # --- 2. physics-driven closed loop: size H so max stress ~ allowable ------
    allow = 200.0      # MPa target
    lo, hi = 6.0, 40.0
    best = None
    print("\n--- autosize: bisect beam height H until max von Mises <= %g MPa ---" % allow)
    for it in range(1, 9):
        H = round((lo + hi) / 2.0, 3)
        d = _solve_beam(s, L, b, H, F, allow=allow)
        vm = d["max_von_mises_mpa"]
        ok = vm <= allow
        print("  iter %d  H=%.3f mm -> max vM=%.1f MPa  SF=%.2f  %s"
              % (it, H, vm, d["safety_factor"], "PASS" if ok else "over"))
        if ok:
            best = (H, vm, d["safety_factor"])
            hi = H            # try thinner
        else:
            lo = H            # need thicker
        if abs(vm - allow) / allow < 0.05:
            break
    assert best is not None, "loop never found a passing height"
    print("converged: H=%.3f mm gives max vM=%.1f MPa (target %g, SF=%.2f)"
          % (best[0], best[1], allow, best[2]))
    # analytic height for the target: H = sqrt(6 F L / (b * allow))
    h_analytic = (6.0 * F * L / (b * allow)) ** 0.5
    print("analytic height for target: %.2f mm (FEM-sized %.2f mm)" % (h_analytic, best[0]))

    print("FEM SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_fem"):
    main()
