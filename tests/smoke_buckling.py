"""Buckling smoke -- Euler critical load of a slender column vs closed form.

A compression column fails by buckling at P_cr = pi^2 E I / (K L)^2 about its
weakest axis. We validate against the exact rectangle/circle formulas:

  * a b x h x L rectangular column buckles about its weak axis (I_min =
    min(b h^3, h b^3)/12); P_cr, slenderness K L / r_min and critical stress
    pi^2 E / lambda^2 all match the closed form ;
  * the end-fixity factor K scales P_cr by 1/K^2 (fixed-fixed K=0.5 -> 4x) ;
  * a round bar matches pi^2 E (pi r^4/4)/L^2 to tessellation ;
  * a missing modulus is refused loudly.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=1e-4, abs_=1e-6):
    return abs(a - b) <= max(abs_, rel * abs(b))


def main():
    s = new_session("buckling")
    print("FreeCAD", s.registry.kernel.freecad_version)

    b, h, L, E = 20.0, 40.0, 600.0, 70000.0   # mm, mm, mm, MPa (Al)
    s.act("solid.box", {"name": "col", "length": b, "width": h, "height": L})
    r = s.act("solid.buckling", {"name": "col", "modulus": E})
    assert r.ok, r.error
    d = r.data
    area = b * h
    i_min = min(b * h ** 3, h * b ** 3) / 12.0   # weak axis
    i_max = max(b * h ** 3, h * b ** 3) / 12.0
    pcr = math.pi ** 2 * E * i_min / L ** 2
    rmin = math.sqrt(i_min / area)
    lam = L / rmin
    assert _close(d["length"], L), d["length"]
    assert _close(d["area"], area), d["area"]
    assert _close(d["I_min"], i_min) and _close(d["I_max"], i_max), d
    assert _close(d["critical_load"], pcr, rel=1e-3), (d["critical_load"], pcr)
    assert _close(d["slenderness_ratio"], lam, rel=1e-3), (d["slenderness_ratio"], lam)
    assert _close(d["critical_stress"], pcr / area, rel=1e-3), d["critical_stress"]
    assert _close(d["critical_stress"], math.pi ** 2 * E / lam ** 2, rel=1e-3), d
    print("rect column: I_min=%.0f P_cr=%.1f N lambda=%.1f sigma_cr=%.2f MPa (closed form)"
          % (i_min, pcr, lam, pcr / area))

    # ---- end fixity K scales P_cr by 1/K^2 ----------------------------- #
    rk = s.act("solid.buckling", {"name": "col", "modulus": E, "K": 0.5}).data
    assert _close(rk["critical_load"], pcr / 0.5 ** 2, rel=1e-3), (rk["critical_load"], pcr)
    print("fixed-fixed (K=0.5) raises P_cr 4x: %.1f N" % rk["critical_load"])

    # ---- round bar matches pi^2 E (pi r^4/4)/L^2 ----------------------- #
    R, Lr = 12.0, 500.0
    s.act("solid.cylinder", {"name": "rod", "radius": R, "height": Lr})
    rr = s.act("solid.buckling", {"name": "rod", "modulus": E}).data
    icirc = math.pi * R ** 4 / 4.0
    pcr_c = math.pi ** 2 * E * icirc / Lr ** 2
    assert _close(rr["I_min"], icirc, rel=2e-3), (rr["I_min"], icirc)
    assert _close(rr["critical_load"], pcr_c, rel=3e-3), (rr["critical_load"], pcr_c)
    print("round column: I=%.0f P_cr=%.1f N (== pi^2 E pi r^4/4 / L^2)" % (icirc, pcr_c))

    # ---- a missing modulus is refused ---------------------------------- #
    bad = s.act("solid.buckling", {"name": "col"})
    assert not bad.ok and "modulus" in (bad.error or "").lower(), bad.error
    print("missing modulus refused: %s" % bad.error)

    print("BUCKLING SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_buckling"):
    main()
