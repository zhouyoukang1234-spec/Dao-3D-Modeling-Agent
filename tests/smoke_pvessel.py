"""Pressure-vessel smoke — thin-wall membrane stresses (Barlow/boiler formula).

``solid.pressure_vessel`` gives the membrane stresses of a thin shell under
internal pressure. Validated against the closed forms:

  * cylinder: hoop = p*r/t, longitudinal = p*r/(2t), hoop is twice longitudinal;
  * sphere:   both = p*r/(2t);
  * von Mises of (sigma_h, sigma_l) and the yield safety factor sy/vm;
  * r/t >= 10 flags ``thin_wall`` True.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-4):
    return abs(got - exp) <= rel * abs(exp) + 1e-4


def main():
    s = new_session("pvessel")
    print("FreeCAD", s.registry.kernel.freecad_version)

    p, r, t = 2.0, 500.0, 10.0       # 2 MPa, r=500 mm, t=10 mm  (r/t=50)
    cyl = s.act("solid.pressure_vessel",
                {"pressure": p, "radius": r, "thickness": t,
                 "kind": "cylinder", "yield_strength": 250.0}).data
    print("cyl:", cyl["hoop_stress"], cyl["longitudinal_stress"],
          cyl["von_mises"], cyl["safety_factor"])
    assert _close(cyl["hoop_stress"], p * r / t), cyl                  # 100
    assert _close(cyl["longitudinal_stress"], p * r / (2 * t)), cyl    # 50
    assert _close(cyl["hoop_stress"], 2 * cyl["longitudinal_stress"]), cyl
    vm = math.sqrt((p * r / t) ** 2 - (p * r / t) * (p * r / (2 * t))
                   + (p * r / (2 * t)) ** 2)
    assert _close(cyl["von_mises"], vm), cyl
    assert _close(cyl["safety_factor"], 250.0 / vm), cyl
    assert cyl["thin_wall"] is True, cyl

    sph = s.act("solid.pressure_vessel",
                {"pressure": p, "radius": r, "thickness": t,
                 "kind": "sphere"}).data
    print("sph:", sph["hoop_stress"], sph["von_mises"])
    assert _close(sph["hoop_stress"], p * r / (2 * t)), sph            # 50
    assert _close(sph["longitudinal_stress"], p * r / (2 * t)), sph
    assert _close(sph["von_mises"], p * r / (2 * t)), sph              # equibiaxial

    # thick wall flagged
    thick = s.act("solid.pressure_vessel",
                  {"pressure": p, "radius": 30.0, "thickness": 10.0}).data
    assert thick["thin_wall"] is False, thick

    # missing radius/thickness -> a guided error, not a raw TypeError on float(None).
    bad = s.act("solid.pressure_vessel", {"pressure": p, "thickness": t})
    assert not bad.ok and "radius" in (bad.error or ""), bad.error
    assert "TypeError" not in (bad.error or ""), bad.error
    print("missing radius refused cleanly: %s" % bad.error)

    print("PVESSEL SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_pvessel"):
    main()
