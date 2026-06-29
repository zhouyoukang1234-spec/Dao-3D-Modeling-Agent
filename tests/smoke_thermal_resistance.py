"""Thermal-resistance smoke -- steady conduction R=L/(kA) vs closed form.

Validated on real kernel solids:

  * a rectangular bar conducts R = L/(kA) and drops dT = Q R exactly ;
  * given temperature_drop instead, the conducted heat_flow = dT/R is recovered ;
  * a round rod uses the true section area pi r^2 ;
  * a film coefficient adds a convective resistance off the lateral area ;
  * missing/zero conductivity is refused.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=2e-3, abs_=1e-9):
    return abs(a - b) <= max(abs_, rel * abs(b))


def main():
    s = new_session("thermal_resistance")
    print("FreeCAD", s.registry.kernel.freecad_version)

    b, h, L, k = 20.0, 30.0, 100.0, 0.2   # mm, mm, mm, W/(mm.K) (~aluminium)
    s.act("solid.box", {"name": "bar", "length": b, "width": h, "height": L})
    A = b * h
    R = L / (k * A)
    Q = 50.0
    r = s.act("solid.thermal_resistance",
              {"name": "bar", "conductivity": k, "heat_flow": Q})
    assert r.ok, r.error
    d = r.data
    assert _close(d["area"], A), d["area"]
    assert _close(d["conduction_resistance"], R), (d["conduction_resistance"], R)
    assert _close(d["conductance"], 1.0 / R), d["conductance"]
    assert _close(d["temperature_drop"], Q * R), (d["temperature_drop"], Q * R)
    print("bar: R=%.5f K/W  Q=%.0f -> dT=%.3f K (closed form L/kA)"
          % (d["conduction_resistance"], Q, d["temperature_drop"]))

    # inverse: prescribe dT, recover Q = dT/R
    dT = 40.0
    di = s.act("solid.thermal_resistance",
               {"name": "bar", "conductivity": k, "temperature_drop": dT}).data
    assert _close(di["heat_flow"], dT / R), (di["heat_flow"], dT / R)
    print("inverse: dT=%.0f -> Q=%.3f W (= dT/R)" % (dT, di["heat_flow"]))

    # round rod uses the true circular section area
    rr, Lr = 10.0, 50.0
    s.act("solid.cylinder", {"name": "rod", "radius": rr, "height": Lr})
    dr = s.act("solid.thermal_resistance", {"name": "rod", "conductivity": k}).data
    Rr = Lr / (k * math.pi * rr ** 2)
    assert _close(dr["area"], math.pi * rr ** 2, rel=3e-3), dr["area"]
    assert _close(dr["conduction_resistance"], Rr, rel=3e-3), (dr["conduction_resistance"], Rr)
    print("rod: A=%.2f (pi r^2)  R=%.5f K/W" % (dr["area"], dr["conduction_resistance"]))

    # convective add-on off the lateral area (4*b? here box lateral = perimeter*L)
    hc = 2.5e-5
    dc = s.act("solid.thermal_resistance",
               {"name": "bar", "conductivity": k, "film_coefficient": hc}).data
    a_lat = dc["lateral_area"]
    assert _close(dc["convection_resistance"], 1.0 / (hc * a_lat), rel=3e-3), dc
    assert _close(dc["total_resistance"], dc["conduction_resistance"] + dc["convection_resistance"]), dc
    print("convection: A_lat=%.0f  R_conv=%.4f  R_total=%.4f K/W"
          % (a_lat, dc["convection_resistance"], dc["total_resistance"]))

    bad = s.act("solid.thermal_resistance", {"name": "bar"})
    assert not bad.ok and "conductivity" in (bad.error or ""), bad.error
    zero = s.act("solid.thermal_resistance", {"name": "bar", "conductivity": 0})
    assert not zero.ok, zero.error
    print("guards: missing/zero conductivity refused")

    print("THERMAL RESISTANCE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_thermal_resistance"):
    main()
