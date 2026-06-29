"""Hydrostatics smoke — free-floating draft, buoyancy and metacentre.

``solid.hydrostatics`` solves the still-water plane from Archimedes' balance and
reads draft / submerged volume / B / waterplane / BM / GM off the real cut
solids. Validated against the closed forms for a floating box bxLxH at density
ratio r = rho_part/rho_fluid:

  * draft T = r*H;  submerged volume = b*L*T;
  * KB = T/2 (centroid of the submerged box);
  * waterplane area = b*L;  BMt = b^2/(12*T) (b = smaller plan side);
  * a part denser than the fluid reports floats == False.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=2e-3):
    return abs(got - exp) <= rel * abs(exp) + 1e-4


def main():
    s = new_session("hydro")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # box 40 (x) by 30 (y) by 20 (z), half-density -> floats half submerged.
    L, b, H = 40.0, 30.0, 20.0
    s.act("solid.box", {"name": "hull", "length": L, "width": b, "height": H})
    r = s.act("solid.hydrostatics", {"name": "hull", "density": 0.5,
                                     "fluid_density": 1.0}).data
    print("box  : draft", r["draft"], "Vsub", r["submerged_volume"],
          "KB", r["KB"], "Awp", r["waterplane_area"], "BM", r["BM"], "GM", r["GM"])
    T = 0.5 * H
    assert r["floats"] is True, r
    assert _close(r["draft"], T), r
    assert _close(r["submerged_volume"], L * b * T), r
    assert _close(r["KB"], T / 2.0), r
    assert _close(r["waterplane_area"], L * b), r
    assert _close(r["BM"], b**2 / (12.0 * T)), r        # transverse uses smaller side
    assert _close(r["GM"], r["KB"] + r["BM"] - r["KG"]), r

    # a third-density box sits shallower: T = H/3.
    r3 = s.act("solid.hydrostatics", {"name": "hull", "density": 1.0,
                                      "fluid_density": 3.0}).data
    print("1/3  : draft", r3["draft"])
    assert _close(r3["draft"], H / 3.0), r3

    # denser than water -> sinks.
    sink = s.act("solid.hydrostatics", {"name": "hull", "density": 2.7,
                                        "fluid_density": 1.0}).data
    assert sink["floats"] is False, sink

    print("HYDRO SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_hydro"):
    main()
