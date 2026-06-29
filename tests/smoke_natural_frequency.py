"""Natural-frequency smoke -- Euler-Bernoulli beam bending modes vs closed form.

f_n = (beta_n L)^2/(2 pi) sqrt(E I/(rho A L^4)). Validated on a real kernel
rectangular beam:

  * simply-supported: f_1 = (pi/2/L^2) sqrt(E I/(rho A)) exactly, modes 1:4:9 ;
  * cantilever first mode matches (1.875104)^2/(2 pi) sqrt(...) ;
  * weak-axis bending gives a lower frequency than the strong axis ;
  * missing modulus/density and an unknown support are refused.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=2e-3, abs_=1e-9):
    return abs(a - b) <= max(abs_, rel * abs(b))


def main():
    s = new_session("natural_frequency")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # steel beam: mm, MPa(N/mm^2), tonne/mm^3 -> frequencies in consistent units
    b, h, L = 20.0, 50.0, 800.0
    E, rho = 210000.0, 7.85e-9
    s.act("solid.box", {"name": "beam", "length": b, "width": h, "height": L})
    Istrong = b * h ** 3 / 12.0
    A = b * h
    base = math.sqrt(E * Istrong / (rho * A * L ** 4))

    r = s.act("solid.natural_frequency",
              {"name": "beam", "modulus": E, "density": rho,
               "support": "simply_supported", "modes": 3})
    assert r.ok, r.error
    f = r.data["frequencies_hz"]
    f1_exact = (math.pi / 2 / L ** 2) * math.sqrt(E * Istrong / (rho * A))
    assert _close(f[0], f1_exact), (f[0], f1_exact)
    # simply-supported modes follow 1:4:9
    assert _close(f[1] / f[0], 4.0) and _close(f[2] / f[0], 9.0), f
    print("simply-supported f = %s Hz (f1 exact=%.4f, ratios 1:4:9)" % (f, f1_exact))

    dc = s.act("solid.natural_frequency",
               {"name": "beam", "modulus": E, "density": rho,
                "support": "cantilever", "modes": 1}).data
    f1c = (1.8751040687 ** 2) / (2 * math.pi) * base
    assert _close(dc["frequencies_hz"][0], f1c), (dc["frequencies_hz"], f1c)
    print("cantilever f1 = %.4f Hz (closed form)" % dc["frequencies_hz"][0])

    # weak axis is softer -> lower frequency
    dw = s.act("solid.natural_frequency",
               {"name": "beam", "modulus": E, "density": rho,
                "support": "cantilever", "bending": "min", "modes": 1}).data
    assert dw["frequencies_hz"][0] < dc["frequencies_hz"][0], (dw, dc)
    print("weak axis lower: %.4f < %.4f Hz" % (dw["frequencies_hz"][0], dc["frequencies_hz"][0]))

    miss = s.act("solid.natural_frequency", {"name": "beam", "modulus": E})
    assert not miss.ok and "density" in (miss.error or ""), miss.error
    bad = s.act("solid.natural_frequency",
                {"name": "beam", "modulus": E, "density": rho, "support": "nope"})
    assert not bad.ok and "support" in (bad.error or ""), bad.error
    print("guards: missing density and unknown support refused")

    print("NATURAL FREQUENCY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_natural_frequency"):
    main()
