"""Torsion smoke -- shaft twist & shear vs closed form.

Circular-shaft theory: phi = T L/(G J), tau = T c/J, k = G J/L. Validated on
real kernel sections:

  * a solid round shaft: J = pi r^4/2, c = r -> phi, tau, stiffness exact, and
    the section is flagged circular (equal principal moments) ;
  * a hollow tube: J = pi(ro^4 - ri^4)/2, still exact and circular ;
  * doubling the torque doubles twist & shear (linearity) ;
  * a square bar is flagged non-circular (polar moment != torsion constant) ;
  * missing torque/shear modulus is refused.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=1e-3, abs_=1e-9):
    return abs(a - b) <= max(abs_, rel * abs(b))


def main():
    s = new_session("torsion")
    print("FreeCAD", s.registry.kernel.freecad_version)

    R, L, G, T = 20.0, 500.0, 80000.0, 1.0e6   # mm, mm, MPa (steel), N.mm
    s.act("solid.cylinder", {"name": "shaft", "radius": R, "height": L})
    r = s.act("solid.torsion", {"name": "shaft", "torque": T, "shear_modulus": G})
    assert r.ok, r.error
    d = r.data
    J = math.pi * R ** 4 / 2.0
    assert _close(d["polar_moment"], J, rel=2e-3), (d["polar_moment"], J)
    assert _close(d["outer_radius"], R, rel=2e-3), d["outer_radius"]
    assert _close(d["angle_of_twist"], T * L / (G * J), rel=3e-3), d["angle_of_twist"]
    assert _close(d["max_shear_stress"], T * R / J, rel=3e-3), d["max_shear_stress"]
    assert _close(d["torsional_stiffness"], G * J / L, rel=3e-3), d["torsional_stiffness"]
    assert d["circular"] is True, d
    print("solid shaft: J=%.0f phi=%.5e rad tau=%.2f MPa k=%.0f circular=%s"
          % (J, d["angle_of_twist"], d["max_shear_stress"], d["torsional_stiffness"], d["circular"]))

    # ---- hollow tube: J = pi(ro^4 - ri^4)/2 --------------------------- #
    ro, ri = 20.0, 14.0
    s.act("solid.cylinder", {"name": "outer", "radius": ro, "height": L})
    s.act("solid.cylinder", {"name": "inner", "radius": ri, "height": L})
    s.act("solid.cut", {"a": "outer", "b": "inner", "out": "tube"})
    dt = s.act("solid.torsion", {"name": "tube", "torque": T, "shear_modulus": G}).data
    Jt = math.pi * (ro ** 4 - ri ** 4) / 2.0
    assert _close(dt["polar_moment"], Jt, rel=3e-3), (dt["polar_moment"], Jt)
    assert _close(dt["max_shear_stress"], T * ro / Jt, rel=4e-3), dt["max_shear_stress"]
    assert dt["circular"] is True, dt
    print("hollow tube: J=%.0f (== pi(ro^4-ri^4)/2) tau=%.2f MPa circular=%s"
          % (Jt, dt["max_shear_stress"], dt["circular"]))

    # ---- linearity: 2x torque -> 2x twist & shear --------------------- #
    d2 = s.act("solid.torsion", {"name": "shaft", "torque": 2 * T, "shear_modulus": G}).data
    assert _close(d2["angle_of_twist"], 2 * d["angle_of_twist"]), d2
    assert _close(d2["max_shear_stress"], 2 * d["max_shear_stress"]), d2
    print("linearity: 2T -> twist %.5e (=2x), shear %.2f (=2x)"
          % (d2["angle_of_twist"], d2["max_shear_stress"]))

    # ---- square bar flagged non-circular ------------------------------ #
    s.act("solid.box", {"name": "bar", "length": 30, "width": 30, "height": L})
    db = s.act("solid.torsion", {"name": "bar", "torque": T, "shear_modulus": G}).data
    assert db["circular"] is False, db
    print("square bar: circular=%s (polar moment != torsion constant)" % db["circular"])

    # ---- guards ------------------------------------------------------- #
    bad = s.act("solid.torsion", {"name": "shaft", "torque": T})
    assert not bad.ok and "shear_modulus" in (bad.error or ""), bad.error
    print("missing shear modulus refused: %s" % bad.error)

    print("TORSION SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_torsion"):
    main()
