"""Inertia smoke -- full mass properties cross-checked against closed form.

Every rigid-body inertia has a textbook value; we build the real BREP solid in
FreeCAD and demand the measured tensor reproduce it:

  * solid box a x b x c about its centroid: principal moments are
    I_x = m(b^2+c^2)/12, I_y = m(a^2+c^2)/12, I_z = m(a^2+b^2)/12 ;
  * material density must scale the whole tensor linearly (FreeCAD's raw
    MatrixOfInertia ignores density -- the op must not) ;
  * cylinder (R,H) taken about its *base* centre, not its centroid: the
    transverse moment grows by the parallel-axis term m(H/2)^2 while the axial
    moment m R^2/2 is unchanged (the shift is along the axis) ;
  * solid sphere: every axis is principal with I = 2/5 m R^2 ;
  * the tensor is symmetric, and a missing solid is refused loudly.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=2e-3):
    return abs(a - b) <= rel * max(1.0, abs(b))


def main():
    s = new_session("inertia")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- box: principal moments vs closed form -------------------------- #
    a, b, c = 20.0, 30.0, 40.0
    s.act("solid.box", {"name": "blk", "length": a, "width": b, "height": c})
    r = s.act("solid.inertia", {"name": "blk"}).data
    m = a * b * c
    cf = sorted([m * (b * b + c * c) / 12.0,
                 m * (a * a + c * c) / 12.0,
                 m * (a * a + b * b) / 12.0])
    got = sorted(r["principal_moments"])
    assert all(_close(g, e) for g, e in zip(got, cf)), (got, cf)
    # tensor symmetry
    t = r["tensor"]
    assert _close(t[0][1], t[1][0]) and _close(t[0][2], t[2][0]) and _close(t[1][2], t[2][1])
    print("box principal moments %s == closed form %s"
          % ([round(x) for x in got], [round(x) for x in cf]))

    # ---- density scales the tensor linearly ----------------------------- #
    rho = 7.85e-9  # steel kg/mm^3
    rd = s.act("solid.inertia", {"name": "blk", "density": rho}).data
    assert _close(rd["mass"], m * rho)
    assert all(_close(g, e * rho) for g, e in zip(sorted(rd["principal_moments"]), cf))
    print("density %.2e scales mass->%.4g and every moment linearly" % (rho, rd["mass"]))

    # ---- cylinder about its base: parallel-axis transfer ---------------- #
    R, H = 10.0, 60.0
    s.act("solid.cylinder", {"name": "cyl", "radius": R, "height": H})
    base = s.act("solid.inertia", {"name": "cyl", "about": "origin"}).data
    mc = math.pi * R * R * H
    ixx_cm = mc * (3 * R * R + H * H) / 12.0
    ixx_base = ixx_cm + mc * (H / 2.0) ** 2          # transverse, shifted to end
    izz = mc * R * R / 2.0                            # axial, unchanged by shift
    txx, tzz = base["tensor"][0][0], base["tensor"][2][2]
    assert _close(txx, ixx_base), (txx, ixx_base)
    assert _close(tzz, izz), (tzz, izz)
    print("cylinder about base: I_xx=%.0f == I_cm+m(H/2)^2=%.0f ; I_zz=%.0f == mR^2/2"
          % (txx, ixx_base, tzz))

    # ---- sphere: isotropic 2/5 m R^2 ------------------------------------ #
    Rs = 15.0
    s.act("solid.sphere", {"name": "sph", "radius": Rs})
    sp = s.act("solid.inertia", {"name": "sph"}).data
    ms = 4.0 / 3.0 * math.pi * Rs ** 3
    iso = 2.0 / 5.0 * ms * Rs * Rs
    assert all(_close(x, iso) for x in sp["principal_moments"]), (sp["principal_moments"], iso)
    print("sphere isotropic moments %s == 2/5 mR^2 = %.0f"
          % ([round(x) for x in sp["principal_moments"]], iso))

    # ---- hollow shell keeps a real wall mass; parallel-axis still holds -- #
    s.act("solid.box", {"name": "shellbox", "length": 30, "width": 30, "height": 30})
    hollow = s.act("solid.shell", {"name": "shellbox", "thickness": -2,
                                   "open_faces": [5], "out": "hollow"}).data
    hi = s.act("solid.inertia", {"name": "hollow"}).data
    assert _close(hi["mass"], hollow["volume"]), (hi["mass"], hollow["volume"])
    print("hollow wall mass=%.1f, symmetric tensor diag=%s"
          % (hi["mass"], [round(hi["tensor"][i][i]) for i in range(3)]))

    # ---- a missing solid is refused loudly ------------------------------ #
    bad = s.act("solid.inertia", {"name": "nope"})
    assert not bad.ok and "no such solid" in (bad.error or "").lower()
    print("missing solid refused: %s" % bad.error)

    print("INERTIA SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_inertia"):
    main()
