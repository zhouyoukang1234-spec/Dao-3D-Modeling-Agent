"""Section-modulus smoke -- cross-section bending properties vs closed form.

The beam numbers an engineer sizes with, read off a real kernel section cut:

  * a b x h rectangular bar cut perpendicular to its length reproduces the
    textbook rectangle exactly -- I = b h^3/12, S = b h^2/6, r = h/sqrt(12),
    polar J = I_x + I_y -- with the strong axis paired to the tall side ;
  * a solid round bar reproduces the circle to tessellation -- I = pi r^4/4,
    S = pi r^3/4, r_gyr = R/2, both principal moments equal ;
  * a plane that misses the solid is refused loudly.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=1e-4, abs_=1e-6):
    return abs(a - b) <= max(abs_, rel * abs(b))


def main():
    s = new_session("section_modulus")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- rectangle b x h, cut perpendicular to length (z) -------------- #
    b, h, L = 40.0, 60.0, 100.0
    s.act("solid.box", {"name": "bar", "length": b, "width": h, "height": L})
    r = s.act("solid.section_modulus", {"name": "bar", "normal": [0, 0, 1]})
    assert r.ok, r.error
    d = r.data
    assert _close(d["area"], b * h), d["area"]
    Ix, Iy = b * h ** 3 / 12.0, h * b ** 3 / 12.0
    Sx, Sy = b * h ** 2 / 6.0, h * b ** 2 / 6.0
    moments = sorted(p["second_moment"] for p in d["principal"])
    assert _close(moments[0], Iy) and _close(moments[1], Ix), (moments, Iy, Ix)
    # the strong axis (larger I) must carry the larger section modulus
    strong = max(d["principal"], key=lambda p: p["second_moment"])
    weak = min(d["principal"], key=lambda p: p["second_moment"])
    assert _close(strong["section_modulus"], Sx), (strong, Sx)
    assert _close(weak["section_modulus"], Sy), (weak, Sy)
    assert _close(strong["radius_of_gyration"], h / math.sqrt(12)), strong
    assert _close(d["polar_moment"], Ix + Iy), d["polar_moment"]
    # section is cut at the centroid (z = L/2) by default
    assert _close(d["centroid"][2], L / 2.0), d["centroid"]
    print("rect: A=%.0f Ix=%.0f Sx=%.0f Iy=%.0f Sy=%.0f J=%.0f (closed form)"
          % (d["area"], Ix, Sx, Iy, Sy, Ix + Iy))

    # ---- solid round bar: I = pi r^4/4, S = pi r^3/4 ------------------- #
    R = 25.0
    s.act("solid.cylinder", {"name": "rod", "radius": R, "height": 80})
    rc = s.act("solid.section_modulus", {"name": "rod"}).data
    Icirc = math.pi * R ** 4 / 4.0
    Scirc = math.pi * R ** 3 / 4.0
    assert _close(rc["area"], math.pi * R * R, rel=1e-3), rc["area"]
    for p in rc["principal"]:
        assert _close(p["second_moment"], Icirc, rel=2e-3), (p, Icirc)
        assert _close(p["section_modulus"], Scirc, rel=5e-3), (p, Scirc)
        assert _close(p["radius_of_gyration"], R / 2.0, rel=2e-3), p
    print("circle: A=%.1f I=%.0f S=%.0f r_gyr=%.2f (== pi r^4/4 etc.)"
          % (rc["area"], Icirc, Scirc, R / 2.0))

    # ---- disconnected section: twin square bars (a compound) ----------- #
    # The section is two separate 20x20 squares, centres at x=10 and x=70,
    # so makeFace yields a *compound* of two faces -- the props must aggregate
    # by the parallel-axis theorem, not crash on the missing global inertia.
    a_ = 20.0
    s.act("solid.box", {"name": "t1", "length": a_, "width": a_, "height": 200})
    s.act("solid.box", {"name": "t2", "length": a_, "width": a_, "height": 200})
    s.act("solid.translate", {"name": "t2", "vector": [60, 0, 0]})
    s.act("solid.compound", {"names": ["t1", "t2"], "out": "twin"})
    tw = s.act("solid.section_modulus", {"name": "twin"})
    assert tw.ok, tw.error
    td = tw.data
    assert _close(td["area"], 2 * a_ * a_), td["area"]
    assert td["regions"] == 2, td["regions"]
    assert _close(td["centroid"][0], 40.0) and _close(td["centroid"][1], 10.0), td["centroid"]
    self_I = a_ * a_ ** 3 / 12.0                       # each square about its own axis
    I_weak = 2 * self_I                                # bending about the shared (x-spread) axis
    I_strong = 2 * (self_I + a_ * a_ * 30.0 ** 2)      # parallel axis, d=30 in x
    tm = sorted(p["second_moment"] for p in td["principal"])
    assert _close(tm[0], I_weak, rel=1e-3), (tm[0], I_weak)
    assert _close(tm[1], I_strong, rel=1e-3), (tm[1], I_strong)
    print("twin bars (compound section): A=%.0f regions=%d I=[%.0f, %.0f] (parallel-axis)"
          % (td["area"], td["regions"], tm[0], tm[1]))

    # ---- a plane that misses the solid is refused --------------------- #
    miss = s.act("solid.section_modulus",
                 {"name": "bar", "normal": [0, 0, 1], "point": [0, 0, 10 * L]})
    assert not miss.ok and "miss" in (miss.error or "").lower(), miss.error
    print("plane missing the solid refused: %s" % miss.error)

    # ---- a zero cut-plane normal is refused with a guided error, not a --
    #      bare OCCError "gp_Dir() ... zero norm" leak -------------------- #
    zn = s.act("solid.section_modulus", {"name": "bar", "normal": [0, 0, 0]})
    assert not zn.ok and "non-zero 'normal'" in (zn.error or ""), zn.error
    assert "OCCError" not in (zn.error or ""), zn.error
    print("zero normal refused (no raw OCCError leak): %s" % zn.error)

    print("SECTION MODULUS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_section_modulus"):
    main()
