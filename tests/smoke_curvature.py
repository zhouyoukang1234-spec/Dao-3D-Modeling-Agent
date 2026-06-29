"""Curvature smoke -- differential-geometry surface analysis vs closed form.

Every analytic surface has a textbook curvature; we build the real BREP solid
in FreeCAD and demand ``solid.curvature`` reproduce it:

  * sphere of radius R: both principal curvatures are 1/R, so the Gaussian
    curvature is K = 1/R^2 (> 0, elliptic) and |mean| = 1/R, and the global
    minimum radius of curvature is exactly R ;
  * cylinder of radius R: the lateral face is developable -- one principal
    curvature 1/R and one 0, hence K = 0 (parabolic) and its local radius is R ;
  * a plain box is faces-only of planes: zero curvature everywhere, so there is
    no finite radius of curvature at all ;
  * a torus (major Rmaj, minor Rmin) has a tube that always curves at 1/Rmin,
    so the tightest feature -- the global min radius of curvature -- is Rmin ;
  * an invalid sampling grid, and a missing solid, are both refused loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(a, b, rel=3e-3):
    return abs(a - b) <= rel * max(1.0, abs(b))


def main():
    s = new_session("curvature")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- sphere: K = 1/R^2, |mean| = 1/R, min radius = R ---------------- #
    R = 12.0
    s.act("solid.sphere", {"name": "sph", "radius": R})
    r = s.act("solid.curvature", {"name": "sph"}).data
    assert _close(r["min_radius_of_curvature"], R), r["min_radius_of_curvature"]
    assert _close(r["max_abs_curvature"], 1.0 / R), r["max_abs_curvature"]
    face = r["detail"][0]
    assert face["class"] == "elliptic", face
    assert _close(face["gaussian"], 1.0 / (R * R)), face["gaussian"]
    assert _close(abs(face["mean"]), 1.0 / R), face["mean"]
    print("sphere R=%g: K=%.5g == 1/R^2=%.5g, min radius=%.3f == R"
          % (R, face["gaussian"], 1.0 / (R * R), r["min_radius_of_curvature"]))

    # ---- cylinder: developable lateral face (K=0), local radius = R ----- #
    Rc, Hc = 8.0, 40.0
    s.act("solid.cylinder", {"name": "cyl", "radius": Rc, "height": Hc})
    rc = s.act("solid.curvature", {"name": "cyl"}).data
    assert _close(rc["min_radius_of_curvature"], Rc), rc["min_radius_of_curvature"]
    lat = [f for f in rc["detail"] if f["surface"] == "Cylinder"]
    assert len(lat) == 1, rc["detail"]
    assert lat[0]["class"] == "parabolic", lat[0]
    assert _close(lat[0]["gaussian"], 0.0), lat[0]["gaussian"]
    assert _close(lat[0]["min_radius"], Rc), lat[0]["min_radius"]
    planar = [f for f in rc["detail"] if f["surface"] == "Plane"]
    assert planar and all(f["class"] == "planar" for f in planar), rc["detail"]
    print("cylinder R=%g: lateral K=0 (developable), local radius=%.3f == R, %d flat caps"
          % (Rc, lat[0]["min_radius"], len(planar)))

    # ---- box: all planar, no finite radius of curvature ----------------- #
    s.act("solid.box", {"name": "blk", "length": 20, "width": 30, "height": 40})
    rb = s.act("solid.curvature", {"name": "blk"}).data
    assert rb["min_radius_of_curvature"] is None, rb["min_radius_of_curvature"]
    assert _close(rb["max_abs_curvature"], 0.0), rb["max_abs_curvature"]
    assert all(f["class"] == "planar" for f in rb["detail"]), rb["detail"]
    print("box: %d planar faces, max|k|=0 -> no finite radius of curvature"
          % rb["faces"])

    # ---- torus: tightest feature is the tube (min radius = Rmin) -------- #
    Rmaj, Rmin = 20.0, 4.0
    s.act("solid.torus", {"name": "tor", "radius1": Rmaj, "radius2": Rmin})
    rt = s.act("solid.curvature", {"name": "tor"}).data
    assert _close(rt["min_radius_of_curvature"], Rmin), rt["min_radius_of_curvature"]
    assert _close(rt["max_abs_curvature"], 1.0 / Rmin), rt["max_abs_curvature"]
    tf = rt["detail"][0]
    assert tf["surface"] == "Toroid" and _close(tf["minor_radius"], Rmin), tf
    print("torus major=%g minor=%g: tightest radius=%.3f == Rmin"
          % (Rmaj, Rmin, rt["min_radius_of_curvature"]))

    # ---- an invalid sampling grid is refused loudly --------------------- #
    badgrid = s.act("solid.curvature", {"name": "blk", "grid": 0})
    assert not badgrid.ok and "grid must be" in (badgrid.error or "").lower()
    print("invalid grid refused: %s" % badgrid.error)

    # ---- an unbounded grid is refused (DoS guard, not a kernel hang) ---- #
    huge = s.act("solid.curvature", {"name": "blk", "grid": 99999})
    assert not huge.ok and "<= 512" in (huge.error or ""), huge.error
    print("oversized grid refused (no kernel hang): %s" % huge.error)

    # ---- a missing solid is refused loudly ------------------------------ #
    bad = s.act("solid.curvature", {"name": "nope"})
    assert not bad.ok and "no such solid" in (bad.error or "").lower()
    print("missing solid refused: %s" % bad.error)

    print("CURVATURE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_curvature"):
    main()
