"""Surface / Draft / Points+ReverseEngineering coverage smoke.

Three workbenches that the solid/param families never reached, now driven as
fusable ops and proven to feed downstream (a Draft array meshes; a reversed
surface registers as a real shape). Malformed input stays guided -- no raw
TypeError / OCCError / AttributeError leaks.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

_RAW = ("TypeError", "AttributeError", "could not convert", "has no attribute",
        "KeyError", "OCCError", "Standard_", "NullShape")


def _guided(r, token):
    err = r.error or ""
    assert not r.ok, "expected failure, got %r" % (r.data,)
    assert not any(x in err for x in _RAW), "raw leak: %r" % err
    assert token in err, "error %r lacks %r" % (err, token)


def main():
    s = new_session("surface")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # surface.fill: a non-planar boundary loop -> a real face with area.
    f = s.act("surface.fill", {"out": "Sail", "points": [
        [0, 0, 0], [20, 0, 5], [20, 20, 0], [0, 20, 5]]})
    assert f.ok, f.error
    assert f.data["area"] > 0, f.data
    print("surface.fill non-planar loop -> area %.2f" % f.data["area"])

    # draft.ortho_array of a real solid, then proven fusable (it meshes).
    assert s.act("solid.box", {"name": "Cell", "length": 4, "width": 4,
                               "height": 4}).ok
    g = s.act("draft.ortho_array", {"source": "Cell", "out": "Grid",
                                    "dx": 8, "dy": 8, "nx": 3, "ny": 2})
    assert g.ok, g.error
    assert g.data["solids"] == 6, g.data
    m = s.act("mesh.analyze", {"name": "Grid"})
    assert m.ok and m.data["watertight"], m.error or m.data
    print("draft.ortho_array 3x2 -> %d solids, meshes watertight" % g.data["solids"])

    # draft.polar_array about a centre.
    assert s.act("solid.cylinder", {"name": "Hub", "radius": 2, "height": 4}).ok
    p = s.act("draft.polar_array", {"source": "Hub", "out": "Ring",
                                    "count": 6, "angle": 360, "center": [0, 0, 0]})
    assert p.ok, p.error
    assert p.data["solids"] == 6, p.data
    print("draft.polar_array 6x -> %d solids" % p.data["solids"])

    # points.cloud + reverse-engineered BSpline surface from scan data.
    pts = [[i * 5.0, j * 5.0,
            3.0 * math.sin(i * 5.0 / 10.0) + 2.0 * math.cos(j * 5.0 / 12.0)]
           for i in range(8) for j in range(8)]
    c = s.act("points.cloud", {"name": "Scan", "points": pts})
    assert c.ok and c.data["points"] == 64, c.error or c.data
    rev = s.act("points.reverse", {"cloud": "Scan", "out": "RevSurf",
                                   "u_poles": 6, "v_poles": 6})
    assert rev.ok, rev.error
    assert rev.data["area"] > 0 and rev.data["fit_points"] == 64, rev.data
    # the reversed surface is a first-class shape (perceivable downstream).
    insp = s.act("analyze.section", {"name": "RevSurf", "plane": "XY", "offset": 5})
    assert insp.ok, insp.error
    print("points.reverse 64-pt cloud -> BSpline area %.1f (fusable)" % rev.data["area"])

    # ---- malformed input stays guided ------------------------------------ #
    _guided(s.act("surface.fill", {"points": "x"}), "list of")
    _guided(s.act("surface.fill", {"points": [[0, 0, 0], [1, 1, 1]]}), "at least 3")
    _guided(s.act("surface.fill", {"points": [[0, 0, 0], [0, 0, 0], [0, 0, 0]]}),
            "coincident")
    _guided(s.act("draft.ortho_array", {"source": "Cell", "nx": "x"}), "number")
    _guided(s.act("draft.ortho_array", {"source": "Nope"}), "no such solid")
    _guided(s.act("draft.ortho_array", {"source": "Cell", "nx": 0}), ">= 1")
    _guided(s.act("draft.polar_array", {"source": "Cell", "angle": "x"}), "number")
    _guided(s.act("draft.polar_array", {"source": "Cell", "center": [1, 2]}), "3 numbers")
    _guided(s.act("points.cloud", {"name": "C", "points": 5}), "list of")
    _guided(s.act("points.reverse", {"cloud": "Ghost"}), "no cloud named")
    _guided(s.act("points.reverse", {"points": [[0, 0, 0]] * 3}), "at least 9")
    _guided(s.act("points.reverse", {"points": pts, "u_poles": 2, "u_degree": 3}),
            "exceed degree")
    print("malformed surface/draft/points input all guided (no raw leaks)")

    print("SURFACE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_surface"):
    main()
