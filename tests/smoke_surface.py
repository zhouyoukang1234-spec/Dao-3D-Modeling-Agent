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

    # surface.ruled: span two open profiles -> a ruled shell with area.
    ru = s.act("surface.ruled", {"out": "Web",
                                 "edge1": [[0, 0, 0], [20, 0, 0], [20, 0, 8]],
                                 "edge2": [[0, 20, 0], [20, 20, 0], [20, 20, 8]]})
    assert ru.ok, ru.error
    assert ru.data["area"] > 0 and ru.data["faces"] >= 1, ru.data
    print("surface.ruled two profiles -> area %.2f, %d faces"
          % (ru.data["area"], ru.data["faces"]))

    # surface.interpolate: a smooth BSpline through a rectangular grid.
    grid = [[[i * 6.0, j * 6.0,
              2.5 * math.sin(i * 0.9) + 1.5 * math.cos(j * 0.7)]
             for j in range(5)] for i in range(5)]
    it = s.act("surface.interpolate", {"out": "Canopy", "grid": grid})
    assert it.ok, it.error
    assert it.data["area"] > 0 and it.data["grid"] == [5, 5], it.data
    print("surface.interpolate 5x5 grid -> area %.1f" % it.data["area"])

    # surface.offset: grow a solid's faces into a parallel shell, then perceive.
    assert s.act("solid.box", {"name": "Core", "length": 10, "width": 10,
                               "height": 10}).ok
    off = s.act("surface.offset", {"source": "Core", "out": "Skin",
                                   "distance": 2})
    assert off.ok, off.error
    # 10mm cube offset out by 2mm -> faces grow well beyond the original 600.
    assert off.data["area"] > 600 and off.data["faces"] >= 6, off.data
    sec = s.act("analyze.section", {"name": "Skin", "plane": "XY", "offset": 5})
    assert sec.ok, sec.error
    print("surface.offset +2mm -> shell area %.1f (fusable)" % off.data["area"])

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
    _guided(s.act("surface.ruled", {"edge1": [[0, 0, 0]], "edge2": [[0, 1, 0]]}),
            "at least 2")
    _guided(s.act("surface.interpolate", {"grid": "x"}), "grid")
    _guided(s.act("surface.interpolate", {"grid": [[[0, 0, 0], [1, 0, 0]]]}),
            "at least 2 rows")
    _guided(s.act("surface.interpolate",
                  {"grid": [[[0, 0, 0], [1, 0, 0]], [[0, 1, 0]]]}),
            "not rectangular")
    _guided(s.act("surface.offset", {"source": "Nope", "distance": 1}),
            "no such solid")
    _guided(s.act("surface.offset", {"source": "Core", "distance": 0}),
            "non-zero")
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
