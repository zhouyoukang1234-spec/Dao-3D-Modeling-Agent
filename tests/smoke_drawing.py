"""TechDraw drawing smoke — multi-view orthographic page + overall dimensions.

A single front projection is not a shop drawing. ``draw.techdraw`` now lays out
several standard projections (front/top/right/iso) on one page and stamps an
overall-dimensions block. The block is validated against the B-rep extents so a
wrong number cannot ship; the page must export to a non-empty DXF.

It also adds a cross-section view. The section's cut geometry is validated by
slicing the real solid with the cut plane: a bored part reveals an internal
contour (more wires) that a solid block of the same envelope does not.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

W, D, H = 100.0, 60.0, 12.0


def main():
    s = new_session("drawing")
    print("FreeCAD", s.registry.kernel.freecad_version)
    if "draw.techdraw" not in s.tools():
        print("DRAWING SMOKE SKIP (TechDraw not available)")
        return

    assert s.act("solid.box", {"name": "part", "length": W, "width": D, "height": H}).ok
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_drawing.dxf")
    r = s.act("draw.techdraw", {"name": "part", "views": ["front", "top", "right", "iso"],
                                "dimensions": True, "path": out, "scale": 1.0})
    assert r.ok, r.error
    d = r.data
    print("page=%s  views=%s  dims=%s  dxf=%s bytes (err=%s)"
          % (d["page"], d["views"], d.get("dimensions"), d.get("bytes"), d.get("export_error")))

    assert len(d["views"]) == 4, ("expected 4 projections", d["views"])
    dim = d.get("dimensions") or {}
    assert abs(dim.get("length", -1) - W) < 1e-6, dim
    assert abs(dim.get("width", -1) - D) < 1e-6, dim
    assert abs(dim.get("height", -1) - H) < 1e-6, dim
    assert not d.get("export_error") and d.get("bytes", 0) > 0, d

    # --- cross-section view: bored block reveals an inner contour ---------- #
    assert s.act("solid.box", {"name": "blk", "length": 80, "width": 60, "height": 40,
                               "pos": [-40, -30, 0]}).ok
    assert s.act("solid.cylinder", {"name": "bore", "radius": 7, "height": 82,
                                    "pos": [-41, 0, 20], "dir": [1, 0, 0]}).ok
    assert s.act("solid.cut", {"a": "blk", "b": "bore", "out": "blk"}).ok
    rs = s.act("draw.techdraw", {"name": "blk", "page": "SecPage", "views": ["front"],
                                 "section": {"normal": [1, 0, 0]}})
    assert rs.ok, rs.error
    sec = rs.data.get("section")
    print("section=%s (err=%s)" % (sec, rs.data.get("section_error")))
    assert not rs.data.get("section_error"), rs.data
    assert sec and sec["wires"] >= 2, ("section should reveal the inner bore", sec)

    # control: a solid block of the same envelope has only the outline contour
    assert s.act("solid.box", {"name": "solid", "length": 80, "width": 60, "height": 40,
                               "pos": [-40, -30, 0]}).ok
    rc = s.act("draw.techdraw", {"name": "solid", "page": "SolidPage", "views": ["front"],
                                 "section": {"normal": [1, 0, 0]}})
    assert rc.ok and rc.data["section"]["wires"] == 1, rc.data["section"]

    print("DRAWING SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_drawing"):
    main()
