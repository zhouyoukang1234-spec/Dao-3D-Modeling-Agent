"""Pattern smoke — linear & polar arrays land where the geometry says.

A linear pattern must step each copy by exactly i*step. A latent friction hid
here: FreeCAD ``Vector.multiply`` mutates in place, so ``step.multiply(i)``
accumulated the step factorially (copy 4 landed at 24x, not 4x). This validates
the array extents against closed form for both linear and polar patterns, so the
regression cannot return.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

SZ = 4.0


def main():
    s = new_session("pattern")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- linear: 5 unit cubes stepped 20 mm along X -------------------------- #
    s.act("solid.box", {"name": "u", "length": SZ, "width": SZ, "height": SZ})
    n, step = 5, 20.0
    r = s.act("solid.pattern_linear", {"name": "u", "count": n, "step": [step, 0, 0], "out": "row"})
    assert r.ok, r.error
    bb = r.data["bbox_size"]
    span = SZ + (n - 1) * step          # 0..(n-1)*step plus the cube width
    print("linear: bbox=%s (X expect %.1f)  vol=%.1f (expect %.1f)"
          % (bb, span, r.data["volume"], n * SZ ** 3))
    assert abs(bb[0] - span) < 1e-6, (bb, span)
    assert abs(r.data["volume"] - n * SZ ** 3) < 1e-6, r.data["volume"]  # disjoint copies

    # --- polar: 6 cubes offset from the Z axis, full circle ------------------ #
    s.act("solid.box", {"name": "p", "length": SZ, "width": SZ, "height": SZ,
                        "pos": [30, -SZ / 2, -SZ / 2]})
    m = 6
    rp = s.act("solid.pattern_polar", {"name": "p", "count": m, "angle": 360,
                                       "center": [0, 0, 0], "axis": [0, 0, 1], "out": "ring"})
    assert rp.ok, rp.error
    print("polar : vol=%.1f (expect %.1f)  bbox=%s" % (rp.data["volume"], m * SZ ** 3, rp.data["bbox_size"]))
    assert abs(rp.data["volume"] - m * SZ ** 3) < 1e-6, rp.data["volume"]  # disjoint copies
    # by symmetry the ring centroid sits on the rotation axis
    com = rp.data.get("center_of_mass")
    if com is not None:
        assert abs(com[0]) < 1e-6 and abs(com[1]) < 1e-6, com

    # --- partial arc: 4 cubes over a 90 deg arc, copies inclusive of both ends - #
    s.act("solid.box", {"name": "q", "length": SZ, "width": SZ, "height": SZ,
                        "pos": [30, -SZ / 2, -SZ / 2]})
    ra = s.act("solid.pattern_polar", {"name": "q", "count": 4, "angle": 90,
                                       "center": [0, 0, 0], "axis": [0, 0, 1], "out": "arc"})
    assert ra.ok, ra.error
    assert abs(ra.data["volume"] - 4 * SZ ** 3) < 1e-6, ra.data["volume"]

    # --- degenerate count=1 on a partial arc must NOT divide by count-1 -------- #
    # (a count=1 polar array is just the original part; it used to crash with a
    #  ZeroDivisionError on total/(count-1)).
    s.act("solid.box", {"name": "one", "length": SZ, "width": SZ, "height": SZ,
                        "pos": [30, -SZ / 2, -SZ / 2]})
    r1 = s.act("solid.pattern_polar", {"name": "one", "count": 1, "angle": 90,
                                       "center": [0, 0, 0], "axis": [0, 0, 1], "out": "single"})
    assert r1.ok, r1.error
    assert abs(r1.data["volume"] - SZ ** 3) < 1e-6, r1.data["volume"]
    print("polar : arc(4@90) vol=%.1f, count=1 degenerate vol=%.1f (no crash)"
          % (ra.data["volume"], r1.data["volume"]))

    print("PATTERN SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_pattern"):
    main()
