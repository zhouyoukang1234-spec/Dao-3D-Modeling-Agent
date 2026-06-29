"""Primitive dimension-guard smoke -- bad dims fail loud, valid apex survives.

Practice exposed that ``solid.cylinder`` with a negative height built an invalid
shape and then threw a cryptic ``FreeCADError`` while reading ``Volume``, and
``solid.cone`` leaked a bare ``OCCDomainError``. Both now validate dimensions up
front with a guided ``ValueError`` -- while a cone with a single zero radius (a
pointed apex, used across the suite) stays valid.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bad(r, token):
    err = r.error or ""
    assert not r.ok, "expected failure, got %r" % (r.data,)
    assert "FreeCADError" not in err and "OCCDomainError" not in err, err
    assert "KeyError" not in err and "TypeError" not in err, err
    assert token in err, "error %r lacks %r" % (err, token)


def main():
    s = new_session("primitive_guards")
    print("FreeCAD", s.registry.kernel.freecad_version)

    _bad(s.act("solid.cylinder", {"name": "c", "radius": 5, "height": -3}), "cylinder")
    _bad(s.act("solid.cylinder", {"name": "c", "radius": -5, "height": 3}), "cylinder")
    _bad(s.act("solid.cylinder", {"name": "c", "radius": 0, "height": 3}), "cylinder")
    _bad(s.act("solid.cone", {"name": "k", "radius1": 5, "radius2": 3, "height": -1}), "cone")
    _bad(s.act("solid.cone", {"name": "k", "radius1": 0, "radius2": 0, "height": 3}), "cone")
    _bad(s.act("solid.cone", {"name": "k", "radius1": -2, "radius2": 3, "height": 3}), "cone")
    _bad(s.act("solid.sphere", {"name": "sp", "radius": -4}), "sphere")
    _bad(s.act("solid.sphere", {"name": "sp", "radius": 0}), "sphere")
    _bad(s.act("solid.torus", {"name": "to", "radius1": -10, "radius2": 3}), "torus")
    _bad(s.act("solid.torus", {"name": "to", "radius1": 10, "radius2": -3}), "torus")
    # a non-string solid name used to leak 'TypeError: argument 2 must be str,
    # not int' from addObject; it must be refused with guidance.
    _bad(s.act("solid.box", {"name": 123, "length": 5, "width": 5, "height": 5}),
         "must be a string")
    print("bad cylinder/cone/sphere/torus dims all refused cleanly")

    # --- malformed-input guards (no_raw_leak): non-numeric dims, malformed
    #     vectors and non-string paths used to leak raw 'could not convert string
    #     to float' / IndexError / TypeError instead of a guided message. -------
    def _num_guard(r, token):
        err = r.error or ""
        assert not r.ok, "expected failure, got %r" % (r.data,)
        for raw in ("could not convert", "TypeError", "IndexError",
                    "invalid literal", "not iterable", "expected str"):
            assert raw not in err, "leaked raw %s: %r" % (raw, err)
        assert token in err, "error %r lacks %r" % (err, token)

    s.act("solid.box", {"name": "G", "length": 20, "width": 10, "height": 5})
    _num_guard(s.act("solid.box", {"name": "Z", "length": "x", "width": 5, "height": 5}),
               "length")
    _num_guard(s.act("solid.cylinder", {"name": "Z", "radius": "x", "height": 5}), "radius")
    _num_guard(s.act("solid.sphere", {"name": "Z", "radius": [1, 2]}), "radius")
    _num_guard(s.act("solid.translate", {"name": "G", "vector": [1, 2], "out": "T"}),
               "3 components")
    _num_guard(s.act("solid.translate", {"name": "G", "vector": ["x", 0, 0], "out": "T"}),
               "numbers")
    _num_guard(s.act("solid.rotate", {"name": "G", "axis": "x", "angle": 30, "out": "R"}),
               "axis")
    _num_guard(s.act("solid.fillet", {"name": "G", "radius": "x"}), "radius")
    _num_guard(s.act("solid.shell", {"name": "G", "thickness": "x", "out": "Sh"}),
               "thickness")
    _num_guard(s.act("solid.pattern_polar", {"name": "G", "count": "x", "out": "P"}),
               "count")
    _num_guard(s.act("solid.curvature", {"name": "G", "grid": "x"}), "grid")
    _num_guard(s.act("solid.inertia", {"name": "G", "density": "x"}), "density")
    _num_guard(s.act("solid.export", {"names": ["G"], "path": 123}), "path")
    _num_guard(s.act("solid.import_step", {"path": 123}), "path")
    _num_guard(s.act("view.render", {"names": ["G"], "path": 123}), "path")
    _num_guard(s.act("view.render", {"names": ["G"], "path": "/tmp/x.png",
                                     "tolerance": "x"}), "number")
    _num_guard(s.act("view.render", {"names": ["G"], "path": "/tmp/x.png",
                                     "size": "x"}), "number")
    _num_guard(s.act("view.scene", {"names": 123}), "names")
    _num_guard(s.act("view.scene", {"names": ["G"], "tolerance": "x"}), "number")
    # non-list names and non-string names on the compound/export/reverse paths
    # used to leak 'int object is not iterable' / "+: 'int' and 'str'".
    _num_guard(s.act("solid.compound", {"names": 123}), "list of solid names")
    _num_guard(s.act("solid.export", {"names": 123, "path": "/tmp/x.step"}),
               "list of solid names")
    _num_guard(s.act("solid.reverse", {"name": 123}), "solid name")
    _num_guard(s.act("solid.reverse_build", {"name": 123}), "solid name")
    _num_guard(s.act("doc.save", {"path": 123}), "path")
    # malformed _profile_face specs (extrude/revolve/loft) used to leak a bare
    # 'not enough values to unpack', 'could not convert' or an OCCError.
    _num_guard(s.act("solid.extrude", {"name": "E", "profile": {"rect": "x"},
                                       "length": 5}), "rect")
    _num_guard(s.act("solid.extrude", {"name": "E", "profile": {"rect": [10]},
                                       "length": 5}), "rect")
    _num_guard(s.act("solid.extrude", {"name": "E", "profile": {"circle": "x"},
                                       "length": 5}), "circle")
    _num_guard(s.act("solid.extrude", {"name": "E", "profile": {"polygon": "x"},
                                       "length": 5}), "polygon")
    _num_guard(s.act("solid.extrude", {"name": "E",
                                       "profile": {"polygon": [[0, 0]]},
                                       "length": 5}), "at least 3 points")
    _num_guard(s.act("solid.extrude", {"name": "E", "profile": {"slot": [10]},
                                       "length": 5}), "slot")
    _num_guard(s.act("solid.revolve", {"name": "R", "profile": {"circle": "x"},
                                       "angle": 90}), "circle")
    print("malformed numeric/vector/path/profile inputs across "
          "solid.*/view.*/doc.* refused cleanly")

    # a pointed cone (one zero radius) is legitimate and must still build.
    apex = s.act("solid.cone", {"name": "apex", "radius1": 0, "radius2": 10, "height": 30})
    assert apex.ok, apex.error
    exp = math.pi * 10.0 ** 2 * 30.0 / 3.0           # full cone volume
    assert abs(apex.data["volume"] - exp) < 1e-3, (apex.data["volume"], exp)
    print("pointed cone (r1=0) volume %.1f matches pi r^2 h / 3" % apex.data["volume"])

    # valid sphere and torus still build with their closed-form volumes.
    sph = s.act("solid.sphere", {"name": "ball", "radius": 5})
    assert sph.ok and abs(sph.data["volume"] - 4.0 / 3.0 * math.pi * 125) < 1e-2, sph.error
    tor = s.act("solid.torus", {"name": "ring", "radius1": 10, "radius2": 2})
    assert tor.ok and abs(tor.data["volume"] - 2 * math.pi ** 2 * 10 * 4) < 1e-1, tor.error
    print("valid sphere/torus build: V=%.1f / %.1f" % (sph.data["volume"], tor.data["volume"]))

    print("PRIMITIVE GUARDS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_primitive_guards"):
    main()
