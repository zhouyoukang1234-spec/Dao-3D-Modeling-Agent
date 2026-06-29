"""Missing-argument guard smoke for the build/transform ops.

Practice exposed that extrude/revolve/loft/shell/translate/pattern_linear/
pattern_polar leaked a bare ``KeyError`` (e.g. ``KeyError: 'profile'``,
``KeyError: 'sections'``, ``KeyError: 'vector'``) when a required argument was
omitted. They now reject with a guided ``ValueError`` while valid calls keep
working.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bad(r, token):
    err = r.error or ""
    assert not r.ok, "expected failure, got %r" % (r.data,)
    assert "KeyError" not in err and "TypeError" not in err, err
    assert token in err, "error %r lacks %r" % (err, token)


def main():
    s = new_session("op_arg_guards")
    print("FreeCAD", s.registry.kernel.freecad_version)
    s.act("solid.box", {"name": "A", "length": 10, "width": 10, "height": 10})

    _bad(s.act("solid.extrude", {"name": "E"}), "profile")
    _bad(s.act("solid.revolve", {"name": "R"}), "profile")
    _bad(s.act("solid.loft", {"name": "L"}), "sections")
    _bad(s.act("solid.loft", {"name": "L", "sections": [{"profile": {"rect": [5, 5]}}]}), "sections")
    _bad(s.act("solid.loft", {"name": "L", "sections": [{"offset": 0}, {"offset": 5}]}), "profile")
    _bad(s.act("solid.loft", {"name": "L", "sections": [
        {"profile": {"rect": [10, 10]}, "offset": 0},
        {"profile": {"rect": [4, 4]}, "offset": 0}]}), "distinct 'offset'")
    _bad(s.act("solid.shell", {"name": "A"}), "thickness")
    _bad(s.act("solid.translate", {"name": "A"}), "vector")
    _bad(s.act("solid.pattern_linear", {"name": "A"}), "count")
    _bad(s.act("solid.pattern_linear", {"name": "A", "step": [5, 0, 0]}), "count")
    _bad(s.act("solid.pattern_linear", {"name": "A", "count": 0, "step": [5, 0, 0]}), ">= 1")
    _bad(s.act("solid.pattern_polar", {"name": "A", "count": 0}), ">= 1")

    # zero-magnitude sweeps used to leak a raw OCCError BRepSweep_*::Constructor.
    _bad(s.act("solid.extrude", {"name": "E0", "profile": {"rect": [10, 6]}, "height": 0}), "non-zero")
    _bad(s.act("solid.extrude", {"name": "Ed", "profile": {"rect": [10, 6]}, "dir": [0, 0, 0]}), "non-zero")
    _bad(s.act("solid.revolve", {"name": "V0", "profile": {"rect": [4, 6]},
                                 "axis_pos": [20, 0, 0], "axis_dir": [0, 1, 0], "angle": 0}), "non-zero")
    for r in (s.act("solid.extrude", {"name": "E0", "profile": {"rect": [10, 6]}, "height": 0}),
              s.act("solid.revolve", {"name": "V0", "profile": {"rect": [4, 6]},
                                      "axis_pos": [20, 0, 0], "axis_dir": [0, 1, 0], "angle": 0})):
        assert "OCCError" not in (r.error or ""), r.error
    # zero axis/normal on a transform used to leak a raw OCCError gp_Dir() zero
    # norm; they must fail loud with guidance instead.
    _bad(s.act("solid.rotate", {"name": "A", "axis": [0, 0, 0], "angle": 45}), "non-zero")
    _bad(s.act("solid.mirror", {"name": "A", "normal": [0, 0, 0]}), "non-zero")
    _bad(s.act("solid.pattern_polar", {"name": "A", "axis": [0, 0, 0], "count": 6}), "non-zero")
    for r in (s.act("solid.rotate", {"name": "A", "axis": [0, 0, 0], "angle": 45}),
              s.act("solid.mirror", {"name": "A", "normal": [0, 0, 0]}),
              s.act("solid.pattern_polar", {"name": "A", "axis": [0, 0, 0], "count": 6})):
        assert "OCCError" not in (r.error or ""), r.error
    # a non-numeric rotate angle used to leak a raw 'TypeError: must be real
    # number, not str' from Shape.rotate; it must fail loud with guidance.
    _bad(s.act("solid.rotate", {"name": "A", "axis": [0, 0, 1], "angle": "ninety"}),
         "angle")
    print("missing/invalid build-op args all refused cleanly")

    # valid calls still work
    assert s.act("solid.extrude", {"name": "E", "profile": {"rect": [10, 6]}, "height": 4}).ok
    assert s.act("solid.translate", {"name": "A", "vector": [5, 0, 0], "out": "At"}).ok
    assert s.act("solid.loft", {"name": "L", "sections": [
        {"profile": {"rect": [10, 10]}, "offset": 0},
        {"profile": {"rect": [4, 4]}, "offset": 20}]}).ok
    pl = s.act("solid.pattern_linear", {"name": "A", "count": 3, "step": [15, 0, 0], "out": "Ap"})
    assert pl.ok, pl.error
    print("valid extrude/translate/loft/pattern_linear still build")

    print("OP ARG GUARDS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_op_arg_guards"):
    main()
