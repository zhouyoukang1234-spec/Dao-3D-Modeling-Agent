"""Boolean / fillet / chamfer guard smoke -- bad inputs fail loud and clear.

Practice exposed that the boolean ops leaked a bare ``KeyError: 'a'`` when an
operand was omitted, and ``fillet``/``chamfer`` leaked a bare ``KeyError`` on a
missing arg and a cryptic ``OCCError StdFail_NotDone`` on a negative or
too-large radius/size. All now raise a guided ``ValueError`` while valid
operations keep working.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bad(r, token):
    err = r.error or ""
    assert not r.ok, "expected failure, got %r" % (r.data,)
    # must be a guided ValueError, never a raw KeyError/TypeError. (The
    # too-large cases deliberately append the OCC detail for context, so the
    # guidance token below is what proves the message is actionable.)
    assert err.startswith("ValueError"), err
    for leak in ("KeyError", "TypeError"):
        assert leak not in err, "%s leaked: %s" % (leak, err)
    assert token in err, "error %r lacks %r" % (err, token)


def main():
    s = new_session("bool_fillet_guards")
    print("FreeCAD", s.registry.kernel.freecad_version)
    s.act("solid.box", {"name": "A", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "B", "length": 6, "width": 6, "height": 6, "pos": [5, 0, 0]})

    # boolean operand validation
    _bad(s.act("solid.union", {"a": "A", "out": "u"}), "operand")
    _bad(s.act("solid.cut", {"b": "B", "out": "c"}), "operand")
    _bad(s.act("solid.common", {"out": "i"}), "operand")

    # empty boolean results must fail loud, not store a 0-volume part that
    # downstream ops/export silently choke on.
    s.act("solid.box", {"name": "D", "length": 10, "width": 10, "height": 10, "pos": [100, 0, 0]})
    s.act("solid.box", {"name": "Big", "length": 60, "width": 60, "height": 60, "pos": [-25, -25, -25]})
    _bad(s.act("solid.common", {"a": "A", "b": "D", "out": "x"}), "empty intersection")
    _bad(s.act("solid.cut", {"a": "A", "b": "Big", "out": "y"}), "nothing remains")

    # fillet / chamfer arg + dimension guards
    _bad(s.act("solid.fillet", {"name": "A"}), "radius")
    _bad(s.act("solid.fillet", {"name": "A", "radius": -2}), "positive")
    _bad(s.act("solid.fillet", {"name": "A", "radius": 999}), "too large")
    _bad(s.act("solid.chamfer", {"name": "A"}), "size")
    _bad(s.act("solid.chamfer", {"name": "A", "size": -1}), "positive")
    _bad(s.act("solid.chamfer", {"name": "A", "size": 999}), "too large")
    print("bad boolean/fillet/chamfer inputs all refused cleanly")

    # valid operations still work
    u = s.act("solid.union", {"a": "A", "b": "B", "out": "u"})
    assert u.ok, u.error
    f = s.act("solid.fillet", {"name": "A", "radius": 1.0, "out": "Af"})
    assert f.ok, f.error
    c = s.act("solid.chamfer", {"name": "B", "size": 1.0, "out": "Bc"})
    assert c.ok, c.error
    print("valid union/fillet/chamfer still build: %s / %s / %s"
          % (u.ok, f.ok, c.ok))

    print("BOOLEAN/FILLET GUARDS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_boolean_fillet_guards"):
    main()
