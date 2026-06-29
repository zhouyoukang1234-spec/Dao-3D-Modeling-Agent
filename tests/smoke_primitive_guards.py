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
    print("bad cylinder/cone/sphere/torus dims all refused cleanly")

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
