"""Mechanism-op guard smoke -- missing required args fail loud, not raw.

Practice exposed that several kinematic ops accessed required parameters with a
bare ``a["..."]``, leaking ``KeyError`` to the caller instead of the guided
``ValueError`` the rest of the library raises. This locks in that each refuses a
missing required argument with an informative ValueError and never a raw
KeyError/TypeError.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


# (op, args-missing-something, a token that must appear in the error)
CASES = [
    ("solid.fourbar", {"crank": 30, "coupler": 90, "rocker": 80}, "ground"),  # no angle/ground
    ("solid.geartrain", {"input_rpm": 100}, "meshes"),
    ("solid.cam", {"angle": 45}, "rise"),
    ("solid.cam_profile", {"rise": 10, "base_radius": 20}, "name"),
    ("solid.planetary", {"teeth_sun": 20, "sun_rpm": 100, "carrier_rpm": 0}, "teeth_ring"),
    ("solid.geneva", {"angle": 10}, "slots"),
]


def main():
    s = new_session("mechanism_guards")
    print("FreeCAD", s.registry.kernel.freecad_version)
    for op, args, token in CASES:
        r = s.act(op, args)
        err = r.error or ""
        assert not r.ok, "%s should have failed on %r" % (op, args)
        assert "KeyError" not in err and "TypeError" not in err, (op, err)
        assert token in err, "%s error %r lacks %r" % (op, err, token)
        print("%-22s refused cleanly: %s" % (op, err.split(":", 1)[-1].strip()[:70]))
    print("MECHANISM GUARDS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_mechanism_guards"):
    main()
