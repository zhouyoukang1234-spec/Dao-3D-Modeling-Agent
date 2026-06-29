"""Export-selector smoke -- singular ``name`` exports only that solid.

Practice exposed that ``solid.export`` honoured only the plural ``names`` and,
given the library-wide singular ``name``, silently fell back to dumping *every*
solid in the session. This locks in that ``name`` selects exactly one solid (a
STEP round-trip recovers a single solid of the right volume) while ``names``
still selects a list, and an empty session refuses loudly.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("export_selector")
    print("FreeCAD", s.registry.kernel.freecad_version)
    d = tempfile.mkdtemp()

    # three distinct solids live in the session at once
    s.act("solid.box", {"name": "one", "length": 20, "width": 30, "height": 40})
    s.act("solid.box", {"name": "two", "length": 10, "width": 10, "height": 10})
    s.act("solid.box", {"name": "three", "length": 5, "width": 5, "height": 5})
    v_one = s.act("solid.measure", {"name": "one"}).data["volume"]

    # singular name -> exactly that one solid round-trips back
    p = os.path.join(d, "one.step")
    er = s.act("solid.export", {"name": "one", "path": p})
    assert er.ok, er.error
    ir = s.act("solid.import_step", {"path": p, "name": "back"})
    assert ir.ok, ir.error
    assert ir.data["solids"] == 1, ir.data            # not all 3 dumped
    v_back = s.act("solid.measure", {"name": "back"}).data["volume"]
    assert abs(v_back - v_one) < 1e-6, (v_back, v_one)
    print("name='one' exported 1 solid, volume %.1f round-tripped" % v_back)

    # plural names still works as a list selector
    p2 = os.path.join(d, "two.step")
    assert s.act("solid.export", {"names": ["two"], "path": p2}).ok

    # empty session refuses loudly rather than writing an empty file
    s2 = new_session("export_empty")
    bad = s2.act("solid.export", {"path": os.path.join(d, "empty.step")})
    assert not bad.ok and "nothing" in (bad.error or ""), bad.error
    print("empty session refused cleanly: %s" % bad.error)

    print("EXPORT SELECTOR SMOKE OK", s.summary())
    s.registry.kernel.shutdown()
    s2.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_export_selector"):
    main()
