"""DFM report smoke — the unified, process-aware manufacturability verdict.

``solid.dfm_report`` is the orchestration layer over the per-pillar DFM tools:
it owns the domain knowledge of which checks gate which process and folds them
into a single ``manufacturable`` verdict plus a human-readable issue list. This
suite proves the routing and the gating on parts whose answer is known:

  * a flanged, well-tapered open-top tub (loft + cut) is a clean two-plate
    moulding -> ``injection`` manufacturable, with draft/undercut/thickness all
    passing;
  * the same family with the taper reversed traps the core -> ``injection`` not
    manufacturable, and the issue list names the undercut;
  * a plain box prints support-free -> ``print`` manufacturable;
  * a sphere overhangs across its whole lower hemisphere -> ``print`` not
    manufacturable, issue list names the overhang;
  * a process the tool does not know raises.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _tub(s, name, outer, cavity):
    (ow0, oh0, oz0), (ow1, oh1, oz1) = outer
    (cw0, ch0, cz0), (cw1, ch1, cz1) = cavity
    s.act("solid.loft", {"name": name + "_o", "sections": [
        {"profile": {"rect": [ow0, oh0]}, "offset": oz0},
        {"profile": {"rect": [ow1, oh1]}, "offset": oz1}]})
    s.act("solid.loft", {"name": name + "_c", "sections": [
        {"profile": {"rect": [cw0, ch0]}, "offset": cz0},
        {"profile": {"rect": [cw1, ch1]}, "offset": cz1}]})
    return s.act("solid.cut", {"a": name + "_o", "b": name + "_c", "out": name})


def main():
    s = new_session("dfm_report")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # well-tapered tub -> injection-mouldable.
    assert _tub(s, "good", ((40, 40, 0), (44, 44, 20)),
                ((36, 36, 2), (41, 41, 24))).ok
    g = s.act("solid.dfm_report", {"name": "good", "process": "injection",
                                   "min_draft": 3}).data
    print("good/injection -> manufacturable=%s issues=%s" % (g["manufacturable"], g["issues"]))
    assert g["manufacturable"], g
    assert all(g["checks"][k]["pass"] for k in ("draft", "undercut", "thickness")), g

    # reversed taper -> the core is trapped -> not mouldable, undercut named.
    assert _tub(s, "bad", ((44, 44, 0), (40, 40, 20)),
                ((40, 40, 2), (36, 36, 22))).ok
    bad = s.act("solid.dfm_report", {"name": "bad", "process": "injection",
                                     "min_draft": 3}).data
    print("bad/injection  -> manufacturable=%s issues=%s" % (bad["manufacturable"], bad["issues"]))
    assert not bad["manufacturable"], bad
    assert not bad["checks"]["undercut"]["pass"], bad
    assert any("undercut" in m for m in bad["issues"]), bad

    # a plain box prints support-free.
    s.act("solid.box", {"name": "blk", "length": 20, "width": 20, "height": 20})
    pb = s.act("solid.dfm_report", {"name": "blk", "process": "print"}).data
    print("box/print      -> manufacturable=%s issues=%s" % (pb["manufacturable"], pb["issues"]))
    assert pb["manufacturable"] and pb["checks"]["overhang"]["pass"], pb

    # a sphere overhangs over its whole lower hemisphere -> needs support.
    s.act("solid.sphere", {"name": "ball", "radius": 15})
    ps = s.act("solid.dfm_report", {"name": "ball", "process": "print"}).data
    print("sphere/print   -> manufacturable=%s issues=%s" % (ps["manufacturable"], ps["issues"]))
    assert not ps["manufacturable"] and not ps["checks"]["overhang"]["pass"], ps
    assert any("overhang" in m for m in ps["issues"]), ps

    # an unknown process is rejected.
    bad_proc = s.act("solid.dfm_report", {"name": "blk", "process": "telepathy"})
    assert not bad_proc.ok, bad_proc

    print("DFM REPORT SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_dfm_report"):
    main()
