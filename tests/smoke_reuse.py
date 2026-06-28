"""Reuse smoke -- 先检索复用、再从零建模, end to end.

``reuse`` fuses the three halves of the pipeline: find the closest catalogued
part (by shape *or* by mounting feature), recover it with ``reverse_build`` into
an editable replay program, and hand that back -- so a new requirement starts
from an existing design to adapt, not a blank sheet. We build a tiny library on
disk and check both query modes plus that a returned program really replays.

  * bracket = 60x40x12 block - 4 phi6 through-holes  (reverse-buildable exactly)
  * plate   = 50x50x10 block - 2 phi3.2 holes + a phi10 boss  (boss -> approx)
  * blank   = a plain block                                   (a box primitive)
"""
import copy
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("reuse")
    print("FreeCAD", s.registry.kernel.freecad_version)
    lib = tempfile.mkdtemp(prefix="dao_reuse_")

    s.act("solid.box", {"name": "br", "length": 60, "width": 40, "height": 12})
    for i, (x, y) in enumerate([(8, 8), (52, 8), (8, 32), (52, 32)]):
        s.act("solid.cylinder", {"name": "bh%d" % i, "radius": 3, "height": 24, "pos": [x, y, -6]})
        s.act("solid.cut", {"a": "br", "b": "bh%d" % i, "out": "br"})
    s.act("solid.export", {"names": ["br"], "path": os.path.join(lib, "bracket.step")})

    s.act("solid.box", {"name": "pl", "length": 50, "width": 50, "height": 10})
    for i, (x, y) in enumerate([(10, 25), (40, 25)]):
        s.act("solid.cylinder", {"name": "ph%d" % i, "radius": 1.6, "height": 20, "pos": [x, y, -5]})
        s.act("solid.cut", {"a": "pl", "b": "ph%d" % i, "out": "pl"})
    s.act("solid.cylinder", {"name": "pb", "radius": 5, "height": 8, "pos": [25, 25, 10]})
    s.act("solid.union", {"a": "pl", "b": "pb", "out": "pl"})
    s.act("solid.export", {"names": ["pl"], "path": os.path.join(lib, "plate.step")})

    s.act("solid.box", {"name": "bk", "length": 20, "width": 20, "height": 20})
    s.act("solid.export", {"names": ["bk"], "path": os.path.join(lib, "blank.step")})

    # ---- feature mode: "I need a part with >=4 holes" -> the bracket -------- #
    rf = s.act("solid.reuse", {"dir": lib, "min_holes": 4}).data
    assert rf["mode"] == "feature", rf
    assert rf["returned"] == 1, rf
    hit = rf["reusable"][0]
    assert hit["label"] == "bracket.step", hit
    assert hit["volume_match"] is True, hit
    assert hit["recipe_kind"] == "billet:box-minus-holes", hit
    assert len(hit["program"]["cuts"]) == 4, hit
    print("feature reuse: %s -> %s (%d cuts)"
          % (hit["label"], hit["recipe_kind"], len(hit["program"]["cuts"])))

    # the returned program is a real, replayable recipe ---------------------- #
    rp = s.act("solid.replay", {"program": hit["program"], "out": "reused_part"}).data
    assert rp["volume"] > 0 and rp["cuts"] == 4, rp
    print("replayed reused program: vol=%g cuts=%d" % (rp["volume"], rp["cuts"]))

    # an edited program (bigger holes) removes more material ----------------- #
    prog2 = copy.deepcopy(hit["program"])
    for c in prog2["cuts"]:
        c["r"] = c["r"] + 1.0
    re = s.act("solid.replay", {"program": prog2, "out": "reused_adapted"}).data
    assert re["volume"] < rp["volume"], (re, rp)
    print("adapted (phi8 holes) reused program: vol=%g" % re["volume"])

    # ---- shape mode: "find me something like THIS part" -------------------- #
    s.act("solid.box", {"name": "q", "length": 60, "width": 40, "height": 12})
    for i, (x, y) in enumerate([(8, 8), (52, 8), (8, 32), (52, 32)]):
        s.act("solid.cylinder", {"name": "qh%d" % i, "radius": 3, "height": 24, "pos": [x, y, -6]})
        s.act("solid.cut", {"a": "q", "b": "qh%d" % i, "out": "q"})
    rs = s.act("solid.reuse", {"name": "q", "dir": lib, "top": 3}).data
    assert rs["mode"] == "shape", rs
    best = rs["reusable"][0]
    assert best["label"] == "bracket.step" and best["same_key"] is True, best
    assert best["volume_match"] is True and best["program"]["cuts"], best
    print("shape reuse best: %s dist=%g same_key=%s"
          % (best["label"], best["distance"], best["same_key"]))

    # the plate (a boss) is still returned, flagged honestly, not dropped ---- #
    plate = [r for r in rs["reusable"] if r["label"] == "plate.step"]
    if plate:
        assert plate[0]["volume_match"] is False and "note" in plate[0], plate[0]
        print("plate returned honestly: volume_match=%s note=%s"
              % (plate[0]["volume_match"], bool(plate[0].get("note"))))

    print("REUSE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_reuse"):
    main()
