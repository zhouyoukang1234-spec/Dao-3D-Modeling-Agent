"""Library-query smoke -- search a catalogue by *mounting feature*, not shape.

``library_match`` finds parts that look like a query solid; ``library_query``
answers the complementary, intent-level question -- "which catalogued parts
carry the feature I need?". We build a tiny three-part library on disk with
*known* features, index it with feature extraction, and check the queries:

  * bracket   = block - 4 phi6 through-holes
  * plate     = block - 2 phi3.2 through-holes + a phi10 boss
  * blank     = a plain block (no features)

so "phi3.2 hole" hits only the plate, "a boss" hits only the plate, ">=4 holes"
hits only the bracket, and a feature-less index is refused loudly.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("library_query")
    print("FreeCAD", s.registry.kernel.freecad_version)
    lib = tempfile.mkdtemp(prefix="dao_libq_")

    # bracket: 60x40x12 block with 4 phi6 corner through-holes
    s.act("solid.box", {"name": "br", "length": 60, "width": 40, "height": 12})
    for i, (x, y) in enumerate([(8, 8), (52, 8), (8, 32), (52, 32)]):
        s.act("solid.cylinder", {"name": "bh%d" % i, "radius": 3, "height": 24, "pos": [x, y, -6]})
        s.act("solid.cut", {"a": "br", "b": "bh%d" % i, "out": "br"})
    s.act("solid.export", {"names": ["br"], "path": os.path.join(lib, "bracket.step")})

    # plate: 50x50x10 block with 2 phi3.2 through-holes and a phi10 boss
    s.act("solid.box", {"name": "pl", "length": 50, "width": 50, "height": 10})
    for i, (x, y) in enumerate([(10, 25), (40, 25)]):
        s.act("solid.cylinder", {"name": "ph%d" % i, "radius": 1.6, "height": 20, "pos": [x, y, -5]})
        s.act("solid.cut", {"a": "pl", "b": "ph%d" % i, "out": "pl"})
    s.act("solid.cylinder", {"name": "pb", "radius": 5, "height": 8, "pos": [25, 25, 10]})
    s.act("solid.union", {"a": "pl", "b": "pb", "out": "pl"})
    s.act("solid.export", {"names": ["pl"], "path": os.path.join(lib, "plate.step")})

    # blank: plain block, no features
    s.act("solid.box", {"name": "bk", "length": 20, "width": 20, "height": 20})
    s.act("solid.export", {"names": ["bk"], "path": os.path.join(lib, "blank.step")})

    # ---- index the library with feature extraction ------------------------- #
    idx = os.path.join(lib, "_feat.index.json")
    ri = s.act("solid.library_index", {"dir": lib, "features": True, "out": idx}).data
    assert ri["features"] is True and ri["indexed"] == 3, ri
    assert os.path.isfile(idx), idx
    print("indexed %d parts with features" % ri["indexed"])

    # ---- query: which parts carry a phi3.2 hole? --> only the plate -------- #
    q1 = s.act("solid.library_query", {"index": idx, "hole_diam": 3.2}).data
    assert q1["matched"] == 1, q1
    assert q1["hits"][0]["label"] == "plate.step", q1
    print("phi3.2 hole -> %s" % [h["label"] for h in q1["hits"]])

    # ---- query: which parts have a boss? --> only the plate ---------------- #
    q2 = s.act("solid.library_query", {"index": idx, "boss": True}).data
    assert [h["label"] for h in q2["hits"]] == ["plate.step"], q2
    print("has boss -> %s" % [h["label"] for h in q2["hits"]])

    # ---- query: at least 4 holes? --> only the bracket --------------------- #
    q3 = s.act("solid.library_query", {"index": idx, "min_holes": 4}).data
    assert [h["label"] for h in q3["hits"]] == ["bracket.step"], q3
    assert q3["hits"][0]["features"]["through_holes"] == 4, q3
    print("min_holes>=4 -> %s" % [h["label"] for h in q3["hits"]])

    # ---- query: a through hole? --> bracket and plate, not the blank ------- #
    q4 = s.act("solid.library_query", {"index": idx, "through": True}).data
    assert sorted(h["label"] for h in q4["hits"]) == ["bracket.step", "plate.step"], q4
    print("through-hole -> %s" % sorted(h["label"] for h in q4["hits"]))

    # ---- query on the fly (no prebuilt index), phi6 hole -> bracket -------- #
    q5 = s.act("solid.library_query", {"dir": lib, "hole_diam": 6.0}).data
    assert [h["label"] for h in q5["hits"]] == ["bracket.step"], q5
    print("on-the-fly phi6 -> %s" % [h["label"] for h in q5["hits"]])

    # ---- a feature-less index is refused loudly ---------------------------- #
    plain = os.path.join(lib, "_plain.index.json")
    s.act("solid.library_index", {"dir": lib, "out": plain})       # features omitted
    bad = s.act("solid.library_query", {"index": plain, "boss": True})
    assert not bad.ok and "features=true" in (bad.error or "").lower(), bad.error
    print("feature-less index refused: %s" % bad.error)

    print("LIBRARY QUERY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_library_query"):
    main()
