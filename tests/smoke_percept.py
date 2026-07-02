"""Structural 3-D perception smoke test.

Exercises:  percept.topology, percept.features, percept.section,
            percept.relations, percept.scene, percept.describe
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session


def main():
    s = new_session("smoke_percept")

    p_ops = [n for n in s.registry.names() if n.startswith("percept.")]
    assert len(p_ops) >= 6, "expected >= 6 percept ops, got %d: %s" % (len(p_ops), p_ops)
    print("percept ops:", sorted(p_ops))

    # --- plate with a through-hole: box cut by cylinder ---
    r = s.act("solid.box", {"name": "pbase", "length": 30, "width": 20, "height": 10})
    assert r.ok, "solid.box failed: %s" % r
    r = s.act("solid.cylinder", {"name": "pdrill", "radius": 4, "height": 10,
                                 "pos": [15, 10, 0]})
    assert r.ok, "solid.cylinder failed: %s" % r
    r = s.act("solid.cut", {"a": "pbase", "b": "pdrill", "out": "pplate"})
    assert r.ok, "solid.cut failed: %s" % r

    # --- percept.topology: typed faces + convexity-classified edges ---
    r = s.act("percept.topology", {"object": "pplate"})
    assert r.ok, "percept.topology failed: %s" % r
    topo = r.data
    kinds = [f["kind"] for f in topo["faces"]]
    assert kinds.count("plane") == 6, "expected 6 planes, got %s" % kinds
    assert kinds.count("cylinder") == 1, "expected 1 cylinder, got %s" % kinds
    convs = [e.get("convexity") for e in topo["edges"] if e.get("convexity")]
    assert convs and all(c == "convex" for c in convs), \
        "a plate with a through-hole has only convex edges: %s" % convs
    print("topology:", topo["counts"], "convexity:",
          {c: convs.count(c) for c in set(convs)})

    # --- blind hole: floor rim edge must read as concave ---
    r = s.act("solid.box", {"name": "bbase", "length": 30, "width": 20,
                            "height": 10})
    assert r.ok
    r = s.act("solid.cylinder", {"name": "bdrill", "radius": 4, "height": 5,
                                 "pos": [15, 10, 5]})
    assert r.ok
    r = s.act("solid.cut", {"a": "bbase", "b": "bdrill", "out": "pblock"})
    assert r.ok
    r = s.act("percept.topology", {"object": "pblock"})
    assert r.ok
    bconvs = [e.get("convexity") for e in r.data["edges"] if e.get("convexity")]
    assert "concave" in bconvs, "blind-hole floor edge should be concave: %s" % bconvs
    assert "convex" in bconvs
    print("blind-hole convexity:", {c: bconvs.count(c) for c in set(bconvs)})

    r = s.act("percept.features", {"object": "pblock"})
    assert r.ok
    bh = [f for f in r.data["features"] if f["type"] == "blind_hole"]
    assert len(bh) == 1, "expected 1 blind_hole, got %s" % r.data["features"]
    print("blind-hole feature:", bh[0])

    # --- percept.features: the hole is recognized as a through-hole ---
    r = s.act("percept.features", {"object": "pplate"})
    assert r.ok, "percept.features failed: %s" % r
    feats = r.data["features"]
    holes = [f for f in feats if f["type"] == "through_hole"]
    assert len(holes) == 1, "expected 1 through_hole, got %s" % feats
    assert abs(holes[0]["radius"] - 4) < 1e-6
    print("features:", feats)

    # --- percept.section: mid-height slice = outer rectangle + hole circle ---
    r = s.act("percept.section", {"object": "pplate", "normal": [0, 0, 1],
                                  "offset": 5})
    assert r.ok, "percept.section failed: %s" % r
    loops = r.data["loops"]
    assert len(loops) == 2, "expected 2 loops (outline + hole), got %d" % len(loops)
    assert all(lo["closed"] for lo in loops)
    lengths = sorted(lo["length"] for lo in loops)
    assert abs(lengths[1] - 100) < 0.1, "outer perimeter should be 100"
    print("section loops:", [(lo["closed"], lo["length"]) for lo in loops])

    # --- percept.relations: apart / overlap / containment ---
    r = s.act("solid.box", {"name": "pfar", "length": 5, "width": 5, "height": 5,
                            "pos": [100, 0, 0]})
    assert r.ok
    r = s.act("percept.relations", {"objects": ["pplate", "pfar"]})
    assert r.ok, "percept.relations failed: %s" % r
    rel = r.data["relations"][0]
    assert rel["relation"] == "apart" and rel["distance"] > 60
    assert rel["direction_b_from_a"] == "+x"
    print("relation pplate/pfar:", rel)

    r = s.act("solid.box", {"name": "pover", "length": 10, "width": 10,
                            "height": 10, "pos": [25, 15, 0]})
    assert r.ok
    r = s.act("percept.relations", {"objects": ["pplate", "pover"]})
    assert r.ok
    rel = r.data["relations"][0]
    assert rel["relation"] == "overlap" and rel["overlap_volume"] > 0
    print("relation pplate/pover:", rel)

    # --- percept.scene: whole-document digest ---
    r = s.act("percept.scene", {})
    assert r.ok, "percept.scene failed: %s" % r
    assert r.data["object_count"] >= 3
    assert "relations" in r.data
    print("scene:", r.data["object_count"], "objects,",
          len(r.data["relations"]), "relations")

    # --- percept.describe: stable text digest ---
    r = s.act("percept.describe", {"object": "pplate"})
    assert r.ok, "percept.describe failed: %s" % r
    desc = r.data["description"]
    assert "through_hole" in desc and "6 plane" in desc
    print("describe:", desc)

    # --- percept.diff: structural before/after comparison ---
    r = s.act("percept.diff", {"a": "pblock", "b": "pplate"})
    assert r.ok, "percept.diff failed: %s" % r
    d = r.data
    assert d["volume_delta"] < 0, "deeper hole removes material"
    assert any(f["type"] == "through_hole" for f in d["features_gained"]), \
        "diff should report the through-hole as gained: %s" % d
    assert any(f["type"] == "blind_hole" for f in d["features_lost"]), \
        "diff should report the blind-hole as lost: %s" % d
    r = s.act("percept.diff", {"a": "pplate", "b": "pplate"})
    assert r.ok and r.data["identical"], "same shape must diff as identical"
    print("diff ok:", {k: d[k] for k in ("volume_delta", "material_removed")})

    # --- pattern detection: 4 holes on a bolt circle ---
    r = s.act("solid.cylinder", {"name": "pdisc", "radius": 30, "height": 8})
    assert r.ok
    for i, (x, y) in enumerate([(20, 0), (0, 20), (-20, 0), (0, -20)]):
        r = s.act("solid.cylinder", {"name": "pd%d" % i, "radius": 3,
                                     "height": 8, "pos": [x, y, 0]})
        assert r.ok
        r = s.act("solid.cut", {"a": "pdisc", "b": "pd%d" % i, "out": "pdisc"})
        assert r.ok
    r = s.act("percept.features", {"object": "pdisc"})
    assert r.ok
    pats = r.data["patterns"]
    circ = [p for p in pats if p["type"] == "circular_pattern"]
    assert len(circ) == 1 and circ[0]["count"] == 4 \
        and abs(circ[0]["circle_radius"] - 20) < 1e-3, \
        "expected 4-hole bolt circle R=20, got %s" % pats
    print("pattern:", circ[0]["count"], "x r=%s on R=%s"
          % (circ[0]["feature_radius"], circ[0]["circle_radius"]))
    r = s.act("percept.describe", {"object": "pdisc"})
    assert r.ok and "4 x through_hole" in r.data["description"], \
        "describe should collapse the pattern: %s" % r.data
    print("pattern describe ok")

    # --- multi-object interference sweep ---
    r = s.act("solid.interference", {"names": ["pplate", "pfar", "pover"]})
    assert r.ok, "solid.interference names-form failed: %s" % r
    assert r.data["checked"] == 3 and r.data["interfering"]
    assert any(p["a"] == "pplate" and p["b"] == "pover"
               for p in r.data["pairs"])
    print("interference sweep:", r.data)
    r = s.act("solid.interference", {"names": ["pplate"]})
    assert not r.ok, "should reject a single-name interference sweep"

    # --- stepped shaft: shoulder edge must read concave (annulus faces) ---
    r = s.act("solid.cylinder", {"name": "pshaft", "radius": 10, "height": 60})
    assert r.ok
    r = s.act("solid.cylinder", {"name": "pstep", "radius": 16, "height": 20,
                                 "pos": [0, 0, 20]})
    assert r.ok
    r = s.act("solid.union", {"a": "pshaft", "b": "pstep", "out": "pshaft"})
    assert r.ok
    r = s.act("percept.topology", {"object": "pshaft"})
    assert r.ok
    convs = [e.get("convexity") for e in r.data["edges"]
             if e.get("convexity")]
    assert convs.count("concave") == 2, \
        "stepped shaft must show 2 concave shoulder edges: %s" % convs
    print("stepped shaft convexity ok:", convs)

    # --- solid.shell selector form ('zmax' opens the top) ---
    r = s.act("solid.box", {"name": "pbox2", "length": 40, "width": 30,
                            "height": 20})
    assert r.ok
    vol0 = r.data["volume"]
    r = s.act("solid.shell", {"name": "pbox2", "thickness": 3,
                              "faces": "zmax"})
    assert r.ok, "solid.shell selector form failed: %s" % r
    assert r.data["volume"] != vol0
    r = s.act("solid.shell", {"name": "pbox2", "thickness": 3,
                              "open_faces": "diagonal"})
    assert not r.ok, "should reject a malformed shell selector"
    print("shell selector ok")

    # --- filleted union: describe must survive fillet seam edges ---
    r = s.act("solid.fillet", {"name": "pshaft", "radius": 2})
    assert r.ok
    r = s.act("percept.describe", {"object": "pshaft"})
    assert r.ok, "percept.describe must survive fillet edges: %s" % r
    assert r.data["description"]
    print("fillet describe ok")

    # --- guards ---
    r = s.act("percept.topology", {"object": "nonexistent"})
    assert not r.ok, "should reject nonexistent object"
    r = s.act("percept.section", {"object": "pplate", "normal": [0, 0, 0]})
    assert not r.ok, "should reject zero normal"
    r = s.act("percept.relations", {"objects": ["pplate"]})
    assert not r.ok, "should reject < 2 objects"
    print("guards ok: bad inputs rejected")

    print("PERCEPT SMOKE OK", s.summary())


if __name__ == "__main__":
    main()
