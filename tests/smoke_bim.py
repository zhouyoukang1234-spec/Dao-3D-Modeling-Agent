"""BIM / Architecture smoke test.

Exercises:  bim.wall, bim.structure, bim.floor, bim.building, bim.site,
            bim.add, bim.tree
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session


def main():
    s = new_session("smoke_bim")

    # --- bim ops registered ---
    bim_ops = [n for n in s.registry.names() if n.startswith("bim.")]
    assert len(bim_ops) >= 7, "expected >= 7 bim ops, got %d: %s" % (len(bim_ops), bim_ops)
    print("bim ops:", sorted(bim_ops))

    # --- wall ---
    r = s.act("bim.wall", {"length": 4000, "width": 200, "height": 3000, "name": "ExteriorWall"})
    assert r.ok, "bim.wall failed: %s" % r
    assert r.data["volume"] > 0, "wall has no volume"
    wall_name = r.data["name"]
    print("wall:", wall_name, "vol:", r.data["volume"])

    # --- structure (column) ---
    r = s.act("bim.structure", {"length": 300, "width": 300, "height": 3000, "name": "Column1"})
    assert r.ok, "bim.structure failed: %s" % r
    assert r.data["volume"] > 0, "structure has no volume"
    col_name = r.data["name"]
    print("structure:", col_name, "vol:", r.data["volume"])

    # --- floor from wall + column ---
    r = s.act("bim.floor", {"members": [wall_name, col_name], "name": "GroundFloor"})
    assert r.ok, "bim.floor failed: %s" % r
    floor_name = r.data["name"]
    print("floor:", floor_name, "members:", r.data["members"])

    # --- building from floor ---
    r = s.act("bim.building", {"members": [floor_name], "name": "MainBuilding"})
    assert r.ok, "bim.building failed: %s" % r
    bldg_name = r.data["name"]
    print("building:", bldg_name)

    # --- site from building ---
    r = s.act("bim.site", {"members": [bldg_name], "name": "ProjectSite"})
    assert r.ok, "bim.site failed: %s" % r
    site_name = r.data["name"]
    print("site:", site_name)

    # --- tree ---
    r = s.act("bim.tree", {})
    assert r.ok, "bim.tree failed: %s" % r
    assert r.data["count"] >= 5, "expected >= 5 objects in tree, got %d" % r.data["count"]
    names = [o["name"] for o in r.data["objects"]]
    print("tree:", names)

    # --- second wall + add to floor ---
    r = s.act("bim.wall", {"length": 3000, "width": 200, "height": 3000, "name": "InteriorWall"})
    assert r.ok
    wall2_name = r.data["name"]
    r = s.act("bim.add", {"parent": floor_name, "child": wall2_name})
    assert r.ok, "bim.add failed: %s" % r
    assert r.data["added"]
    print("added", wall2_name, "to", floor_name)

    # --- guards: bad inputs ---
    r = s.act("bim.floor", {"members": ["nonexistent_object"]})
    assert not r.ok, "bim.floor should reject nonexistent member"
    r = s.act("bim.add", {"parent": "nope", "child": wall2_name})
    assert not r.ok, "bim.add should reject nonexistent parent"
    print("guards ok: bad members/parent rejected")

    print("BIM SMOKE OK", s.summary())


if __name__ == "__main__":
    main()
