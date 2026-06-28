"""Library-match smoke -- search a folder of model files before modelling.

反者道之动: the cheapest part to make is one you already have. This builds a tiny
on-disk model library (primitives exported to STEP), then asks ``library_match``
to find, for a query solid, the file holding the same shape family:

  * a query box (moved and scaled x1.5) retrieves the box STEP first, with
    same_key=True and volume_ratio reporting the size relation ;
  * a sphere/cylinder/cone in the library sit strictly farther away ;
  * a non-existent path and a non-solid file are reported in ``skipped`` rather
    than aborting the whole search ;
  * an empty path list and a query with no candidates are refused loudly.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("library_match")
    print("FreeCAD", s.registry.kernel.freecad_version)
    lib = tempfile.mkdtemp(prefix="dao_lib_")

    # ---- author a tiny model library on disk --------------------------- #
    catalog = {}
    s.act("solid.box", {"name": "lib_box", "length": 20, "width": 30, "height": 40})
    s.act("solid.cylinder", {"name": "lib_cyl", "radius": 10, "height": 50})
    s.act("solid.sphere", {"name": "lib_sph", "radius": 15})
    s.act("solid.cone", {"name": "lib_cone", "radius1": 18, "radius2": 0, "height": 40})
    for nm in ("lib_box", "lib_cyl", "lib_sph", "lib_cone"):
        p = os.path.join(lib, nm + ".step")
        s.act("solid.export", {"names": [nm], "path": p})
        catalog[nm] = p
    paths = list(catalog.values())
    print("library: %d STEP files in %s" % (len(paths), lib))

    # ---- query: a box at a different pose AND 1.5x size ---------------- #
    s.act("solid.box", {"name": "q", "length": 30, "width": 45, "height": 60})
    s.act("solid.rotate", {"name": "q", "center": [0, 0, 0], "axis": [1, 2, 3], "angle": 33})
    s.act("solid.translate", {"name": "q", "vector": [120, -40, 15]})

    r = s.act("solid.library_match", {"name": "q", "paths": paths}).data
    print("best=%s dist=%.4f" % (r["best"], r["best_distance"]))
    assert r["best"] == "lib_box.step", r["ranking"]
    top = r["ranking"][0]
    assert top["same_key"] is True, top
    # library box is 1/1.5 the query's linear size -> (1/1.5)^3 the volume
    assert abs(top["volume_ratio"] - (1.0 / 1.5) ** 3) < 0.01, top
    # every non-box candidate is strictly farther
    others = [x for x in r["ranking"] if x["label"] != "lib_box.step"]
    assert all(x["distance"] > top["distance"] + 1e-6 for x in others), r["ranking"]
    assert all(x["same_key"] is False for x in others), r["ranking"]
    print("box family retrieved first (same_key, vol_ratio=%.2f); sphere/cyl/cone farther"
          % top["volume_ratio"])

    # ---- a bad path is skipped, not fatal ------------------------------- #
    empty = os.path.join(lib, "empty.txt")
    open(empty, "w").close()
    r2 = s.act("solid.library_match",
               {"name": "q", "paths": paths + [os.path.join(lib, "ghost.step"), empty]}).data
    reasons = {sk["reason"].split(":")[0] for sk in r2["skipped"]}
    assert r2["best"] == "lib_box.step", r2
    assert any("no such file" in sk["reason"] for sk in r2["skipped"]), r2["skipped"]
    print("robust to junk: best still lib_box.step, skipped=%s" % sorted(reasons))

    # ---- loud guards ---------------------------------------------------- #
    bad = s.act("solid.library_match", {"name": "q", "paths": []})
    assert not bad.ok and "paths" in (bad.error or "").lower(), bad.error
    print("empty path list refused: %s" % bad.error)

    none_usable = s.act("solid.library_match", {"name": "q", "paths": [empty]})
    assert not none_usable.ok and "no usable solid" in (none_usable.error or "").lower()
    print("library with no usable solid refused: %s" % none_usable.error)

    print("LIBRARY MATCH SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_library_match"):
    main()
