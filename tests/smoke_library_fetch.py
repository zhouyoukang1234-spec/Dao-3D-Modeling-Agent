"""Library-fetch smoke -- pull community/online models into the local pipeline.

取之尽锱铢: ``library_fetch`` is the bridge from the world's model repositories to
the in-box matching pipeline. We stand up a throwaway HTTP server over a folder of
STEP files (a stand-in for an online 3D repo), then prove the op:

  * downloads every URL into a local cache and fingerprints each solid ;
  * with ``name`` set, ranks the fetched models against a query box -- the box
    URL wins with same_key=True, exactly like ``library_match`` ;
  * with ``out`` set, persists a reusable library index over the cache ;
  * caches: a second fetch of the same name re-uses the file (no duplicate) ;
  * also accepts ``file://`` / local mirrors ;
  * a 404 link and an oversized download land in ``skipped`` without aborting ;
  * an empty ``urls`` list, and a harvest where nothing is usable, are refused.
"""
import functools
import http.server
import os
import socketserver
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _serve(directory):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler,
                                 directory=directory)
    httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), handler)
    httpd.daemon_threads = True
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd, httpd.server_address[1]


def main():
    s = new_session("library_fetch")
    print("FreeCAD", s.registry.kernel.freecad_version)
    src = tempfile.mkdtemp(prefix="dao_src_")
    cache = tempfile.mkdtemp(prefix="dao_cache_")

    # ---- author a tiny remote repo of STEP files ----------------------- #
    s.act("solid.box", {"name": "rep_box", "length": 20, "width": 30, "height": 40})
    s.act("solid.cylinder", {"name": "rep_cyl", "radius": 10, "height": 50})
    s.act("solid.sphere", {"name": "rep_sph", "radius": 15})
    names = ["rep_box", "rep_cyl", "rep_sph"]
    for nm in names:
        s.act("solid.export", {"names": [nm], "path": os.path.join(src, nm + ".step")})
    httpd, port = _serve(src)
    base = "http://127.0.0.1:%d/" % port
    urls = [base + nm + ".step" for nm in names]
    print("serving %d STEP at %s" % (len(names), base))

    # ---- query box (different pose + 1.5x) ----------------------------- #
    s.act("solid.box", {"name": "q", "length": 30, "width": 45, "height": 60})
    s.act("solid.rotate", {"name": "q", "center": [0, 0, 0], "axis": [1, 2, 3], "angle": 33})
    s.act("solid.translate", {"name": "q", "vector": [120, -40, 15]})

    # ---- fetch -> fingerprint -> rank against the query ---------------- #
    idx = os.path.join(cache, "remote.index.json")
    r = s.act("solid.library_fetch",
              {"urls": urls, "cache": cache, "name": "q", "out": idx}).data
    assert r["fetched"] == 3, r
    assert r["best"] == "rep_box.step", r["ranking"]
    assert r["ranking"][0]["same_key"] is True, r["ranking"]
    assert abs(r["ranking"][0]["volume_ratio"] - (1.0 / 1.5) ** 3) < 0.01, r["ranking"]
    assert os.path.isfile(idx), r
    for nm in names:
        assert os.path.isfile(os.path.join(cache, nm + ".step")), nm
    print("fetched %d, best=%s (same_key, vol_ratio=%.2f), index written"
          % (r["fetched"], r["best"], r["ranking"][0]["volume_ratio"]))

    # ---- the cached index queries exactly like a local library --------- #
    ri = s.act("solid.library_match", {"name": "q", "index": idx}).data
    assert ri["best"] == "rep_box.step", ri
    assert abs(ri["best_distance"] - r["best_distance"]) < 1e-9, (ri, r)
    print("cached index matches like a local library (best=%s)" % ri["best"])

    # ---- caching: a second fetch re-uses files, no duplication --------- #
    before = sorted(os.listdir(cache))
    r2 = s.act("solid.library_fetch", {"urls": urls, "cache": cache}).data
    assert r2["fetched"] == 3 and sorted(os.listdir(cache)) == before, (r2, before)
    print("re-fetch re-used cache, no duplicate files")

    # ---- file:// and bare local mirror also resolve -------------------- #
    local_step = os.path.join(src, "rep_box.step")
    rl = s.act("solid.library_fetch",
               {"urls": ["file://" + local_step, local_step], "cache": cache}).data
    assert rl["fetched"] == 2, rl
    print("file:// and local-path mirrors fetched (%d)" % rl["fetched"])

    # ---- a dead link is skipped, not fatal ----------------------------- #
    rd = s.act("solid.library_fetch",
               {"urls": urls + [base + "ghost.step"], "cache": cache, "name": "q"}).data
    assert rd["best"] == "rep_box.step", rd
    assert any("ghost.step" in sk.get("url", "") for sk in rd["skipped"]), rd["skipped"]
    print("dead link skipped, best still rep_box.step")

    # ---- max_bytes ceiling drops oversized downloads ------------------- #
    tiny = s.act("solid.library_fetch",
                 {"urls": urls, "cache": tempfile.mkdtemp(prefix="dao_tiny_"),
                  "max_bytes": 10})
    assert not tiny.ok and "no usable model" in (tiny.error or "").lower(), tiny.error
    print("max_bytes=10 dropped everything: %s" % tiny.error)

    # ---- loud guards --------------------------------------------------- #
    empty = s.act("solid.library_fetch", {"urls": []})
    assert not empty.ok and "urls" in (empty.error or "").lower(), empty.error
    print("empty urls refused: %s" % empty.error)

    httpd.shutdown()
    print("LIBRARY FETCH SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_library_fetch"):
    main()
