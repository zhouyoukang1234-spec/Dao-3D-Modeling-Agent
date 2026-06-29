"""CAM smoke — the ``path.*`` group (needs FreeCAD with the Path workbench).

Design a plate, build a Path Job over it, generate a profile (contour) tool
path, post-process to real G-code, and assert the output is physically sane:
the tool-radius-compensated outside profile of a W x D plate machined with a
tool of diameter T must lie on a path bounded by (W/2 + T/2). Proves the
design -> manufacture path emits inspectable machine instructions, not a stub.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("cam")
    print("FreeCAD", s.registry.kernel.freecad_version)
    pathtools = sorted(t for t in s.tools() if t.startswith("path."))
    print("path ops:", pathtools)
    if "path.gcode" not in pathtools:
        print("PATH SMOKE SKIP (Path workbench not available)")
        return

    W, D, H, T = 60.0, 40.0, 12.0, 6.0
    assert s.act("param.body", {"name": "PL"}).ok
    assert s.act("param.pad", {"body": "PL", "feature": "Slab",
                               "profile": {"rect": [W, D]}, "length": H}).ok

    rj = s.act("path.job", {"target": "PL", "tool_diameter": T})
    assert rj.ok, rj.error
    assert rj.data["tool_diameter_mm"] == T, rj.data
    print("job: %s  tool=%s d=%.1fmm  post=%s"
          % (rj.data["job"], rj.data["tool"], rj.data["tool_diameter_mm"], rj.data["postprocessor"]))

    rp = s.act("path.profile", {"side": "Outside"})
    assert rp.ok, rp.error
    assert rp.data["commands"] > 5, rp.data
    bb = rp.data["path_bbox"]
    # outside profile with tool radius T/2: |X| ~ W/2 + T/2, |Y| ~ D/2 + T/2
    assert abs(bb[3] - (W / 2 + T / 2)) < 1e-3 and abs(bb[0] + (W / 2 + T / 2)) < 1e-3, bb
    assert abs(bb[4] - (D / 2 + T / 2)) < 1e-3, bb
    print("profile: %d commands  path_bbox=%s (expect |X|=%.1f |Y|=%.1f)"
          % (rp.data["commands"], bb, W / 2 + T / 2, D / 2 + T / 2))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_plate.nc")
    rg = s.act("path.gcode", {"path": out})
    assert rg.ok, rg.error
    assert rg.data["feeds_g1"] >= 1 and rg.data["rapids_g0"] >= 1, rg.data
    assert rg.data["chars"] > 200, rg.data
    print("gcode: %d lines  G0=%d  G1=%d  -> %s"
          % (rg.data["lines"], rg.data["rapids_g0"], rg.data["feeds_g1"], out))

    # out-of-order use guides, never leaks a raw RuntimeError: path.profile/
    # drill/gcode before path.job used to surface as "RuntimeError: call
    # path.job first"; the type name must not leak.
    q = new_session("cam_seq")
    for op in ("path.profile", "path.drill", "path.gcode"):
        r = q.act(op, {})
        err = r.error or ""
        assert not r.ok, (op, r.data)
        assert "RuntimeError" not in err, (op, err)
        assert "path.job first" in err, (op, err)
    q.registry.kernel.shutdown()
    print("out-of-order path.* guided cleanly (no raw RuntimeError)")

    # malformed face/hole selectors used to leak a bare 'could not convert',
    # 'str object has no attribute get', 'Expected sequence of size 3' or
    # 'invalid literal' from _select_faces / _select_holes. They must guide.
    def _bad(r, *needles):
        err = r.error or ""
        assert not r.ok, ("expected failure", r.data)
        for raw in ("could not convert", "TypeError", "AttributeError",
                    "KeyError", "invalid literal", "Expected sequence",
                    "not enough values", "string indices"):
            assert raw not in err, (raw, err)
        for nd in needles:
            assert nd in err, (nd, err)

    _bad(s.act("path.profile", {"select": "x"}), "select")
    _bad(s.act("path.profile", {"select": {"index": 1.5}}), "whole face")
    _bad(s.act("path.profile", {"select": {"normal": [0, 0, "x"]}}), "normal")
    _bad(s.act("path.pocket", {"select": {"axis": "w"}}), "axis")
    _bad(s.act("path.pocket", {"select": {"axis": 5}}), "axis")
    _bad(s.act("path.drill", {"select": {"diameter": "x"}}), "diameter")
    _bad(s.act("path.drill", {"select": {"axis_dir": "x"}}), "axis_dir")
    _bad(s.act("path.drill", {"select": 5}), "select")
    _bad(s.act("path.pocket", {"select": {"index": [0]}}), "out of range")
    _bad(s.act("path.pocket", {"select": {"index": [1]}, "start_depth": "x"}),
         "start_depth")
    print("malformed path.* selectors/depths refused cleanly")

    print("PATH SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_path"):
    main()
