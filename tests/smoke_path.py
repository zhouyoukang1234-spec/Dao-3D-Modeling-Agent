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

    print("PATH SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_path"):
    main()
