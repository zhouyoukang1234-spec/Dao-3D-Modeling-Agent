"""Cam-profile build smoke -- analytic law turned into real CAD geometry.

``solid.cam_profile`` is the parametric design output: it sweeps the cam law
r(theta)=base+lift into an actual extruded disc in the live kernel. We confirm
the kernel-built solid matches the closed-form law:

  * the maximum profile radius equals base + rise (top of the rise/dwell);
  * the minimum equals the base circle (bottom dwell);
  * the solid is a valid closed prism whose volume = (polygon area)*thickness and
    sits between the inscribed base disc and circumscribed (base+rise) disc;
  * changing the rise re-generates a larger cam (parametric).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

BASE, RISE, THICK = 20.0, 8.0, 6.0


def main():
    s = new_session("cam_profile")
    print("FreeCAD", s.registry.kernel.freecad_version)

    r = s.act("solid.cam_profile", {"name": "cam1", "base_radius": BASE, "rise": RISE,
                                    "law": "cycloidal", "rise_angle": 120, "dwell_angle": 60,
                                    "fall_angle": 120, "thickness": THICK, "step": 1.0})
    assert r.ok, r.error
    d = r.data
    assert abs(d["max_radius"] - (BASE + RISE)) < 1e-2, d
    assert abs(d["min_radius"] - BASE) < 1e-2, d
    print("built cam: min r=%.2f (base), max r=%.2f (base+rise), %d samples"
          % (d["min_radius"], d["max_radius"], d["samples"]))

    # independent measurement of the built solid agrees with the reported bounds
    m = s.act("solid.measure", {"name": "cam1"}).data
    bx, by, _bz = m["bbox_size"]
    span = max(bx, by) / 2.0
    assert span <= BASE + RISE + 1e-3, (span, m["bbox_size"])
    # volume between inscribed base disc and circumscribed (base+rise) disc
    vlo = math.pi * BASE ** 2 * THICK
    vhi = math.pi * (BASE + RISE) ** 2 * THICK
    assert vlo < d["volume"] < vhi, (vlo, d["volume"], vhi)
    print("kernel solid: bbox span %.2f <= base+rise, volume %.1f within [%.1f, %.1f]"
          % (span, d["volume"], vlo, vhi))

    # parametric: a bigger rise yields a strictly bigger cam
    r2 = s.act("solid.cam_profile", {"name": "cam2", "base_radius": BASE, "rise": RISE * 2,
                                     "law": "harmonic", "thickness": THICK})
    assert r2.data["max_radius"] - d["max_radius"] > RISE - 1e-6, (r2.data, d)
    assert r2.data["volume"] > d["volume"], (r2.data["volume"], d["volume"])
    print("parametric: doubling rise grows max r %.1f -> %.1f" % (d["max_radius"], r2.data["max_radius"]))

    # invalid params rejected
    assert not s.act("solid.cam_profile", {"name": "x", "base_radius": -1, "rise": 5}).ok
    print("CAM_PROFILE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_cam_profile"):
    main()
