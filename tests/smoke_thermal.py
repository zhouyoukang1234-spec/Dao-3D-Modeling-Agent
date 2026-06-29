"""Thermal-expansion smoke — uniform free isotropic growth.

``solid.thermal_expansion`` scales every length by ``1 + alpha*dT``. Validated
against the closed form for a box: each dimension -> ``L*(1+alpha*dT)``, volume
-> ``V*(1+alpha*dT)^3`` and volumetric strain -> ``(1+alpha*dT)^3 - 1``.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-6):
    return abs(got - exp) <= rel * abs(exp) + 1e-6


def main():
    s = new_session("thermal")
    print("FreeCAD", s.registry.kernel.freecad_version)

    L, W, H = 100.0, 50.0, 20.0
    alpha, dt = 23e-6, 80.0          # aluminium, +80 K
    s.act("solid.box", {"name": "blk", "length": L, "width": W, "height": H})
    r = s.act("solid.thermal_expansion",
              {"name": "blk", "cte": alpha, "delta_t": dt}).data
    print("expand:", r["linear_strain"], r["expanded_dims"], r["expanded_volume"])

    eps = alpha * dt
    sc = 1.0 + eps
    assert _close(r["linear_strain"], eps), r
    assert _close(r["volumetric_strain"], sc**3 - 1.0), r
    assert _close(r["expanded_dims"][0], L * sc), r
    assert _close(r["expanded_dims"][1], W * sc), r
    assert _close(r["expanded_dims"][2], H * sc), r
    assert _close(r["delta_dims"][0], L * eps), r
    assert _close(r["expanded_volume"], L * W * H * sc**3, rel=1e-5), r

    # zero temperature delta -> no growth.
    z = s.act("solid.thermal_expansion",
              {"name": "blk", "cte": alpha, "delta_t": 0}).data
    assert _close(z["linear_strain"], 0.0) and _close(z["expanded_dims"][0], L), z

    # missing required args -> a guided error, not a raw KeyError.
    bad = s.act("solid.thermal_expansion", {"name": "blk", "delta_t": dt})
    assert not bad.ok and "cte" in (bad.error or ""), bad.error
    assert "KeyError" not in (bad.error or ""), bad.error
    print("missing cte refused cleanly: %s" % bad.error)

    print("THERMAL SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_thermal"):
    main()
