"""Tolerance-stack smoke — 1-D dimensional stack-up (worst-case + RSS).

``solid.tolerance_stack`` sums a signed dimension chain and reports the gap
nominal, the worst-case limits (arithmetic tolerance sum) and the statistical
RSS limits (root-sum-square). Validated against hand computation:

  * nominal gap = sum(sign*nominal);
  * worst-case +/- = arithmetic sum of the +/- tolerances (signs flip which
    extreme a -1 link pushes toward);
  * RSS +/- = sqrt(sum of squares) <= worst-case;
  * the widest-band link is reported as dominant.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-3):
    return abs(got - exp) <= rel * abs(exp) + 1e-4


def main():
    s = new_session("tolstack")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # housing bore 50 +/-0.05 minus a shaft 49.8 +/-0.02 (sign -1) gives the
    # diametral clearance gap; add a 0.1 +0.03/-0.01 shim that closes it.
    links = [
        {"name": "bore", "nominal": 50.0, "tol": 0.05, "sign": 1},
        {"name": "shaft", "nominal": 49.8, "tol": 0.02, "sign": -1},
        {"name": "shim", "nominal": 0.1, "plus": 0.03, "minus": 0.01, "sign": -1},
    ]
    r = s.act("solid.tolerance_stack", {"links": links}).data
    print("gap  :", r["nominal"], "WC", r["worst_case"], "RSS", r["rss"],
          "dom", r["dominant"])

    assert _close(r["nominal"], 50.0 - 49.8 - 0.1), r          # 0.1
    # worst-case: +link adds its plus to max; -links add their minus to max.
    wc_plus = 0.05 + 0.02 + 0.01
    wc_minus = 0.05 + 0.02 + 0.03
    assert _close(r["worst_case"]["plus"], wc_plus), r
    assert _close(r["worst_case"]["minus"], wc_minus), r
    assert _close(r["worst_case"]["max"], 0.1 + wc_plus), r
    assert _close(r["worst_case"]["min"], 0.1 - wc_minus), r
    # RSS is the quadrature sum and never exceeds worst-case.
    assert _close(r["rss"]["plus"], math.sqrt(0.05**2 + 0.02**2 + 0.01**2)), r
    assert _close(r["rss"]["minus"], math.sqrt(0.05**2 + 0.02**2 + 0.03**2)), r
    assert r["rss"]["plus"] <= r["worst_case"]["plus"] + 1e-9, r
    assert r["dominant"] == "bore", r

    # single symmetric link: gap == nominal, symmetric bands.
    one = s.act("solid.tolerance_stack",
                {"links": [{"nominal": 12.0, "tol": 0.1}]}).data
    assert _close(one["nominal"], 12.0) and _close(one["worst_case"]["plus"], 0.1), one
    assert _close(one["rss"]["plus"], 0.1), one  # one term: RSS == worst-case

    print("TOLSTACK SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_tolstack"):
    main()
