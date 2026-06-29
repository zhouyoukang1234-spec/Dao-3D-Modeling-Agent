"""Fatigue smoke -- mean-stress safety factors vs closed form.

``solid.fatigue`` reduces a cyclic (sigma_min..sigma_max) load to the
alternating/mean pair and applies the Goodman / Soderberg / Gerber mean-stress
criteria. Validated against hand-computed factors for sigma_a=sigma_m=100 with
Se=200, Su=400, Sy=300.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _close(got, exp, rel=1e-4):
    return abs(got - exp) <= rel * abs(exp) + 1e-6


def main():
    s = new_session("fatigue")
    print("FreeCAD", s.registry.kernel.freecad_version)

    Se, Su, Sy = 200.0, 400.0, 300.0
    # stress_max=200, stress_min=0 -> sigma_a=100, sigma_m=100
    base = {"stress_max": 200.0, "stress_min": 0.0,
            "endurance": Se, "ultimate": Su, "yield": Sy}

    g = s.act("solid.fatigue", dict(base, criterion="goodman")).data
    print("goodman:", g["stress_alt"], g["stress_mean"], g["safety_factor"])
    assert _close(g["stress_alt"], 100.0) and _close(g["stress_mean"], 100.0), g
    assert _close(g["safety_factor"], 1.0 / (100 / Se + 100 / Su)), g   # 1.3333
    assert _close(g["safety_factor"], 4.0 / 3.0), g
    assert g["infinite_life"] is True, g
    assert _close(g["yield_safety"], Sy / 200.0), g                      # 1.5

    so = s.act("solid.fatigue", dict(base, criterion="soderberg")).data
    assert _close(so["safety_factor"], 1.0 / (100 / Se + 100 / Sy)), so  # 1.2
    assert _close(so["safety_factor"], 1.2), so

    ge = s.act("solid.fatigue", dict(base, criterion="gerber")).data
    A, B = 100 / Se, (100 / Su) ** 2
    n_exp = (-A + math.sqrt(A * A + 4 * B)) / (2 * B)
    assert _close(ge["safety_factor"], n_exp), (ge["safety_factor"], n_exp)   # 1.6569
    # Gerber (parabola) is the least conservative -> largest factor
    assert ge["safety_factor"] > g["safety_factor"] > so["safety_factor"], (ge, g, so)
    print("gerber > goodman > soderberg:",
          ge["safety_factor"], g["safety_factor"], so["safety_factor"])

    # fully reversed (sigma_m=0): every criterion gives n = Se/sigma_a
    rev = s.act("solid.fatigue",
                {"stress_alt": 50.0, "stress_mean": 0.0,
                 "endurance": Se, "ultimate": Su}).data
    assert _close(rev["safety_factor"], Se / 50.0), rev                  # 4.0

    # Se estimated from Su via se_factor=0.5 matches an explicit Se=0.5*Su
    est = s.act("solid.fatigue",
                {"stress_alt": 80.0, "stress_mean": 0.0, "ultimate": Su}).data
    assert _close(est["endurance"], 0.5 * Su), est                       # 200
    assert _close(est["safety_factor"], (0.5 * Su) / 80.0), est

    # Basquin finite life: N = 0.5 (sigma_a/sigma_f')^(1/b)
    fl = s.act("solid.fatigue",
               {"stress_alt": 300.0, "stress_mean": 0.0, "endurance": Se,
                "ultimate": Su, "fatigue_coeff": 900.0, "fatigue_exp": -0.085}).data
    n_exp = 0.5 * (300.0 / 900.0) ** (1.0 / -0.085)
    assert _close(fl["cycles_to_failure"], n_exp, rel=1e-3), (fl, n_exp)
    print("basquin N:", fl["cycles_to_failure"])

    # guarded errors
    bad = s.act("solid.fatigue", {"stress_alt": 100.0})
    assert not bad.ok and "endurance" in (bad.error or ""), bad.error
    bad2 = s.act("solid.fatigue", {"stress_alt": 100.0, "endurance": Se,
                                   "criterion": "soderberg"})
    assert not bad2.ok and "yield" in (bad2.error or ""), bad2.error
    assert "KeyError" not in (bad.error or "") and "KeyError" not in (bad2.error or "")
    print("guards ok:", bad.error, "|", bad2.error)

    print("FATIGUE SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_fatigue"):
    main()
