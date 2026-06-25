# -*- coding: utf-8 -*-
"""Assemble BOTH anchors from their JSON specs with zero mechanism code.

This is the payoff of the anchor-spec layer: the only things imported are the
generic loader (`uam.spec`) and the generic solver (it runs inside `Spec.solve`).
Neither SR6 nor Stewart contributes a single line of Python here — each is just
a data file.  If both close, "onboarding a new machine = write a spec" holds.
"""
from __future__ import annotations

import os
import sys

import numpy as np

try:  # keep the Chinese coda legible on a cp1252 console without crashing
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

from uam.spec import load_file  # noqa: E402

SPECS = {
    "SR6 (real 6-leg home)": os.path.join(HERE, "sr6", "sr6.spec.json"),
    "Stewart (synthetic 6-6 home)": os.path.join(HERE, "stewart", "stewart.spec.json"),
    "Four-bar (hand-written, novel)": os.path.join(HERE, "fourbar", "fourbar.spec.json"),
    "Slider-crank (hand-written, prismatic)": os.path.join(HERE, "slidercrank", "slidercrank.spec.json"),
}


def check(spec):
    """Re-derive each constraint's residual after solving, honestly."""
    worst = 0.0
    for c in spec.constraints:
        r = float(np.linalg.norm(np.atleast_1d(c.residual())))
        worst = max(worst, r)
    return worst


def main():
    print("=== Assemble both anchors from JSON specs (loader + solver only) ===\n")
    ok = True
    for label, path in SPECS.items():
        spec = load_file(path)
        _, rms = spec.solve(verbose=False)
        worst = check(spec)
        flag = "CLOSES" if worst < 1e-4 else "OPEN"
        print(f"  {label:30s}  parts={len(spec.parts):2d} cons={len(spec.constraints):2d}"
              f"  solver RMS={rms:.2e}  worst constraint resid={worst:.2e}  [{flag}]")
        ok = ok and worst < 1e-4
    print("\n  => one generic loader + one generic solver assemble four unrelated"
          "\n     machines (two serialized, two hand-written) straight from data."
          "\n     the slider-crank is the first to need a prismatic lower pair,"
          "\n     added once to the shared primitive library -- not per machine."
          "\n     器 (the machine) is now pure data;"
          "\n     道 (the solver) is the only code.")
    assert ok, "a spec failed to close"


if __name__ == "__main__":
    main()
