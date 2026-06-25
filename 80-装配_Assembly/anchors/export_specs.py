# -*- coding: utf-8 -*-
"""Serialize the two anchors to declarative JSON specs.

The hand-written `build()` functions in assemble_full.py / stewart/assemble.py
are the source of truth for the geometry; this dumps that geometry into the
plain-data anchor-spec format so the SAME mechanisms can be re-assembled with
ZERO mechanism code, only `uam.spec.load` + `uam.assembly.solve`.  Run
`assemble_from_spec.py` afterwards to prove the data path closes identically.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
sys.path.insert(0, os.path.join(HERE, "sr6"))
sys.path.insert(0, os.path.join(HERE, "stewart"))

import assemble_full as sr6  # noqa: E402
from assemble import base_anchors, leg_lengths, build as stewart_build  # noqa: E402
from synthetic import stewart_home  # noqa: E402


def _l(v):
    return [round(float(x), 6) for x in np.asarray(v, float)]


def sr6_spec() -> dict:
    parts, _, rods = sr6.build()
    ground = parts[0]
    pspec = [{
        "name": "ground", "fixed": True,
        "connectors": {c.name: {"point": _l(c.point), "axis": _l(c.axis)}
                       for c in ground.connectors.values()},
    }]
    world_points, constraints = {}, []
    for name, shaft, piv, arm, link in sr6.LEGS:
        rod = rods[name]
        pspec.append({
            "name": f"rod::{name}",
            "seed": {"t": _l(rod.t), "q": _l(rod.q)},
            "connectors": {"s": {"point": [0, 0, 0], "axis": [1, 0, 0]},
                           "r": {"point": [round(link, 6), 0, 0], "axis": [1, 0, 0]}},
        })
        world_points[f"piv::{name}"] = _l(piv)
        constraints.append({"type": "distance", "a": [f"rod::{name}", "s"],
                            "b": ["ground", f"shaft::{name}"], "d": round(arm, 6)})
        constraints.append({"type": "point_at", "a": [f"rod::{name}", "r"],
                            "target": f"world.piv::{name}"})
    return {"name": "sr6_home_6leg", "parts": pspec,
            "world_points": world_points, "constraints": constraints}


def stewart_spec() -> dict:
    home = stewart_home()
    base = base_anchors()
    legs = leg_lengths(home, base)
    cen = np.mean(list(home.values()), axis=0)
    parts = [{
        "name": "ground", "fixed": True,
        "connectors": {f"B::{k}": {"point": _l(b), "axis": [0, 0, 1]}
                       for k, b in base.items()},
    }, {
        "name": "platform", "seed": {"t": _l(cen)},
        "connectors": {k: {"point": _l(np.asarray(p) - cen), "axis": [0, 0, 1]}
                       for k, p in home.items()},
    }]
    constraints = [{"type": "distance", "a": ["platform", k],
                    "b": ["ground", f"B::{k}"], "d": round(float(legs[k]), 6)}
                   for k in base]
    return {"name": "stewart_home_6_6", "parts": parts, "constraints": constraints}


def main():
    out = {
        os.path.join(HERE, "sr6", "sr6.spec.json"): sr6_spec(),
        os.path.join(HERE, "stewart", "stewart.spec.json"): stewart_spec(),
    }
    for path, spec in out.items():
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(spec, fh, indent=2)
        print("wrote", os.path.relpath(path, HERE), f"({len(spec['parts'])} parts, "
              f"{len(spec['constraints'])} constraints)")


if __name__ == "__main__":
    main()
