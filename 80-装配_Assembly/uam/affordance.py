#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uam/affordance.py — L1: perceived features -> functional affordances.

L0 (perceive.py) yields geometry: holes with (radius, axis, center, length).
L1 asks "what is this hole FOR?" and tags it with a functional type, so L2 can
wire mates by MEANING ("rod pivot to rod pivot") rather than by raw geometry.

The mapping radius->type is a learned prior over a fastener/bearing vocabulary;
it is data, not code, so it generalizes: feed any catalog and the buckets adapt.
Nothing here is SR6-specific — SR6 just happens to use M3/M4/608ZZ like most
hobby mechanisms.
"""
from __future__ import annotations
import os, sys, json
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)

# (label, nominal_radius_mm) — bore radius of standard metric clearance holes,
# common pins and the 608ZZ bearing seat. A hole is tagged with the nearest
# nominal within NEAR_TOL; this is a prior over a fastener vocabulary (data, not
# code) so it adapts to any catalog you feed it.
VOCAB = [
    ("M2",         1.0),    # M2 clearance
    ("M3",         1.6),    # M3 clearance
    ("M4",         2.1),    # M4 clearance (~2.2)
    ("M5",         2.6),    # M5 clearance / small shoulder
    ("pin3",       2.9),    # 3mm pin / shoulder
    ("rod_pivot",  3.6),    # rod-end pivot pin
    ("ball_pivot", 6.0),    # ball-joint / large pivot
    ("bearing608", 19.0),   # 608ZZ outer race seat
]
NEAR_TOL = 0.45   # mm; beyond this a radius is "unknown"
# concentric rings within this xy/axis tolerance are one physical (counterbored) hole
CONC_XY = 0.6


def classify_hole(radius: float):
    cand = min(VOCAB, key=lambda kv: abs(radius - kv[1]))
    return cand[0] if abs(radius - cand[1]) <= NEAR_TOL else "unknown"


def _consolidate(holes):
    """Merge coaxial concentric rings (a counterbored hole detected at several
    radii) into one feature. Bore = smallest radius (the through-hole that a pin
    actually passes), head = largest. Keeps the bore center/axis."""
    groups = []
    for h in holes:
        c = np.array(h["center"]); ax = np.array(h["axis"])
        placed = False
        for g in groups:
            gc = np.array(g[0]["center"]); gax = np.array(g[0]["axis"])
            # same axis direction and same line (perp distance small)
            if abs(abs(float(ax @ gax)) - 1.0) < 0.05:
                d = c - gc
                perp = np.linalg.norm(d - (d @ gax) * gax)
                if perp < CONC_XY:
                    g.append(h); placed = True; break
        if not placed:
            groups.append([h])
    feats = []
    for g in groups:
        g.sort(key=lambda h: h["radius"])
        bore = g[0]
        feats.append({
            "type": classify_hole(bore["radius"]),
            "radius": bore["radius"],
            "head_radius": g[-1]["radius"],
            "center": bore["center"], "axis": bore["axis"],
            "length": max(h["length"] for h in g),
            "rings": len(g),
        })
    return feats


def classify_part(part: dict) -> dict:
    """Return affordance summary for one perceived part."""
    raw = [h for h in part.get("holes", []) if h["kind"] == "hole"]
    feats = _consolidate(raw)
    # group by type
    bytype: dict = {}
    for f in feats:
        bytype.setdefault(f["type"], []).append(f)
    # pivots are the load-bearing mate features (rod/ball/bearing); fasteners fix parts
    pivots = [f for f in feats if f["type"] in ("rod_pivot", "ball_pivot", "bearing608", "pin3", "M5")]
    fasteners = [f for f in feats if f["type"] in ("M2", "M3", "M4")]
    return {
        "name": part["name"], "n_holes": len(feats),
        "types": {k: len(v) for k, v in bytype.items()},
        "pivots": pivots, "fasteners": fasteners, "features": feats,
    }


def pivot_pairs(aff: dict, max_pairs: int = 8):
    """For a link, the candidate mate axes are pairs of pivots; report span + types.
    L2 picks which pair is the link's working axis (usually the longest same-type pair)."""
    pv = aff["pivots"]
    out = []
    for i in range(len(pv)):
        for j in range(i + 1, len(pv)):
            ci = np.array(pv[i]["center"]); cj = np.array(pv[j]["center"])
            out.append({
                "i": i, "j": j, "span": float(np.linalg.norm(ci - cj)),
                "types": (pv[i]["type"], pv[j]["type"]),
            })
    out.sort(key=lambda d: -d["span"])
    return out[:max_pairs]


def main():
    pj = os.path.join(_REPO, "results", "perceive.json")
    d = json.load(open(pj))
    out = {}
    print(f"{'part':26s} {'types (radius->function)'}")
    print("-" * 78)
    for name, part in d.items():
        aff = classify_part(part)
        out[name] = aff
        types = ", ".join(f"{k}×{v}" for k, v in sorted(aff["types"].items()))
        print(f"{name:26s} {types}")
        pp = pivot_pairs(aff, 3)
        for p in pp:
            print(f"{'':28s}pivot pair {p['types']} span={p['span']:.1f}mm")
    res = os.path.join(_REPO, "results", "affordance.json")
    json.dump(out, open(res, "w"), indent=1)
    print(f"\nwrote {res}")


if __name__ == "__main__":
    main()
