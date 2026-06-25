# -*- coding: utf-8 -*-
"""uam/spec.py — declarative anchor-spec loader (the data layer above L2/L3).

The thesis of this whole project is that onboarding a new mechanism should mean
*declaring* it, not *coding* it.  `uam/assembly.py` already removed hand-written
poses; a mechanism is still, however, a Python `build()` function.  This module
removes that too: a machine is a plain dict (JSON/YAML on disk) of

    {
      "name": "...",
      "parts":       [ {name, fixed?, mesh?, seed?:{t,q}, connectors:{c:{point,axis}}} ],
      "world_points":{ "name": [x,y,z], ... },          # optional named anchors
      "constraints": [ {type, a:[part,conn], b:[part,conn]|"world.name", d?}, ... ]
    }

and `load(spec)` turns it into the exact (parts, constraints) the generic solver
consumes.  No mechanism-specific code path exists here either — SR6 and the
Stewart platform are then literally two data files fed to one loader+solver.

A constraint `b` (or a target) may be:
  * ["part", "conn"]        -> that part's connector
  * "world.NAME"            -> a named point from `world_points`
  * [x, y, z]               -> a literal world point / direction
"""
from __future__ import annotations

import json
from typing import Any

import numpy as np

from uam.assembly import (
    Coincident, Distance, Parallel, Part, PointAt, PointOnLine, solve,
)


def _vec(v) -> np.ndarray:
    return np.asarray(v, float)


class Spec:
    """A loaded mechanism: parts + constraints, ready for uam.assembly.solve."""

    def __init__(self, name, parts, constraints, by_name, world_points):
        self.name = name
        self.parts = parts
        self.constraints = constraints
        self.by_name = by_name              # part name -> Part
        self.world_points = world_points    # name -> (3,)

    def solve(self, **kw):
        return solve(self.parts, self.constraints, **kw)

    def world_point(self, part_name, conn):
        return self.by_name[part_name].world_point(conn)


def _resolve_endpoint(ep: Any, by_name, world_points):
    """Return either a (Part, conn) tuple or a fixed (3,) world vector/point."""
    if isinstance(ep, str):
        if ep.startswith("world."):
            return world_points[ep[len("world."):]]
        raise ValueError(f"bad endpoint string {ep!r} (use 'world.NAME')")
    if isinstance(ep, (list, tuple)) and len(ep) == 2 and isinstance(ep[0], str):
        part_name, conn = ep
        if part_name in by_name:
            return (by_name[part_name], conn)
        # fall through: could be a 2-vector? Stewart/SR6 are 3D, so treat as ref.
        raise KeyError(f"unknown part {part_name!r} in endpoint {ep!r}")
    # literal 3-vector
    return _vec(ep)


def load(spec: dict) -> Spec:
    """Build (parts, constraints) from a declarative mechanism dict."""
    world_points = {k: _vec(v) for k, v in spec.get("world_points", {}).items()}

    parts, by_name = [], {}
    for pd in spec["parts"]:
        p = Part(pd["name"], fixed=bool(pd.get("fixed", False)),
                 mesh_name=pd.get("mesh"))
        for cname, c in pd.get("connectors", {}).items():
            p.add(cname, _vec(c["point"]), _vec(c.get("axis", [1, 0, 0])))
        seed = pd.get("seed")
        if seed:
            if "t" in seed:
                p.t = _vec(seed["t"])
            if "q" in seed:
                p.q = _vec(seed["q"])
        parts.append(p)
        by_name[pd["name"]] = p

    constraints = []
    for cd in spec["constraints"]:
        t = cd["type"].lower()
        a = _resolve_endpoint(cd["a"], by_name, world_points)
        if t == "coincident":
            b = _resolve_endpoint(cd["b"], by_name, world_points)
            constraints.append(Coincident(a, b))
        elif t in ("point_at", "pointat"):
            tgt = _resolve_endpoint(cd.get("target", cd.get("b")), by_name, world_points)
            constraints.append(PointAt(a, tgt))
        elif t == "parallel":
            b = _resolve_endpoint(cd["b"], by_name, world_points)
            constraints.append(Parallel(a, b))
        elif t == "distance":
            b = _resolve_endpoint(cd["b"], by_name, world_points)
            constraints.append(Distance(a, b, float(cd["d"])))
        elif t in ("on_line", "online", "point_on_line", "prismatic", "slider"):
            # line given either as {"point":[...],"dir":[...]} or [part,conn]
            ln = cd.get("line", cd.get("b"))
            if isinstance(ln, dict):
                line = (_vec(ln["point"]), _vec(ln["dir"]))
            else:
                line = _resolve_endpoint(ln, by_name, world_points)
            constraints.append(PointOnLine(a, line))
        else:
            raise ValueError(f"unknown constraint type {cd['type']!r}")

    return Spec(spec.get("name", "anchor"), parts, constraints, by_name, world_points)


def load_file(path: str) -> Spec:
    with open(path, "r", encoding="utf-8") as fh:
        return load(json.load(fh))
