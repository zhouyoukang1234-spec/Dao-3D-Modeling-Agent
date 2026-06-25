# -*- coding: utf-8 -*-
"""L5 -- the rigid-consistency VALIDATOR.

The datum layer (uam/datum.py) grounds an assembly at ONE configuration (home),
proving perception and authority describe the same body to ~micron RMS.  But a
machine moves.  This module answers the next, sharper question, and it answers it
the predictive-coding way -- with an honest residual, never a fitted DOF:

    A controller commands each leg INDEPENDENTLY.  Nothing in that per-leg code
    knows the legs share one rigid platform.  Across the whole workspace, how far
    do the controller's independently-commanded attachment points drift from the
    positions a single RIGID body would actually occupy?

If the drift is ~0 everywhere, the control law IS an exact rigid-body solver and
the home datum extends rigidly across the workspace.  Where the drift grows, the
control law is a LINEARISATION about home; the drift is the exact, bounded model
error -- the "surprise" between two models of the same machine.  Reporting it
(instead of hiding it behind a fudge parameter) is the whole point: 反者道之動.

Everything here is mechanism-agnostic.  A specific machine supplies only
  * home_pts  : name -> attachment point in the world frame, at home (an L1/2 datum)
  * commanded : name -> where the CONTROL LAW puts that point for a given input
  * the intended rigid transform (R, t) for that same input
and gets back a per-point residual plus the inter-point distance drift (the
frame-free rigidity invariant a true rigid body must preserve exactly).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
from typing import Callable, Dict, List

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def rigid_transform(translation=(0, 0, 0), rpy=(0, 0, 0)):
    """A proper rigid motion: rotate (intrinsic R = Rz Ry Rx) then translate.

    Returns (R, t).  This is the *model* a true rigid body obeys -- the yardstick
    the control law is measured against, not something fitted to the data.
    """
    rx, ry, rz = rpy
    R = rot_z(rz) @ rot_y(ry) @ rot_x(rx)
    t = np.asarray(translation, float)
    return R, t


@dataclass
class ConsistencyResult:
    """How well a set of independently-commanded points matches one rigid body.

    resid    : per-point ||commanded - rigid_predicted||  (mm)
    dist_drift: for every point pair, |commanded distance - home distance| (mm);
                a true rigid body preserves every pairwise distance exactly, so
                this is a frame-free rigidity error needing no pose assumption.
    """
    keys: List[str]
    resid: Dict[str, float]
    dist_drift: Dict[tuple, float]
    rms: float = field(init=False)
    max_resid: float = field(init=False)
    max_drift: float = field(init=False)

    def __post_init__(self):
        r = np.array(list(self.resid.values()), float)
        self.rms = float(np.sqrt((r ** 2).mean())) if len(r) else 0.0
        self.max_resid = float(r.max()) if len(r) else 0.0
        d = np.array(list(self.dist_drift.values()), float)
        self.max_drift = float(np.abs(d).max()) if len(d) else 0.0


def consistency(home_pts: Dict[str, np.ndarray],
                commanded: Dict[str, np.ndarray],
                R: np.ndarray,
                t: np.ndarray) -> ConsistencyResult:
    """Compare control-law-commanded points against a rigid transform of home.

    home_pts  : name -> world position at home (the datum)
    commanded : name -> world position the control law commands for this input
    (R, t)    : the intended rigid motion for this input

    No parameter is fitted.  resid[name] = || commanded - (R*home + t) ||.
    """
    keys = [k for k in home_pts if k in commanded]
    resid = {}
    for k in keys:
        pred = R @ np.asarray(home_pts[k], float) + t
        resid[k] = float(np.linalg.norm(np.asarray(commanded[k], float) - pred))
    drift = {}
    for a, b in combinations(keys, 2):
        d_home = np.linalg.norm(np.asarray(home_pts[a]) - np.asarray(home_pts[b]))
        d_cmd = np.linalg.norm(np.asarray(commanded[a]) - np.asarray(commanded[b]))
        drift[(a, b)] = float(d_cmd - d_home)
    return ConsistencyResult(keys=keys, resid=resid, dist_drift=drift)


def sweep(home_pts: Dict[str, np.ndarray],
          control_law: Callable[[dict], Dict[str, np.ndarray]],
          intended: Callable[[dict], tuple],
          inputs: List[dict]) -> List[tuple]:
    """Run `consistency` over a list of input dicts.

    control_law(input) -> commanded world points
    intended(input)    -> (R, t) the rigid motion that input is meant to produce
    Returns list of (input, ConsistencyResult).
    """
    out = []
    for u in inputs:
        cmd = control_law(u)
        R, t = intended(u)
        out.append((u, consistency(home_pts, cmd, R, t)))
    return out
