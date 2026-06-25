#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uam/assembly.py — L2/L3 generic mate kernel: connectors, constraints, solver.

NO assembly-specific logic lives here. A part is a rigid body carrying named
MATE CONNECTORS (a point + an axis, taken from perceived features). An assembly
is a set of parts plus CONSTRAINTS between their connectors. The solver finds the
6-DOF pose of every free part that drives all constraint residuals to zero.

This is the layer whose absence caused every prior SR6 failure (see docs/ROOT_CAUSE):
part poses are SOLVED from relationships, never hand-written as absolute offsets.

Pose representation: world_p = R(q) @ local_p + t , with q a unit quaternion.
Residuals are stacked and minimized with scipy.optimize.least_squares (Gauss-Newton/
Levenberg-Marquardt). The firmware/skeleton solution is used as the initial guess —
the "prior" in predictive-coding terms — so the solver just nulls prediction error.
"""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field
from scipy.optimize import least_squares


# ── quaternion (x,y,z,w) ───────────────────────────────────────────────────
def qnorm(q):
    q = np.asarray(q, float)
    return q / (np.linalg.norm(q) + 1e-12)

def qrot(q, v):
    q = qnorm(q); x, y, z, w = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])
    return v @ R.T

def qmat(q):
    q = qnorm(q); x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


@dataclass
class Connector:
    """A named mate frame on a part, in the part's LOCAL coordinates."""
    name: str
    point: np.ndarray          # local origin (on the feature, e.g. hole center)
    axis: np.ndarray           # local unit axis (e.g. hole axis)
    def __post_init__(self):
        self.point = np.asarray(self.point, float)
        self.axis = np.asarray(self.axis, float)
        self.axis = self.axis / (np.linalg.norm(self.axis) + 1e-12)


@dataclass
class Part:
    name: str
    connectors: dict = field(default_factory=dict)   # name -> Connector
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))   # world translation
    q: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1.0]))  # world quat
    fixed: bool = False
    mesh_name: str | None = None

    def add(self, name, point, axis):
        self.connectors[name] = Connector(name, point, axis); return self

    def world_point(self, cname):
        c = self.connectors[cname]
        return qrot(self.q, c.point) + self.t

    def world_axis(self, cname):
        c = self.connectors[cname]
        return qrot(self.q, c.axis)


# ── constraints: each returns a residual vector (0 when satisfied) ──────────
@dataclass
class Coincident:
    """Two connector origins coincide (ball/spherical joint)."""
    a: tuple  # (part, connector)
    b: tuple
    def residual(self):
        (pa, ca), (pb, cb) = self.a, self.b
        return pa.world_point(ca) - pb.world_point(cb)   # (3,)

@dataclass
class PointAt:
    """A connector origin coincides with a fixed world point."""
    a: tuple
    target: np.ndarray
    def residual(self):
        pa, ca = self.a
        return pa.world_point(ca) - np.asarray(self.target, float)

@dataclass
class Parallel:
    """Two axes parallel (cross product zero). For 'axis vs fixed world dir' pass a Part-less dir."""
    a: tuple
    b: object  # (part, connector) OR a fixed 3-vector
    def residual(self):
        pa, ca = self.a
        va = pa.world_axis(ca)
        vb = self.b.world_axis if False else None
        if isinstance(self.b, tuple):
            vb = self.b[0].world_axis(self.b[1])
        else:
            vb = np.asarray(self.b, float); vb /= (np.linalg.norm(vb)+1e-12)
        return np.cross(va, vb)   # (3,) zero when parallel

@dataclass
class Distance:
    """Distance between two connector origins equals d (rigid link length check)."""
    a: tuple
    b: tuple
    d: float
    def residual(self):
        (pa, ca), (pb, cb) = self.a, self.b
        return np.array([np.linalg.norm(pa.world_point(ca) - pb.world_point(cb)) - self.d])

@dataclass
class PointOnLine:
    """A connector origin must lie on the infinite line (point0, dir): the
    prismatic / slider lower pair.  Distance pins a point to a sphere, PointAt
    to a single point, Coincident to another point; none of them expresses
    'free to slide along one axis, pinned in the other two'.  The residual is
    the component of (P - point0) perpendicular to dir -- a 3-vector of rank 2
    (identically zero along dir), so it removes exactly the two translational
    DOF a slider forbids and leaves the one it permits.  `line` may be a fixed
    (point0, dir) pair or a (part, connector) whose origin/axis ride a moving
    part (a slider on a rocking guide)."""
    a: tuple
    line: object  # ((3,)point0, (3,)dir)  OR  (part, connector)
    def residual(self):
        pa, ca = self.a
        p = pa.world_point(ca)
        if isinstance(self.line, tuple) and len(self.line) == 2 and not hasattr(self.line[0], "world_point"):
            p0 = np.asarray(self.line[0], float)
            d = np.asarray(self.line[1], float)
        else:
            lp, lc = self.line
            p0 = lp.world_point(lc)
            d = lp.world_axis(lc)
        d = d / (np.linalg.norm(d) + 1e-12)
        w = p - p0
        return w - np.dot(w, d) * d   # (3,) perpendicular gap, zero on the line


def solve(parts, constraints, max_nfev=4000, verbose=False):
    """Solve free parts' 6-DOF poses to null all constraint residuals.
    Returns (result, rms_residual). Parts are mutated in place with the solution."""
    free = [p for p in parts if not p.fixed]
    # pack: per free part [tx,ty,tz, qx,qy,qz,qw]
    def pack():
        return np.concatenate([np.concatenate([p.t, p.q]) for p in free])
    def unpack(x):
        for i, p in enumerate(free):
            seg = x[7*i:7*i+7]
            p.t = seg[:3]; p.q = qnorm(seg[3:])
    x0 = pack()
    def fun(x):
        unpack(x)
        r = [c.residual() for c in constraints]
        # soft unit-quaternion regularization
        for p in free:
            r.append(np.array([np.linalg.norm(p.q) - 1.0]) * 0.0)
        return np.concatenate(r)
    # Real mechanisms carry redundant DOF (spherical bearings, axial spin), so the
    # constraint system can be underdetermined (residuals < variables). 'lm' refuses
    # that case; 'trf' handles it and converges to a valid member of the solution
    # family. Pick adaptively so the kernel stays general across both regimes.
    n_res = int(fun(x0).size)
    n_var = int(x0.size)
    method = "lm" if n_res >= n_var else "trf"
    res = least_squares(fun, x0, method=method, max_nfev=max_nfev,
                        xtol=1e-12, ftol=1e-12, gtol=1e-12)
    unpack(res.x)
    rms = float(np.sqrt(np.mean(res.fun**2)))
    if verbose:
        print(f"[solve] parts(free)={len(free)} cons={len(constraints)} "
              f"rms={rms:.4f} nfev={res.nfev} status={res.status}")
    return res, rms
