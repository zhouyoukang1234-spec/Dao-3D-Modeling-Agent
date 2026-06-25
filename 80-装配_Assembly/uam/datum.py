# -*- coding: utf-8 -*-
"""L1/2 -- the DATUM layer:  acquire absolute placement from an AUTHORITY,
never fabricate it.

This module is the reusable distillation of the central discovery of the whole
project (validated on the SR6 anchor, see anchors/sr6 and docs/CLOSURE_FINDINGS):

    A part STL fixes RELATIVE geometry (axes, holes, inter-feature offsets) but
    NOT absolute placement.  Absolute placement -- where the assembled body sits
    in the world and how it is oriented -- is information that simply is not in
    the individual part files.  A human gets it for free by glancing at the
    assembled product; an agent must EXPLICITLY ACQUIRE it from an authority and
    must NEVER invent it.

Authorities, in strict priority order (highest first):

    1. CONTROL_LAW   a controller's IK home constants -> EXACT, world-frame-free
    2. CAD           an assembled STEP / F3D            -> exact
    3. BUILD_GUIDE   an official assembly manual / PDF  -> approximate
    4. PHOTO         a single photo of the built device -> coarse

The actual placement is then SOLVED, not guessed: given >=3 non-collinear
features whose positions are known both in the part frame (perceived) and in the
world frame (authority), a single rigid pose (R, t) is recovered by Kabsch.  The
residual is the honest perception<->authority prediction error -- it is reported,
never tuned away.

Four principles this layer enforces (the hard-won ones; see docs):
  P1  A DOF is only legitimate if it is physical.  RMS=0 means nothing if a
      solver DOF does not exist in the real mechanism.  -> require_physical_dof.
  P2  Do not perceive actuator placement from printed-part geometry.  COTS
      bodies are not in the STLs; hole grids are mounts, not shaft axes.
  P3  Control-law-as-datum.  If a controller exists, lift its home constants;
      do not re-derive them from noisy perception.
  P4  An unconstrained assembly DOF must be closed by an authority, never by a
      hand-picked value that merely satisfies a magnitude constraint.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, Optional

import numpy as np


class Authority(IntEnum):
    """Provenance of a datum, ordered by trust (higher = more authoritative)."""
    PHOTO = 1
    BUILD_GUIDE = 2
    CAD = 3
    CONTROL_LAW = 4


def kabsch(P: np.ndarray, Q: np.ndarray):
    """Best PROPER rigid (R, t) mapping rows of P onto rows of Q (no scale).

    det(R) = +1 is enforced, so the result is a rotation, never a reflection --
    the silent reflection bug (det = -1) that mirrored the SR6 receiver onto the
    wrong side is impossible here.
    """
    P = np.asarray(P, float)
    Q = np.asarray(Q, float)
    cp = P.mean(0)
    cq = Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cq - R @ cp
    return R, t


def _collinear(Q: np.ndarray, tol: float = 1e-6) -> bool:
    """True if the authority points are (near) collinear -> pose under-determined."""
    if len(Q) < 3:
        return True
    c = Q - Q.mean(0)
    s = np.linalg.svd(c, compute_uv=False)
    # second singular value tiny relative to the first => all on a line
    return s[1] <= tol * max(s[0], 1e-12)


@dataclass
class PoseFit:
    """Result of placing a part by aligning perceived features onto an authority."""
    R: np.ndarray
    t: np.ndarray
    keys: list
    resid: np.ndarray          # per-feature residual (mm)
    authority: Authority
    rms: float = field(init=False)

    def __post_init__(self):
        self.rms = float(np.sqrt((self.resid ** 2).mean()))

    def apply(self, p: np.ndarray) -> np.ndarray:
        """Map a point from the part frame into the world frame."""
        return self.R @ np.asarray(p, float) + self.t

    def axis_angle(self):
        """Rotation as (unit_axis, angle_deg)."""
        ang = np.arccos(np.clip((np.trace(self.R) - 1.0) / 2.0, -1.0, 1.0))
        if ang < 1e-9:
            return np.array([0.0, 0.0, 1.0]), 0.0
        ax = np.array([self.R[2, 1] - self.R[1, 2],
                       self.R[0, 2] - self.R[2, 0],
                       self.R[1, 0] - self.R[0, 1]])
        ax = ax / np.linalg.norm(ax)
        return ax, float(np.degrees(ang))


def solve_placement(perceived: Dict[str, np.ndarray],
                    authority: Dict[str, np.ndarray],
                    source: Authority,
                    strict: bool = True) -> PoseFit:
    """Solve a part's absolute pose by rigid-fitting perceived features onto
    authority features sharing the same keys.

    perceived : feature -> position in the PART frame  (from L0 perception)
    authority : feature -> position in the WORLD frame (from an Authority)
    source    : which Authority supplied `authority` (recorded in the result)

    Raises (when strict) if the problem is under-determined -- refusing to
    return a confident pose that the data cannot actually support (P4).
    """
    keys = [k for k in perceived if k in authority]
    if strict and len(keys) < 3:
        raise ValueError(
            f"need >=3 shared features to fix a rigid pose, got {len(keys)}: {keys}")
    P = np.array([perceived[k] for k in keys], float)
    Q = np.array([authority[k] for k in keys], float)
    if strict and _collinear(Q):
        raise ValueError("authority features are collinear -> pose under-determined "
                         "(roll about the line is unconstrained; supply an off-line "
                         "feature or a higher authority -- do NOT guess it)")
    R, t = kabsch(P, Q)
    pred = (R @ P.T).T + t
    resid = np.linalg.norm(pred - Q, axis=1)
    return PoseFit(R=R, t=t, keys=keys, resid=resid, authority=source)


def require_physical_dof(name: str, exists_in_mechanism: bool):
    """P1 guard: refuse to use a solver DOF that is not physically real.

    Call this for every free parameter before trusting a low residual; passing
    `exists_in_mechanism=False` raises, which is exactly what should have stopped
    the old horizontal-sweep ball-x trick that faked RMS=0.
    """
    if not exists_in_mechanism:
        raise ValueError(
            f"DOF '{name}' is not physical; a residual driven to zero through it "
            f"is fabricated (P1). Remove it or map it to a real mechanism freedom.")


PRINCIPLES = (
    "P1 a DOF is legitimate only if physical (require_physical_dof)",
    "P2 do not perceive actuator placement from printed-part geometry",
    "P3 control-law-as-datum: lift IK home constants, don't re-derive them",
    "P4 close an unconstrained DOF with an authority, never a hand value",
)
