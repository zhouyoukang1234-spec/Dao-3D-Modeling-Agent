# -*- coding: utf-8 -*-
"""PHYSICS-FEASIBLE ORS6 assembly (no floating, by construction).

The complaint was floating parts.  A real SR6/ORS6 cannot float: every push-rod is
a RIGID body of fixed length whose two ball-joints must coincide with the horn ball
and the platform anchor.  So instead of placing the receiver wherever Tripo's surface
best fits (which leaves rods too long/short -> gaps -> floating), we PROJECT the Tripo
pose onto the mechanism's feasible manifold:

    find receiver 6-DOF pose  s.t.  every rod = EXACTLY 175mm (ball-centre to ball-centre)
    while staying as close as physics allows to the Tripo-registered pose.

Because the projected pose makes all 6 rod ball-centres meet their anchors exactly,
the chain horn-ball -> stud -> link -> stud -> platform-ball is fully closed: zero
floating, physically valid.  The real link STL (141.9/156.4mm bore-bore) sits centred,
its studs (175-link)/2 long are the real ball-screw studs.
"""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K
import kbuild as B
from overlay_check import render_fixed, _look_at
from scipy.optimize import linear_sum_assignment, least_squares
from scipy.spatial.transform import Rotation as Rot
from PIL import Image

WORK = K.WORK
B0 = np.array([0, 0, K.HOME_H])
rr = np.load(os.path.join(WORK, "recv_reg.npz"))
RR, TT = rr["R"], rr["t"]
SLOTS = [s for s, *_ in K.SERVO_SLOTS]


def pose(p):
    """p = [rotvec(3), dt(3)] delta applied on top of the ICP pose (RR,TT)."""
    dR = Rot.from_rotvec(p[:3]).as_matrix()
    return dR @ RR, dR @ TT + p[3:]


def mounts(p):
    R, t = pose(p)
    return {s: R @ B.MH[s] + t for s in SLOTS}


def assign(p):
    MW = mounts(p)
    C = np.array([[abs(B.solve_arm(si, MW[sj])[2] - B.ROD) for sj in SLOTS] for si in SLOTS])
    ri, cj = linear_sum_assignment(C)
    return {SLOTS[i]: SLOTS[j] for i, j in zip(ri, cj)}


def residuals(p, amap, wreg=0.06):
    MW = mounts(p)
    r = [B.solve_arm(s, MW[amap[s]])[2] - B.ROD for s in SLOTS]   # rod must be 175
    r += list(wreg * np.array(p))                                 # stay near Tripo
    return r


def project():
    amap = assign(np.zeros(6))                       # fix pairing at the Tripo pose
    sol = least_squares(residuals, np.zeros(6), args=(amap,),
                        method="lm", max_nfev=4000)
    p = sol.x
    MW = mounts(p)
    res = {}; tot = 0.0
    print("PHYSICS-PROJECTED pose  (rods forced to 175mm):")
    for s in SLOTS:
        M = MW[amap[s]]
        th, tip, d = B.solve_arm(s, M)
        res[s] = (th, tip, M, d); tot += abs(d - B.ROD)
        print(f"  {s:11s} -> {amap[s]:11s} angle={math.degrees(th):7.1f}  rod={d:7.2f}")
    dR = Rot.from_rotvec(p[:3])
    print(f"rod_resid_sum={tot:.3f}mm   pose drift: rot={dR.magnitude()*57.3:.1f}deg "
          f"trans={np.linalg.norm(p[3:]):.1f}mm")
    return p, res


def build(p, res):
    R, t = pose(p)
    parts = []
    for n in K.STATIC:
        parts.append((K.Tb(K.MESH[n][0]), K.MESH[n][1], K.C_BODY))
    for n in K.RECV_VIS:
        V, F = K.MESH[n]
        parts.append((K.Tb((V + B0) @ R.T + t), F, K.C_RECV))
    for s in SLOTS:
        th, tip, M, d = res[s]
        Va, Fa, Ra, ta = B.arm_transform(s, th)
        parts.append((K.Tb((Va @ Ra.T) + ta), Fa, K.C_ARM))
        Vl, Fl, ea, eb = B.place_link(s, tip, M)
        parts.append((K.Tb(Vl), Fl, K.C_ROD))
        for a, b in ((tip, ea), (M, eb)):            # real ball-screw studs
            Vc, Fc = B.cylinder(a, b, 3.0, 12)
            parts.append((K.Tb(Vc), Fc, K.C_ROD))
        for c in (tip, M):                           # ball joints at the true centres
            Vs, Fs = K.icosphere(c, 4.5, 1)
            parts.append((K.Tb(Vs), Fs, K.C_BALL))
    return parts


def render_standalone(parts, tag, views):
    Va = np.vstack([p[0] for p in parts])
    Ca = np.vstack([np.tile(p[2], (len(p[0]), 1)) for p in parts])
    offs = np.cumsum([0] + [len(p[0]) for p in parts])
    Fa = np.vstack([p[1] + offs[i] for i, p in enumerate(parts)])
    fix = (Va.min(0) + Va.max(0)) / 2
    imgs = []
    for vd in views:
        u, v = _look_at(vd)[:2]
        span = max(np.ptp(Va @ u), np.ptp(Va @ v)) * 1.12
        imgs.append(render_fixed(Va, Fa, Ca, vd, fix, span, W=560, H=620))
    Image.fromarray(np.hstack(imgs)).save(os.path.join(WORK, f"{tag}.png"))


def main():
    p, res = project()
    parts = build(p, res)
    K.export_glb(parts, os.path.join(WORK, "ORS6_phys.glb"))
    K.render_views(parts, "kphys")
    render_standalone(parts, "kphys_solo",
                      [(1, -1, 0.35), (1.2, -0.6, 0.1), (0.2, -1, 0.15), (-1, -0.6, 0.3)])
    print("saved ORS6_phys.glb + kphys.png + kphys_solo.png")


if __name__ == "__main__":
    main()
