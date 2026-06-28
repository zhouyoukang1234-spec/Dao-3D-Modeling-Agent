# -*- coding: utf-8 -*-
"""Assembly using the ICP-registered receiver pose (recv_reg.npz).
   body via Tb; receiver = real cradle snapped to its Tripo region; legs solved to
   the resulting (rigid) mounts; real link STLs; render overlay + standalone."""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K
import kbuild as B
from overlay_check import render_fixed, _look_at
from scipy.optimize import linear_sum_assignment
from PIL import Image

WORK = K.WORK
B0 = np.array([0, 0, K.HOME_H])
rr = np.load(os.path.join(WORK, "recv_reg.npz"))
RR, TT = rr["R"], rr["t"]
print("receiver reg rmse", float(rr["rmse"]))


def mount_world(s):
    return RR @ B.MH[s] + TT


def build():
    slots = [s for s, *_ in K.SERVO_SLOTS]
    MW = {s: mount_world(s) for s in slots}
    # resolve the receiver twist/label ambiguity: assign each servo to the mount
    # it can physically reach closest to 175mm (optimal 1-1 assignment).
    cost = np.zeros((6, 6))
    for i, si in enumerate(slots):
        for j, sj in enumerate(slots):
            cost[i, j] = abs(B.solve_arm(si, MW[sj])[2] - B.ROD)
    ri, cj = linear_sum_assignment(cost)
    assign = {slots[i]: slots[j] for i, j in zip(ri, cj)}
    res = {}; tot = 0
    for s in slots:
        M = MW[assign[s]]
        th, tip, d = B.solve_arm(s, M)
        res[s] = (th, tip, M, d); tot += abs(d - B.ROD)
        print(f"  {s:11s} -> mount[{assign[s]:11s}] angle={math.degrees(th):7.1f}  rod={d:6.1f}")
    print("rod_resid_sum", round(tot, 1))

    parts = []
    for n in K.STATIC:
        parts.append((K.Tb(K.MESH[n][0]), K.MESH[n][1], K.C_BODY))
    for n in K.RECV_VIS:
        V, F = K.MESH[n]
        parts.append((K.Tb((V + B0) @ RR.T + TT), F, K.C_RECV))
    for s, *_ in K.SERVO_SLOTS:
        th, tip, M, d = res[s]
        V, F, R, t = B.arm_transform(s, th)
        parts.append((K.Tb((V @ R.T) + t), F, K.C_ARM))
        Vl, Fl, ea, eb = B.place_link(s, tip, M)
        parts.append((K.Tb(Vl), Fl, K.C_ROD))
        # thin connective rods close any small gap between horn-tip/mount and the
        # link's bore centres, so the chain reads continuous (no floating).
        for p, q in ((tip, ea), (M, eb)):
            Vc, Fc = B.cylinder(p, q, 2.4, 10)
            parts.append((K.Tb(Vc), Fc, K.C_ROD))
        # ball joints sit on the link's own bore centres (never floating).
        for p in (ea, eb):
            Vs, Fs = K.icosphere(p, 4.0, 1)
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
        span = max(np.ptp(Va @ _look_at(vd)[0]), np.ptp(Va @ _look_at(vd)[1])) * 1.12
        imgs.append(render_fixed(Va, Fa, Ca, vd, fix, span, W=560, H=620))
    Image.fromarray(np.hstack(imgs)).save(os.path.join(WORK, f"{tag}.png"))


def main():
    parts = build()
    K.export_glb(parts, os.path.join(WORK, "ORS6_build2.glb"))
    K.render_views(parts, "kbuild2")          # overlay vs Tripo (4 rows x 3 cols)
    render_standalone(parts, "kbuild2_solo",
                      [(1, -1, 0.35), (1.2, -0.6, 0.1), (0.2, -1, 0.15), (-1, -0.6, 0.3)])
    print("saved ORS6_build2.glb + kbuild2.png + kbuild2_solo.png")


if __name__ == "__main__":
    main()
