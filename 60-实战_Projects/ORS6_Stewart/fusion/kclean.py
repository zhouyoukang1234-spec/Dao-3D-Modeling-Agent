# -*- coding: utf-8 -*-
"""Clean assembly at the firmware-VALID deployed pose (sane angles, rods=175 exact,
   zero floating), using the REAL link STLs, rendered standalone & large from a
   photo-like viewpoint for honest visual judgement against the cover photo."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K
import kbuild as B
from overlay_check import render_fixed, _look_at
from PIL import Image
import trimesh

WORK = K.WORK


def build(tcode):
    tfs, tips, mounts = K.part_transforms(tcode)
    parts = []
    for n, R, t, kind in tfs:
        V, F = (K.ARM_M if n == "Arm_R" else K.ARM_ML if n == "Arm_L" else K.MESH[n])
        col = K.C_BODY if kind == "body" else K.C_ARM if kind == "arm" else K.C_RECV
        parts.append((K.Tb((V @ R.T) + t), F, col))
    for s, *_ in K.SERVO_SLOTS:
        a = np.array(tips[s]); b = np.array(mounts[s])
        Vl, Fl = B.place_link(s, a, b)
        parts.append((K.Tb(Vl), Fl, K.C_ROD))
        for p in (a, b):
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
    tcode = tuple(int(v) for v in np.load(os.path.join(WORK, "kfinal_pose.npz"))["tcode"])
    parts = build(tcode)
    K.export_glb(parts, os.path.join(WORK, "ORS6_clean.glb"))
    # photo-like 3/4 views + side
    render_standalone(parts, "kclean",
                      [(1, -1, 0.35), (1.2, -0.6, 0.1), (0.2, -1, 0.15), (-1, -0.6, 0.3)])
    print("saved ORS6_clean.glb + kclean.png  tcode", tcode)


if __name__ == "__main__":
    main()
