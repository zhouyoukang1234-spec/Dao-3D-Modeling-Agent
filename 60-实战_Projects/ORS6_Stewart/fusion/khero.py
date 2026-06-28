# -*- coding: utf-8 -*-
"""Final hero render of the physics-feasible ORS6 assembly, from the cover-matching
   3/4 angle (cradle left, body right), high resolution."""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kfinal as K
import kphys as PH
from overlay_check import render_fixed, _look_at
from PIL import Image

WORK = K.WORK
p, res = PH.project()
parts = PH.build(p, res)
Va = np.vstack([q[0] for q in parts])
Ca = np.vstack([np.tile(q[2], (len(q[0]), 1)) for q in parts])
offs = np.cumsum([0] + [len(q[0]) for q in parts])
Fa = np.vstack([q[1] + offs[i] for i, q in enumerate(parts)])
fix = (Va.min(0) + Va.max(0)) / 2

views = [(0.05, 1, 0.30), (0.6, 0.8, 0.28), (1, 0.15, 0.30)]
imgs = []
for vd in views:
    u, v = _look_at(vd)[:2]
    span = max(np.ptp(Va @ u), np.ptp(Va @ v)) * 1.1
    imgs.append(render_fixed(Va, Fa, Ca, vd, fix, span, W=720, H=860))
Image.fromarray(np.hstack(imgs)).save(os.path.join(WORK, "khero.png"))
print("saved khero.png")
