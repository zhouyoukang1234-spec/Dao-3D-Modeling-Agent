# -*- coding: utf-8 -*-
"""Render the physics-feasible assembly from a ring of azimuths and montage with the
   real SR6 cover photo, to self-verify the gross structure matches reality."""
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

el = 0.32
tiles = []
for az in np.linspace(0, 2 * math.pi, 8, endpoint=False):
    vd = (math.cos(az), math.sin(az), el)
    u, v = _look_at(vd)[:2]
    span = max(np.ptp(Va @ u), np.ptp(Va @ v)) * 1.13
    tiles.append(render_fixed(Va, Fa, Ca, vd, fix, span, W=440, H=500))
row1 = np.hstack(tiles[:4]); row2 = np.hstack(tiles[4:])
grid = Image.fromarray(np.vstack([row1, row2]))

cover = Image.open(os.path.join("C:/Users/Administrator/ors6_work/pdf_pages/p00.png")).convert("RGB")
cover = cover.resize((int(cover.width * grid.height / cover.height), grid.height))
combo = Image.new("RGB", (cover.width + grid.width, grid.height), (255, 255, 255))
combo.paste(cover, (0, 0)); combo.paste(grid, (cover.width, 0))
combo.save(os.path.join(WORK, "kcompare.png"))
print("saved kcompare.png")
