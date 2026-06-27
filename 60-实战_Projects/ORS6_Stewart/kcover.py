# -*- coding: utf-8 -*-
"""kcover — render the physically-correct truth assembly posed/angled to match
the real SR6 cover photo (body right, cradle upper-left, black sleeve seated).

The mechanism geometry is the regression-tested truth_assembly (6 rods = 175mm,
neat symmetric truss, zero floating).  Here we only (a) add the recognizable
black sleeve into the cradle, (b) rigidly rotate the whole rigid assembly to the
cover's resting pose, and (c) render from a cover-matching camera.
"""
from __future__ import annotations
import os
import numpy as np
from PIL import Image

from .render import Part, render, hex_rgb, cylinder
from .truth_assembly import build, RECV_LIFT

HERE = os.path.dirname(__file__)
COVER = r"C:/Users/Administrator/ors6_work/pdf_pages/p00.png"
OUT = os.path.join(HERE, "output", "truth")


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def sleeve_parts():
    """Tapered ribbed black sleeve seated in the cradle (native +Z axis)."""
    parts = []
    base = np.array([0.0, 0.0, 22.0]) + RECV_LIFT     # just above cradle floor
    top = np.array([0.0, 0.0, 150.0]) + RECV_LIFT
    n = 12
    # stacked rings -> ribbed look, slight flare
    segs = 14
    for k in range(segs):
        z0 = base + (top - base) * (k / segs)
        z1 = base + (top - base) * ((k + 1) / segs)
        r0 = 31.0 + 3.0 * (k / segs)
        r1 = 31.0 + 3.0 * ((k + 1) / segs)
        # rib bulge every other
        rr = 1.6 if k % 2 == 0 else 0.0
        V0, F0 = cylinder(z0, z1, r=max(r0, r1) + rr, n=28)
        parts.append(Part(V0, F0, hex_rgb(0x141414), f"sleeve{k}"))
    # rounded cap
    cap0 = top
    cap1 = top + np.array([0.0, 0.0, 10.0])
    Vc, Fc = cylinder(cap0, cap1, r=33.0, n=28)
    parts.append(Part(Vc, Fc, hex_rgb(0x141414), "sleeve_cap"))
    return parts


def export_glb(parts, path):
    import trimesh
    scene = trimesh.Scene()
    for p in parts:
        if len(p.F) == 0:
            continue
        m = trimesh.Trimesh(vertices=p.V, faces=p.F, process=False)
        col = (np.array([*p.color, 1.0]) * 255).astype(np.uint8)
        m.visual.face_colors = np.tile(col, (len(p.F), 1))
        scene.add_geometry(m, geom_name=p.name or "part")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    scene.export(path)
    return path


def final():
    os.makedirs(OUT, exist_ok=True)
    parts = build() + sleeve_parts()
    Rg = rot_z(np.radians(-90)) @ rot_x(np.radians(60))
    rp = [Part(p.V @ Rg.T, p.F, p.color, p.name) for p in parts]
    allv = np.vstack([p.V for p in rp])
    b = (allv.min(0), allv.max(0))
    # hero from cover-matching camera
    hero = render(rp, view_dir=(0.0, -1.0, 0.35), W=1000, H=1150, bounds=b)
    Image.fromarray(hero).save(os.path.join(OUT, "ORS6_cover_hero.png"))
    # side-by-side cover | model
    cover = Image.open(COVER).convert("RGB")
    Hs = 1150
    cov = cover.resize((int(cover.width * Hs / cover.height), Hs))
    sbs = Image.new("RGB", (cov.width + 1000, Hs), "white")
    sbs.paste(cov, (0, 0))
    sbs.paste(Image.fromarray(hero), (cov.width, 0))
    sbs.save(os.path.join(OUT, "ORS6_cover_compare.png"))
    # GLB with sleeve, in upright assembly frame (not rotated)
    export_glb(parts, os.path.join(OUT, "ORS6_cover.glb"))
    print("wrote hero + compare + glb")


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = build() + sleeve_parts()

    cover = Image.open(COVER).convert("RGB")

    # candidate global orientations (rigid) + views
    cands = [
        ("a", rot_z(np.radians(-90)) @ rot_x(np.radians(60)), (0.0, -1.0, 0.35)),
        ("b", rot_z(np.radians(-90)) @ rot_x(np.radians(70)), (0.2, -1.0, 0.30)),
        ("c", rot_x(np.radians(62)), (0.0, -1.0, 0.25)),
        ("d", rot_z(np.radians(90)) @ rot_x(np.radians(62)), (0.0, -1.0, 0.30)),
        ("e", rot_z(np.radians(-90)) @ rot_x(np.radians(55)), (-0.3, -1.0, 0.30)),
        ("f", rot_z(np.radians(-60)) @ rot_x(np.radians(60)), (0.0, -1.0, 0.30)),
    ]
    Wt = 560
    imgs = []
    for tag, Rg, vd in cands:
        rp = [Part(p.V @ Rg.T, p.F, p.color, p.name) for p in parts]
        allv = np.vstack([p.V for p in rp])
        b = (allv.min(0), allv.max(0))
        im = render(rp, view_dir=vd, W=Wt, H=Wt, bounds=b)
        pim = Image.fromarray(im)
        imgs.append((tag, pim))
        pim.save(os.path.join(OUT, f"cover_cand_{tag}.png"))

    # montage: cover (left) + 6 candidates grid
    cov = cover.resize((Wt, int(cover.height * Wt / cover.width)))
    grid = Image.new("RGB", (Wt * 4, Wt * 2), "white")
    grid.paste(cov.crop((0, 0, Wt, min(Wt, cov.height))), (0, 0))
    for i, (tag, pim) in enumerate(imgs):
        col = 1 + i % 3
        row = i // 3
        grid.paste(pim, (col * Wt, row * Wt))
    grid.save(os.path.join(OUT, "cover_candidates.png"))
    print("wrote", os.path.join(OUT, "cover_candidates.png"))


if __name__ == "__main__":
    main()
