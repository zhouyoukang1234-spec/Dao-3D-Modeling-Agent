"""v2/hero.py -- hero render of the body datum + side-by-side vs reference photo."""
import os, sys, glob, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(__file__))
from render import render_views
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")
OUT = os.path.join(os.path.dirname(__file__), "..", "results")
REF = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "ref", "ayva_3d_ref.png")

BODY = ["Base", "Lid", "LFrame", "RFrame", "Receiver"]
COLORS = {"Base": "#9aa0a6", "Lid": "#3c4043", "LFrame": "#1f6fbf",
          "RFrame": "#1f6fbf", "Receiver": "#2a7fd0"}


def main():
    M = {n: trimesh.load(os.path.join(STL_DIR, n + ".stl"), process=True) for n in BODY}
    parts = [(M[n].vertices, M[n].faces, COLORS[n]) for n in BODY]
    render_views(parts, os.path.join(OUT, "v2_body_hero.png"),
                 title="OSR6 body (feature-shared datum)",
                 views=[("perspective", 18, -58)], figsize=(7, 7))

    # side-by-side vs reference
    hero = mpimg.imread(os.path.join(OUT, "v2_body_hero.png"))
    ref = mpimg.imread(REF)
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ax[0].imshow(ref); ax[0].set_title("reference (Ayva 3D)"); ax[0].axis("off")
    ax[1].imshow(hero); ax[1].set_title("v2 body datum render"); ax[1].axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "v2_body_vs_ref.png"), dpi=110)
    print("saved v2_body_hero.png, v2_body_vs_ref.png")


if __name__ == "__main__":
    main()
