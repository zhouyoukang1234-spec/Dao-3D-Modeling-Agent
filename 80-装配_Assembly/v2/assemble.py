"""v2/assemble.py -- feature-based OSR6 assembly on the shared body datum.

Discovery (v2_identity.png): Base, Lid, LFrame, RFrame, Receiver are already in a
SHARED assembly frame -- they sit together as the real machine body. Only the moving
linkage parts (Arm, LPitcher, RPitcher, MainLink_Alpha, PitcherLink_Alpha,
BearingMainLink, BearingPitcherLink) are each exported in their own local frame and
must be PLACED by mating their real mesh features onto the body.

This module: (1) loads the body datum, (2) will place moving parts by feature mates.
"""
import os, sys, glob, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(__file__))
from render import render_views
from cylinders import detect_cylinders

STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")
OUT = os.path.join(os.path.dirname(__file__), "..", "results")

BODY = ["Base", "Lid", "LFrame", "RFrame", "Receiver"]
COLORS = {
    "Base": "#9aa0a6", "Lid": "#3c4043",
    "LFrame": "#1f77b4", "RFrame": "#1f77b4",
    "Arm": "#202124", "LPitcher": "#202124", "RPitcher": "#202124",
    "MainLink_Alpha": "#202124", "PitcherLink_Alpha": "#202124",
    "BearingMainLink": "#5f6368", "BearingPitcherLink": "#5f6368",
    "Receiver": "#1f77b4",
}


def load_all():
    M = {}
    for p in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
        name = os.path.splitext(os.path.basename(p))[0]
        M[name] = trimesh.load(p, process=True)
    return M


def main():
    M = load_all()
    parts = [(M[n].vertices, M[n].faces, COLORS[n]) for n in BODY]
    render_views(parts, os.path.join(OUT, "v2_body.png"),
                 title="OSR6 body datum (shared assembly frame, identity poses)")
    print("saved v2_body.png")


if __name__ == "__main__":
    main()
