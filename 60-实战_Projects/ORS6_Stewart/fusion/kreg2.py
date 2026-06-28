# -*- coding: utf-8 -*-
"""Body registration v2: same trimmed-ICP shape fit as kreg.py, BUT resolve the
top/bottom flip ambiguity (the body box is near-symmetric, so pure shape ICP can
land upside-down) by selecting, among near-optimal fits, the orientation whose
assembly +Z (base->lid->receiver) points toward the detected Tripo ring.  This is
the root fix: with the wrong flip the receiver/legs are built on the wrong side.
Saves kfit_body.npz (R,t,rmse)."""
import os, sys, numpy as np
os.environ.setdefault("ORS6_STL_ROOT", r"C:\Users\Administrator\ors6_assets\STLs")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
sys.path.insert(0, ROOT); sys.path.insert(0, WORK); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trimesh
from scipy.spatial import cKDTree
from PIL import Image
from overlay_check import render_fixed, _look_at, load_concat
from kreg import load_tripo, kabsch, trimmed_icp, fib_rots


def main():
    Vt, Ft = load_tripo()
    tree = cKDTree(Vt)
    shell = load_concat(["Base", "L_Frame", "R_Frame", "Lid", "PowerBus"])
    src = np.asarray(shell.sample(9000), float)
    csrc = src.mean(0)

    rc = np.load(os.path.join(WORK, "ring_circle.npz"))
    ring_ctr = rc["center"]

    xq = np.quantile(Vt[:, 0], 0.45)
    cbody = Vt[Vt[:, 0] > xq].mean(0)

    cands = []
    for R0 in fib_rots(120):
        t0 = cbody - (R0 @ csrc)
        R, t, md = trimmed_icp(src, tree, R0, t0, keep=0.6, iters=40)
        cands.append((R, t, md))
    best_md = min(c[2] for c in cands)
    print("best shape md", round(best_md, 2), "candidates", len(cands))

    # keep near-optimal shape fits, choose orientation pointing toward ring
    keep = [c for c in cands if c[2] <= best_md * 1.5 + 0.5]
    scored = []
    for R, t, md in keep:
        bc = (R @ csrc) + t
        zdir = R @ np.array([0, 0, 1.0])           # assembly +Z in Tripo
        toward = ring_ctr - bc; toward = toward / (np.linalg.norm(toward) or 1)
        score = float(zdir @ toward)               # +1 => +Z points at ring
        scored.append((score, md, R, t))
    scored.sort(key=lambda s: (-s[0], s[1]))
    print("kept", len(scored), "best score/md", round(scored[0][0], 3), round(scored[0][1], 2),
          "| worst-kept score", round(scored[-1][0], 3))
    _, _, R, t = scored[0]
    # final polish at high keep
    R, t, md = trimmed_icp(src, tree, R, t, keep=0.72, iters=250)
    bc = (R @ csrc) + t; zdir = R @ np.array([0, 0, 1.0])
    toward = ring_ctr - bc; toward /= (np.linalg.norm(toward) or 1)
    print("FINAL md", round(md, 2), "z.toward_ring", round(float(zdir @ toward), 3))

    np.savez(os.path.join(WORK, "kfit_body.npz"), R=R, t=t, rmse=md)

    Vs = np.asarray(shell.vertices); Fs = np.asarray(shell.faces)
    Vsx = (R @ Vs.T).T + t
    Cs = np.tile([0.85, 0.12, 0.12], (len(Vsx), 1)); Ct = np.tile([0.20, 0.55, 0.95], (len(Vt), 1))
    allV = np.vstack([Vsx, Vt]); center = (allV.min(0) + allV.max(0)) / 2
    rows = []
    for vd in [(1, -1, 0.5), (0, -1, 0.1), (1, 0, 0.1), (0, 0, 1)]:
        span = max(np.ptp(allV @ _look_at(vd)[0]), np.ptp(allV @ _look_at(vd)[1])) * 1.1
        it = render_fixed(Vt, Ft, Ct, vd, center, span, W=440, H=440)
        Vo = np.vstack([Vsx, Vt]); Fo = np.vstack([Fs, Ft + len(Vsx)]); Co = np.vstack([Cs, Ct])
        io = render_fixed(Vo, Fo, Co, vd, center, span, W=440, H=440)
        rows.append(np.hstack([it, io]))
    Image.fromarray(np.vstack(rows)).save(os.path.join(WORK, "kreg2_body.png"))
    print("saved kreg2_body.png")


if __name__ == "__main__":
    main()
