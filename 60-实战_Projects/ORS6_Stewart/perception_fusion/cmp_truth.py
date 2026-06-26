# -*- coding: utf-8 -*-
"""道.感.校 — 真值装配体 vs 实物照片. 搜位姿, 算 IoU, 出对照图."""
from __future__ import annotations
import os, sys
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import dao_jiao as DJ          # noqa: E402
import build_truth as BT       # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PHOTO = r"C:\Users\Administrator\attachments\1e3e689a-718b-47ac-a271-445caac3a39d\SmartSelect_20260626_115856_Baidu.jpg"


def main(label="home", **kw):
    pm, rgb = DJ.load_photo(PHOTO)
    V, F, VC, rep = BT.assemble(label=label, export=False, **kw)
    print("rods:", rep["rods_mm"], "max_err", rep["rod_max_err_mm"])
    pf = DJ.PoseFitter(V, F, VC)
    iou, az, el, mir, roll = pf.search(pm)
    print(f"best IoU={iou:.3f} @ az{az} el{el} mir{mir} roll{roll}")
    img, mm = pf.fitted_render(az, el, mir, roll, W=560)
    pmf, mmf = DJ.fit_norm(pm), DJ.fit_norm(mm)
    ov = np.ones((*pmf.shape, 3))
    ov[pmf & ~mmf] = [0.9, 0.2, 0.2]; ov[mmf & ~pmf] = [0.2, 0.7, 0.2]
    ov[pmf & mmf] = [0.5, 0.5, 0.5]

    fig, ax = plt.subplots(1, 4, figsize=(17, 5))
    ax[0].imshow(rgb); ax[0].set_title("real photo")
    ax[1].imshow(pm, cmap="gray"); ax[1].set_title("perceived silhouette")
    ax[2].imshow(img); ax[2].set_title(f"truth model @az{az}el{el}")
    ax[3].imshow(ov); ax[3].set_title(f"overlay IoU={iou:.3f}\n(red=photo grn=model)")
    for a in ax:
        a.axis("off")
    out = os.path.join(_HERE, "..", "output", f"truth_cmp_{label}.png")
    plt.tight_layout(); plt.savefig(out, dpi=95); plt.close()
    print("saved", os.path.normpath(out))
    return iou


if __name__ == "__main__":
    main()
