# -*- coding: utf-8 -*-
"""三向 1:1:1 证据图: 原始照片 → Tripo 图转三维 → 我们自建可用模型.
三者各自用 道.感 搜最佳相机位姿、对照同一张照片算轮廓 IoU, 横排同框 + 叠加 + 指标表.
自建模型为几何自洽位姿 (6 杆 = 175mm 全可达)。"""
from __future__ import annotations
import os, sys, json
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import dao_jiao as DJ          # noqa: E402
import build_truth as BT       # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib import font_manager      # noqa: E402

PHOTO = r"C:\Users\Administrator\attachments\1e3e689a-718b-47ac-a271-445caac3a39d\SmartSelect_20260626_115856_Baidu.jpg"


def _overlay(pm, mm):
    pmf, mmf = DJ.fit_norm(pm), DJ.fit_norm(mm)
    ov = np.ones((*pmf.shape, 3))
    ov[pmf & ~mmf] = [0.90, 0.20, 0.20]
    ov[mmf & ~pmf] = [0.20, 0.65, 0.25]
    ov[pmf & mmf] = [0.50, 0.50, 0.50]
    return ov


def fit(V, F, VC, pm):
    pf = DJ.PoseFitter(V, F, VC)
    iou, az, el, mir, roll = pf.search(pm, log=lambda *a: None)
    img, mm = pf.fitted_render(az, el, mir, roll, W=560)
    return iou, img, mm, (az, el, mir, roll)


def main():
    pm, rgb = DJ.load_photo(PHOTO)

    # 方向二: Tripo 图转三维
    d = np.load(os.path.join(_HERE, "tripo_prepped.npz"))
    iou_t, img_t, mm_t, cam_t = fit(d["V"].astype(float), d["F"].astype(int),
                                    d["vcol"].astype(float), pm)
    print(f"Tripo  IoU={iou_t:.3f} cam={cam_t}")

    # 方向三: 我们自建可用模型 (几何自洽 home, 6 杆=175mm)
    V, F, VC, rep = BT.assemble(label="home", export=False)
    iou_m, img_m, mm_m, cam_m = fit(V, F, VC, pm)
    print(f"Built  IoU={iou_m:.3f} cam={cam_m} rods={rep['rods_mm']} "
          f"maxerr={rep['rod_max_err_mm']} unreach={rep['unreachable']}")

    ov_t, ov_m = _overlay(pm, mm_t), _overlay(pm, mm_m)

    fig = plt.figure(figsize=(16, 8.4))
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1], hspace=0.12, wspace=0.06)
    panels = [
        (gs[0, 0], rgb, "① real photo (本源实物)"),
        (gs[0, 1], img_t, f"② Tripo web mesh (图转三维)\nIoU={iou_t:.3f}"),
        (gs[0, 2], img_m, f"③ our built model (自建可用)\nIoU={iou_m:.3f}  6 rods=175.0mm"),
        (gs[1, 0], pm, "perceived silhouette (道.感分割)"),
        (gs[1, 1], ov_t, f"② overlay vs photo  IoU={iou_t:.3f}"),
        (gs[1, 2], ov_m, f"③ overlay vs photo  IoU={iou_m:.3f}"),
    ]
    for spec, im, title in panels:
        ax = fig.add_subplot(spec)
        ax.imshow(im, cmap="gray" if im.ndim == 2 else None)
        ax.set_title(title, fontsize=11)
        ax.axis("off")
    fig.suptitle("三向 1:1:1  原始照片 ≡ Tripo 图转三维 ≡ 我们自建可用模型   "
                 "(red=photo  green=model  gray=both)", fontsize=13)
    out = os.path.join(_HERE, "..", "output", "three_way_111.png")
    plt.savefig(out, dpi=95, bbox_inches="tight")
    plt.close()
    print("saved", os.path.normpath(out))

    summary = {
        "photo": "real hardware photo",
        "tripo": {"iou": round(float(iou_t), 3), "cam": list(cam_t)},
        "built": {"iou": round(float(iou_m), 3), "cam": list(cam_m),
                  "rods_mm": rep["rods_mm"], "rod_max_err_mm": rep["rod_max_err_mm"],
                  "unreachable": rep["unreachable"], "recv_z": rep["recv_z"]},
    }
    with open(os.path.join(_HERE, "..", "output", "three_way_111.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    try:
        font_manager.fontManager.addfont(r"C:\Windows\Fonts\msyh.ttc")
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
        plt.rcParams["axes.unicode_minus"] = False
    except Exception:
        pass
    main()
