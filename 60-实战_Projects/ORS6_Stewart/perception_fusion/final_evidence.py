# -*- coding: utf-8 -*-
"""三向 1:1:1 终证 — 照片 | Tripo图转三维 | 自建可用模型(ORS6_home, 项目gen_deliverables产出)
全部置于与实物照片最贴合的相机位姿下渲染, 叠加轮廓, 列指标.

道.感 是裁判: 不靠肉眼, 用分割轮廓 IoU 量化三方向一致性.
"""
import os, sys, json, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "00-本源_Origin")))
import trimesh
import dao_jiao as DJ
import fastfit as FF
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
CJK = None
for _fp in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
    if os.path.exists(_fp):
        CJK = _fm.FontProperties(fname=_fp)
        break
plt.rcParams["axes.unicode_minus"] = False

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "output")
PHOTO = r"C:\Users\Administrator\attachments\1e3e689a-718b-47ac-a271-445caac3a39d\SmartSelect_20260626_115856_Baidu.jpg"

pm, rgb = DJ.load_photo(PHOTO)
pf_photo = DJ.fit_norm(pm)


def overlay(model_mask):
    """red=photo, green=model, yellow=both (both fit-normalized to S×S)."""
    mm = DJ.fit_norm(model_mask)
    S = pf_photo.shape[0]
    ov = np.ones((S, S, 3))
    ov[pf_photo] = [0.90, 0.25, 0.22]
    ov[mm] = [0.20, 0.70, 0.25]
    ov[pf_photo & mm] = [0.95, 0.85, 0.15]
    return ov, DJ.iou(pf_photo, mm)


CACHE = os.path.join(OUT, "three_way_final.json")


def fit_and_render(P, V, F, vcol=None, els=(0, 10, 20, 30, 45), label="", cached=None):
    pf = DJ.PoseFitter(V, F, vcol)
    if cached is not None:
        az, el, mir, roll, iou = cached["az"], cached["el"], cached["mir"], cached["roll"], cached["iou"]
    else:
        ff = FF.FastFitter(P)
        t0 = time.time()
        iou, az, el, mir, roll = ff.search(pm, az_step=20, els=els, coarse_res=170, fine=True, log=None)
        print(f"{label}: IoU={iou:.4f} @az{az}el{el}m{mir}r{roll} ({time.time()-t0:.0f}s)")
    img, mm = pf.fitted_render(az, el, mir, roll, W=520)
    return dict(iou=float(iou), az=int(az), el=int(el), mir=int(mir), roll=int(roll), img=img, mask=mm)


def main():
    np.random.seed(7)
    cache = json.load(open(CACHE)) if (os.path.exists(CACHE) and "--refit" not in sys.argv) else {}
    # ② Tripo
    d = np.load(os.path.join(os.path.dirname(__file__), "tripo_prepped.npz"))
    Vt, Ft, vct = np.asarray(d["V"], float), np.asarray(d["F"], int), np.asarray(d["vcol"], float)
    Pt, _ = trimesh.sample.sample_surface(trimesh.Trimesh(Vt, Ft, process=False), 45000)
    rt = fit_and_render(np.asarray(Pt, float), Vt, Ft, vct, label="② Tripo", cached=cache.get("tripo"))

    # ③ 自建可用模型 (official ORS6_home.stl)
    mh = trimesh.load(os.path.join(OUT, "ORS6_home.stl"), force="mesh")
    Vh, Fh = np.asarray(mh.vertices, float), np.asarray(mh.faces, int)
    Ph, _ = trimesh.sample.sample_surface(trimesh.Trimesh(Vh, Fh, process=False), 45000)
    rh = fit_and_render(np.asarray(Ph, float), Vh, Fh, None, label="③ 自建", cached=cache.get("selfbuilt"))

    ov_t, iou_t = overlay(rt["mask"])
    ov_h, iou_h = overlay(rh["mask"])

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3, height_ratios=[3, 2])
    a = fig.add_subplot(gs[0, 0]); a.imshow(rgb); a.axis("off"); a.set_title("① 实物照片 (real hardware)", fontsize=13, fontproperties=CJK)
    a = fig.add_subplot(gs[0, 1]); a.imshow(rt["img"]); a.axis("off"); a.set_title(f"② Tripo 图转三维  IoU={rt['iou']:.3f}", fontsize=13, fontproperties=CJK)
    a = fig.add_subplot(gs[0, 2]); a.imshow(rh["img"]); a.axis("off"); a.set_title(f"③ 自建可用模型(ORS6_home)  IoU={rh['iou']:.3f}", fontsize=13, fontproperties=CJK)
    a = fig.add_subplot(gs[1, 1]); a.imshow(ov_t); a.axis("off"); a.set_title(f"②叠加 红=实物/绿=Tripo  IoU={iou_t:.3f}", fontsize=11, fontproperties=CJK)
    a = fig.add_subplot(gs[1, 2]); a.imshow(ov_h); a.axis("off"); a.set_title(f"③叠加 红=实物/绿=自建  IoU={iou_h:.3f}", fontsize=11, fontproperties=CJK)
    txt = fig.add_subplot(gs[1, 0]); txt.axis("off")
    lines = [
        "三向 1:1:1 量化对照", "",
        f"② Tripo   轮廓IoU = {rt['iou']:.3f}",
        f"③ 自建     轮廓IoU = {rh['iou']:.3f}", "",
        "尺寸互证(三方向咬合):",
        " 接收环球铰半径  STL 59.98mm",
        "             -> Tripo视觉 59.3mm (Δ0.7mm)",
        " 接收环外径  CAD Ø114 -> Tripo Ø118.7", "",
        "③ 物理自洽(真零件装配):",
        " 6 杆全部 = 175.0mm  零不可达",
        " 底座/臂/杆/环 接口接触 不悬空",
    ]
    txt.text(0.0, 0.98, "\n".join(lines), va="top", ha="left", fontsize=11, fontproperties=CJK)
    plt.tight_layout()
    out = os.path.join(OUT, "_three_way_final.png")
    plt.savefig(out, dpi=95, bbox_inches="tight")
    print("saved three_way_final.png")

    json.dump({"tripo": {k: rt[k] for k in ("iou", "az", "el", "mir", "roll")},
               "selfbuilt": {k: rh[k] for k in ("iou", "az", "el", "mir", "roll")},
               "overlay_iou": {"tripo": float(iou_t), "selfbuilt": float(iou_h)}},
              open(os.path.join(OUT, "three_way_final.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
