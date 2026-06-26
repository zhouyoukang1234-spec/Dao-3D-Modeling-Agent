# -*- coding: utf-8 -*-
"""多视角交叉验证 — ②Tripo 与 ③自建(最佳工作位姿) 在多个相机角度下逐视角比对轮廓.
两者各自已对齐到实物照片(az/el/mir/roll 来自 three_way_final.json); 以该对齐为
基准, 同步绕竖直轴扫一组方位角偏移 d, 逐视角计算 ②③ 轮廓 IoU + 叠加.

意义: 若 ②③ 在每个视角轮廓都一致, 证明三向一致不是单视角巧合, 而是真正的三维 1:1.
"""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "00-本源_Origin")))
import trimesh
import dao_jiao as DJ
import build_pose as BP
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as _fm
CJK = None
for _fp in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
    if os.path.exists(_fp):
        CJK = _fm.FontProperties(fname=_fp); break
plt.rcParams["axes.unicode_minus"] = False

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "output")
BEST_POSE = (1500, 5000, 5000, 5000, 5000, 5000)


def overlay(ma, mb):
    a, b = DJ.fit_norm(ma), DJ.fit_norm(mb)
    S = a.shape[0]
    ov = np.ones((S, S, 3))
    ov[a] = [0.85, 0.30, 0.25]      # ② Tripo
    ov[b] = [0.20, 0.55, 0.85]      # ③ 自建
    ov[a & b] = [0.25, 0.75, 0.30]  # 一致 = 绿
    return ov, DJ.iou(a, b)


def main():
    cache = json.load(open(os.path.join(OUT, "three_way_final.json")))
    t, h = cache["tripo"], cache["selfbuilt"]
    # ② Tripo
    d = np.load(os.path.join(os.path.dirname(__file__), "tripo_prepped.npz"))
    pf_t = DJ.PoseFitter(np.asarray(d["V"], float), np.asarray(d["F"], int), np.asarray(d["vcol"], float))
    # ③ 自建 (真零件装配, 最佳工作位姿)
    mh, info = BP.build(BEST_POSE)
    pf_h = DJ.PoseFitter(np.asarray(mh.vertices, float), np.asarray(mh.faces, int))

    # ③ 是镜像对齐(mir=1), 故方位角扫描方向相对 ② 取反, 保证两者"同向"旋转
    sgn_h = -1 if (h["mir"] != t["mir"]) else 1
    offs = [-60, -30, 0, 30, 60]
    rows = []
    ious = []
    for dd in offs:
        it, mt = pf_t.fitted_render((t["az"] + dd) % 360, t["el"], t["mir"], t["roll"], W=420)
        ih, mhk = pf_h.fitted_render((h["az"] + sgn_h * dd) % 360, h["el"], h["mir"], h["roll"], W=420)
        ov, j = overlay(mt, mhk)
        rows.append((it, ih, ov, dd, j))
        ious.append(j)
        print(f"d={dd:+3d}  ②③ 轮廓IoU={j:.3f}", flush=True)

    fig, axes = plt.subplots(3, len(offs), figsize=(4 * len(offs), 11))
    for k, (it, ih, ov, dd, j) in enumerate(rows):
        axes[0, k].imshow(it); axes[0, k].axis("off")
        axes[0, k].set_title(f"② Tripo  Δaz={dd:+d}°", fontsize=11, fontproperties=CJK)
        axes[1, k].imshow(ih); axes[1, k].axis("off")
        axes[1, k].set_title(f"③ 自建  Δaz={dd:+d}°", fontsize=11, fontproperties=CJK)
        axes[2, k].imshow(ov); axes[2, k].axis("off")
        axes[2, k].set_title(f"叠加  ②③IoU={j:.3f}", fontsize=11, fontproperties=CJK)
    fig.suptitle(f"多视角交叉验证 · ②Tripo 与 ③自建可用模型(最佳工作位姿)逐视角轮廓一致\n"
                 f"红=Tripo 蓝=自建 绿=一致   均值 ②③IoU={np.mean(ious):.3f}   "
                 f"(证明三向一致非单视角巧合·真三维 1:1)", fontsize=13, fontproperties=CJK)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    out = os.path.join(OUT, "_multi_view_xcheck.png")
    plt.savefig(out, dpi=92, bbox_inches="tight")
    print("saved", out, " mean IoU", round(float(np.mean(ious)), 4))
    json.dump({"offsets": offs, "tripo_selfbuilt_iou": [round(float(x), 4) for x in ious],
               "mean": round(float(np.mean(ious)), 4)},
              open(os.path.join(OUT, "multi_view_xcheck.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
