# -*- coding: utf-8 -*-
"""道.感.校 位姿求解: 遍历固件 15 个标准 T-Code 位姿, 每个用真零件装配→点云→
相机位姿搜索→对照实物照片轮廓 IoU. 找出与照片最一致的真实舵机位姿."""
import os, sys, time, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dao_jiao as DJ
import fastfit as FF
import build_pose as BP
from ORS6_Stewart.poses import MOTION_POSES

PHOTO = r"C:\Users\Administrator\attachments\1e3e689a-718b-47ac-a271-445caac3a39d\SmartSelect_20260626_115856_Baidu.jpg"


def main():
    pm, rgb = DJ.load_photo(PHOTO)
    results = []
    for entry in MOTION_POSES:
        name, pose = entry[0], tuple(entry[1:])
        t = time.time()
        try:
            pts, info = BP.sample(pose, n=32000)
        except Exception as e:
            print(f"{name}: build FAIL {e!r}"); continue
        ff = FF.FastFitter(pts)
        iou, az, el, mir, roll = ff.search(pm, az_step=30, els=(-20, 0, 20, 40, 60),
                                           coarse_res=150, fine=True, log=None)
        rmax = max(abs(x - 175.0) for x in info["rod_lens"])
        results.append({"name": name, "pose": list(pose), "iou": round(float(iou), 4),
                        "az": az, "el": el, "mir": mir, "roll": roll,
                        "rod_dev": round(rmax, 4)})
        print(f"{name:12s} IoU={iou:.4f} @az{az}el{el}m{mir}r{roll} "
              f"rod_dev={rmax:.3f} ({time.time()-t:.0f}s)", flush=True)
    results.sort(key=lambda r: -r["iou"])
    with open(os.path.join(os.path.dirname(__file__), "..", "output", "pose_solve.json"),
              "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print("\nBEST:", results[0])


if __name__ == "__main__":
    main()
