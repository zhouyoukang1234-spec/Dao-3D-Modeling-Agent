# -*- coding: utf-8 -*-
"""平台位姿网格搜索: 对每个 (tx,ty,roll,pitch,twist) 装配→IK→道.感 搜相机位姿→IoU.
照片是展开/倾斜位姿; 找出最贴合的平台位姿, 三向收敛."""
from __future__ import annotations
import os, sys, json, itertools
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import dao_jiao as DJ          # noqa: E402
import build_truth as BT       # noqa: E402

PHOTO = r"C:\Users\Administrator\attachments\1e3e689a-718b-47ac-a271-445caac3a39d\SmartSelect_20260626_115856_Baidu.jpg"


def run():
    pm, rgb = DJ.load_photo(PHOTO)
    # 网格: 接收环倾斜(roll绕Y) + 横向偏移(ty) + twist(平台相对底座转角→交叉杆)
    grid = []
    for roll in (0.0, 12.0, 23.0):
        for ty in (0.0, -18.0):
            for twist in (0.0, 18.0):
                grid.append(dict(roll=roll, ty=ty, twist=twist))
    results = []
    for i, kw in enumerate(grid):
        V, F, VC, rep = BT.assemble(label=f"g{i}", export=False, **kw)
        unreach = rep["unreachable"]
        pf = DJ.PoseFitter(V, F, VC)
        iou, az, el, mir, roll_c = pf.search(pm)
        tag = f"roll{kw['roll']:.0f} ty{kw['ty']:.0f} tw{kw['twist']:.0f}"
        print(f"[{i}] {tag:22s} IoU={iou:.3f} cam(az{az} el{el} m{mir} r{roll_c}) "
              f"maxerr={rep['rod_max_err_mm']} unreach={unreach}")
        results.append(dict(i=i, iou=float(iou), cam=[az, el, mir, roll_c],
                            unreach=unreach, **kw))
    results.sort(key=lambda r: -r["iou"])
    best = results[0]
    print("\nBEST:", json.dumps(best, ensure_ascii=False))
    with open(os.path.join(_HERE, "..", "output", "pose_search.json"), "w",
              encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    return best


if __name__ == "__main__":
    run()
