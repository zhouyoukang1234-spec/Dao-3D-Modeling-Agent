#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_verify_fem.py — FreeCAD live-kernel CalculiX 多物理验道 · 万法归一 30 门

道法自然 · 以闭式解为镜, 不以图像为凭.

一条 live FreeCAD 内核 (freecadcmd + CalculiX ccx), 四个独立失效模式,
每个都对解析闭式解校验 (validate to closed form, never eyeball):

  1 旋转  fem.spin    旋转圆盘中心应力   (3+nu)/8 * rho*omega^2*R^2
  2 屈曲  fem.buckle  Euler 临界载荷因子  pi^2 EI/(KL)^2  (K=2 悬臂)
  3 热    fem.thermal 受约束杆热应力      E*alpha*dT
  4 静力  fem.solve   悬臂弯曲根部应力    6FL/bH^2

运行:  python 30-验证_Verify/_verify_fem.py
需真实 FreeCAD (FREECADCMD 环境变量或标准安装路径); CalculiX 随 FreeCAD 自带.
"""
import math
import sys
from pathlib import Path

_DAO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "_paths.py").is_file())
sys.path.insert(0, str(_DAO_ROOT))
import _paths  # noqa: E402,F401  五层 sys.path 注入

from cad_agent import new_session  # noqa: E402

E, NU = 210000.0, 0.30
ALPHA = 1.2e-5
RHO_TMM3 = 7900.0 * 1e-12  # t/mm^3
TOL = 0.05  # 5% closed-form tolerance


def _chk(tag, fem, closed, tol=TOL):
    err = abs(fem / closed - 1.0)
    ok = err <= tol
    print("  [%s] %-9s FEM=%.4f  closed=%.4f  err=%.2f%%  (tol %.0f%%)"
          % ("PASS" if ok else "FAIL", tag, fem, closed, err * 100, tol * 100))
    return ok


def v_spin():
    s = new_session("v_spin")
    R, H, rpm = 120.0, 10.0, 6000.0
    s.act("solid.cylinder", {"name": "full", "radius": R, "height": H})
    s.act("solid.box", {"name": "corner", "length": R + 1, "width": R + 1,
                        "height": H + 2, "pos": [0, 0, -1]})
    s.act("solid.common", {"a": "full", "b": "corner", "out": "disk"})
    s.act("fem.setup", {"target": "disk", "material": "steel", "order": 2, "mesh_size": R / 16.0})
    s.act("fem.support", {"select": {"normal": [-1, 0, 0]}, "fix": ["x"]})
    s.act("fem.support", {"select": {"normal": [0, -1, 0]}, "fix": ["y"]})
    s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]})
    s.act("fem.spin", {"rpm": rpm, "axis": [0, 0, 1]})
    fem = s.act("fem.solve", {}).data["max_von_mises_mpa"]
    omega = 2 * math.pi * rpm / 60.0
    closed = (3 + NU) / 8.0 * RHO_TMM3 * omega ** 2 * R ** 2
    s.registry.kernel.shutdown()
    return _chk("spin", fem, closed)


def v_buckle():
    s = new_session("v_buckle")
    L, b = 200.0, 10.0
    s.act("solid.box", {"name": "col", "length": b, "width": b, "height": L})
    s.act("fem.setup", {"target": "col", "material": "steel", "order": 2, "mesh_size": b / 2})
    s.act("fem.fix", {"select": {"axis": "z", "side": "min"}})
    s.act("fem.load", {"select": {"axis": "z", "side": "max"}, "kind": "force",
                       "value": 1000.0, "direction": [0, 0, -1]})
    fac = s.act("fem.buckle", {"modes": 1}).data["critical_factor"]
    euler = (math.pi ** 2 * E * (b ** 4 / 12.0) / (2 * L) ** 2) / 1000.0
    s.registry.kernel.shutdown()
    return _chk("buckle", fac, euler)


def v_thermal():
    s = new_session("v_thermal")
    L = 100.0
    s.act("solid.box", {"name": "bar", "length": L, "width": 10, "height": 10})
    s.act("fem.setup", {"target": "bar", "material": "steel", "order": 2, "mesh_size": 5})
    s.act("fem.support", {"select": {"axis": "x", "side": "min"}, "fix": ["x"]})
    s.act("fem.support", {"select": {"axis": "y", "side": "min"}, "fix": ["y"]})
    s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]})
    s.act("fem.support", {"select": {"axis": "x", "side": "max"}, "fix": ["x"]})
    dT = 80.0
    s.act("fem.temperature", {"value": dT, "ref": 0.0})
    fem = s.act("fem.thermal", {}).data["max_von_mises_mpa"]
    closed = E * ALPHA * dT
    s.registry.kernel.shutdown()
    return _chk("thermal", fem, closed)


def v_static():
    s = new_session("v_static")
    L, b, h, F = 100.0, 10.0, 10.0, 1000.0
    s.act("solid.box", {"name": "beam", "length": L, "width": b, "height": h})
    s.act("fem.setup", {"target": "beam", "material": "steel", "order": 2, "mesh_size": 4})
    s.act("fem.fix", {"select": {"axis": "x", "side": "min"}})
    s.act("fem.load", {"select": {"axis": "x", "side": "max"}, "kind": "force",
                       "value": F, "direction": [0, 0, -1]})
    fem = s.act("fem.solve", {}).data["max_von_mises_mpa"]
    closed = 6 * F * L / (b * h ** 2)
    s.registry.kernel.shutdown()
    return _chk("static", fem, closed)


def main():
    print("FreeCAD live-kernel CalculiX 多物理验道 (闭式解校验)")
    checks = [("rotational", v_spin), ("buckling", v_buckle),
              ("thermal", v_thermal), ("static", v_static)]
    results = []
    for name, fn in checks:
        try:
            results.append(fn())
        except Exception as exc:  # noqa: BLE001
            print("  [FAIL] %-9s raised %r" % (name, exc))
            results.append(False)
    npass = sum(results)
    print("\n%d/%d PASS" % (npass, len(results)))
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
