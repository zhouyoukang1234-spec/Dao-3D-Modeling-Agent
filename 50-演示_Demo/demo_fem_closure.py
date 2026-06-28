"""闭环演示 · 一条 live FreeCAD 内核, 四个物理失效模式, 每个对闭式解校验,
并渲染应力云图 · 万法归一 50 门.

Run:  python 50-演示_Demo/demo_fem_closure.py
(经 FREECADCMD / 标准安装路径拉起 freecadcmd; CalculiX 随 FreeCAD 自带)
"""
import math
import os
import sys
from pathlib import Path

_DAO_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "_paths.py").is_file())
sys.path.insert(0, str(_DAO_ROOT))
import _paths  # noqa: E402,F401  五层 sys.path 注入

from cad_agent import new_session  # noqa: E402

OUT = str(_paths.ROOT / "output" / "fem_demo")
os.makedirs(OUT, exist_ok=True)
E, NU = 210000.0, 0.30


def banner(t):
    print("\n" + "=" * 64 + "\n  " + t + "\n" + "=" * 64)


def spin_disk():
    banner("1/4  ROTATIONAL  fem.spin  (spinning disk, *DLOAD CENTRIF)")
    s = new_session("demo_spin")
    R, H, rpm = 120.0, 10.0, 6000.0
    s.act("solid.cylinder", {"name": "full", "radius": R, "height": H})
    s.act("solid.box", {"name": "corner", "length": R + 1, "width": R + 1,
                        "height": H + 2, "pos": [0, 0, -1]})
    s.act("solid.common", {"a": "full", "b": "corner", "out": "disk"})
    s.act("fem.setup", {"target": "disk", "material": "steel",
                        "order": 2, "mesh_size": R / 16.0})
    s.act("fem.support", {"select": {"normal": [-1, 0, 0]}, "fix": ["x"]})
    s.act("fem.support", {"select": {"normal": [0, -1, 0]}, "fix": ["y"]})
    s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]})
    s.act("view.render", {"names": ["disk"], "view": "iso",
                          "path": os.path.join(OUT, "disk_model.png")})
    s.act("fem.spin", {"rpm": rpm, "axis": [0, 0, 1]})
    sv = s.act("fem.solve", {})
    fem = sv.data["max_von_mises_mpa"]
    omega = 2 * math.pi * rpm / 60.0
    closed = (3 + NU) / 8.0 * (7900.0 * 1e-12) * omega ** 2 * R ** 2
    png = os.path.join(OUT, "spin_contour.png")
    s.act("fem.contour", {"path": png, "view": "top"})
    print("  rpm=%.0f R=%.0f  FEM centre vM=%.3f MPa  closed (3+v)/8 rho w^2 R^2=%.3f  err=%.2f%%"
          % (rpm, R, fem, closed, abs(fem / closed - 1) * 100))
    print("  contour -> %s" % png)
    s.registry.kernel.shutdown()
    return png


def buckle_column():
    banner("2/4  BUCKLING  fem.buckle  (Euler column, *BUCKLE)")
    s = new_session("demo_buck")
    L, b = 200.0, 10.0
    s.act("solid.box", {"name": "col", "length": b, "width": b, "height": L})
    s.act("fem.setup", {"target": "col", "material": "steel", "order": 2, "mesh_size": b / 2})
    s.act("fem.fix", {"select": {"axis": "z", "side": "min"}})
    s.act("fem.load", {"select": {"axis": "z", "side": "max"}, "kind": "force",
                       "value": 1000.0, "direction": [0, 0, -1]})
    bk = s.act("fem.buckle", {"modes": 1})
    fac = bk.data["critical_factor"]
    euler = (math.pi ** 2 * E * (b ** 4 / 12.0) / (2 * L) ** 2) / 1000.0
    print("  L=%.0f b=%.0f  FEM factor=%.3f  Euler pi^2 EI/(KL)^2=%.3f  err=%.2f%%  Pcr=%.0f N"
          % (L, b, fac, euler, abs(fac / euler - 1) * 100, fac * 1000))
    s.registry.kernel.shutdown()


def thermal_bar():
    banner("3/4  THERMAL  fem.thermal  (constrained bar, *COUPLED TEMP-DISP)")
    s = new_session("demo_therm")
    L = 100.0
    s.act("solid.box", {"name": "bar", "length": L, "width": 10, "height": 10})
    s.act("fem.setup", {"target": "bar", "material": "steel", "order": 2, "mesh_size": 5})
    # statically-determinate rollers (one component each) -> uniaxial stress;
    # blocking the far x face as well jams the axial thermal growth.
    s.act("fem.support", {"select": {"axis": "x", "side": "min"}, "fix": ["x"]})
    s.act("fem.support", {"select": {"axis": "y", "side": "min"}, "fix": ["y"]})
    s.act("fem.support", {"select": {"axis": "z", "side": "min"}, "fix": ["z"]})
    s.act("fem.support", {"select": {"axis": "x", "side": "max"}, "fix": ["x"]})
    dT = 80.0
    s.act("fem.temperature", {"value": dT, "ref": 0.0})
    tr = s.act("fem.thermal", {})
    s.act("fem.contour", {"path": os.path.join(OUT, "thermal_contour.png"), "view": "iso"})
    fem = tr.data["max_von_mises_mpa"]
    closed = E * 1.2e-5 * dT
    print("  dT=%.0fK (axially blocked)  FEM vM=%.3f MPa  closed E*alpha*dT=%.3f  err=%.2f%%"
          % (dT, fem, closed, abs(fem / closed - 1) * 100))
    s.registry.kernel.shutdown()


def static_beam():
    banner("4/4  YIELD/STATIC  fem.solve  (cantilever bending)")
    s = new_session("demo_static")
    L, b, h, F = 100.0, 10.0, 10.0, 1000.0
    s.act("solid.box", {"name": "beam", "length": L, "width": b, "height": h})
    s.act("fem.setup", {"target": "beam", "material": "steel", "order": 2, "mesh_size": 4})
    s.act("fem.fix", {"select": {"axis": "x", "side": "min"}})
    s.act("fem.load", {"select": {"axis": "x", "side": "max"}, "kind": "force",
                       "value": F, "direction": [0, 0, -1]})
    sv = s.act("fem.solve", {})
    s.act("fem.contour", {"path": os.path.join(OUT, "static_contour.png"), "view": "iso"})
    fem = sv.data["max_von_mises_mpa"]
    closed = 6 * F * L / (b * h ** 2)
    print("  F=%.0fN L=%.0f  FEM vM=%.1f MPa  closed 6FL/bh^2=%.1f  err=%.2f%%  SF=%.2f"
          % (F, L, fem, closed, abs(fem / closed - 1) * 100, sv.data["safety_factor"]))
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "demo_closure"):
    print("DAO FreeCAD agent - closed-loop multi-physics demo (live CalculiX kernel)")
    png = spin_disk()
    buckle_column()
    thermal_bar()
    static_beam()
    banner("CLOSURE: 4 physics modes, each validated to closed form")
    print("stress contour image: %s" % png)
