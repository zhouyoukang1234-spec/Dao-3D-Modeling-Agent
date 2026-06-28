"""Assembly mass-properties smoke — center of mass vs closed form.

An assembly engineer needs the assembly's center of mass (balance / CG), not
just its volume. ``asm.measure`` now returns the volume-weighted centroid and,
given densities, the mass-weighted center of mass + total mass. Validated on two
equal cubes at a known spacing:

  * uniform density -> CoM at the geometric midpoint;
  * 1:3 density ratio -> CoM shifts toward the heavy cube by the exact lever-arm
    rule  x = (rho_a*x_a + rho_b*x_b) / (rho_a + rho_b).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

A = 20.0            # cube edge
XB = 60.0           # cube B center offset along +X (A is centered at origin)


def main():
    s = new_session("asm_massprops")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # two equal cubes: A centered at origin, B centered at (XB,0,0)
    s.act("solid.box", {"name": "cubeA", "length": A, "width": A, "height": A,
                        "pos": [-A / 2, -A / 2, -A / 2]})
    s.act("solid.box", {"name": "cubeB", "length": A, "width": A, "height": A,
                        "pos": [XB - A / 2, -A / 2, -A / 2]})
    s.act("asm.create", {"name": "Asm"})
    assert s.act("asm.add", {"name": "A", "body": "cubeA"}).ok
    assert s.act("asm.add", {"name": "B", "body": "cubeB"}).ok

    # 1) uniform material: CoM at the geometric midpoint
    m = s.act("asm.measure", {"density": 0.00785}).data
    print("uniform: centroid=%s  com=%s  mass=%.2f g" % (m["centroid"], m["center_of_mass"], m["mass"]))
    assert abs(m["centroid"][0] - XB / 2.0) < 1e-3, m["centroid"]
    assert abs(m["center_of_mass"][0] - XB / 2.0) < 1e-3, m["center_of_mass"]
    vA = A ** 3
    assert abs(m["mass"] - 0.00785 * 2 * vA) < 1e-3, m["mass"]

    # 2) multi-material 1:3 -> CoM toward the heavy cube B
    rho_a, rho_b = 1.0, 3.0
    mm = s.act("asm.measure", {"densities": {"A": rho_a, "B": rho_b}}).data
    expect_x = (rho_a * 0.0 + rho_b * XB) / (rho_a + rho_b)
    print("1:3   : com=%s  (closed-form x=%.3f)" % (mm["center_of_mass"], expect_x))
    assert abs(mm["center_of_mass"][0] - expect_x) < 1e-3, (mm["center_of_mass"], expect_x)
    assert abs(mm["mass"] - (rho_a + rho_b) * vA) < 1e-3, mm["mass"]

    print("ASM MASSPROPS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_asm_massprops"):
    main()
