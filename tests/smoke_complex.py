"""Complex multi-body assembly smoke — a gearbox built and verified end to end.

Exercises the whole stack on one non-trivial model: a housing with a bored
bearing seat, a through shaft seated by a real cylindrical-axis coaxial mate,
and a meshing involute spur-gear pair, then couples the assembly to FEM.

It also pins down a real bug found by practice (道法自然): the coaxial mate used
the raw (arbitrarily-signed) cylinder-face axis, which could inject a spurious
180-degree flip and seat the shaft on the wrong side of the bore. The
``shaft seats through the bore`` assertion guards against that regression.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402


def main():
    s = new_session("gearbox")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- housing: a block with a central Ø24 through-bore (bearing seat) ------
    assert s.act("param.body", {"name": "Housing"}).ok
    assert s.act("param.pad", {"body": "Housing", "feature": "Block",
                               "profile": {"rect": [80, 60]}, "length": 40}).ok
    assert s.act("param.pocket", {"body": "Housing", "feature": "Bore",
                                  "profile": {"circle": 12, "at": [0, 0]},
                                  "length": 40, "through": True}).ok
    # --- through shaft + a meshing involute gear pair (m=2, z=20 & z=30) ------
    assert s.act("solid.cylinder", {"name": "Shaft", "radius": 10, "height": 120}).ok
    m, zA, zB = 2.0, 20, 30
    for nm, z in (("GearA", zA), ("GearB", zB)):
        assert s.act("param.body", {"name": nm}).ok
        assert s.act("param.pad", {"body": nm, "feature": nm + "f",
                                   "profile": {"gear": {"module": m, "teeth": z}},
                                   "length": 12}).ok

    # --- assemble -----------------------------------------------------------
    assert s.act("asm.create", {"name": "Box"}).ok
    assert s.act("asm.add", {"assembly": "Box", "body": "Housing", "name": "housing", "fixed": True}).ok
    for nm in ("Shaft", "GearA", "GearB"):
        assert s.act("asm.add", {"assembly": "Box", "body": nm, "name": nm.lower()}).ok

    # real cylindrical-axis mate: seat the shaft into the housing bore
    cx = s.act("asm.coaxial", {"hole": "housing", "pin": "shaft"})
    assert cx.ok, cx.error

    # REGRESSION GUARD: the shaft must remain coaxial through the bore, not be
    # flipped to the far side. A gear placed concentric on the shaft at z=60
    # must therefore overlap it (a press fit) -> non-zero clash.
    assert s.act("asm.place", {"name": "geara", "pos": [0, 0, 60]}).ok
    clashes = {(c["a"], c["b"]): c["overlap_volume"]
               for c in s.act("asm.interference", {"assembly": "Box"}).data["clashes"]}
    shaft_gear = clashes.get(("shaft", "geara")) or clashes.get(("geara", "shaft"))
    assert shaft_gear and shaft_gear > 100, ("shaft flipped away from bore?", clashes)
    print("coaxial seats shaft through bore: shaft<->gearA overlap = %.1f mm^3" % shaft_gear)

    # place the gear pair at the meshing centre distance a = m(zA+zB)/2 = 50 mm,
    # and PHASE GearB (due +X of GearA -> beta=0) so its teeth interleave GearA's
    # rather than jamming tip-to-tip.
    a_center = m * (zA + zB) / 2.0
    assert s.act("asm.place", {"name": "gearb", "pos": [a_center, 0, 60]}).ok
    assert s.act("asm.rotate", {"name": "gearb", "axis": [0, 0, 1],
                                "angle": meshing_phase_deg(0.0, zA, zB),
                                "at": [a_center, 0, 60]}).ok

    def _mesh():
        cl = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"]
              for c in s.act("asm.interference", {"assembly": "Box"}).data["clashes"]}
        return cl, cl.get(("geara", "gearb"), 0.0)

    clashes, mesh = _mesh()
    # phased -> teeth interleave: ~0 interference (a true mesh, not a jam)
    assert mesh < 8.0, ("gears jam (not phased)", clashes)
    # the gears sit above the housing -> no gear/housing collision
    assert ("gearb", "housing") not in clashes and ("geara", "housing") not in clashes, clashes
    print("gear pair at centre distance %.0f mm: phased mesh overlap = %.1f mm^3"
          % (a_center, mesh))

    # engagement proof: mis-phase GearB half a tooth -> the pair jams
    assert s.act("asm.rotate", {"name": "gearb", "axis": [0, 0, 1],
                                "angle": 180.0 / zB, "at": [a_center, 0, 60]}).ok
    _, jam = _mesh()
    assert jam > 50.0, ("mis-phase should jam the pair", jam)
    print("mis-phased GearB jams = %.0f mm^3 (engagement is real)" % jam)
    assert s.act("asm.rotate", {"name": "gearb", "axis": [0, 0, 1],
                                "angle": -180.0 / zB, "at": [a_center, 0, 60]}).ok

    bom = s.act("asm.bom", {"assembly": "Box", "density": 0.00785})
    assert bom.data["component_count"] == 4, bom.data
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))

    # --- couple the assembly to FEM: load the housing through the bearing -----
    if "fem.solve" in s.tools():
        assert s.act("fem.setup", {"target": "Housing", "material": "aluminum"}).ok
        assert s.act("fem.fix", {"select": {"axis": "z", "side": "min"}}).ok
        assert s.act("fem.load", {"select": {"axis": "y", "side": "max"},
                                  "kind": "force", "value": 2000, "direction": [0, -1, 0]}).ok
        fr = s.act("fem.solve", {"allowable_mpa": 200})
        assert fr.ok and fr.data["passed"], fr.data
        print("housing FEM: max vM = %.2f MPa  SF = %.1f  passed = %s"
              % (fr.data["max_von_mises_mpa"], fr.data["safety_factor"], fr.data["passed"]))

    # --- render the assembly as evidence ------------------------------------
    if "view.render" in s.tools():
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out", "smoke_gearbox.png")
        # render the assembly so parts show at their assembled placements
        rr = s.act("view.render", {"assembly": "Box", "view": "iso", "path": out})
        assert rr.ok and rr.data["bytes"] > 5000, rr.data
        print("render -> %s (%d bytes)" % (out, rr.data["bytes"]))

    print("COMPLEX SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_complex"):
    main()
