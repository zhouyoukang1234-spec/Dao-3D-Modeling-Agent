"""Large engineering assembly smoke — a two-stage parallel-shaft gear reducer.

This is the "big" closed-loop engineering build: a real speed reducer with a
bored housing, three bearing-seated shafts and a four-gear train across two
reduction stages, verified for gear meshing, press fits, absence of spurious
collisions, mass (BOM) and an FEM check on the output shaft.

Layout (three parallel shafts along Z, axes spaced along X):

    input  x=0      pinion P1 (z=18) @ plane A
    inter  x=a1     gear   G1 (z=36) @ plane A   (meshes P1)
                    pinion P2 (z=18) @ plane B
    output x=a1+a2  gear   G2 (z=54) @ plane B   (meshes P2)

Stage centre distance a = m*(z_p + z_g)/2.  Overall ratio = (36/18)*(54/18) = 6.

It also pins down a real defect found by practice (道法自然): a z>=36 involute
gear sketch carried one Coincident constraint per flank sample (~800), which
drove the Sketcher solver super-linear and timed the pad out. The fix builds the
dense generated profile constraint-free (endpoints already coincide exactly), so
this whole 8-body train pads and assembles in seconds; the test guards that.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session            # noqa: E402
from tests._gearmath import meshing_phase_deg  # noqa: E402

M = 2.0
Z_P1, Z_G1, Z_P2, Z_G2 = 18, 36, 18, 54
A1 = M * (Z_P1 + Z_G1) / 2.0                 # 54 mm
A2 = M * (Z_P2 + Z_G2) / 2.0                 # 72 mm
X_IN, X_MID, X_OUT = 0.0, A1, A1 + A2        # 0, 54, 126 mm
PLANE_A, PLANE_B = 40.0, 75.0
GW = 12.0
SHAFT_R, BORE_R, SHAFT_H = 8.0, 9.0, 130.0


def _clash_map(s):
    out = s.act("asm.interference", {"assembly": "Reducer"})
    assert out.ok, out.error
    return {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"]
            for c in out.data["clashes"]}


def main():
    s = new_session("reducer")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # --- housing: long block with three vertical bearing bores ---------------
    assert s.act("param.body", {"name": "Housing"}).ok
    assert s.act("param.pad", {"body": "Housing", "feature": "Block",
                               "profile": {"rect": [X_OUT + 40, 80], "at": [X_OUT / 2, 0]},
                               "length": 20}).ok
    for i, x in enumerate((X_IN, X_MID, X_OUT)):
        assert s.act("param.pocket", {"body": "Housing", "feature": "Bore%d" % i,
                                      "profile": {"circle": BORE_R * 2, "at": [x, 0]},
                                      "length": 20, "through": True}).ok

    # --- three shafts --------------------------------------------------------
    for nm in ("ShaftIn", "ShaftMid", "ShaftOut"):
        assert s.act("solid.cylinder", {"name": nm, "radius": SHAFT_R, "height": SHAFT_H}).ok

    # --- four gears (padded involute profiles) -------------------------------
    # REGRESSION GUARD: the z=36 / z=54 gears must pad without the old
    # constraint-explosion timeout. If the fix regresses, these .ok assertions
    # raise a TimeoutError here.
    for nm, z in (("P1", Z_P1), ("G1", Z_G1), ("P2", Z_P2), ("G2", Z_G2)):
        assert s.act("param.body", {"name": nm}).ok
        rp = s.act("param.pad", {"body": nm, "feature": nm + "f",
                                 "profile": {"gear": {"module": M, "teeth": z}},
                                 "length": GW})
        assert rp.ok, (nm, z, rp.error)

    # --- assemble ------------------------------------------------------------
    assert s.act("asm.create", {"name": "Reducer"}).ok
    assert s.act("asm.add", {"assembly": "Reducer", "body": "Housing",
                             "name": "housing", "fixed": True}).ok
    for nm in ("ShaftIn", "ShaftMid", "ShaftOut", "P1", "G1", "P2", "G2"):
        assert s.act("asm.add", {"assembly": "Reducer", "body": nm, "name": nm.lower()}).ok

    # seat every shaft into a bore via the real cylindrical-axis mate, then
    # spread each onto its own shaft axis (coaxial snaps them all to bore #0).
    for shaft in ("shaftin", "shaftmid", "shaftout"):
        cx = s.act("asm.coaxial", {"hole": "housing", "pin": shaft})
        assert cx.ok, (shaft, cx.error)
    assert s.act("asm.place", {"name": "shaftin", "pos": [X_IN, 0, 0]}).ok
    assert s.act("asm.place", {"name": "shaftmid", "pos": [X_MID, 0, 0]}).ok
    assert s.act("asm.place", {"name": "shaftout", "pos": [X_OUT, 0, 0]}).ok

    # gears concentric on their shafts at the two reduction planes; each driven
    # gear (G1, G2) is PHASED so its teeth interleave the pinion's instead of
    # jamming tip-to-tip (the driven gear sits due +X of its pinion -> beta=0).
    assert s.act("asm.place", {"name": "p1", "pos": [X_IN, 0, PLANE_A]}).ok
    assert s.act("asm.place", {"name": "g1", "pos": [X_MID, 0, PLANE_A]}).ok
    assert s.act("asm.rotate", {"name": "g1", "axis": [0, 0, 1],
                                "angle": meshing_phase_deg(0.0, Z_P1, Z_G1),
                                "at": [X_MID, 0, PLANE_A]}).ok
    assert s.act("asm.place", {"name": "p2", "pos": [X_MID, 0, PLANE_B]}).ok
    assert s.act("asm.place", {"name": "g2", "pos": [X_OUT, 0, PLANE_B]}).ok
    assert s.act("asm.rotate", {"name": "g2", "axis": [0, 0, 1],
                                "angle": meshing_phase_deg(0.0, Z_P2, Z_G2),
                                "at": [X_OUT, 0, PLANE_B]}).ok

    cm = _clash_map(s)

    # both stages truly mesh (interleave) -> ~0 interference, not a jam
    stage1 = cm.get(("g1", "p1"), 0.0)
    stage2 = cm.get(("g2", "p2"), 0.0)
    assert stage1 < 8.0, ("stage 1 jams (not phased)", cm)
    assert stage2 < 8.0, ("stage 2 jams (not phased)", cm)
    print("stage1 P1<->G1 phased mesh = %.1f ; stage2 P2<->G2 phased mesh = %.1f mm^3"
          % (stage1, stage2))

    # engagement proof: mis-phase G1 half a tooth -> stage 1 jams
    assert s.act("asm.rotate", {"name": "g1", "axis": [0, 0, 1],
                                "angle": 180.0 / Z_G1, "at": [X_MID, 0, PLANE_A]}).ok
    jam = _clash_map(s).get(("g1", "p1"), 0.0)
    assert jam > 50.0, ("mis-phase should jam stage 1", jam)
    print("mis-phased G1 jams stage 1 = %.0f mm^3 (engagement is real)" % jam)
    assert s.act("asm.rotate", {"name": "g1", "axis": [0, 0, 1],
                                "angle": -180.0 / Z_G1, "at": [X_MID, 0, PLANE_A]}).ok

    # each gear is press-fit on its own shaft (solid disc over the shaft)
    press = math.pi * SHAFT_R ** 2 * GW
    for gear, shaft in (("p1", "shaftin"), ("g1", "shaftmid"),
                        ("p2", "shaftmid"), ("g2", "shaftout")):
        ov = cm.get(tuple(sorted((gear, shaft))), 0.0)
        assert abs(ov - press) < 1.0, (gear, shaft, ov, press)

    # NO spurious cross-shaft / gear-housing collisions: every clashing pair is
    # an intended mesh or press fit, never e.g. input pinion vs output shaft.
    allowed = {("g1", "p1"), ("g2", "p2"), ("p1", "shaftin"), ("g1", "shaftmid"),
               ("p2", "shaftmid"), ("g2", "shaftout")}
    stray = set(cm) - allowed
    assert not stray, ("unexpected collisions", stray)

    bom = s.act("asm.bom", {"assembly": "Reducer", "density": 0.00785})
    assert bom.data["component_count"] == 8, bom.data
    print("BOM: %d components, total mass(steel) = %.1f g"
          % (bom.data["component_count"], bom.data["total_mass"]))

    ratio = (Z_G1 / Z_P1) * (Z_G2 / Z_P2)
    assert abs(ratio - 6.0) < 1e-9, ratio
    print("overall reduction ratio = %.2f:1" % ratio)

    # --- couple to FEM: axial thrust on the output shaft (its end-face normal
    # is the load's direction reference; a transverse load has no such planar
    # reference on a cylinder, so an axial thrust is the physical, supported
    # load here) --------------------------------------------------------------
    if "fem.solve" in s.tools():
        assert s.act("fem.setup", {"target": "ShaftOut", "material": "steel"}).ok
        assert s.act("fem.fix", {"select": {"axis": "z", "side": "min"}}).ok
        assert s.act("fem.load", {"select": {"axis": "z", "side": "max"},
                                  "kind": "force", "value": 1500, "direction": [0, 0, -1]}).ok
        fr = s.act("fem.solve", {"allowable_mpa": 250})
        assert fr.ok, fr.error
        print("output-shaft FEM: max vM = %.2f MPa  SF = %.1f  passed = %s"
              % (fr.data["max_von_mises_mpa"], fr.data["safety_factor"], fr.data["passed"]))

    # --- render the whole reducer as evidence --------------------------------
    if "view.render" in s.tools():
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "_out", "smoke_reducer.png")
        # render the assembly so every part is shown at its assembled placement
        rr = s.act("view.render", {"assembly": "Reducer", "view": "iso", "path": out})
        assert rr.ok and rr.data["bytes"] > 5000, rr.data
        print("render -> %s (%d bytes)" % (out, rr.data["bytes"]))

    print("ENGINEERING SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_engineering"):
    main()
