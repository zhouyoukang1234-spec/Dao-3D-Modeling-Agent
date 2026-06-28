import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("asm")
    k = s.registry.kernel
    print("asm ops:", [o for o in k.ops if o.startswith("asm.")])

    # two parts: a base plate and a block
    assert s.act("param.body", {"name": "Base"}).ok
    assert s.act("param.pad", {"body": "Base", "feature": "BasePad",
                              "profile": {"rect": [50, 50]}, "length": 6}).ok
    assert s.act("param.body", {"name": "Block"}).ok
    assert s.act("param.pad", {"body": "Block", "feature": "BlockPad",
                              "profile": {"rect": [20, 20]}, "length": 20}).ok

    # assemble
    assert s.act("asm.create", {"name": "Asm"}).ok
    assert s.act("asm.add", {"name": "base", "body": "Base", "fixed": True}).ok
    assert s.act("asm.add", {"name": "block", "body": "Block"}).ok
    # initially overlapping (both centered at origin region) -> expect clash
    r = s.act("asm.interference", {})
    print("pre-stack clashes:", r.data["clash_count"])

    # stack block on top of base -> no clash, block sits at z = base top
    r = s.act("asm.stack", {"base": "base", "top": "block", "gap": 0})
    print("after stack block pos:", r.data["placement"])
    r = s.act("asm.interference", {})
    print("post-stack clashes:", r.data["clash_count"], r.data["clashes"])
    assert r.data["clash_count"] == 0, r.data["clashes"]

    # bom (steel density g/mm^3)
    r = s.act("asm.bom", {"density": 0.00785})
    print("BOM:", r.data["line_items"], "total_mass(g):", r.data["total_mass"])
    assert r.data["component_count"] == 2

    # overall measure
    r = s.act("asm.measure", {})
    print("assembly bbox:", r.data["bbox_size"])
    assert abs(r.data["bbox_size"][2] - 26) < 1e-3  # 6 + 20

    # tree + solve
    print("tree:", s.act("asm.tree", {}).data)
    r = s.act("asm.solve", {})
    print("solve:", r.data)
    assert r.data["solved"]

    # export whole assembly
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)
    r = s.act("asm.export", {"path": os.path.join(out, "assembly.step")})
    print("export bytes:", r.data["bytes"])
    assert r.data["bytes"] > 0

    print("ASM SMOKE OK", s.summary())
    k.shutdown()

    # fastener: a self-made threaded stud + hex nut, assembled coaxially.
    # fresh session so the component count is scoped to just this assembly.
    f = new_session("fastener")
    fk = f.registry.kernel
    assert f.act("param.body", {"name": "Stud"}).ok
    assert f.act("param.pad", {"body": "Stud", "feature": "Rod",
                               "profile": {"circle": 5}, "length": 30}).ok
    assert f.act("param.sweep", {"body": "Stud", "feature": "Thread", "cut": True,
                                 "profile": {"circle": 1.0},
                                 "path": {"helix": {"pitch": 3, "height": 32,
                                                    "radius": 5, "z": -1}}}).ok
    hexpts = [[round(9.5 * math.cos(math.radians(60 * i)), 4),
               round(9.5 * math.sin(math.radians(60 * i)), 4)] for i in range(6)]
    assert f.act("param.body", {"name": "Nut"}).ok
    assert f.act("param.pad", {"body": "Nut", "feature": "Hex",
                               "profile": {"polygon": hexpts}, "length": 8}).ok
    assert f.act("param.pocket", {"body": "Nut", "feature": "Bore",
                                  "profile": {"circle": 5.6}, "through": True}).ok
    assert f.act("param.sweep", {"body": "Nut", "feature": "IntThread", "cut": True,
                                 "profile": {"circle": 0.9},
                                 "path": {"helix": {"pitch": 3, "height": 10,
                                                    "radius": 5.6, "z": -1}}}).ok
    assert f.act("asm.create", {"name": "Fastener"}).ok
    assert f.act("asm.add", {"name": "stud", "body": "Stud", "fixed": True}).ok
    assert f.act("asm.add", {"name": "nut", "body": "Nut"}).ok
    assert f.act("asm.place", {"name": "nut", "pos": [0, 0, 22]}).ok
    fr = f.act("asm.interference", {})
    assert fr.data["clash_count"] == 0, fr.data["clashes"]  # clearance bore -> slip fit
    fb = f.act("asm.bom", {"density": 0.00785})
    assert fb.data["component_count"] == 2
    print("fastener BOM mass(g):", round(fb.data["total_mass"], 1), "clashes:", fr.data["clash_count"])
    fk.shutdown()

    # semantic mate: align a pin to an OFF-CENTER bore's true axis (not the bbox).
    g = new_session("coaxial")
    gk = g.registry.kernel
    assert g.act("param.body", {"name": "Plate"}).ok
    assert g.act("param.pad", {"body": "Plate", "feature": "Slab",
                               "profile": {"rect": [50, 50]}, "length": 10}).ok
    assert g.act("param.pocket", {"body": "Plate", "feature": "Bore",
                                  "profile": {"circle": 5, "at": [12, 0]}, "through": True}).ok
    assert g.act("param.body", {"name": "Pin"}).ok
    assert g.act("param.pad", {"body": "Pin", "feature": "Shaft",
                               "profile": {"circle": 4.8}, "length": 20}).ok
    assert g.act("asm.create", {"name": "PinAsm"}).ok
    assert g.act("asm.add", {"name": "plate", "body": "Plate", "fixed": True}).ok
    assert g.act("asm.add", {"name": "pin", "body": "Pin"}).ok
    cx = g.act("asm.coaxial", {"hole": "plate", "pin": "pin", "axis": "z", "seat": "bottom"})
    assert cx.ok, cx.error
    # the pin must land on the hole axis [12,0], NOT the plate bbox center [0,0]
    assert abs(cx.data["placement"][0] - 12) < 1e-6 and abs(cx.data["placement"][1]) < 1e-6
    assert g.act("asm.interference", {}).data["clash_count"] == 0  # clearance fit
    print("coaxial pin placement:", cx.data["placement"])
    gk.shutdown()

    # rotational coaxial: seat a VERTICAL pin into a HORIZONTAL (Y-axis) bore;
    # the mate must rotate the pin's axis onto the bore axis, not just translate.
    h = new_session("rotmate")
    hk = h.registry.kernel
    assert h.act("param.body", {"name": "Brk"}).ok
    assert h.act("param.pad", {"body": "Brk", "feature": "Slab",
                               "profile": {"rect": [40, 30]}, "length": 20}).ok
    assert h.act("param.pocket", {"body": "Brk", "feature": "Bore", "plane": "XZ",
                                  "profile": {"circle": 5, "at": [0, 10]}, "through": True}).ok
    assert h.act("param.body", {"name": "Pin"}).ok
    assert h.act("param.pad", {"body": "Pin", "feature": "Shaft",
                               "profile": {"circle": 4.8}, "length": 40}).ok
    assert h.act("asm.create", {"name": "RotAsm"}).ok
    assert h.act("asm.add", {"name": "brk", "body": "Brk", "fixed": True}).ok
    assert h.act("asm.add", {"name": "pin", "body": "Pin"}).ok
    rc = h.act("asm.coaxial", {"hole": "brk", "pin": "pin"})
    assert rc.ok, rc.error
    assert abs(abs(rc.data["axis"][1]) - 1) < 1e-6, rc.data["axis"]  # pin axis now +/-Y
    assert h.act("asm.interference", {}).data["clash_count"] == 0  # clearance fit
    print("rotational coaxial axis:", rc.data["axis"])
    hk.shutdown()

    # hinge: two knuckle tubes made coaxial bore-to-bore but slid apart along the
    # shared axis (offset=), then a pin threaded through both -> no clashes.
    j = new_session("hinge")
    jk = j.registry.kernel
    for nm in ("K1", "K2"):
        assert j.act("param.body", {"name": nm}).ok
        assert j.act("param.pad", {"body": nm, "feature": "Barrel",
                                   "profile": {"circle": 6}, "length": 10}).ok
        assert j.act("param.pocket", {"body": nm, "feature": "Bore",
                                      "profile": {"circle": 3}, "through": True}).ok
    assert j.act("param.body", {"name": "Pin"}).ok
    assert j.act("param.pad", {"body": "Pin", "feature": "Shaft",
                               "profile": {"circle": 2.8}, "length": 30}).ok
    assert j.act("asm.create", {"name": "Hinge"}).ok
    assert j.act("asm.add", {"name": "k1", "body": "K1", "fixed": True}).ok
    assert j.act("asm.add", {"name": "k2", "body": "K2"}).ok
    assert j.act("asm.add", {"name": "pin", "body": "Pin"}).ok
    jc = j.act("asm.coaxial", {"hole": "k1", "pin": "k2",
                               "hole_pick": "min", "pin_pick": "min", "offset": 10})
    assert jc.ok, jc.error
    assert abs(jc.data["placement"][2] - 10) < 1e-6  # k2 slid +10 up the axis
    assert j.act("asm.coaxial", {"hole": "k1", "pin": "pin",
                                 "hole_pick": "min", "pin_pick": "max", "seat": "bottom"}).ok
    ji = j.act("asm.interference", {})
    assert ji.data["clash_count"] == 0, ji.data["clashes"]  # clearance bores + slid apart
    print("hinge clashes:", ji.data["clash_count"], "bbox:", j.act("asm.measure", {}).data["bbox_size"])
    jk.shutdown()

    # gear pair: two involute gears meshed at the standard center distance
    # a = m*(z1+z2)/2; the driven gear is phased by half a tooth (asm.rotate
    # about its own centre) so its teeth seat in the driver's spaces -> 0 overlap.
    g1, g2, mod = 18, 12, 2.0
    dist = mod * (g1 + g2) / 2.0
    p = new_session("gearpair")
    pk = p.registry.kernel

    def _gear(name, z):
        assert p.act("param.body", {"name": name}).ok
        assert p.act("param.pad", {"body": name, "feature": "Blank",
                                   "profile": {"gear": {"module": mod, "teeth": z}}, "length": 8}).ok
        assert p.act("param.pocket", {"body": name, "feature": "Bore",
                                      "profile": {"circle": 3}, "through": True}).ok
    _gear("G1", g1)
    _gear("G2", g2)
    assert p.act("asm.create", {"name": "Pair"}).ok
    assert p.act("asm.add", {"name": "g1", "body": "G1", "fixed": True}).ok
    assert p.act("asm.add", {"name": "g2", "body": "G2"}).ok

    def _overlap(phase):
        p.act("asm.place", {"name": "g2", "pos": [dist, 0, 0]})  # reset pose
        p.act("asm.rotate", {"name": "g2", "axis": [0, 0, 1], "at": [dist, 0, 0], "angle": phase})
        cl = p.act("asm.interference", {}).data
        return sum(c["overlap_volume"] for c in cl["clashes"])

    meshed = _overlap(360.0 / g2 / 2.0)   # half-tooth phase -> teeth seat in gaps
    clashing = _overlap(0.0)              # unphased -> tooth meets tooth
    assert meshed < 1e-6, ("phased gears should not overlap", meshed)
    assert clashing > meshed, ("unphased gears should clash", clashing)
    print("gear pair center dist:", dist, "meshed overlap:", round(meshed, 4),
          "unphased overlap:", round(clashing, 1))
    pk.shutdown()


if __name__ == "__main__":
    main()
