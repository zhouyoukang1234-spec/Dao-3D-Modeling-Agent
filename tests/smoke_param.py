import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("param")
    k = s.registry.kernel
    print("ops with param.*:", [o for o in k.ops if o.startswith("param.")])

    # 1) parametric plate: pad a centered 60x40 rect to height 8
    assert s.act("param.body", {"name": "P"}).ok
    r = s.act("param.pad", {"body": "P", "feature": "Plate",
                            "profile": {"rect": [60, 40]}, "length": 8})
    assert r.ok, r.error
    print("pad volume", r.data["volume"], "dof", r.data["dof"])
    assert abs(r.data["volume"] - 60 * 40 * 8) < 1e-3
    assert r.data["dof"] == 0, "rect sketch should be fully constrained"

    # 2) pocket a centered circular hole r8 through
    r = s.act("param.pocket", {"body": "P", "feature": "Hole",
                              "profile": {"circle": 8}, "through": True})
    assert r.ok, r.error
    print("after pocket volume", r.data["volume"])
    assert abs(r.data["volume"] - (60 * 40 * 8 - math.pi * 64 * 8)) < 1.0

    # 3) diagnose — should be fully healthy (DoF 0, no conflicts)
    d = s.act("param.diagnose", {})
    print("diagnose all_healthy", d.data["all_healthy"], "total_dof", d.data["total_dof"])
    assert d.data["all_healthy"], d.data["sketches"]

    # 4) parametric re-edit: list params, change plate height 8 -> 12
    p = s.act("param.params", {})
    print("params", p.data["params"])
    assert "Plate.length" in p.data["params"]
    assert s.act("param.set", {"param": "Plate.length", "value": 12}).ok
    m = s.act("param.measure", {"body": "P"})
    print("after height 8->12 volume", m.data["volume"])
    # through-all hole tracks the new 12mm thickness
    assert abs(m.data["volume"] - (60 * 40 * 12 - math.pi * 64 * 12)) < 1.0

    # 5) parametric fillet dressup
    r = s.act("param.fillet", {"body": "P", "feature": "Round", "edges": None, "radius": 1.5})
    print("fillet feature ok", r.ok, "faces", r.data.get("faces"))

    # 6) revolve: a rect revolved 360 about its vertical edge -> a disc/tube
    assert s.act("param.body", {"name": "R"}).ok
    r = s.act("param.revolve", {"body": "R", "feature": "Rev",
                               "profile": {"polygon": [[10, 0], [20, 0], [20, 5], [10, 5]]},
                               "angle": 360})
    print("revolve ok", r.ok, r.error or ("V=%.1f" % r.data["volume"]))
    assert r.ok, r.error

    # 7) loft circle r12 -> square 30 -> circle r6 over 50mm
    assert s.act("param.body", {"name": "L"}).ok
    r = s.act("param.loft", {"body": "L", "feature": "Trans", "sections": [
        {"profile": {"circle": 12}, "offset": 0},
        {"profile": {"rect": [30, 30]}, "offset": 25},
        {"profile": {"circle": 6}, "offset": 50}]})
    print("loft ok", r.ok, r.error or ("V=%.1f" % r.data["volume"]))
    assert r.ok, r.error

    # 8) sweep circle r5 along L-shaped path (sharp corner -> round transition)
    assert s.act("param.body", {"name": "S"}).ok
    r = s.act("param.sweep", {"body": "S", "feature": "Pipe", "profile": {"circle": 5},
                             "path": {"plane": "XZ", "points": [[0, 0], [0, 30], [25, 30]]}})
    print("sweep ok", r.ok, r.error or ("V=%.1f bbox=%s" % (r.data["volume"], r.data["bbox_size"])))
    assert r.ok, r.error
    # both legs should contribute: bbox spans ~25 in X and ~30 in Z
    bs = r.data["bbox_size"]
    assert bs[0] > 20 and bs[2] > 25, ("sweep did not turn corner: %s" % bs)
    # volume should clearly exceed a single 30mm leg (pi*25*30 = 2356)
    print("sweep volume vs single-leg 2356:", r.data["volume"])
    assert r.data["volume"] > 3000, "sharp-corner sweep degenerated (only one leg)"

    # 9) bolt-circle flange: off-center hole + polar pattern (PartDesign feature)
    assert s.act("param.body", {"name": "F"}).ok
    assert s.act("param.pad", {"body": "F", "feature": "Plate",
                               "profile": {"circle": 40}, "length": 10}).ok
    base = s.act("param.pocket", {"body": "F", "feature": "Bore",
                                  "profile": {"circle": 10}, "through": True})
    assert base.ok, base.error
    one = s.act("param.pocket", {"body": "F", "feature": "BoltHole",
                                 "profile": {"circle": 3.25, "at": [30, 0]}, "through": True})
    assert one.ok, one.error
    hole_vol = base.data["volume"] - one.data["volume"]  # volume of a single bolt hole
    assert hole_vol > 0
    r = s.act("param.pattern_polar", {"body": "F", "feature": "Bolts",
                                      "originals": ["BoltHole"], "count": 6, "angle": 360})
    print("polar ok", r.ok, r.error or ("V=%.1f faces=%d" % (r.data["volume"], r.data["faces"])))
    assert r.ok, r.error
    # tip must advance to the pattern and exactly 6 holes must be cut (no overlap)
    assert s.act("param.tree", {"body": "F"}).data["tip"] == "Bolts"
    removed = base.data["volume"] - r.data["volume"]
    assert abs(removed - 6 * hole_vol) < 1.0, ("expected 6 holes, removed %.1f (1 hole=%.1f)"
                                               % (removed, hole_vol))
    # parametric re-edit: drive the bolt count up and confirm it recuts
    up = s.act("param.set", {"param": "Bolts.occurrences", "value": 8})
    assert up.ok, up.error
    r8 = s.act("param.measure", {"body": "F"})
    removed8 = base.data["volume"] - r8.data["volume"]
    print("re-edit to 8 bolts removed:", round(removed8, 1))
    assert abs(removed8 - 8 * hole_vol) < 1.0, "param.set Occurrences did not recut"

    # 10) body-qualified param keys: a 2nd body re-using feature name "Bolts"
    #     must not clobber the first body's parameter target.
    assert s.act("param.body", {"name": "F2"}).ok
    assert s.act("param.pad", {"body": "F2", "feature": "Plate",
                               "profile": {"circle": 40}, "length": 10}).ok
    b2 = s.act("param.pocket", {"body": "F2", "feature": "Bore",
                                "profile": {"circle": 10}, "through": True}).data["volume"]
    assert s.act("param.pocket", {"body": "F2", "feature": "BoltHole",
                                  "profile": {"circle": 3.25, "at": [30, 0]}, "through": True}).ok
    assert s.act("param.pattern_polar", {"body": "F2", "feature": "Bolts",
                                         "originals": ["BoltHole"], "count": 4, "angle": 360}).ok
    # drive ONLY F2 via its qualified key; F (set to 8 above) must stay at 8
    assert s.act("param.set", {"param": "F2.Bolts.occurrences", "value": 5}).ok
    f2_removed = b2 - s.act("param.measure", {"body": "F2"}).data["volume"]
    assert abs(f2_removed - 5 * hole_vol) < 1.0, "qualified key did not target F2"
    f_removed = base.data["volume"] - s.act("param.measure", {"body": "F"}).data["volume"]
    assert abs(f_removed - 8 * hole_vol) < 1.0, "F2 edit leaked into F (key collision)"
    print("qualified keys: F=%d bolts, F2=%d bolts (no collision)"
          % (round(f_removed / hole_vol), round(f2_removed / hole_vol)))

    # 11) mirror feature: one offset hole mirrored across YZ -> symmetric pair
    assert s.act("param.body", {"name": "M"}).ok
    assert s.act("param.pad", {"body": "M", "feature": "Plate",
                               "profile": {"rect": [80, 40]}, "length": 8}).ok
    mbase = s.act("param.measure", {"body": "M"}).data["volume"]
    mh = mbase - s.act("param.pocket", {"body": "M", "feature": "Hole",
                                        "profile": {"circle": 4, "at": [30, 0]},
                                        "through": True}).data["volume"]
    mr = s.act("param.mirror", {"body": "M", "feature": "Mir",
                                "originals": ["Hole"], "plane": "YZ"})
    assert mr.ok, mr.error
    assert s.act("param.tree", {"body": "M"}).data["tip"] == "Mir"
    assert abs((mbase - mr.data["volume"]) - 2 * mh) < 1.0, "mirror did not duplicate the hole"
    print("mirror -> holes:", round((mbase - mr.data["volume"]) / mh, 2))

    # 12) vented cover plate: a single-sketch nx*ny hole grid (one pocket)
    assert s.act("param.body", {"name": "V"}).ok
    assert s.act("param.pad", {"body": "V", "feature": "Plate",
                               "profile": {"rect": [100, 80]}, "length": 4}).ok
    vbase = s.act("param.measure", {"body": "V"}).data["volume"]
    vg = s.act("param.pocket", {"body": "V", "feature": "Vents", "through": True,
                                "profile": {"grid": {"circle": 3, "nx": 5, "ny": 4,
                                                     "dx": 18, "dy": 18}}})
    assert vg.ok, vg.error
    single = (3.0 ** 2) * 3.14159265 * 4  # pi r^2 * thickness, one vent hole
    removed = vbase - vg.data["volume"]
    assert abs(removed - 20 * single) < 5.0, ("expected 20 vents, removed %.1f (1=%.1f)"
                                              % (removed, single))
    vents_sk = next(d for d in s.act("param.diagnose", {}).data["sketches"]
                    if d["sketch"] == "Vents_sk")
    assert vents_sk["fully_constrained"], "grid sketch not fully constrained (DoF>0)"
    print("grid vents -> holes:", round(removed / single, 1), "DoF=0")

    # 13) stacked features at height: a boss padded on TOP of the base via offset
    assert s.act("param.body", {"name": "T"}).ok
    assert s.act("param.pad", {"body": "T", "feature": "Base",
                               "profile": {"circle": 20}, "length": 10}).ok
    assert s.act("param.pad", {"body": "T", "feature": "Boss", "offset": 10,
                               "profile": {"circle": 10}, "length": 10}).ok
    tm = s.act("param.measure", {"body": "T"}).data
    assert abs(tm["bbox_size"][2] - 20) < 0.01, "offset sketch did not stack the boss on top"
    assert abs(tm["volume"] - (math.pi * 400 * 10 + math.pi * 100 * 10)) < 1.0, "stacked volume wrong"
    print("stacked step height:", tm["bbox_size"][2])

    # 14) subtractive revolve (Groove): an O-ring groove turned into a shaft
    assert s.act("param.body", {"name": "Shaft"}).ok
    assert s.act("param.pad", {"body": "Shaft", "feature": "Rod",
                               "profile": {"circle": 15}, "length": 40}).ok
    rod = s.act("param.measure", {"body": "Shaft"}).data["volume"]
    gr = s.act("param.groove", {"body": "Shaft", "feature": "ORing", "plane": "XZ",
                                "angle": 360,
                                "profile": {"polygon": [[13, 18], [15, 18], [15, 22], [13, 22]]}})
    assert gr.ok, gr.error
    assert s.act("param.tree", {"body": "Shaft"}).data["tip"] == "ORing"
    ring = math.pi * (15 ** 2 - 13 ** 2) * 4  # annular groove volume
    assert abs((rod - gr.data["volume"]) - ring) < 1.0, "groove did not cut the expected ring"
    print("groove ring removed:", round(rod - gr.data["volume"], 1))

    # 15) helical sweep: a coil spring (Part::Helix spine + AdditivePipe)
    assert s.act("param.body", {"name": "Spring"}).ok
    co = s.act("param.sweep", {"body": "Spring", "feature": "Coil",
                               "profile": {"circle": 2.5},
                               "path": {"helix": {"pitch": 8, "height": 48, "radius": 20}}})
    assert co.ok, co.error
    assert co.data["turns"] == 6.0, "helix turns wrong"
    coil_exp = math.pi * 2.5 ** 2 * 6 * math.sqrt((2 * math.pi * 20) ** 2 + 8 ** 2)
    assert abs(co.data["volume"] - coil_exp) < coil_exp * 0.02, "coil volume off (>2%)"
    print("coil turns/volume:", co.data["turns"], round(co.data["volume"], 1))

    # 16) subtractive helical sweep: thread a stud (helix overruns both ends)
    assert s.act("param.body", {"name": "Stud"}).ok
    assert s.act("param.pad", {"body": "Stud", "feature": "Rod",
                               "profile": {"circle": 5}, "length": 30}).ok
    rodv = s.act("param.measure", {"body": "Stud"}).data["volume"]
    thr = s.act("param.sweep", {"body": "Stud", "feature": "Thread", "cut": True,
                                "profile": {"circle": 1.0},
                                "path": {"helix": {"pitch": 3, "height": 32, "radius": 5, "z": -1}}})
    assert thr.ok, thr.error
    assert s.act("param.tree", {"body": "Stud"}).data["tip"] == "Thread"
    assert 0 < (rodv - thr.data["volume"]) < rodv, "thread cut removed an implausible volume"
    print("thread cut removed:", round(rodv - thr.data["volume"], 1), "turns:", thr.data["turns"])

    # 17) involute spur gear profile: a module-2, 18-tooth gear with a center bore
    assert s.act("param.body", {"name": "Gear"}).ok
    assert s.act("param.pad", {"body": "Gear", "feature": "Blank",
                               "profile": {"gear": {"module": 2, "teeth": 18}}, "length": 8}).ok
    assert s.act("param.pocket", {"body": "Gear", "feature": "Bore",
                                  "profile": {"circle": 4}, "through": True}).ok
    gm = s.act("param.measure", {"body": "Gear"})
    assert gm.ok, gm.error
    assert abs(gm.data["bbox_size"][0] - 40) < 0.5, gm.data["bbox_size"]  # tip dia = 2*(rp+m)
    # below the pitch disc (gaps cut more than teeth add) and above the root disc
    disc = math.pi * 18 ** 2 * 8 - math.pi * 16 * 8
    root = math.pi * 15.5 ** 2 * 8 - math.pi * 16 * 8
    assert root < gm.data["volume"] < disc, (gm.data["volume"], root, disc)
    assert gm.data["faces"] > 100  # one pair of flank faces per tooth + tip/root
    print("gear volume:", round(gm.data["volume"], 1), "tip dia:", gm.data["bbox_size"][0])

    # 18) thin-wall shell (PartDesign Thickness): open-top enclosure + re-edit
    assert s.act("param.body", {"name": "Box"}).ok
    assert s.act("param.pad", {"body": "Box", "feature": "Block",
                               "profile": {"rect": [40, 30]}, "length": 20}).ok
    solidv = s.act("param.measure", {"body": "Box"}).data["volume"]
    assert abs(solidv - 24000) < 1e-6, solidv
    sh = s.act("param.shell", {"body": "Box", "feature": "Wall", "thickness": 2, "open": "+Z"})
    assert sh.ok, sh.error
    assert sh.data["opened_faces"], sh.data
    # 2mm inward wall, open top: outer 40x30x20 minus inner 36x26x18 cavity
    assert abs(sh.data["volume"] - (24000 - 36 * 26 * 18)) < 1e-3, sh.data["volume"]
    # parametric re-edit: thicken the wall 2 -> 3 and recheck the cavity volume
    assert s.act("param.set", {"param": "Wall.value", "value": 3}).ok
    v3 = s.act("param.measure", {"body": "Box"}).data["volume"]
    assert abs(v3 - (24000 - 34 * 24 * 17)) < 1e-3, v3
    assert s.act("param.diagnose", {"body": "Box"}).data["all_healthy"]
    # a tube: shelling a cylinder open at BOTH ends
    assert s.act("param.body", {"name": "Tube"}).ok
    assert s.act("param.pad", {"body": "Tube", "feature": "Cyl",
                               "profile": {"circle": 15}, "length": 40}).ok
    tb = s.act("param.shell", {"body": "Tube", "feature": "W", "thickness": 3,
                               "open": ["+Z", "-Z"]})
    assert tb.ok and len(tb.data["opened_faces"]) == 2, tb.error or tb.data
    print("shell open-top vol:", round(sh.data["volume"], 1),
          "thickened:", round(v3, 1), "tube faces opened:", tb.data["opened_faces"])

    # 19) full enclosure: shelled case + 4 corner standoffs (grid pad) each with
    # a pilot hole (grid pocket). exercises features stacked on a shelled body.
    cw, cd, ch, ct = 60, 40, 25, 2.5
    bx, by = cw / 2 - 8, cd / 2 - 8
    assert s.act("param.body", {"name": "Case"}).ok
    assert s.act("param.pad", {"body": "Case", "feature": "Outer",
                               "profile": {"rect": [cw, cd]}, "length": ch}).ok
    assert s.act("param.shell", {"body": "Case", "feature": "W", "thickness": ct}).ok
    boss = {"grid": {"circle": 4, "nx": 2, "ny": 2, "dx": 2 * bx, "dy": 2 * by}}
    pil = {"grid": {"circle": 1.5, "nx": 2, "ny": 2, "dx": 2 * bx, "dy": 2 * by}}
    assert s.act("param.pad", {"body": "Case", "feature": "Bosses",
                               "profile": boss, "offset": ct, "length": ch - ct - 3}).ok
    assert s.act("param.pocket", {"body": "Case", "feature": "Pilots",
                                  "profile": pil, "offset": ch - 3, "length": ch - ct - 3}).ok
    assert s.act("param.diagnose", {"body": "Case"}).data["all_healthy"]
    cm = s.act("param.measure", {"body": "Case"})
    assert cm.ok and cm.data["faces"] > 12, cm.data
    # chaining a transform feature into a mirror must be rejected clearly (the
    # kernel can't compose transforms; grid is the route used above)
    s.act("param.pad", {"body": "Case", "feature": "Tab",
                        "profile": {"circle": 3, "at": [bx, by]}, "offset": ct, "length": 4})
    m1 = s.act("param.mirror", {"body": "Case", "feature": "MX",
                               "originals": ["Tab"], "plane": "YZ"})
    assert m1.ok, m1.error
    chained = s.act("param.mirror", {"body": "Case", "feature": "MY",
                                     "originals": ["Tab", "MX"], "plane": "XZ"})
    assert not chained.ok and "chaining transforms" in (chained.error or ""), chained.error
    print("enclosure faces:", cm.data["faces"], "vol:", round(cm.data["volume"], 1),
          "| transform-chain rejected:", chained.error.split(";")[0])

    print("PARAM SMOKE OK", s.summary())
    k.shutdown()


if __name__ == "__main__":
    main()
