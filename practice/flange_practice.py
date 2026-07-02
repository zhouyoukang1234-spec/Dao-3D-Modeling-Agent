"""Practice round 1: bolted flange coupling assembly, perceive-verify each step.

Not a smoke test: a *practice campaign*. Builds a realistic multi-part
assembly purely through agent ops, reads every intermediate result back
through percept.*, and records any friction/deficiency encountered into a
deficiency log for the improve-loop.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from cad_agent import new_session

DEFECTS = []


def defect(msg):
    DEFECTS.append(msg)
    print("DEFECT:", msg)


def act(s, tool, args, expect_ok=True):
    r = s.act(tool, args)
    if r.ok != expect_ok:
        defect("%s %s -> %s" % (tool, args, r.error or r.data))
    return r


def main():
    s = new_session("flange_practice")

    # ---------------- Flange A: disc + hub + center bore + 6 bolt holes ----
    act(s, "solid.cylinder", {"name": "discA", "radius": 60, "height": 15})
    act(s, "solid.cylinder", {"name": "hubA", "radius": 25, "height": 25,
                              "pos": [0, 0, 15]})
    act(s, "solid.union", {"a": "discA", "b": "hubA", "out": "flangeA"})
    act(s, "solid.cylinder", {"name": "boreA", "radius": 10, "height": 40})
    act(s, "solid.cut", {"a": "flangeA", "b": "boreA", "out": "flangeA"})

    # bolt circle: 6 holes r=5 on R=45
    import math
    for i in range(6):
        ang = i * math.pi / 3
        act(s, "solid.cylinder", {
            "name": "holeA%d" % i, "radius": 5, "height": 15,
            "pos": [45 * math.cos(ang), 45 * math.sin(ang), 0]})
        act(s, "solid.cut", {"a": "flangeA", "b": "holeA%d" % i,
                             "out": "flangeA"})

    # perceive: expect 7 through-holes (bore + 6 bolt holes)
    r = act(s, "percept.features", {"object": "flangeA"})
    holes = [f for f in r.data["features"] if f["type"] == "through_hole"]
    print("flangeA holes:", len(holes),
          sorted(round(h["radius"], 1) for h in holes))
    if len(holes) != 7:
        defect("expected 7 through-holes on flangeA, percept saw %d"
               % len(holes))

    r = act(s, "percept.describe", {"object": "flangeA"})
    print("flangeA:", r.data["description"])

    # ---------------- Flange B: mirror twin, offset along +z ---------------
    act(s, "solid.mirror", {"name": "flangeA", "normal": [0, 0, 1],
                            "base": [0, 0, 0], "out": "flangeB"})
    act(s, "solid.translate", {"name": "flangeB", "vector": [0, 0, -3]})

    # ---------------- Gasket between the faces -----------------------------
    act(s, "solid.cylinder", {"name": "gasket", "radius": 60, "height": 3,
                              "pos": [0, 0, -3]})
    act(s, "solid.cylinder", {"name": "gbore", "radius": 10, "height": 3,
                              "pos": [0, 0, -3]})
    act(s, "solid.cut", {"a": "gasket", "b": "gbore", "out": "gasket"})
    for i in range(6):
        ang = i * math.pi / 3
        act(s, "solid.cylinder", {
            "name": "gh%d" % i, "radius": 5, "height": 3,
            "pos": [45 * math.cos(ang), 45 * math.sin(ang), -3]})
        act(s, "solid.cut", {"a": "gasket", "b": "gh%d" % i, "out": "gasket"})

    # ---------------- Bolts through the stack ------------------------------
    for i in range(6):
        ang = i * math.pi / 3
        act(s, "solid.cylinder", {
            "name": "bolt%d" % i, "radius": 4.8, "height": 33,
            "pos": [45 * math.cos(ang), 45 * math.sin(ang), -18]})

    # ---------------- Perceive the assembly --------------------------------
    parts = ["flangeA", "flangeB", "gasket"] + ["bolt%d" % i for i in range(6)]
    r = act(s, "percept.relations", {"objects": parts})
    rels = {(x["a"], x["b"]): x for x in r.data["relations"]}
    fa_g = rels[("flangeA", "gasket")]
    print("flangeA-gasket:", fa_g)
    if fa_g["relation"] != "contact":
        defect("flangeA/gasket should be in contact, got %s" % fa_g)
    fa_b0 = rels[("flangeA", "bolt0")]
    print("flangeA-bolt0:", fa_b0)
    if not (fa_b0["relation"] == "apart" and 0 < fa_b0["distance"] < 0.5):
        defect("bolt0 should sit in its hole with 0.2 clearance, got %s"
               % fa_b0)

    # coaxial mate: seat bolt0 onto its hole's real cylindrical axis
    act(s, "asm.create", {"name": "coupling"})
    for pname in parts:
        act(s, "asm.add", {"body": pname, "name": "c_" + pname})
    r = act(s, "asm.coaxial", {"pin": "c_bolt0", "hole": "c_flangeA"})
    print("asm.coaxial bolt0/flangeA:", r.ok, r.data if r.ok else r.error)

    # interference: none expected anywhere
    r = act(s, "solid.interference", {"names": parts})
    print("interference:", r.data)

    # scene digest
    r = act(s, "percept.scene", {"relations": False})
    print("scene objects:", r.data["object_count"])

    # section through the bolt circle plane: read the stack like a CT
    r = act(s, "percept.section", {"object": "flangeA", "normal": [0, 0, 1],
                                   "offset": 7.5})
    print("flangeA mid-section loops:", r.data["loop_count"])
    if r.data["loop_count"] != 8:
        defect("mid-plate section should show 8 loops (rim+bore+6 holes), "
               "got %d" % r.data["loop_count"])

    # ---------------- Persist + BOM + drawing ------------------------------
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)
    r = act(s, "doc.save", {"path": os.path.join(out_dir, "flange.FCStd")})
    r = act(s, "solid.export", {"name": "flangeA",
                                "path": os.path.join(out_dir, "flangeA.step")})
    print("export:", r.data if r.ok else r.error)

    # ---------------- percept.diff: verify an edit structurally ------------
    act(s, "solid.cylinder", {"name": "flangeA2", "radius": 60, "height": 15})
    act(s, "solid.cylinder", {"name": "extra", "radius": 3, "height": 15,
                              "pos": [30, 0, 0]})
    act(s, "solid.cut", {"a": "flangeA2", "b": "extra", "out": "flangeA2"})
    act(s, "solid.cylinder", {"name": "flangeA1", "radius": 60, "height": 15})
    r = act(s, "percept.diff", {"a": "flangeA1", "b": "flangeA2"})
    print("diff disc->disc+hole:", {k: r.data[k] for k in (
        "volume_delta", "material_removed", "face_count_delta")},
        "gained:", [f["type"] for f in r.data["features_gained"]])
    if not any(f["type"] == "through_hole"
               for f in r.data["features_gained"]):
        defect("percept.diff should report the drilled hole as gained")

    # ---------------- Round 2: FEM on the flange plate ---------------------
    r = s.act("fem.setup", {"target": "flangeA", "material": "steel"})
    print("fem.setup:", r.ok, str(r.data if r.ok else r.error)[:200])
    if r.ok:
        r = act(s, "fem.fix", {"select": {"axis": "z", "side": "min"}})
        print("fem.fix:", r.ok, r.data if r.ok else r.error)
        r = act(s, "fem.load", {"select": {"axis": "z", "side": "max"},
                                "kind": "force", "value": 1000,
                                "direction": [0, 0, -1]})
        print("fem.load:", r.ok, r.data if r.ok else r.error)
        r = s.act("fem.solve", {})
        print("fem.solve:", r.ok, str(r.data if r.ok else r.error)[:300])
        if not r.ok:
            defect("fem.solve failed on flangeA: %s" % r.error)
    else:
        defect("fem.setup failed on flangeA: %s" % r.error)

    # ---------------- Round 3: TechDraw drawing ----------------------------
    r = s.act("draw.techdraw", {"name": "flangeA",
                                "views": ["front", "top", "iso"],
                                "path": os.path.join(out_dir, "flangeA.svg")})
    print("draw.techdraw:", r.ok, str(r.data if r.ok else r.error)[:200])
    if not r.ok:
        defect("draw.techdraw failed: %s" % r.error)

    print("\n==== DEFICIENCY LOG (%d) ====" % len(DEFECTS))
    for d in DEFECTS:
        print("-", d)
    print("PRACTICE ROUND 1 DONE", s.summary())
    return DEFECTS


if __name__ == "__main__":
    main()
