"""Build a spreadsheet-driven bolted flange coupling through the agent.

Exercises the parametric feature tree end to end, including the off-center
circle profile (``at``) and the PartDesign polar pattern (``param.pattern_polar``)
that a real bolt-circle flange needs. Produces two mating flanges, drives their
dimensions and bolt count parametrically, mates them into an assembly, then
writes ``coupling.FCStd``. Run with system Python (it spawns the kernel):

    FREECADCMD=".../freecadcmd.exe" python examples/build_coupling.py [out.FCStd]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session


def _flange(a, name, specs):
    """A flange = disc plate + central bore + a hub + a bolt circle of holes."""
    a("param.body", {"name": name})
    a("param.pad", {"body": name, "feature": "Plate",
                    "profile": {"circle": specs["od"] / 2.0}, "length": specs["thk"]})
    # concentric hub stacked on the plate (taller centered cylinder)
    a("param.pad", {"body": name, "feature": "Hub",
                    "profile": {"circle": specs["hub_d"] / 2.0},
                    "length": specs["thk"] + specs["hub_len"]})
    a("param.pocket", {"body": name, "feature": "Bore",
                       "profile": {"circle": specs["bore"] / 2.0}, "through": True})
    # one bolt hole on the bolt circle, then pattern it around the axis
    a("param.pocket", {"body": name, "feature": "BoltHole",
                       "profile": {"circle": specs["bolt_d"] / 2.0,
                                   "at": [specs["bcd"] / 2.0, 0]}, "through": True})
    a("param.pattern_polar", {"body": name, "feature": "Bolts",
                              "originals": ["BoltHole"], "count": specs["bolts"], "angle": 360})
    return a("param.measure", {"body": name}).data


def build(out_path: str) -> None:
    s = new_session("coupling")
    a = s.act

    specs = {"od": 90, "thk": 12, "hub_d": 40, "hub_len": 20,
             "bore": 25, "bcd": 68, "bolt_d": 9, "bolts": 6}

    print("flange A:", _flange(a, "FlangeA", specs))
    print("flange B:", _flange(a, "FlangeB", specs))
    print("diagnose:", a("param.diagnose", {}).data)

    # parametric re-edit: bump the bolt count on flange A only and confirm it
    # recuts. Both flanges share the feature name "Bolts", so use the
    # body-qualified param key to target FlangeA unambiguously.
    a("param.set", {"param": "FlangeA.Bolts.occurrences", "value": 8})
    print("flange A after 8-bolt re-edit:", a("param.measure", {"body": "FlangeA"}).data)
    a("param.set", {"param": "FlangeA.Bolts.occurrences", "value": 6})  # back to matching pair

    # mate the two flanges face to face into a real assembly
    a("asm.create", {"name": "Coupling"})
    a("asm.add", {"name": "flangeA", "body": "FlangeA", "fixed": True})
    a("asm.add", {"name": "flangeB", "body": "FlangeB"})
    a("asm.stack", {"base": "flangeA", "top": "flangeB"})
    print("interference:", a("asm.interference", {}).data)
    print("bom:", a("asm.bom", {"density": 0.00785}).data)

    r = a("doc.save", {"path": out_path})
    print("saved:", r.data)
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "coupling.FCStd")
    build(out)
