"""Build a self-made fastener: a threaded stud + a hex nut, assembled coaxially.

Demonstrates the full pipeline end-to-end:
  * subtractive helical sweep to cut a real thread on a stud (param.sweep cut=True)
  * a hex nut = polygon pad + through bore + internal thread
  * a real Assembly container with two App::Link components
  * a coaxial slip-fit placement + interference check + BOM

Run headless:  freecadcmd examples/build_fastener.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("fastener")
    a = s.act

    # threaded stud (outer r5, M-style thread running off both ends)
    a("param.body", {"name": "Stud"})
    a("param.pad", {"body": "Stud", "feature": "Rod", "profile": {"circle": 5}, "length": 30})
    a("param.sweep", {"body": "Stud", "feature": "Thread", "cut": True,
                      "profile": {"circle": 1.0},
                      "path": {"helix": {"pitch": 3, "height": 32, "radius": 5, "z": -1}}})

    # hex nut: across-corner 19 -> circumradius 9.5; clearance bore + internal thread
    hexpts = [[round(9.5 * math.cos(math.radians(60 * i)), 4),
               round(9.5 * math.sin(math.radians(60 * i)), 4)] for i in range(6)]
    a("param.body", {"name": "Nut"})
    a("param.pad", {"body": "Nut", "feature": "Hex", "profile": {"polygon": hexpts}, "length": 8})
    a("param.pocket", {"body": "Nut", "feature": "Bore", "profile": {"circle": 5.6}, "through": True})
    a("param.sweep", {"body": "Nut", "feature": "IntThread", "cut": True,
                      "profile": {"circle": 0.9},
                      "path": {"helix": {"pitch": 3, "height": 10, "radius": 5.6, "z": -1}}})

    # assemble coaxially: stud grounded, nut threaded onto the top
    a("asm.create", {"name": "Fastener"})
    a("asm.add", {"name": "stud", "body": "Stud", "fixed": True})
    a("asm.add", {"name": "nut", "body": "Nut"})
    a("asm.place", {"name": "nut", "pos": [0, 0, 22]})

    clash = a("asm.interference", {}).data
    bom = a("asm.bom", {"density": 0.00785}).data
    print("clashes:", clash["clash_count"])
    print("BOM total_mass(g):", round(bom["total_mass"], 1), "parts:", bom["component_count"])

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_out")
    os.makedirs(out, exist_ok=True)
    a("asm.export", {"path": os.path.join(out, "fastener.step")})
    print("exported", os.path.join(out, "fastener.step"))


if __name__ == "__main__":
    main()
