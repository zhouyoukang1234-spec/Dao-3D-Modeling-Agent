"""Gear-train smoke -- speed ratio of an ordinary gear train.

Pairs with ``solid.coaxial`` (which finds gears stacked on a shaft): once the
spindle is recovered, the train value tells how fast and which way the output
turns. Each mesh multiplies the train value by driver/driven teeth; an external
mesh flips the sign. We check the textbook cases against closed form:

  * a single external pair 20->40  : e = -1/2 (output reversed, half speed)
  * an idler 20->30->40            : e = +1/2 (idler cancels magnitude, 2 flips)
  * a compound train               : products of teeth, exact
  * an internal/ring mesh          : keeps sign
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("geartrain")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # single external pair: 20 -> 40
    r = s.act("solid.geartrain", {"meshes": [{"driver": 20, "driven": 40}], "input_rpm": 100}).data
    assert abs(r["train_value"] - (-0.5)) < 1e-9, r
    assert r["reversing"] and abs(r["output_rpm"] - (-50)) < 1e-9, r
    assert abs(r["reduction"] - 2.0) < 1e-9, r
    print("external pair 20/40: e=%.3f, out=%.1f rpm, reduction=%.1f" % (r["train_value"], r["output_rpm"], r["reduction"]))

    # idler train 20 -> 30 -> 40: idler cancels magnitude, two external flips -> +
    r = s.act("solid.geartrain", {"meshes": [
        {"driver": 20, "driven": 30}, {"driver": 30, "driven": 40}]}).data
    assert abs(r["train_value"] - 0.5) < 1e-9 and not r["reversing"], r
    print("idler train 20/30/40: e=%.3f (idler cancels, sign restored)" % r["train_value"])

    # compound train: (20/40) on shaft, compounded 15/60 -> e = 0.5*0.25 = 0.125
    r = s.act("solid.geartrain", {"meshes": [
        {"driver": 20, "driven": 40}, {"driver": 15, "driven": 60}], "input_rpm": 1200}).data
    assert abs(r["train_value"] - 0.125) < 1e-9, r
    assert abs(r["output_rpm"] - 150) < 1e-9, r  # two flips -> positive
    print("compound train: e=%.4f, 1200->%.0f rpm" % (r["train_value"], r["output_rpm"]))

    # pitch radii give the same ratio as tooth counts
    r = s.act("solid.geartrain", {"meshes": [{"driver_radius": 10, "driven_radius": 20}]}).data
    assert abs(r["train_value"] - (-0.5)) < 1e-9, r

    # internal/ring mesh keeps the sign (no reversal)
    r = s.act("solid.geartrain", {"meshes": [{"driver": 18, "driven": 72, "internal": True}]}).data
    assert not r["reversing"] and abs(r["train_value"] - 0.25) < 1e-9, r
    print("internal/ring mesh 18/72: e=%.3f (non-reversing)" % r["train_value"])

    # invalid input is rejected loudly
    bad = s.act("solid.geartrain", {"meshes": [{"driver": 0, "driven": 10}]})
    assert not bad.ok and "positive" in (bad.error or ""), bad
    print("GEARTRAIN SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_geartrain"):
    main()
