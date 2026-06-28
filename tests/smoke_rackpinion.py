"""Rack-and-pinion smoke -- rotary <-> linear conversion, the drivetrain's tail.

A pinion of pitch radius r rolls without slipping on a rack: x = r*theta. We
check the closed form (one revolution = one pitch circumference), the exact
invertibility (angle -> travel -> angle), the module/teeth pitch-radius form,
and that a whole gearbox composes: motor rpm -> geartrain -> pinion -> rack
linear speed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def main():
    s = new_session("rackpinion")
    print("FreeCAD", s.registry.kernel.freecad_version)

    r = 12.0
    # one full revolution moves the rack one pitch circumference
    full = s.act("solid.rackpinion", {"pitch_radius": r, "angle": 360}).data
    assert abs(full["travel"] - 2 * math.pi * r) < 1e-3, full
    assert abs(full["travel_per_rev"] - 2 * math.pi * r) < 1e-3, full
    # half revolution -> half circumference
    half = s.act("solid.rackpinion", {"pitch_radius": r, "angle": 180}).data
    assert abs(half["travel"] - math.pi * r) < 1e-3, half
    print("pinion r=%.0f: 360 deg -> travel %.3f = 2 pi r" % (r, full["travel"]))

    # exact invertibility: travel -> angle round-trips
    back = s.act("solid.rackpinion", {"pitch_radius": r, "travel": math.pi * r}).data
    assert abs(back["angle"] - 180) < 1e-3, back
    print("invertible: travel %.3f -> %.1f deg" % (math.pi * r, back["angle"]))

    # module/teeth form: r = module*teeth/2 = 2*15/2 = 15
    mt = s.act("solid.rackpinion", {"module": 2, "teeth": 15, "angle": 360}).data
    assert abs(mt["pitch_radius"] - 15) < 1e-3, mt
    assert abs(mt["travel"] - 2 * math.pi * 15) < 1e-3, mt
    print("module/teeth: m=2, z=15 -> r=%.0f, travel/rev=%.3f" % (mt["pitch_radius"], mt["travel_per_rev"]))

    # compose: motor 1200 rpm through a 4:1 reduction, pinion r=10 -> rack mm/min
    e = s.act("solid.geartrain", {"meshes": [{"driver": 10, "driven": 40}], "input_rpm": 1200}).data
    pinion_rpm = abs(e["output_rpm"])           # 300 rpm
    rack = s.act("solid.rackpinion", {"pitch_radius": 10, "angle": 360}).data
    feed = pinion_rpm * rack["travel_per_rev"]  # mm per minute
    assert abs(pinion_rpm - 300) < 1e-9, e
    assert abs(feed - 300 * 2 * math.pi * 10) < 0.1, feed  # travel/rev rounded to 4dp, x300
    print("drivetrain: 1200rpm /4 = %.0f rpm pinion -> rack feed %.1f mm/min" % (pinion_rpm, feed))

    # invalid inputs are rejected loudly
    assert not s.act("solid.rackpinion", {"angle": 90}).ok
    assert not s.act("solid.rackpinion", {"pitch_radius": 5}).ok
    assert not s.act("solid.rackpinion", {"pitch_radius": -1, "angle": 10}).ok
    print("RACKPINION SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_rackpinion"):
    main()
