"""Planetary (epicyclic) gear smoke -- the Willis equation with a moving carrier.

A sun-planet-ring set is 2-DOF: fix any member and the other two trade speed by
a closed-form ratio. We check the textbook operating modes against closed form
for N_sun=24, N_ring=72 (so N_planet=24):

  * ring fixed   -> reduction w_sun/w_carrier = 1 + N_ring/N_sun = 4 ;
  * carrier fixed-> ordinary train value sun->ring = -N_sun/N_ring = -1/3 ;
  * sun fixed    -> w_ring/w_carrier = 1 + N_sun/N_ring = 4/3 ;
  * self-consistency: solving for any member reproduces the Willis identity.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402

NS, NR = 24.0, 72.0


def willis(d):
    # (w_s - w_c)/(w_r - w_c) must equal -N_ring/N_sun
    return (d["sun_rpm"] - d["carrier_rpm"]) / (d["ring_rpm"] - d["carrier_rpm"])


def main():
    s = new_session("planetary")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ring fixed, sun driven at 100 -> carrier slows by 1 + Nr/Ns = 4
    r1 = s.act("solid.planetary", {"teeth_sun": NS, "teeth_ring": NR,
                                   "sun_rpm": 100, "ring_rpm": 0}).data
    assert abs(r1["carrier_rpm"] - 25) < 1e-6, r1
    assert abs(100 / r1["carrier_rpm"] - (1 + NR / NS)) < 1e-6, r1
    assert abs(r1["teeth_planet"] - 24) < 1e-6, r1
    print("ring fixed: sun 100 -> carrier %.1f (reduction %.0f:1)" % (r1["carrier_rpm"], 100 / r1["carrier_rpm"]))

    # carrier fixed -> ordinary train, sun->ring = -Ns/Nr
    r2 = s.act("solid.planetary", {"teeth_sun": NS, "teeth_ring": NR,
                                   "sun_rpm": 90, "carrier_rpm": 0}).data
    assert abs(r2["ring_rpm"] - (-NS / NR * 90)) < 1e-6, r2
    assert abs(willis(r2) - (-NR / NS)) < 1e-6, r2
    print("carrier fixed: sun 90 -> ring %.1f (ordinary -Ns/Nr)" % r2["ring_rpm"])

    # sun fixed -> w_ring/w_carrier = 1 + Ns/Nr
    r3 = s.act("solid.planetary", {"teeth_sun": NS, "teeth_ring": NR,
                                   "sun_rpm": 0, "carrier_rpm": 30}).data
    assert abs(r3["ring_rpm"] / 30 - (1 + NS / NR)) < 1e-6, r3
    print("sun fixed: carrier 30 -> ring %.1f (1 + Ns/Nr)" % r3["ring_rpm"])

    # solving for the carrier reproduces Willis exactly
    r4 = s.act("solid.planetary", {"teeth_sun": NS, "teeth_ring": NR,
                                   "sun_rpm": 120, "ring_rpm": -40}).data
    assert abs(willis(r4) - (-NR / NS)) < 1e-6, r4
    print("carrier solved: sun 120, ring -40 -> carrier %.2f (Willis holds)" % r4["carrier_rpm"])

    # boundary rejections
    assert not s.act("solid.planetary", {"teeth_sun": 24, "teeth_ring": 72, "sun_rpm": 1}).ok  # only 1 speed
    assert not s.act("solid.planetary", {"teeth_sun": 72, "teeth_ring": 24,
                                         "sun_rpm": 1, "ring_rpm": 0}).ok                       # ring<=sun
    assert not s.act("solid.planetary", {"teeth_sun": 24, "teeth_ring": 72, "teeth_planet": 99,
                                         "sun_rpm": 1, "ring_rpm": 0}).ok                       # bad tooth constraint
    print("PLANETARY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_planetary"):
    main()
