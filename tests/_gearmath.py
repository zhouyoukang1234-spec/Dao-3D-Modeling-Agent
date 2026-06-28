"""Gear-train phasing math shared by the meshing/assembly smokes.

The gear profile generator places tooth #0 pointing +x (phase 0). For two gears to
*mesh* (one's teeth sitting in the other's spaces) rather than *jam* (tooth tip on
tooth tip), the driven gear must be rotated about its own axis by a specific phase.

Derivation: at the line of centres the reference gear's tooth centre must face the
driven gear's space centre. With the reference at phase 0 and the driven gear's
centre at angle ``beta`` (deg) from the reference centre, the meshing condition
``a_ref + a_driven == 0.5 (mod 1)`` gives

    phi = beta + 180 - (180 - beta*z_ref) / z_driven      (mod 360/z_driven)

Verified against a brute-force interference sweep: a planet (z18) meshing a sun
(z24) at beta=0 needs phi=10 deg = half a tooth, where the sun/ring overlaps both
fall to zero.
"""


def meshing_phase_deg(beta_deg, z_ref, z_driven):
    """Rotation (deg, about its own axis) to apply to a ``z_driven`` gear whose
    centre lies at angle ``beta_deg`` from a phase-0 ``z_ref`` gear so the pair
    meshes instead of jamming. Result is reduced into ``[0, 360/z_driven)``."""
    phi = beta_deg + 180.0 - (180.0 - beta_deg * z_ref) / float(z_driven)
    pitch = 360.0 / float(z_driven)
    return phi % pitch
