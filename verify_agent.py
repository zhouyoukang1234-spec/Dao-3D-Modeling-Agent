"""One-shot closed-loop verifier.

Runs every smoke suite against the live FreeCAD kernel and prints a single
green/red summary — the "perceive -> act -> verify" loop exercised end to end
across solid / param / asm / advanced groups. Use this on a machine with
FreeCAD installed (set FREECADCMD if it is not on PATH).
"""
import importlib
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SUITES = ["tests.smoke_kernel", "tests.smoke_param", "tests.smoke_asm",
          "tests.smoke_advanced", "tests.smoke_fem", "tests.smoke_path",
          "tests.smoke_complex", "tests.smoke_engineering",
          "tests.smoke_epicyclic", "tests.smoke_planetary",
          "tests.smoke_pgbox", "tests.smoke_kinematics",
          "tests.smoke_helical", "tests.smoke_rack",
          "tests.smoke_bevel", "tests.smoke_cam",
          "tests.smoke_fourbar", "tests.smoke_slidercrank",
          "tests.smoke_engine", "tests.smoke_interop",
          "tests.smoke_gear_fem", "tests.smoke_pressure_vessel",
          "tests.smoke_worm", "tests.smoke_buckling",
          "tests.smoke_thermal", "tests.smoke_rotordisk", "tests.smoke_modal",
          "tests.smoke_advmodel", "tests.smoke_cam_pocket",
          "tests.smoke_drawing", "tests.smoke_sketch_constraint",
          "tests.smoke_cam_drill", "tests.smoke_asm_massprops",
          "tests.smoke_pattern", "tests.smoke_draft",
          "tests.smoke_thickness", "tests.smoke_undercut",
          "tests.smoke_dfm_housing", "tests.smoke_section",
          "tests.smoke_beam_theory", "tests.smoke_overhang",
          "tests.smoke_dfm_report", "tests.smoke_fem_guard",
          "tests.smoke_reverse", "tests.smoke_mechanism",
          "tests.smoke_drive", "tests.smoke_step_reverse",
          "tests.smoke_recognize", "tests.smoke_reverse_pipeline",
          "tests.smoke_coaxial", "tests.smoke_fourbar",
          "tests.smoke_geartrain", "tests.smoke_gearmesh",
          "tests.smoke_rackpinion", "tests.smoke_cam",
          "tests.smoke_planetary", "tests.smoke_geneva",
          "tests.smoke_cam_profile", "tests.smoke_gearbox",
          "tests.smoke_spatial_mobility", "tests.smoke_inertia",
          "tests.smoke_curvature", "tests.smoke_obb",
          "tests.smoke_symmetry", "tests.smoke_fingerprint",
          "tests.smoke_match", "tests.smoke_chirality",
          "tests.smoke_complexity_guard", "tests.smoke_library_match",
          "tests.smoke_holes", "tests.smoke_fillets",
          "tests.smoke_design_intent", "tests.smoke_library_query",
          "tests.smoke_reverse_build", "tests.smoke_reuse",
          "tests.smoke_projarea", "tests.smoke_hydro", "tests.smoke_tolstack",
          "tests.smoke_clearance", "tests.smoke_thermal", "tests.smoke_pvessel",
          "tests.smoke_library_fetch", "tests.smoke_section_modulus",
          "tests.smoke_buckling", "tests.smoke_beam_deflection",
          "tests.smoke_torsion", "tests.smoke_natural_frequency",
          "tests.smoke_thermal_resistance", "tests.smoke_contact_stress",
          "tests.smoke_mechanism_guards"]


def main() -> int:
    results = []
    for mod_name in SUITES:
        try:
            mod = importlib.import_module(mod_name)
            mod.main()
            results.append((mod_name, True, ""))
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            results.append((mod_name, False, repr(exc)))
    print("\n==================== VERIFY SUMMARY ====================")
    ok = 0
    for name, passed, err in results:
        print("  %-22s %s %s" % (name, "PASS" if passed else "FAIL", err))
        ok += int(passed)
    print("  %d/%d suites passed" % (ok, len(results)))
    print("========================================================")
    return 0 if ok == len(results) else 1


# freecadcmd runs a script with __name__ set to the module basename rather than
# "__main__", so accept both to make `freecadcmd verify_agent.py` work directly.
if __name__ in ("__main__", "verify_agent"):
    _rc = main()
    sys.stdout.flush()  # freecadcmd exits without flushing block-buffered stdout
    sys.exit(_rc)
