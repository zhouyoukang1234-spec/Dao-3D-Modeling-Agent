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
          "tests.smoke_drawing"]


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
