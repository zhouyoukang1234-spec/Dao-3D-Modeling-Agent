"""FreeCAD capability-map smoke -- the foundation reverse-engineered once and
frozen, then guarded.

This proves three things about :mod:`cad_agent.capability`:

* the live kernel introspection finds the foundational surface we expect (the
  Part / OCCT core, the parametric and mesh workbenches, the BRep shape API);
* the frozen ``capability_map.json`` committed to the repo still matches a live
  scan exactly -- so the artifact is honest and any FreeCAD API drift is caught
  here rather than discovered mid-feature; and
* ``coverage`` cleanly maps every registered operator onto a kernel domain (no
  unrecognised prefixes) and reports the still-uncovered modules as a fact.
"""
from cad_agent import capability
from cad_agent import new_session


def main():
    s = new_session("capability")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- live scan finds the foundational surface ------------------------- #
    live = capability.scan()
    mods = live["modules"]
    for core in ("Part", "PartDesign", "Mesh", "Sketcher", "Surface", "Points",
                 "Draft", "TechDraw", "Fem", "Path"):
        assert core in mods, core
    # the OCCT BRep core: every topological type carries a rich method surface.
    for cls in ("Shape", "Solid", "Face", "Edge", "Wire", "Vertex", "Compound"):
        assert cls in live["shape_api"], cls
        assert len(live["shape_api"][cls]) > 50, (cls, len(live["shape_api"][cls]))
    # Part is the kernel root -- it must expose the most classes of any module.
    assert len(mods["Part"]["classes"]) >= 40, mods["Part"]["classes"]
    t = live["totals"]
    assert t["modules"] >= 18 and t["classes"] >= 120 and t["functions"] >= 400, t

    # ---- the frozen map is honest: it still matches a live scan ----------- #
    frozen = capability.load_map()
    assert frozen == live, "capability_map.json is stale; regenerate via " \
        "freecadcmd -c 'from cad_agent import capability; capability.snapshot()'"

    # ---- coverage maps every operator onto a kernel domain ---------------- #
    cov = capability.coverage(s.registry.names())
    assert cov["operators"] == len(s.registry.names())
    assert cov["unknown_prefix"] == [], cov["unknown_prefix"]
    for dom in ("Part", "PartDesign", "Mesh", "Surface", "Points", "Fem"):
        assert dom in cov["by_domain"] and cov["by_domain"][dom] > 0, cov
    # the gap is reported as a fact (modules no operator touches yet).
    assert "ReverseEngineering" in cov["uncovered_modules"], cov
    assert isinstance(cov["uncovered_modules"], list)

    print("capability map: %(modules)d modules, %(classes)d classes, "
          "%(functions)d functions, %(shape_methods)d shape methods" % t)
    print("coverage: %d operators over %d kernel domains, %d modules uncovered" % (
        cov["operators"], len(cov["covered_modules"]), len(cov["uncovered_modules"])))
    print("CAPABILITY SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ == "__main__":
    main()
