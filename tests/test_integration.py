"""End-to-end integration tests against a live FreeCAD kernel.

Skipped automatically when freecadcmd is unavailable (e.g. on CI). The
exhaustive flows live in the smoke_*.py scripts; these assert the headline
behaviours of every tool group through the public session API.
"""
import math
import os

from conftest import requires_freecad


@requires_freecad
class TestSolid:
    def test_box_volume(self, freecad_session):
        s = freecad_session
        s.act("solid.box", {"name": "ibox", "length": 10, "width": 10, "height": 10})
        assert s.verify("ibox", {"volume": 1000, "valid": True}).ok

    def test_boolean_cut(self, freecad_session):
        s = freecad_session
        s.act("solid.box", {"name": "ip", "length": 20, "width": 20, "height": 10})
        s.act("solid.cylinder", {"name": "ic", "radius": 4, "height": 10, "pos": [10, 10, 0]})
        s.act("solid.cut", {"a": "ip", "b": "ic", "out": "iflange"})
        expected = 20 * 20 * 10 - math.pi * 16 * 10
        assert s.verify("iflange", {"volume": expected}, tol=1e-3).ok


@requires_freecad
class TestParam:
    def test_pad_pocket_diagnose(self, freecad_session):
        s = freecad_session
        assert s.act("param.body", {"name": "IB"}).ok
        r = s.act("param.pad", {"body": "IB", "feature": "IPad",
                                "profile": {"rect": [60, 40]}, "length": 8})
        assert r.ok and abs(r.data["volume"] - 19200) < 1e-3 and r.data["dof"] == 0
        d = s.act("param.diagnose", {})
        assert d.data["all_healthy"] and d.data["total_dof"] == 0

    def test_param_reedit(self, freecad_session):
        s = freecad_session
        assert s.act("param.set", {"param": "IPad.length", "value": 12}).ok
        m = s.act("param.measure", {"body": "IB"})
        assert abs(m.data["volume"] - 60 * 40 * 12) < 1e-3

    def test_sweep_sharp_corner(self, freecad_session):
        s = freecad_session
        assert s.act("param.body", {"name": "ISw"}).ok
        r = s.act("param.sweep", {"body": "ISw", "feature": "ISweep",
                                  "profile": {"circle": 5},
                                  "path": {"plane": "XZ", "points": [[0, 0], [0, 30], [25, 30]]}})
        assert r.ok, r.error
        # both legs present: spans X and Z
        bs = r.data["bbox_size"]
        assert bs[0] > 20 and bs[2] > 25


@requires_freecad
class TestAssembly:
    def test_stack_no_clash_and_bom(self, freecad_session):
        s = freecad_session
        assert s.act("param.body", {"name": "ABase"}).ok
        assert s.act("param.pad", {"body": "ABase", "feature": "ABp",
                                   "profile": {"rect": [50, 50]}, "length": 6}).ok
        assert s.act("param.body", {"name": "ABlk"}).ok
        assert s.act("param.pad", {"body": "ABlk", "feature": "ABkp",
                                   "profile": {"rect": [20, 20]}, "length": 20}).ok
        assert s.act("asm.create", {"name": "IAsm"}).ok
        assert s.act("asm.add", {"name": "ibase", "body": "ABase", "fixed": True}).ok
        assert s.act("asm.add", {"name": "iblk", "body": "ABlk"}).ok
        s.act("asm.stack", {"base": "ibase", "top": "iblk"})
        assert s.act("asm.interference", {}).data["clash_count"] == 0
        bom = s.act("asm.bom", {"density": 0.00785})
        assert bom.data["component_count"] == 2

    def test_coaxial_seats_pin_without_spurious_flip(self, freecad_session):
        # a cylinder's face axis has an arbitrary sign; the coaxial mate must not
        # let that flip the pin 180deg onto the wrong side of the bore.
        s = freecad_session
        s.act("param.body", {"name": "Plate"})
        s.act("param.pad", {"body": "Plate", "feature": "Slab",
                            "profile": {"rect": [60, 60]}, "length": 40})
        s.act("param.pocket", {"body": "Plate", "feature": "Bore",
                               "profile": {"circle": 12, "at": [0, 0]},
                               "length": 40, "through": True})
        s.act("solid.cylinder", {"name": "Pin", "radius": 10, "height": 120})
        s.act("asm.create", {"name": "A"})
        s.act("asm.add", {"assembly": "A", "body": "Plate", "name": "plate", "fixed": True})
        s.act("asm.add", {"assembly": "A", "body": "Pin", "name": "pin"})
        assert s.act("asm.coaxial", {"hole": "plate", "pin": "pin"}).ok
        # the pin still runs concentric through the plate, so a probe disc placed
        # concentric on it overlaps it -> the pin was not flipped away.
        s.act("solid.cylinder", {"name": "Disc", "radius": 9, "height": 4})
        s.act("asm.add", {"assembly": "A", "body": "Disc", "name": "disc"})
        s.act("asm.place", {"name": "disc", "pos": [0, 0, 60]})
        clashes = s.act("asm.interference", {"assembly": "A"}).data["clashes"]
        pairs = {tuple(sorted((c["a"], c["b"]))): c["overlap_volume"] for c in clashes}
        assert pairs.get(("disc", "pin"), 0.0) > 100.0, pairs


@requires_freecad
class TestAdvanced:
    def test_spreadsheet_drives_geometry(self, freecad_session):
        s = freecad_session
        assert s.act("param.body", {"name": "SS"}).ok
        assert s.act("param.pad", {"body": "SS", "feature": "SSPad",
                                   "profile": {"rect": [40, 30]}, "length": 5}).ok
        assert s.act("ss.create", {"cells": {"thk": 5}}).ok
        assert s.act("ss.bind", {"param": "SSPad.length", "alias": "thk"}).ok
        assert s.act("ss.set", {"alias": "thk", "value": 9}).ok
        m = s.act("param.measure", {"body": "SS"})
        assert abs(m.data["volume"] - 40 * 30 * 9) < 1e-3

    def test_mesh_watertight_and_render(self, freecad_session, tmp_path):
        s = freecad_session
        s.act("solid.box", {"name": "mb", "length": 10, "width": 10, "height": 10})
        r = s.act("mesh.analyze", {"name": "mb"})
        assert r.data["watertight"]
        png = str(tmp_path / "mb.png")
        rr = s.act("view.render", {"names": ["mb"], "view": "iso", "path": png})
        assert rr.data["bytes"] > 0 and os.path.exists(png)


@requires_freecad
class TestFem:
    def test_cantilever_matches_beam_theory(self, freecad_session):
        s = freecad_session
        if "fem.solve" not in s.tools():
            import pytest
            pytest.skip("FEM (Fem/ccx) not available in this FreeCAD build")
        L, b, H, F = 100.0, 10.0, 10.0, 1000.0
        s.act("solid.box", {"name": "fbeam", "length": L, "width": b, "height": H})
        assert s.act("fem.setup", {"target": "fbeam", "material": "steel"}).ok
        assert s.act("fem.fix", {"select": {"axis": "x", "side": "min"}}).ok
        assert s.act("fem.load", {"select": {"axis": "x", "side": "max"},
                                  "kind": "force", "value": F, "direction": [0, 0, -1]}).ok
        r = s.act("fem.solve", {"allowable_mpa": 250})
        assert r.ok, r.error
        analytic = 6.0 * F * L / (b * H * H)   # MPa
        # quadratic-element FEM lands within ~40% of slender-beam theory
        assert 0.8 * analytic <= r.data["max_von_mises_mpa"] <= 1.6 * analytic, (r.data, analytic)
        assert r.data["max_disp_mm"] > 0.1


@requires_freecad
class TestPath:
    def test_profile_gcode_is_tool_compensated(self, freecad_session):
        s = freecad_session
        if "path.gcode" not in s.tools():
            import pytest
            pytest.skip("Path workbench not available in this FreeCAD build")
        W, D, H, T = 60.0, 40.0, 12.0, 6.0
        s.act("param.body", {"name": "PL"})
        s.act("param.pad", {"body": "PL", "feature": "Slab",
                            "profile": {"rect": [W, D]}, "length": H})
        assert s.act("path.job", {"target": "PL", "tool_diameter": T}).ok
        rp = s.act("path.profile", {"side": "Outside"})
        assert rp.ok, rp.error
        bb = rp.data["path_bbox"]
        # outside contour offset by the tool radius T/2
        assert abs(bb[3] - (W / 2 + T / 2)) < 1e-3, bb
        rg = s.act("path.gcode", {})
        assert rg.ok and rg.data["feeds_g1"] >= 1 and rg.data["chars"] > 200, rg.data
