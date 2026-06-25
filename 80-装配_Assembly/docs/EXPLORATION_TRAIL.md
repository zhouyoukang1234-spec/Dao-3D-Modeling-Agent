# Exploration trail — the honest record (反者道之動)

The `anchors/sr6/` directory holds many `closure_*.py` scripts.  They are **not**
redundant clutter to be deleted; they are the falsification trail by which the
architecture was found.  Each one encodes a hypothesis about "how does the SR6
home pose close?", and most encode a hypothesis that was **proven wrong** by its
own honest residual.  Keeping them (rather than quietly erasing the dead ends)
is the point: 反者道之動 — the reversal is how the way moved.  Deleting the
mistakes would hide *why* the canonical files are shaped the way they are.

This index says which scripts are **canonical** (the current architecture) and
which are **superseded** (kept as archaeology).  Run any of them; none lie about
their residuals.

## Canonical (the architecture as it now stands)

| file | layer | what it proves |
|---|---|---|
| `uam/datum.py` | L½ | Kabsch placement + 5 principle guards (general) |
| `uam/assembly.py` | L2/L3 | declarative mate solver (general, adaptive lm/trf) |
| `uam/kinematics.py` | L5 | rigid-consistency validator (general) |
| `anchors/sr6/constants.py` | L4 | firmware IK decode → arm/rod/home geometry |
| `anchors/sr6/closure_kabsch.py` | L½·SR6 | receiver home pose, RMS 0.0141mm (datum) |
| `anchors/sr6/closure_firmware_6leg.py` | SR6 | bespoke 6-leg closure, RMS 0 (the baseline) |
| `anchors/sr6/assemble_full.py` | L2/L3·SR6 | **6-leg closure via the GENERAL solver**, RMS 2e-12 |
| `anchors/sr6/rigid_consistency.py` | L5·SR6 | SR6 main subsystem fed to the general validator |
| `anchors/stewart/synthetic.py` | L5·anchor2 | validator generality (no SR6 DNA) |
| `anchors/stewart/assemble.py` | L2/L3·anchor2 | **solver generality**: Stewart FK via the GENERAL solver |

## Superseded — kept as the honest falsification trail

| file | hypothesis it tested | how its own residual killed it |
|---|---|---|
| `closure.py` | sense servo axes from frame holes | x=±76.4 was the mean of unrelated mount holes, not an axis (P2) |
| `closure_phys.py` | planar Y-Z arm sweep closes 4 main legs | "RMS=0" used a non-physical x-slide DOF — a fabricated zero (P1) |
| `closure_honest.py` | rod-length + sensed mounts fix the pose | only a non-physical roll≈78.5° closed it → mounts ≠ pivots |
| `closure_roll.py` | read home roll from the 55mm pitch offset | any roll satisfies a length; orientation is unreadable from it (P4) |
| `closure_datum.py` | "arms horizontal @ home" (PDF p.24) as datum | a calibration zero, not the loaded-home pose; superseded by P3 |
| `closure_grounded.py` | mixed sensed/firmware grounding | transitional; folded into closure_firmware_6leg |
| `closure_firmware.py` | firmware as forward kinematics, 4 main legs | correct but partial; generalised by closure_firmware_6leg |
| `assemble.py` | first mate-graph sketch | predates the general kernel; replaced by assemble_full.py |

Five principles crystallised out of these dead ends (see `CLOSURE_FINDINGS.md`
and `uam/datum.py` guards): **P1** every solver DOF must be physical; **P2** never
sense a COTS actuator's placement from printed-part holes; **P3** when a
controller exists, its IK home constants ARE the placement datum; **P4** close a
free assembly DOF with authority, never a hand-picked value; **P5** preserve
honest residuals.  Each principle is a tombstone for a script above.
