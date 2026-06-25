# SR6 / ORS6 — Phase 2B Bottom-Core Rebuild · HANDOFF

> Status: **IN PROGRESS — root cause located, topology corrected, solver/renders WIP.**
> This folder is a handoff package for the next agent. Phase 1 (firmware IK port) and
> Phase 2 (33-STL full assembly, links = 175/186 mm exact) are already merged into PR #1.
> Phase 2B fixes the **bottom core module** (base + L/R frames + 6 servos), which Phase 2
> got visually-close but topologically wrong.

---

## 1. The problem (user report)

The upper mechanism (receiver ring + arms + rods) matches the PDF. The **bottom core
module is wrong**: servo box / servo layout is scrambled, it "can't move", doesn't match
reality. Previous attempts (including other agents) all stall here — the overall shape
exists but the underlying bottom structure cannot be reconstructed correctly.

## 2. Root cause (confirmed by firmware + STL + PDF cross-check)

The Phase 2 bottom model used **guessed servo coordinates** (hand-picked from 40+
false-positive auto hole detections) and an **incorrect servo topology**. The correct
architecture, now confirmed three ways, is:

### 2a. Two frames are LEFT / RIGHT (split along X), each an inward-tilted A-frame
- Firmware names them Left / Right (not front/back).
- `R-Frame` STL bounds (local/print): X∈[47.4,109.9], Y∈[-56,56], Z∈[11.9,65.9] — an
  A-shaped tent. `L-Frame` is the mirror across X=0.
- In PDF p23 the photo is shot from front-above, so a **front** servo appears lower in
  the image and a **back** servo higher → the firmware labels "Lower"/"Upper" most likely
  correspond to **front/back depth (±Y)**, NOT vertical Z stacking. (VERIFY against the
  detected bore Z-spans — see §4 open question.)

### 2b. 4 ball joints on the receiver, NOT 6  (this is the key fix)
Receiver STL = a horizontal ring (axis ‖ Z at home) with:
- **2 main lugs** sticking out sideways at local **(±60, 0, 0)**, pivot axis ‖ X.
- **2 pitch towers** at the rear rising up to **(±60, −14.235, 53.126)**, pivot axis ‖ X.

Note `sqrt(14.235² + 53.126²) = 55.0` at `atan2(53.126,14.235)=75°`, and the firmware uses
`5500` (=55 mm) and `0.2618 rad` (=15°, and 75°=90°−15°). The receiver geometry ties
exactly to the firmware pitch constants. ✓

### 2c. Per side, TWO main servos share ONE main ball (5-bar / 2-DOF positioner)
This is the OSR2-style mechanism. Firmware `SetMainServo(x,y)` is a per-servo 2D arm+rod
IK that places the arm tip at the **shared ball's** 2D position in that servo's swing
plane. Decisive evidence — at non-home the four main calls get **differential** targets:
```
out1 (L-lower): (16248-fwd, 1500 + thrust + roll)
out2 (L-upper): (16248-fwd, 1500 - thrust - roll)
out5 (R-upper): (16248-fwd, 1500 - thrust + roll)
out6 (R-lower): (16248-fwd, 1500 + thrust - roll)
```
The two left servos move their plane-y target in **opposite** directions for the same
stroke → they are two arms straddling one common ball (mirrored plane convention), not two
independent balls. `thrust` moves the ball along plane-y, `fwd` along plane-x, `roll`
tilts L vs R oppositely.

So: **mainL ball** ← L-lower + L-upper (2 rods, 175 mm each); **mainR ball** ← R-lower +
R-upper; **pitchL ball** ← L-pitch (1 rod); **pitchR ball** ← R-pitch (1 rod). 6 rods total.

### 2d. Firmware ↔ 3D rigid gap is real and quantified (do NOT try to "fix" it)
`SetMainServo` is a **pure 2D planar IK**; it ignores the ~25 mm out-of-plane (X) offset
between a main servo shaft and its receiver ball. The unavoidable per-leg length error at
home is exactly `sqrt(175² + 25²) − 175 ≈ 1.78 mm`. This is the true cost of "2D
controller driving 3D hardware", already quantified per-leg in Phase 2 — it is the honest
closure result, NOT a calibration failure.

## 3. Measured / firmware-derived ground truth (anchors — do not guess)

| quantity | value | source |
|---|---|---|
| main arm (horn→ball) | **50.0 mm** | STL measured |
| pitch arm (horn→ball) | **75.0 mm** | STL measured |
| rod (eye-to-eye) | **175 mm** main, 186 mm pitch | STL + firmware |
| receiver main pivots | (±60, 0, 0), axis ‖X | STL measured |
| receiver pitch pivots | (±60, −14.235, 53.126), axis ‖X | STL measured |
| firmware home main | `SetMainServo(16248,1500)` → ball at (162.48, 15.00) in plane | firmware |
| firmware pitch consts | 5500 = 55 mm, 0.2618 = 15°, arm 75 | firmware |
| R-Frame bounds (local) | X[47.4,109.9] Y[-56,56] Z[11.9,65.9] | STL |
| Base bounds | X[-58.3,58.3] Y[-68,69] Z[2.5,74] | STL |

## 4. OPEN QUESTION the next agent must resolve first

**Are the two main servos per frame stacked in Z (Upper/Lower) or split in depth ±Y
(front/back)?** p23 labels say "Upper/Lower" but the camera angle could make front/back
read as upper/lower. This decides the frame standing rotation and the 6 servo world
positions. Resolve by running `scripts/detect_bores.py "R-Frame"` (already written) to get
the 3 servo bore centers + axes in local coords, then check whether the two **main** bores
differ in local Z (→ Upper/Lower) or local Y (→ front/back). `detect_bores.py` currently
prints candidate cylinders per principal axis; tune radius/cluster thresholds to isolate
the 3 servo shaft bores (~6 mm) or horn recesses (~12 mm).

Last run start (incomplete): `R-Frame faces=6080 bounds=[[47.4,-56,11.9],[109.9,56,65.9]]`.

## 5. What is DONE in this folder

- **Topology corrected** to 4-ball / 5-bar shared-main (above) — locked.
- `scripts/solve_home.py` — per-leg IK + 6-DOF least-squares receiver-pose closure.
- `scripts/build_core.py` — assembly builder, generalized `build(pose)` to accept full
  6-DOF receiver poses (heave/surge/sway/roll/pitch/yaw).
- `scripts/motion_sim.py` — 108-pose trajectory across all 6 DOF; **self-consistent**
  closure (rod deviation ≤ 2.84e-14 mm, 108/108 reachable). ⚠ uses the OLD servo positions
  (placeholder), so it proves the math closes but NOT yet physical correctness.
- `scripts/render_bottom.py` — bottom-only multiview vs PDF p24.
- `scripts/view_part.py`, `scripts/detect_bores.py` — STL inspection / bore extraction.
- `renders/` — `part_receiver.png` (4-ball ring confirmed), `part_rframe.png` (A-frame),
  `sr6_bottom_home.png`, `sr6_core_v4.png`, `sr6_motion.gif`, `motion_report.json`.

## 6. What REMAINS (next agent's checklist)

1. Run `detect_bores.py` → resolve §4, extract the **6 servo bore centers + axes** in
   frame-local coords (3 per frame, mirror for L).
2. Determine the frame→world standing transform (A-frame inward tilt) that (a) places the
   2 frames into the base cradle without interpenetration and (b) makes the 4 main arm tips
   lie on a single horizontal "straight line" as in PDF p24, arms horizontal at home.
3. Rewrite `solve_home.py` `SERVO_O` / `SERVO_AXIS` / plane basis from the measured bores
   (replace placeholder positions). Keep the 5-bar shared-ball constraint.
4. Re-solve home; re-run `motion_sim.py`; verify all 6 rods = 175 mm across workspace AND
   that the model is physically consistent with measured servo world positions.
5. Place the 6 real link STLs + receiver STL on the FK transforms; interference check
   (scipy cKDTree, all gaps > 0).
6. Render bottom-only vs p24/p29 and full assembly vs p32 side-by-side; export the motion
   GIF as proof of true motion.
7. Wire corrected build into `closed_loop/assembly/` + `assembly_transforms.json`; update
   PR #1; CI green (Gates 1-4).

## 7. Reference files in the repo
- Firmware: `SR6-Alpha4_ESP32.ino` (`SetMainServo` ~L755-844, `SetPitchServo` ~L851-862).
- PDF: SR6 Build Instructions — p23 (servo wiring), p24 ("straight line" home), p29 (frame
  assembly), p32 (final).
- Existing solver/assembly: `closed_loop/true_kinematics.py`, `closed_loop/assembly/`.
- 33 STLs are NOT in the repo (large); set `SR6_STL_ROOT` to the parts dir to reproduce.

*道法自然 · 反者道之动 — measurement-driven, no guessing.*
