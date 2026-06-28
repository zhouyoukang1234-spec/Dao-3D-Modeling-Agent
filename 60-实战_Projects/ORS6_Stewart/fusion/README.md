# ORS6 — Truth-fused kinematic assembly

Assemble the ORS6 Stewart mechanism so it fuses 1:1 with the image-to-3D
(Tripo) reconstruction of the real device. The Tripo mesh is the geometric
ground truth; nothing is placed by firmware assumption.

## Principle (反者道之动 · 无为而无不为)
The whole linkage is one closed kinematic mechanism, not independently-placed
parts. A single receiver pose drives per-leg inverse kinematics; every rod is
exactly 175 mm and connects servo-arm-tip ↔ receiver-mount by construction, so
nothing floats.

## Pipeline
1. `kreg.py` — rigid (scale-locked) ICP of the static body shell
   (Base + Frames + Lid) to Tripo. Canonical frame = Tripo mm. → `data/kfit_body.npz`
2. `kfit.py` — receiver fit. The receiver is the **real device's simple ring**,
   pinned to the Tripo-detected ring circle (center+normal+radius). Only its
   orientation (incl. twist) is free; per-leg IK solves each arm angle for the
   fixed 175 mm rod, branch chosen by proximity to Tripo. → `data/kfit_pose.npz`
3. `kassemble.py` — builds the per-part colored assembly (red body/frame/rods,
   white arms, chrome ball-joints, ring receiver), renders 4-view assembly +
   overlay on Tripo (blue), prints symmetric chamfer, exports `data/ORS6_fused.glb`.
4. `diag_chamfer.py` — per-part chamfer breakdown (body / ring / rod / arm).

## Key data finding
The repo's `Receiver` STL is a deep cup / Twist gear-head — a **different part
version** from the simple ring on the photographed device. The receiver is
therefore rebuilt as the actual detected ring so it fuses with Tripo.

## Run
```
set ORS6_STL_ROOT=<path to STLs>   # or export on POSIX
python fusion/kreg.py && python fusion/kfit.py && python fusion/kassemble.py
```
