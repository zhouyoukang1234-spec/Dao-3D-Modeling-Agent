# CONNECTED_FIX — why the rods splayed past the ring, and the mesh-snap closure

## The defect humans saw instantly (and the critics now quantify)

Every prior render put the receiver ring off to one side with the 6 rods
**splaying past it without touching it** — "平躺 / 爆炸头 / 散架". The eye reads
this immediately as *not connected*; the residual-based pipeline never did,
because it only optimised rod length (=175 mm) and IK residual (→1e-14).

## Root cause (measured, not guessed)

The firmware 2-D IK emits abstract anchor points that **do not lie on the real
STL meshes**:

| anchor | firmware value | real mesh | gap |
|---|---|---|---|
| `arm_tips` (rod ↔ horn) | local (18.3, 0, 42.3) | Arm horn x∈[53.5, 81.5] | **35.5 mm outside** |
| `recv_mounts` (rod ↔ ring) | circle **radius 97.7 mm** | Receiver ring reaches **radius ~70 mm** | **~34–39 mm outside** |

So the rods reached toward a **phantom mount circle larger than the actual
ring**, and started from a phantom horn tip 35 mm proud of the horn. Both ends
floated → the splayed/disconnected look. This is present even in the
341-test-passing `ORS6_Stewart` fused output, because those tests only check rod
length and IK residual, never **mesh contact**.

## The fix — 道法自然: let the abstract points return to the real geometry

`assemble_connected.py` keeps the firmware IK *direction/topology* (which servo
connects to which mount, the 3-pair fan) but **snaps each rod endpoint onto the
nearest point of its real mesh**:

```
tip  = nearest_point(placed_arm_horn_mesh,  firmware_arm_tip)
mount= nearest_point(placed_receiver_mesh,  firmware_recv_mount)
link = cylinder(tip, mount, r=6mm)   # red push-link, like the photo
```

The mesh is the single source of truth; firmware points are merely the seed for
a nearest-surface projection. No hand-placed constants.

## Result — dual-critic gate PASSES (was FAIL on every prior form)

**Perceptual (2-D gestalt vs real photo `ref_machine.jpg`):**

| descriptor | reference | connected model | verdict |
|---|---|---|---|
| connected components | 1 | 1 | ok |
| solidity | 0.726 | 0.725 | ok |
| elongation | 1.67 | 1.31 | ok |
| two-mass (box+ring) | 0.262 | 0.237 | ok |
| **VERDICT** | | | **PASS** |

**Structural (3-D, mesh contact):**

| link | length mm | gap→horn | gap→ring |
|---|---|---|---|
| LowerLeft / LowerRight | 170.7 | 0.00 | 0.00 |
| UpperLeft / UpperRight | 173.5 | 0.00 | 0.00 |
| LeftPitch / RightPitch | 151.4 | — | 0.00 |

L/R mirror symmetry exact (Δ = 0.00 mm on all 3 pairs); main axis body→receiver
clean. All 6 links physically touch both the horn and the ring.

See `proof_vs_real.png` (real photo ‖ model), `connected_views.png` (3 views),
`connected_panel.png` (silhouette gate).

## Reproduce

```
python assemble_connected.py     # build + print snap distances / link lengths
python render_connected.py       # -> connected_views.png  (3 horizontal views)
python run_connected.py          # dual-critic gate -> connected_panel.png
```
