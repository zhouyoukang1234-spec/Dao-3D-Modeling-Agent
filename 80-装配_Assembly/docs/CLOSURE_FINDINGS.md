# SR6 Closure — Honest Findings (datum-grounded)

This documents what the closure solver actually proves, with honest residuals.
No world coordinate here is hand-fabricated; every one is either **perceived**
(STL mating interfaces), a **firmware metric prior** (arm/rod lengths, no world
coords), or a **datum** (the build-guide HOME definition).

## 1. The missing input (root cause of "总是组装不起来")

Printed parts + firmware constants fix only **relative** geometry; they do not
fix **absolute placement**. The prior architecture papered over this by
hardcoding/inventing servo world positions (its own `HALLUCINATION_MAP.md`
admits up to 53 mm error). The missing input is a **datum**: the assembled HOME
configuration. Acquired (never fabricated) from two authoritative sources:

- **Build-guide PDF (user's 原图), p.24 step 9** — at HOME every servo arm is
  horizontal (arm-end hole + axle hole on a line parallel to the frame base).
- **Firmware IK (SR6-Alpha4_ESP32.ino)** — decoded exactly:
  - main: arm = 50, rod = 175, shaft→pivot home distance c = hypot(162.48,15.0) = 163.171
  - pitch: arm = 75, rod = **175** (bearing-centre to bearing-centre)

## 2. CORRECTION: the earlier "4-main RMS=0" was a FABRICATED zero

`closure.solve_main_closure` swept each ball on a **horizontal** circle about a
*vertical* axis: `ball = (sx + arm·cosθ, sy + arm·sinθ, Zs)`. That gives the ball
a translational degree of freedom in **x** that the real linkage does not have —
every servo horn here turns about a **horizontal (X) shaft** (Arm.stl /
L-RPitcher.stl holes are all bored along X), so the ball sweeps in the **Y-Z
plane at the fixed wall x = ±76.4**. Inspecting that "RMS=0" solution, the balls
had slid `ball_x − shaft_x = ±38…±47 mm` off the wall — the fake x-DOF absorbed
the error and manufactured the zero. **Rejected.** (This is the same failure
class as the old shared-offset trick; the lesson is that RMS=0 means nothing
unless every DOF in the model is physical.)

## 3. PHYSICAL closure (Y-Z sweep about X, rod = 175 for all 6)

Two authority-backed corrections, then re-solve (`closure_phys.py`):
1. **rod = 175 for the pitch links too** — PDF p.26 says all 6 links are 175 mm
   bearing-centre to bearing-centre. The 185 mm I had used was the kinked Alpha
   **grommet** variant `PitcherLink_Alpha.stl` (60 mm offset + 175 → 185 straight
   line) — a **variant trap**, exactly what L1 affordance is meant to catch.
2. **ball sweeps Y-Z about the X shaft**; the pitch horn carries a +3.8 mm along-
   shaft (inward) offset = the "kink" (PDF p.24: kink points upwards).

With arm angles free, all six legs close at a level, centred receiver:
```
receiver t = [0.0, 0.0, 208.48]   roll/pitch/yaw = 0,0,0   RMS = 0.0000 mm
pitch balls sit UP at z≈100 (kink up); old model forced a 46.8° tilt — resolved.
```
This fixes the old "pitch frontier": the 46.8° tilt was an artefact of TWO
compounding errors (rod 185 not 175, and the wrong sweep plane), not a real
mechanism gap.

## 3b. The perceived servo position was a BUG; the firmware IS the datum

A first "datum test" using the **perceived** servo position (±76.4) left a ~10 mm
(6 %) rod discrepancy. Chasing it exposed the real bug: ±76.4 is just the **mean
of an unrelated grid of vertical mounting holes** (x = −51.9 and −100.9 on the
frame), with no physical meaning as a shaft location. Perceiving an *actuator's*
placement from printed-part holes is unreliable — the servo is a COTS body whose
output shaft is nowhere in the printed-part STL.

The authoritative absolute-placement datum is the **control law itself**. Reading
the firmware IK (`SR6-Alpha4_ESP32.ino`, `SetMainServo`/`SetPitchServo`):
```
SetMainServo(x=16248-fwd, y=1500+/-thrust+/-roll):
    csq=x^2+y^2;  beta=acos((csq-28125)/(100*c))
    -> 2a=100 => arm a=50 ;  a^2-b^2=-28125 => rod b=175
    home (fwd=thrust=roll=0): pivot offset = (vertical 162.48, horizontal 15.0) from shaft
SetPitchServo(x=16248-fwd, y=4500-thrust, z=side-1.5roll, pitch):
    bsq=36250-(75+z)^2;  beta=acos((csq+5625-bsq)/(150*c))
    -> 2a=150 => arm a=75 ;  bsq(z=0)=30625 => rod b=175 ;  36250=175^2+75^2
    home: pivot offset = (vertical 162.48, horizontal 45.0, lateral 0) from shaft
```
Key facts, all exact and authority-backed (no perception, no fabrication):
- shaft z = receiver_z - 162.48 = 208.48 - 162.48 = **46.0** (main and pitch).
- main: arm 50, rod 175, pivot 15 mm horizontal from shaft -> hypot(162.48, 15+50)
  = hypot(162.48, 65) = **175.00** with the arm **horizontal** at home (matches PDF p.24).
- pitch: arm 75, rod 175, pivot 45 mm horizontal; home servo angle decodes to
  ~ -23.4 deg, i.e. the pitch arm is **not** horizontal at running home (the PDF
  "arms horizontal" is the 1500 us calibration zero, distinct from the loaded home).

So the firmware is an internally **exact** closed kinematic chain; there is no
10 mm residual once the servo placement is taken from the control law instead of
from mis-perceived holes. The earlier "free closure @ RMS=0 with arms 12-46 deg
off horizontal" was the solver compensating for the wrong shaft position.

## 3c. OPEN frontier: the exact 3D embedding of each servo plane

What the firmware gives is each servo's **in-plane** home offset (vertical 162.48,
horizontal 15/45) and the metric chain (arm/rod), which close exactly. What it
does **not** give (by design — it needs no world frame) is the **orientation of
each servo's working plane in the world** (which world axis the "horizontal 15"
points along, and the shaft-axis direction). Resolving that 3D embedding so the
balls land physically *inside* the frame is the next step, and is exactly the
kind of thing an assembly CAD/STEP datum would pin in one shot. This is the
honest current boundary; it is NOT papered over with a free parameter.

## 4. Architecture takeaway (general, beyond SR6)

The decisive layer is **L½ DATUM**: a model can have perfect relative geometry
and still be unplaceable without an externally-acquired neutral configuration.
A general AI-assembly stack must (a) perceive mating interfaces, (b) carry a
metric prior from the control law if one exists, and (c) **acquire** the datum
from an authority (CAD / host-kinematics / build guide) — never invent it. Free
parameters that drive residuals to zero (the old shared-offset trick: RMS 0 with
|offset| 171 mm) are fabrication and must be rejected in favour of honest,
non-zero, diagnostic residuals.

Three reusable principles crystallised on this anchor:
1. **A DOF is only legitimate if it is physical.** RMS=0 means nothing if any
   solver DOF (the old horizontal-sweep ball-x) does not exist in the mechanism.
   Audit every free parameter against the real kinematics before trusting a zero.
2. **Do not perceive actuator placement from printed-part geometry.** COTS bodies
   (servos, motors, bearings) are not in the printed STLs; hole-grids on the
   frame are mounting features, not shaft axes. Mis-using them fabricates poses.
3. **Control-law-as-datum.** When a controller exists, its IK home constants ARE
   the absolute-placement datum for the actuated DOF — exact, authority-backed,
   and world-frame-free. Lift them; don't re-derive them from noisy perception.
   (SR6: 162.48/15 main, 162.48/45 pitch, arm 50/75, rod 175 — an exact chain.)

---

## §5  Receiver home ORIENTATION solved from the control law (not guessed)

The 4 main legs pin the receiver position (x,y,z) and level it, but they leave
the **roll about the main-pivot axis** free — a real DOF. Twice I filled that
DOF by hand (a +60° then a +64° tilt). Both were fabrication: a magnitude
constraint (the 55 mm pitch offset) is satisfied by *any* roll, so roll cannot
be read off it — it must come from an authority.

The authority is the firmware itself, read as **forward** kinematics at the home
input (`roll=pitch=fwd=thrust=side=0`):

* `SetMainServo(16248,1500)` → `out≈0`: main servos sit at neutral, arms
  horizontal; main pivot world = `(±59.5, 0, 208.48)`.
* `SetPitchServo(16248,4500,0,0)` → the `x+=5500·sin(15°)`, `y-=5500·cos(15°)`
  offset places each pitch pivot at `main + (vertical +14.23, horizontal 53.13)`
  → pitch pivot world = `(±61, 53.13, 222.71)`.

These 4 non-collinear world pivots fix the receiver pose **uniquely**. A Kabsch
fit of the *perceived* receiver pivots onto them (`closure_kabsch.py`) returns:

```
R = Rx(-90.00°)   t = (0, 0, 208.48)   perception-vs-firmware RMS = 0.0141 mm
```

So the home pose is the **proper** rotation `Rx(-90°)` + lift — *derived*, not
assumed. The 0.014 mm residual is the honest prediction error between the
printed part and the control law: they describe the same machine to 14 microns.

My earlier code used `world=(rx,-rz,-ry)`, a **det = -1 reflection** that
mirrored the receiver and put the pitch pivots on the wrong (−Y) side. The
correct proper rotation is `world=(rx, rz, -ry)+lift`.

**Visual confirmation** (`render_6leg.py`, real STL meshes at solved poses):
the front view shows the Receiver as a **vertical ring centred on x=0**, axis
horizontal along the stroke direction Y, suspended symmetrically by the four
crossing main links — the canonical SR6 home. All 6 legs close at RMS 0.000 mm
with the only free parameter per leg being its physical servo-arm angle (4 main
arms 0.0° horizontal, 2 pitch arms +8.5°).

Principle (4th, added): **An unconstrained assembly DOF must be closed by an
authority, never by a hand-picked value that merely satisfies a magnitude.**
Position came from the 4-main closure; orientation came from the control law's
forward map. Nothing was tuned to make the picture look right.

---

## §6  Does the closure hold across the WORKSPACE? (motion_sweep.py)

A home snapshot can be a coincidence. The decisive test of "is this a real
kinematic model" is whether the control law keeps the receiver rigid as it
*moves*. The firmware solves each main servo **independently in its own plane**
(lines 765-768); nothing there enforces that the six per-leg targets belong to
one rigid body. So I drove the real control law across its full input range and
reconstructed each firmware-commanded main pivot in world coords with **no fitted
DOF**, then checked the rigid invariant (the two main pivots must stay 119.0 mm
apart, and the two servos sharing a side must agree).

```
pure translation (fwd in [-3000,3000], thrust in [-6000,6000]):
    distance error = 0.0e+00 at EVERY amplitude ; per-side gap = 0
pure roll:
    roll 250  ( 2.4 deg)  dist 119.105  err +0.105 mm
    roll 1000 ( 9.5 deg)  dist 120.669  err +1.669 mm
    roll 3000 (26.8 deg)  dist 133.270  err +14.27 mm
```

Two findings, both honest:

1. **Pure translation is EXACTLY rigid** at all amplitudes. The home datum is
   not an isolated lucky pose — it extends rigidly along the fwd/thrust axes. The
   per-side servo pair never disagrees. This is strong evidence the perceived
   geometry and the control law are the same machine, not just at one point but
   along a 2-D slice of the workspace.

2. **Rotation is a first-order linearisation.** Firmware "roll" shifts the two
   main pivots oppositely in y at constant z. To first order that *is* a rigid
   rotation (the yaw-equiv column), so 119 mm is preserved to first order; the
   error grows only ~quadratically (`dist = hypot(119, 2*roll/100)`). The +14 mm
   at full roll is the firmware's **open-loop approximation error**, absorbed in
   practice by the receiver settling to a least-squares pose over all 6 legs.

**Architectural conclusion (control-law-as-datum, sharpened).** A controller
gives two things: an *exact* home configuration (the L½ datum, RMS 0.014 mm) and
a motion law that is *exact under translation* and *first-order accurate under
rotation* about that datum. That is exactly the structure of a good predictive
model: zero surprise at the operating point, bounded and diagnostic surprise as
you leave it. We neither fabricated a zero nor hid the quadratic residual — we
measured it and explained its origin.

---

## §7  L5 generalised: firmware-vs-rigid consistency (`uam/kinematics.py`)

§6 lived inside an SR6 script.  The reusable distillation now sits in
`uam/kinematics.py` (the L5 validator) with SR6 as a thin instance
(`anchors/sr6/rigid_consistency.py`).  The general primitive:

```
consistency(home_pts, commanded_pts, R, t)
    home_pts      name -> world position at the datum (L½)
    commanded_pts name -> where the CONTROL LAW puts that point for an input
    (R, t)        the rigid motion the input is MEANT to produce
  -> per-point resid = ||commanded - (R*home + t)||      (needs a pose)
  -> dist_drift = |commanded pair distance - home distance|  (frame-free)
```

No parameter is fitted — `resid` and `dist_drift` are pure prediction error.
`dist_drift` needs *no* pose assumption (a rigid body preserves every pairwise
distance), so it is the cleanest headline number.

### Result on the SR6 main subsystem (4 servos → 2 receiver points)

```
home:                       resid=0      drift=0      gap=0
pure translation (Y stroke, Z heave), incl. coupled:
                            resid=0      drift=0      gap=0   at EVERY amplitude
pure roll (yaw):
   yaw  2.4 deg             resid 0.05   drift +0.105
   yaw  9.7 deg             resid 0.85   drift +1.669
   yaw 19.6 deg             resid 3.46   drift +6.543
   yaw 30.3 deg             resid 8.12   drift +14.270
coupled roll+thrust+fwd:    error == the pure-roll error  (translation adds 0)
drift vs closed form sqrt(119^2+(2d)^2)-119:  match to 4 decimals
```

Three sharp, honest conclusions:

1. **Translation is exactly rigid**, even when coupled with anything else, at all
   amplitudes — the L½ datum extends *exactly* along the heave+stroke plane.
2. **Rotation is a pure first-order linearisation**, and its error is *not a
   mystery*: it equals the closed-form chord-minus-arc of a yaw at radius 59.5 to
   4 decimals.  The firmware holds the pivots' x fixed and offsets only y; the
   true rigid body rides the arc.  The gap is geometry, fully accounted for.
3. **The decoupled control law superimposes linearly** — coupled-input error
   equals the single rotational term because every translational term is exact.
   So the entire non-rigidity of the SR6 control law is one scalar: the yaw
   chord-arc defect.

Visual proof: `anchors/sr6/rigid_consistency.png` — left, the chord (firmware)
vs arc (rigid) in top-down X-Y as yaw grows; right, measured drift dots sitting
exactly on the closed-form curve, with translation drift flat at zero.

### Why this matters for the universal architecture

This is the L5 answer to "is my assembled+actuated model *true*?", phrased as
predictive coding and answered with an honest residual:

* a controller is an **exact** datum at its operating point (RMS 0.014 mm),
* and a motion model that is **exact for the DOF it represents linearly**
  (translations) and **first-order for the DOF it linearises** (rotations),
* with the linearisation error **bounded, closed-form, and zero at home**.

The validator that establishes this is mechanism-agnostic: give it home points,
a control law, and the intended rigid motion, and it returns the surprise.  SR6
merely supplies those three things from its firmware.

---

## §8  Generality proof: a SECOND anchor (`anchors/stewart/synthetic.py`)

Every section above lives on the SR6.  A claim of a *general* architecture that
has only ever run on its origin example is a sample of size one (道生一).  So we
added a second, independent anchor (一生二): a synthetic 6-6 Gough-Stewart
platform with arbitrary radii — **zero** SR6 geometry, constants, or firmware.
The **unchanged** `uam.kinematics` validator judges two control laws driving it:

```
EXACT control law (full rotation R), 8 random poses, |trans|<=40mm, |rpy|<=35deg:
    max_resid = 0           max_drift <= 2.84e-14 mm   (machine epsilon)

LINEAR control law (R ~ I + [w]x), pure roll, growing angle:
    angle    measured drift   exact closed form sqrt(|d|^2+|w x d|^2)-|d|   |diff|
      2 deg     0.07060            0.07060                                  0
     10 deg     1.75218            1.75218                                  0
     30 deg    14.92762           14.92762                                  0
    per-pair max |measured - closed form| @ 20 deg = 1.4e-14 mm
```

Two conclusions, both on a machine with no SR6 DNA:

1. **The validator is honest.** A genuinely rigid control law drifts by machine
   epsilon even under large arbitrary 6-DOF rotation.  Therefore the SR6's
   non-zero rotation drift (§6, §7) is a real property of *its firmware's
   linearisation*, not an artefact of how we measure.  The yardstick is true.
2. **The drift law is universal.** A first-order control law drifts by exactly
   `sqrt(|d|^2 + |w x d|^2) - |d|` — the very chord-minus-arc signature the SR6
   showed (`sqrt(119^2+(2d)^2)-119`), now reproduced to 1e-14 on an unrelated
   mechanism.  Leading term `½ θ² |k x d|² / |d|`: the quadratic growth.

> **反者道之動 in passing.** The first run *failed* an assertion: measured drift
> disagreed with my "closed form" by 0.2 mm at 20°.  The discrepancy was the
> signal — it showed my *formula* was the truncated leading term, while the data
> was exact.  Replacing the formula with the full `sqrt(...)` closed form
> collapsed the gap to 1e-14.  The residual told the truth; I corrected the
> model, not the measurement.

**Architectural payoff.** "Inter-point distance drift measures the linearisation
ORDER of a control law, independent of the mechanism" is now established on two
machines.  The validator (`uam/kinematics.py`) and the datum layer
(`uam/datum.py`) carry no SR6 specifics; SR6 and the Stewart platform are both
thin anchors supplying (home points, control law, intended motion).  This is the
general layer showing through the instance — 大制無割.

## §9  The general SOLVER assembles BOTH machines (`assemble_full.py`, `stewart/assemble.py`)

§7–§8 proved the *validator* (L5) is mechanism-agnostic.  This section proves the
*mate solver* (L2/L3, `uam/assembly.py`) is too — the layer whose **absence** was
the root cause of every prior "装不起来" (see `ROOT_CAUSE.md`).  Earlier closures
hand-rolled per-leg trigonometry; here nothing is hand-solved.  Each machine is
declared as a **mate graph** (parts carrying connectors + constraints) and handed
to the *same* `uam.assembly.solve`, which knows nothing about either mechanism.

**SR6 — `anchors/sr6/assemble_full.py`** (6 legs, declared, not hand-derived):
```
each leg:  ground.shaft --(Distance = arm len)--> rod.s
           rod.r --(Coincident)--> receiver.pivot     (rod = rigid body, len baked)
result:    general-solver constraint RMS = 2.2e-12 mm
           main legs arm_tilt -0.0deg (horizontal)  |  pitch legs +8.5deg
           — independently reproduces the bespoke closure_firmware_6leg.py tilts
```

**Stewart — `anchors/stewart/assemble.py`** (6-6 platform, no SR6 DNA):
```
platform = one free rigid body w/ 6 joints; ground = 6 fixed base anchors
constraints:  Distance(platform.Pi, ground.Bi) == leg_len_i   (the 6 leg lengths)
home closure:        solver RMS = 0,  joint err = 0
forward kinematics:  recover full 6-DOF pose from leg lengths ALONE,
                     worst joint error over 6 random poses = 1.3e-11 mm
```

**Two regimes, one engine.** A real mechanism carries redundant freedom.  The SR6
rod-ends are spherical, so a leg keeps a genuine swing-about-its-axis DOF: the
constraint system is **underdetermined** (24 residuals < 42 vars).  A Stewart
platform pinned by six leg lengths is **exactly determined** (6 = 6).  `solve`
now picks its method adaptively — `trf` when underdetermined, `lm` when square —
so the kernel is sound across both.  This was a real generalisation of the
solver, made *because* the SR6 demanded it (P1: the leftover DOF is physical, a
spherical bearing's freedom, not a fudge factor — so it is left free, not faked
away).

**What this closes.** The architecture no longer rests on one hand-tuned script.
A new machine is onboarded by supplying *data only* — its datum points, its
joints, and which connects to which — and the general L½/L2/L3/L5 layers do the
rest.  道生一 (SR6) → 一生二 (Stewart) → the layers are the 道, the anchors merely
器.  大制無割.

---

## §10 The machine becomes DATA — `uam/spec.py` (大制無割)

§9 still left each mechanism as a Python `build()` function.  §10 removes even
that: a machine is now a plain dict (`*.spec.json`) of *parts → connectors*,
*world anchors*, and *constraints*.  One generic loader (`uam/spec.py`) turns it
into the exact `(parts, constraints)` the solver consumes — there is **no
mechanism-specific code path anywhere** below the data file.

```
uam/spec.py    load(dict) -> Spec(parts, constraints)         # the only code
anchors/sr6/sr6.spec.json          7 parts, 12 constraints    # pure data (器)
anchors/stewart/stewart.spec.json  2 parts,  6 constraints    # pure data (器)
```

`anchors/assemble_from_spec.py` imports *only* the loader + solver and feeds it
both data files:

```
SR6 (real 6-leg home)         solver RMS = 1.2e-11   worst constraint resid = 3.3e-11  CLOSES
Stewart (synthetic 6-6 home)  solver RMS = 2.2e-14   worst constraint resid = 2.8e-14  CLOSES
```

Constraint endpoints in the spec are uniform and declarative:
`["part","conn"]` references a connector, `"world.NAME"` a named anchor, a bare
`[x,y,z]` a literal point/direction.  Adding a third machine now means writing a
JSON file — no import, no `build()`, no trig.  器 (the machine) is pure data;
道 (the loader+solver) is the only code, and it is machine-blind.  This is the
operational form of the project's whole claim: *let an agent assemble a complex
mechanism by declaring its relationships, not by hand-coding its geometry.*

---

## §11 The claim, falsified the only way that counts — a machine written by hand (反者道之動)

§10 proved the loader+solver are machine-blind, but the two specs it ate were
**serialized from existing `build()` code** — the geometry had already lived in
Python.  A skeptic's honest objection: of course a round-trip closes; the spec
is just a transcript.  The real claim — *"onboarding a machine = authoring a
spec, with zero new code"* — is only tested when the machine has **never existed
in the code at all**.

So I typed a brand-new mechanism by hand: `anchors/fourbar/fourbar.spec.json`,
a planar four-bar (ground 120, crank 40, coupler 70, rocker 70).  Nothing about
a four-bar appears anywhere in this repo's code.  Each revolute joint is
declared as the two primitives the kernel already had — `Coincident` (shared
hinge point) + `Parallel` (shared `z` hinge axis, which also keeps the loop
planar).  The seeds are a deliberately **open** configuration, so the solver has
to actually close the loop, not sit on a pre-closed answer.

```
Four-bar (hand-written, novel)   parts=4 cons=8   solver RMS = 1.1e-14
  ground link (A-B)  authored=120.00  solved=120.0000  err=+0.0e+00
  crank              authored= 40.00  solved= 40.0000  err=-5.6e-11
  coupler            authored= 70.00  solved= 70.0000  err=-5.2e-12
  rocker             authored= 70.00  solved= 70.0000  err=-2.2e-10
  planarity (max |z|) = 6.4e-28
  solved crank angle  = 72.37 deg
```

Closure here is checked the way a *linkage* demands, not just by constraint
residual: every rigid link keeps its authored length (≤2e-10 mm drift), and the
whole loop stays planar (|z| ~ 1e-28).  The solver settled on crank = 72.37° —
**not** the 60° the geometry was originally figured at — because a four-bar has
one real degree of freedom and `trf` slid along that 1-DOF curve to the closed
branch nearest the open seed.  That free angle is the mechanism's genuine
mobility (principle P1: every free DOF must be physical), so it is *found*, not
fixed.

The kernel never knew four-bars existed.  道生一 (SR6) → 一生二 (Stewart) →
三生萬物: the third machine arrived as nothing but a text file, and assembled.
The architecture's whole purpose — *an agent builds a complex assembly by
declaring relationships, not by hand-coding geometry* — is now demonstrated on a
machine whose first and only existence was its declaration.

---

## §12 — The pitch leg's "185 vs 175" is resolved by perception, not by a chosen number

The one honest open frontier left after §11 was the pitch leg.  Every other
closure used either firmware-authority or perceived geometry, but the pitch rod
carried a number I could never fully justify: the physical `PitcherLink` spans
**185 mm** between bearing centres, while the firmware's planar IK uses an
effective rod of **175 mm**.  Earlier sessions hand-waved this as "the L-shape
folds the rod", which is exactly the kind of unverified story principle P5
forbids.  So I read the answer straight off the printed parts with L0 perception
(`anchors/sr6/probe_bellcrank.py`, using `uam.cylinders` — no new perception
code), and the geometry settles it with zero tuning:

```
LPitcher (servo lever)
  pivot-seat A = [-7.5  30.0  46.4]      (revolute about z)
  lever tip  C = [-39.74 97.72 50.25]
  arm A->C = 75.099  mm                  (== firmware PITCH_ARM 75)

PitcherLink_Alpha (the rod)
  end-1 = [ 63.25 -175.0  0.0]           both ends revolute about x
  end-2 = [  3.25    0.0  0.0]           (pins PARALLEL)
  3D centre distance      = 185.000      (the physical 185)
  offset ALONG pin axis   =  60.000      (lateral — sits on the pin axis)
  lever PERP to pin axis  = 175.000      (== firmware effective rod 175)
  hypot(175, 60)          = 185.000
```

The mechanism is not a mysterious "folded" rod.  `PitcherLink`'s two bearing
bores are **revolute and parallel**, both about the lateral (x) axis.  A planar
linkage living in the sagittal (pitch) plane only feels the component of the
link **perpendicular to its pin axis**; the 60 mm that makes 185 differ from 175
lies **along** the pin axis (a lateral stagger so the rod clears its neighbour),
and a parallel-pin planar loop is blind to motion along the pin.  So the
kinematically real lever is the perpendicular projection — exactly 175, exactly
the firmware number.  185 and 175 were never in conflict; they are the 3-D
centre-distance and its in-plane projection of the **same** measured part.

Two architectural lessons, both general (not SR6 trivia):

1. **For a revolute link, the kinematic length is the projection perpendicular
   to the pin axis, not the 3-D centre distance.**  A pure `distance` constraint
   (which would pin 185) is the *wrong* primitive for a parallel-pin planar
   loop; the right reading falls out automatically once the joint is modelled as
   revolute.  This is the pitch-leg analogue of the four-bar's `Coincident +
   Parallel` revolute encoding from §11.

2. **An "L-shaped" or otherwise bent rigid link is, for a ball/parallel-pin
   loop, identical to the straight link through its joint centres.**  Material
   between the bearings matters for *interference*, never for *loop closure*.
   So `assemble_full`'s straight-185 placeholder was geometrically sound for
   closure all along — what was missing was the *proof* that it was sound, which
   perception now supplies.  The bend only needs to re-enter when we add an L5
   interference check, not for kinematics.

The pitch leg therefore has the **same arm+rod topology as the four main legs**
(servo arm `LPitcher` = 75, rod `PitcherLink` with in-plane lever 175), not a
special bellcrank kinematic class.  No fabricated geometry remains anywhere in
the SR6 anchor: every length is now either a firmware-authority constant or a
number perceived from a printed part, and the only "conflict" turned out to be
two honest projections of one measured body.

---

## §13 — Motion validation: the firmware control law is an honest rigid-body solver, exact for translation and a bounded linearisation for rotation

Home closure (§8–§12) proves the assembly is *consistent at one pose*. It cannot,
by itself, distinguish a true kinematic model from a lucky static fit — a straight
rod and an L-bellcrank are identical at rest (§12). The discriminant is **motion**:
drive the firmware control law across the full workspace and ask whether the four
reconstructed receiver pivots move as one rigid body.

`anchors/sr6/motion_full.py` does exactly this, reusing only authority-backed data
(receiver pivots perceived in §8; servo→world embedding from the firmware home IK,
Kabsch-validated to 0.014 mm in §8) and the **mechanism-agnostic** L5 validator
`uam.kinematics.consistency`. It introduces **no fitted DOF**: every pivot is
placed by the firmware's own `SetMainServo`/`SetPitchServo` equations, and the only
reported quantity is `max|Δ pairwise-distance|` vs home — the frame-free rigidity
invariant (a true rigid body preserves all six pairwise distances exactly).

```
  fwd  = ±3000    max|dist drift| = 0.0000 mm     (pure z translation, EXACT)
  thrust = ±6000  max|dist drift| = 0.0000 mm     (pure y translation, EXACT)
  pitch = +2500   max|dist drift| = 2.05 mm       (~25°, rotation about main axis)
  pitch = -2500   max|dist drift| = 18.21 mm      (asymmetric: servo-arc geometry)
  roll (main-only) drift = 0.42 / 1.67 / 6.54 / 14.27 mm at roll = 500/1000/2000/3000
```

**Readings (nothing here was hand-tuned; the bug that first inflated `thrust` to
44 mm was a `copysign(horiz, sy)` sign-discard in my reconstruction, corrected to
`sign(sy)·horiz` — 反者道之動: I changed my code, not the data):**

1. **Translation is exact.** `fwd` shifts every pivot equally in z; `thrust` shifts
   every pivot equally in −y. All six pairwise distances are preserved to
   floating-point. The decoupled control law is a *mathematically exact* rigid-body
   command for the two translational DOF, and the §8 home datum therefore extends
   rigidly across the entire translational workspace — not an approximation.

2. **Pitch is a real rotation about the main-pivot axis.** Pure pitch leaves the
   two main pivots fixed (their servo args don't depend on `pitch`) and swings the
   two pitch pivots. That is precisely a rigid rotation about the `main_L–main_R`
   line. The residual (2 mm at +25°, 18 mm at −25°) is the gap between the servo's
   nonlinear arc and the exact rigid arc; its strong asymmetry comes from the pitch
   arm's `5500·sin(15°±θ)` geometry, **not** from any modelling freedom.

3. **Roll is a first-order linearisation, and the residual proves it.** The main
   subsystem moves its two pivots oppositely in y at constant x,z (a shear); a true
   rotation would also pull them inward in x. The held distance therefore drifts as
   `√(119² + (0.02·roll)²) − 119`, which grows **quadratically** (≈4× per doubling:
   0.42 → 1.67 → 6.54 → 14.27). Quadratic growth from zero at home is the signature
   of an exact first-order linearisation — the same chord-vs-arc law the synthetic
   Stewart anchor reproduced to 1e-14 in §10, now confirmed on real firmware.

4. **An honest, explicitly-deferred gap.** Under roll the firmware feeds the pitch
   servos a lateral term `z = −1.5·roll`. *Which world axis that offset rides* is
   not pinned by the firmware (it is a CAD-authority datum, P3/P4). Rather than pick
   a sign to make a pretty number, `motion_full` restricts the roll test to the
   unambiguous main subsystem and flags the pitch-under-roll embedding as a standing
   CAD dependency. (The first run *did* leak this ambiguity into a contaminated,
   suspiciously-linear full-4-pivot roll drift; recognising that the growth was
   linear-not-quadratic is what exposed the leak.)

**Architectural conclusion.** The firmware control law is a genuine rigid-body
solver, not a hand-fit: exact for translation, and a bounded, computable,
home-anchored linearisation for rotation. The mechanism-agnostic validator returns
these residuals with zero knowledge of the SR6 — the same invariant judged the
synthetic Stewart (§10) and the hand-written four-bar (§11). Motion does **not**
require the bellcrank model (§12): the straight-rod placeholder reproduces every
pose to the firmware's own linearisation order. The bend re-enters only for an L5
*interference* check, never for closure. The 5-layer architecture now carries one
machine from raw STL through perception, datum, generic mate closure, and
full-workspace motion validation with no mechanism-specific solver code anywhere.

---

## §14 — The prismatic lower pair, and a 4th machine that needed it

The four primitives the kernel shipped with (`Coincident`, `PointAt`,
`Parallel`, `Distance`) can express every joint in SR6, the Stewart platform and
the four-bar — because all of those are built from spherical and revolute pairs
plus rigid links. They **cannot** express a *slider*: a point free to translate
along one axis while pinned in the other two. No combination of "coincide here",
"pin to that world point", "be parallel" or "hold this distance" forbids exactly
two translational DOF and permits exactly one.

So the kernel grew a fifth primitive — `PointOnLine` — once, in the shared
library (`uam/assembly.py`), **not** per machine:

```python
@dataclass
class PointOnLine:           # the prismatic / slider lower pair
    a: tuple                 # (part, connector) whose origin must ride the line
    line: object             # ((3,)point0,(3,)dir)  OR  (part, connector)
    def residual(self):
        p  = self.a[0].world_point(self.a[1])
        p0, d = <resolve fixed tuple OR moving part-connector>
        d  = d / |d|
        w  = p - p0
        return w - (w·d) d   # (3,) perpendicular gap: 0 along d, rank-2
```

The residual is the component of `P − p0` perpendicular to the line direction.
It is identically zero along `d` (the permitted travel) and rank-2 transverse,
so it removes precisely the two DOF a slider forbids — no more, no less. The
line may be a fixed `(point, dir)` tuple *or* a `(part, connector)` whose origin
and axis ride a moving part (a slider on a rocking guide), with no code change.

To check the primitive on a machine that **never existed in code**, a planar
slider-crank was hand-authored as pure JSON
(`anchors/slidercrank/slidercrank.spec.json`): ground crank-pivot at the origin,
crank `r=30`, connecting rod `L=90`, the rod's wrist-pin sliding on the ground's
x-axis. Two revolute joints (`Coincident` + `Parallel` about z) and one slider
(`PointOnLine`, rod.b on the x-line). 1 real DOF (crank angle); the seed pose is
deliberately **open** so the solver must close the loop rather than sit on it.

The same machine-blind loader+solver closes it: **worst constraint residual
1.7e-10, RMS 4.1e-11**, the crank settling on the closed branch nearest the open
seed (P1 — that free angle is the mechanism's one true DOF, solved-for not
filled-in). Four unrelated machines now assemble from data alone with the *same*
code, and onboarding a joint type the library lacked cost one dataclass shared by
all machines — never a per-machine special case. 道生一,一生二,二生三,三生萬物。
