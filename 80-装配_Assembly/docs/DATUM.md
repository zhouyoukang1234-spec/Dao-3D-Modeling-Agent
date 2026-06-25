# The Datum Problem — why 3D assembly "never comes together" for an agent

*(architectural finding, this session — the deepest root cause so far)*

## One sentence
Individual part meshes + the firmware's metric constants are **sufficient for
relative structure** (lengths, pivot-to-pivot geometry) but **insufficient for
absolute placement**; the missing ingredient is a **datum** — the assembled
*neutral / home configuration* — which a human acquires for free by *seeing* the
assembled object, and which an agent must explicitly *acquire*, never fabricate.

## The two honest facts that prove it

**Fact A — relative structure is rock-solid (perception ↔ prior agree to 0.03mm).**
The Receiver mesh, perceived independently, places its pitch pivot exactly
`55.0 mm @ 15.0°` from its main pivot. The firmware encodes the identical number
as `5500*sin(0.2618)`. Agreement: **0.034 mm, 0.028°**. Zero fudge. The relative
geometry layer works.

**Fact B — absolute placement is ungrounded.**
Take the firmware-exact home distances (main `163.171`, pitch `176.902` mm) and
the perceived servo mount-screw footprints as anchors; solve for the only free
unknown, the receiver 6-DOF pose:

| imposed receiver pose | closure RMS |
|---|---|
| level / home (identity) | **15.50 mm**  ✗ |
| 78.5° rolled            | **0.005 mm**  ✓ |

The data on hand selects a **78.5° rolled** receiver, not the physical level
home. Reason: the mount-*screw* footprints are **not** the kinematic *pivots*
(arm rotation centres + neutral arm direction). Those live in the assembled
configuration, which is absent from individual part files + the ESP32 solver.

## Why this is *the* root cause of all prior failures
The prior session reacted to exactly this gap by **fabricating** the servo world
coordinates (its own `HALLUCINATION_MAP.md` admits errors up to 53 mm). That is
the only way to get a level assembly out of ungrounded data — invent the datum.
Invented datum ⇒ rods don't meet ⇒ "总是组装不起来". The failure was never
modelling skill; it was an **unacknowledged missing input** silently fabricated.

## The general principle (predictive coding)
Inference from sensory data alone is multimodal / biased: many world
configurations explain the same measurements. Disambiguation requires at least
one **grounding observation** (a prior anchored to the world). In assembly the
grounding observation is the **neutral configuration** of the assembled product.

> Free/ungrounded parameters absorb error and manufacture false agreement.
> A closure metric is honest only when every quantity is either **perceived**
> (from mesh) or a **prior constant** (firmware) or the **datum** (acquired from
> a grounding observation) — and the one remaining unknown is the pose we solve.

## Architecture evolution → add an explicit **Datum layer (L½)**
```
L0 Perception      raw mesh -> cylindrical features            (have)
L½ DATUM           acquire neutral/home config of the ASSEMBLY (NEW, missing)
                   sources, in order of authority:
                     1. assembled CAD (STEP/F3D) — exact
                     2. host kinematics (Ayva/OSR global->per-servo map) — exact
                     3. build-guide assembly images (SR6 PDF) — approximate
                     4. a single grounding photo of the device at home
                   NEVER: invent it from part files alone (= hallucination)
L1 Affordance      features -> function                        (have)
L2/L3 Mates+Solver constraints -> pose, datum-grounded          (have, needs L½)
L4 Kinematics      firmware IK as metric prior                  (have)
L5 Validation      perception-vs-prior residual = surprise      (have)
```

## Next (autonomous)
Acquire the datum from an **authoritative, non-fabricated** source — the
open-source OSR6/Ayva host geometry (which defines the global receiver-pose →
per-servo planar-target mapping, i.e. the servo pivot positions + neutral
orientations) and the SR6 build-guide assembly images — then re-solve closure
with the pose *grounded*, not invented, and report the honest residual.
