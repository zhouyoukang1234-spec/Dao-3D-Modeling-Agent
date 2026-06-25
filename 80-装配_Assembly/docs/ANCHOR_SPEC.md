# Anchor-Spec — declaring a machine as data

A mechanism is a plain dict (JSON/YAML on disk). `uam.spec.load` turns it into
the `(parts, constraints)` that `uam.assembly.solve` consumes. No Python is
written per machine — onboarding a new one means authoring a spec.

```jsonc
{
  "name": "my_machine",

  // Optional named world anchors. Reference them as "world.NAME".
  "world_points": { "piv::main_L": [-59.5, 0.0, 208.48] },

  "parts": [
    {
      "name": "ground",
      "fixed": true,                      // ground never moves
      "mesh": "Base.stl",                 // optional, for rendering only
      "connectors": {
        "shaft::main_L": { "point": [-59.5, 0, 162.48], "axis": [1,0,0] }
      }
    },
    {
      "name": "rod::main_L",
      "seed": { "t": [x,y,z], "q": [w,x,y,z] },   // optional starting guess only
      "connectors": {
        "s": { "point": [0,0,0],   "axis": [1,0,0] },   // arm-end (body frame)
        "r": { "point": [175,0,0], "axis": [1,0,0] }     // receiver-end
      }
    }
  ],

  "constraints": [
    // endpoints: ["part","conn"] | "world.NAME" | [x,y,z] (literal)
    { "type": "distance",  "a": ["rod::main_L","s"], "b": ["ground","shaft::main_L"], "d": 50.0 },
    { "type": "point_at",  "a": ["rod::main_L","r"], "target": "world.piv::main_L" },
    { "type": "coincident","a": ["rod::main_L","r"], "b": ["receiver","pivot_L"] },
    { "type": "parallel",  "a": ["rod::main_L","s"], "b": ["ground","shaft::main_L"] }
  ]
}
```

## Rules that keep it honest
- **Connector points are in the part's body frame.** A `fixed` part's frame is
  the world; everything else is solved.
- **`seed` is a guess, never an answer.** It only sets the solver's starting
  pose; the converged pose is determined by the constraints. Omit it and the
  part starts at identity.
- **Every free DOF must be physical (P1).** If a part keeps a genuine residual
  freedom (e.g. a spherical rod-end's swing), the system is underdetermined and
  `solve` picks `trf`; do not invent a constraint to square it away.
- **Absolute placement comes from authority (P3/P4).** `world_points` and fixed
  connectors must trace to a datum source (firmware IK home, assembly CAD, build
  guide), never a value hand-picked to make a leg close.

## Worked examples
- `anchors/sr6/sr6.spec.json` — real SR6, 6 legs (underdetermined → `trf`).
- `anchors/stewart/stewart.spec.json` — synthetic 6-6 platform (square → `lm`).
- `anchors/assemble_from_spec.py` — loads both, asserts closure, importing only
  the generic loader + solver.
- `anchors/export_specs.py` — regenerates the two JSON files from the canonical
  `build()` geometry.
