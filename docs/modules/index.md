# Modules

> Code is truth — values below describe; YAML/source linked wins on conflict.

Three kinds of things live in this repo. Mixing them up is the usual
confusion, so fix the mental model first:

- **Module** — reusable, system-agnostic library under `src/`
  (`localisation`, `control`, `common`). A module never names a world,
  a launch file, or a second system. Alpha composes modules; a future
  gamma system would compose the same ones.
- **System** — one rover stack under `src/systems/<name>/` plus its
  `parameters/systems/<name>/` tuning (see Systems/Alpha — later work
  package). Systems own the ESKF model, the driver, and the supervisor.
- **Environment** — one world under `worlds/` (see Environments — later
  work package). Modules and systems never hardcode an environment.

```mermaid
graph TD
    mod[Modules: localisation / control / common] --> sys[System: alpha]
    env[Environment: lunar_surface] --> sys
    sys --> run[One composed AlphaNode process]
```

## Where to go

- [Localisation](localisation/index.md) — measurement producers plus the
  model-agnostic Kalman engine. The rover-specific ESKF model belongs to
  Systems/Alpha, not here.
- [Control](control/index.md) — Ackermann steering geometry plus the
  display-only estimate filter.
- [Common](common/index.md) — console logging macros and diagnostics
  helpers shared by every node.
