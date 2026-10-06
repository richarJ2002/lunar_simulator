# Environments

> Code is truth — values below describe; source linked wins on conflict.

Purpose: worlds the rover drives in. `crater_field` is the only
supported environment today; `plain_stub` is an explicit TODO placeholder,
not a second world.

Where: `environment/`
([GitHub](https://github.com/richarJ2002/space_robotics_simulator/blob/main/environment/)).

## Worlds

| World | Status |
|---|---|
| [Lunar crater field](crater-field.md) | Present — Alpha drives here. |
| [Martian plain stub](plain-stub.md) | TODO placeholder — planned, not present. |

## Launcher matrix

`./scripts/launch_simulator.sh [environment] [system]` — defaults are
`crater_field` + `alpha`, so a bare call launches Alpha on crater_field.
Each selector resolves in 3 steps: absolute path (must exist), else
CWD-relative path (if exists), else a bare name under `environment/` (a
world SDF) or `src/systems/` (a rover system).

| Environment | System | Launch command | Status |
|---|---|---|---|
| `crater_field` | `alpha` | `./scripts/launch_simulator.sh crater_field alpha` | Supported |
| `plain_stub` | `alpha` | `./scripts/launch_simulator.sh plain_stub alpha` | Resolves; unsupported stub |
| Mars | TODO | TODO | Planned, not present |
