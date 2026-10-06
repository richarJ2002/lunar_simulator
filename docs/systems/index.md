# Systems

> Code is truth — values below describe; YAML/source linked wins on conflict.

Purpose: one rover system is one composing entry point plus its own model,
parameters, bridge config, and launch file. Alpha is the only system today;
this page is the checklist for adding the next one (e.g. gamma), trimmed
from README "Adding New Systems".

Where: `src/systems/alpha/`
([GitHub](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/)).

## How to add gamma

1. Create `src/systems/gamma/gamma_node/main.cpp`, following
   `alpha_node/main.cpp`'s pattern (compose, `spin()`,
   `rclcpp::init`/`shutdown` around it).
2. Compose only the node classes gamma wants from the static libraries
   under `src/localisation/` and `src/control/`, plus gamma's own
   driver/model nodes, on one owned executor.
3. Create `launch/gamma_launch.py` declaring `system_name` (default
   `gamma`) and passing it to the single node executable.
4. Give gamma its own `parameters/systems/gamma/` tree, bridge config,
   and model; update the README systems table.

Full authority: [README "Adding New Systems"](https://github.com/richarJ2002/space_robotics_simulator/blob/main/README.md)
and [`src/systems/alpha/alpha_node/`](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/alpha_node/).

## `system_name` limits

`system_name` changes node-computed topic defaults only (e.g.
`/<system>/localisation/kalman_filter/odometry`). Launch paths, model and
bridge assets, parameter files, and absolute `/alpha/...` topic overrides
stay Alpha-specific — renaming alone does not create an independent second
system (per repo `AGENTS.md`).
