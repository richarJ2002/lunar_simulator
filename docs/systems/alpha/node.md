# Alpha node

> Code is truth — values below describe; YAML/source linked wins on conflict.

Purpose: Alpha's single entry point. It waits out the spawn-settle
transient, then constructs every node in the stack and spins them together
in one process.

Where: `src/systems/alpha/alpha_node/`
([GitHub](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/alpha_node/)).

## Topics

None of its own — each composed node declares its own topics (see the
system subpages). Names only, no QoS claims — see source.

## Key files

| Class / unit | Path | GitHub |
|---|---|---|
| `AlphaNode` | `src/systems/alpha/alpha_node/objects/AlphaNodeClass.h` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/alpha_node/objects/AlphaNodeClass.h) |
| Entry point | `src/systems/alpha/alpha_node/main.cpp` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/alpha_node/main.cpp) |
| Spin | `src/systems/alpha/alpha_node/methods/spin.cc` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/systems/alpha/alpha_node/methods/spin.cc) |

No function signatures in v1 — see source.

## Parameters

None — `main.cpp` takes no ROS parameters. The fixed 5 s sleep before
construction lets the spawned model settle under gravity before inertial
start-up calibration sees any samples; it is a plain constant, not a
tuning knob.

## Executor and callback groups

One `rclcpp::executors::MultiThreadedExecutor` shared by all nine nodes.
Each node keeps its own mutually-exclusive default callback group, so
nodes progress independently while each node's own callbacks stay
serialized (which e.g. the ESKF's internal state requires).

```mermaid
graph LR
    wait[5 s settle] --> construct[construct 9 nodes]
    construct --> exec[one MultiThreadedExecutor]
    exec --> cb[mutually-exclusive group per node]
```
