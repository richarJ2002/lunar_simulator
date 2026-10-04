# Lunar Simulator

`lunar_simulator` is a proving ground for Mars-mission rover architectures:
autonomy stacks (localisation, control, supervision) developed and tested in
simulation first, on the Moon today and on Mars surfaces next.

> Code is truth — this README points; the source and YAML on `main` win on
> conflict. Details live in the docs site, not here.

## Systems x environments

| System | Environment | Launch command | Status |
|--------|-------------|----------------|--------|
| Alpha | `lunar_surface` | `./scripts/launch_simulator.sh lunar_surface alpha` | Supported |
| Alpha | Mars | TODO | Planned, not present |

Alpha is the only system: an ExoMars-scale rover (triple-bogie, six
independently driven/steered wheels, front/mast stereo cameras) composed as a
single `alpha_node` process. The only world is `worlds/lunar_surface.sdf`
(low gravity, 1.62 m/s²).

## Prerequisites

Use the same isolation values in every terminal (defaults shown):

```bash
export ROS_DOMAIN_ID=73
export GZ_PARTITION=lunar_simulator_73
```

`ROS_DOMAIN_ID` isolates the global `/clock` topic; `GZ_PARTITION`
independently isolates Gazebo Transport. Explicitly exported values override
both defaults. Requires ROS 2 Jazzy and Gazebo Harmonic.

## Quick start

Three commands, from the repository root:

```bash
# 1. Build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lunar_simulator

# 2. Launch (Alpha on lunar_surface; a bare call is equivalent)
./scripts/launch_simulator.sh lunar_surface alpha

# 3. Wait for READY before commanding motion
python3 scripts/wait_for_system_ready.py --timeout-s 120
```

Motion commands are obeyed only while `/alpha/system/state` is `READY` with a
fresh heartbeat; everything published earlier is dropped, never replayed. See
[command gate](docs/systems/alpha/drivers.md) and
[supervisor](docs/systems/alpha/supervisor.md).

Drive with per-wheel `actuator_msgs/msg/Actuators` on
`/alpha/control/cmd/wheel_joint_states` or higher-level `geometry_msgs/Twist`
on `/alpha/control/cmd/velocity` — wheel order, examples, and tuning live in
the [drivers](docs/systems/alpha/drivers.md) and
[controller](docs/modules/control/ackermann-controller.md) pages.

## Documentation

Full documentation lives in `docs/` (MkDocs Material site):

| Page | Path |
|------|------|
| Quickstart | [docs/quickstart.md](docs/quickstart.md) |
| Architecture | [docs/architecture.md](docs/architecture.md) |
| Modules | [docs/modules/index.md](docs/modules/index.md) |
| Systems / Alpha | [docs/systems/alpha/index.md](docs/systems/alpha/index.md) |
| Launch and bridge | [docs/launch-bridge.md](docs/launch-bridge.md) |
| Post-processing | [docs/post-processing.md](docs/post-processing.md) |
| Compliance | [docs/compliance/index.md](docs/compliance/index.md) |
| TODO (Mars, second system) | [docs/todo.md](docs/todo.md) |
| Contributing docs | [docs/contributing-docs.md](docs/contributing-docs.md) |

Project site: <https://richarJ2002.github.io/lunar_simulator> (live).

## What moved

Launch sequencing, post-processing reports, verification steps, command-gate
semantics, wheel order, Twist examples, and tuning parameters used to live
here; they now live in the pages above. No topic names, QoS settings, or
tuning numbers are duplicated in this file — follow the links, then the code.
