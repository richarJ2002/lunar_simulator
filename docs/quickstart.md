# Quickstart

> Code is truth — commands below describe; the launcher scripts and launch
> files on `main` win on conflict.

## Systems

| System | Status |
|--------|--------|
| Alpha | Supported |

## Environments

| Environment | Status |
|-------------|--------|
| `crater_field` | Supported |
| Mars | Planned, not present |

Launches use `./scripts/launch_simulator.sh [environment] [system]` with defaults `crater_field` + `alpha`, so a bare call is equivalent. See Three commands below.

## Prerequisites

Every terminal uses the same isolation values (defaults shown):

```bash
export ROS_DOMAIN_ID=73
export GZ_PARTITION=space_robotics_simulator_${ROS_DOMAIN_ID}
```

## Three commands

```bash
# 1. Build (from the repository root)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select space_robotics_simulator

# 2. Launch (Alpha on crater_field; bare call is equivalent)
./scripts/launch_simulator.sh crater_field alpha

# 3. Wait for READY before commanding motion
python3 scripts/wait_for_system_ready.py --timeout-s 120
```

Motion commands are obeyed only while `/alpha/system/state` is `READY`;
everything published earlier is dropped, never replayed.
