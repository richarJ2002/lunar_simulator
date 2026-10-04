# Quickstart

> Code is truth — commands below describe; the launcher scripts and launch
> files on `main` win on conflict.

## Systems x environments

| System | Environment | Launch command | Status |
|--------|-------------|----------------|--------|
| Alpha | `lunar_surface` | `./scripts/launch_simulator.sh lunar_surface alpha` | Supported |
| Alpha | Mars | TODO | Planned, not present |

## Prerequisites

Every terminal uses the same isolation values (defaults shown):

```bash
export ROS_DOMAIN_ID=73
export GZ_PARTITION=lunar_simulator_73
```

## Three commands

```bash
# 1. Build (from the repository root)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lunar_simulator

# 2. Launch (Alpha on lunar_surface; bare call is equivalent)
./scripts/launch_simulator.sh lunar_surface alpha

# 3. Wait for READY before commanding motion
python3 scripts/wait_for_system_ready.py --timeout-s 120
```

Motion commands are obeyed only while `/alpha/system/state` is `READY`;
everything published earlier is dropped, never replayed.
