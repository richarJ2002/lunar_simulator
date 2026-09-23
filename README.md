# Lunar Rover Simulator Systems

| System | Node | Topics | Description |
|--------|------|--------|-------------|
| **alpha** | `alpha_node` (single process; composes `continuous_ekf`, `alpha_driver_node`, and every other node below) | `/alpha/control/cmd/wheel_joint_states` (`actuator_msgs/Actuators`, public control input, perturbed with configured actuator noise and forwarded through `/alpha/drivers/cmd/wheel_joint_states`) <br> `/alpha/localisation/ground_truth/{odometry,path}` <br> `/alpha/localisation/kalman_filter/{odometry,path}` <br> `/alpha/drivers/loccam/{left,right}` (bridged images) | ExoMars-scale Alpha rover with triple-bogie, independently walking/steering/driven wheels and front/mast stereo cameras. Every ROS-Gazebo transport endpoint except the standard `/clock` lives under `/alpha/drivers`; processed sensor and localisation outputs retain their public namespaces. |

## How to Launch

```bash
# The project defaults to DDS domain 73 to isolate the global /clock topic.
export ROS_DOMAIN_ID=73
export GZ_PARTITION=lunar_simulator_73

# Alpha is the default; both commands are equivalent
./scripts/launch_simulator.sh
./scripts/launch_simulator.sh lunar_surface alpha

# Also record the LocCam stereo and annotated feature image topics into
# this run's rosbag (core telemetry is always recorded regardless of this
# flag; see "Post-Processing Reports" below for the disk-space tradeoff)
./scripts/launch_simulator.sh --record-images

# Launch every node the Alpha system owns, plus Gazebo and the ROS bridge,
# standalone (scripts/launch_simulator.sh instead delegates just the node
# launching to this same file, after handling Gazebo spawn/pause/unpause
# itself)
ros2 launch lunar_simulator launch/alpha_launch.py
```

Use the same `ROS_DOMAIN_ID` in every terminal that publishes commands or
inspects topics. `GZ_PARTITION` independently isolates Gazebo Transport from
other local simulations. Explicitly exported values override both defaults.

Each `launch_simulator.sh` invocation creates a timestamped artifact directory:

```text
test_runs/YYYY-MM-DD-HH-mm-SS/
|-- parameters/       # Snapshot of parameters/ at launch time
|-- ros/
|   |-- build_logs/   # Colcon logs
|   |-- logs/         # ROS node logs
|   `-- bags/
|       |-- localisation/     # Automatically recorded rosbag2/MCAP bag
|       `-- manifest.json     # World/system/domain/profile/topics/revision for this recording
|-- logs/
|   `-- terminal.txt  # Complete launcher and simulation terminal output
`-- post_processing/  # Generated report site (see "Post-Processing Reports")
```

The launcher exports `TEST_RUN_DIR`, `ROS_LOG_DIR`, `COLCON_LOG_PATH`, and
`LUNAR_SIMULATOR_ROSBAG_DIR` so child processes and future recording tools use
the same run directory. Every launch automatically records a core telemetry
bag to `ros/bags/localisation`; the launcher exits with an error rather than
completing a run whose recorder failed to start or crashed immediately.

When an interactive simulation ends, the launcher asks for an optional test-run
name. Entering `straight-drive`, for example, renames the directory to
`YYYY-MM-DD-HH-mm-SS-straight-drive`; pressing Enter leaves the timestamp-only
name unchanged. Suffixes may contain up to 100 letters, numbers, periods,
underscores, or hyphens. Non-interactive runs skip this prompt.

Every node's topics default to the `/alpha/...` namespace via a shared
`system_name` launch argument (default `alpha`), declared once in
`alpha_launch.py` and passed to the single `alpha_node` process it
starts. Each node `alpha_node` composes builds its own default topic names
from this parameter (e.g. `AlphaKalmanFilterNode`'s `output_topic` defaults to
`/<system_name>/localisation/kalman_filter/odometry`), so
`ros2 launch lunar_simulator launch/alpha_launch.py system_name:=beta`
reuses the same executable under a `/beta/...` namespace instead —
see "Adding New Systems" for the rest of what a second system still needs
(its own model, bridge config and launch file).

## Adding New Systems

To add a new rover system (e.g., gamma):
1. Create `src/systems/gamma/gamma_node/main.cpp`, following
   `alpha_node/main.cpp`'s pattern (construct a composing class, call its
   `spin()`, `rclcpp::init`/`shutdown` around it)
2. Create `src/systems/gamma/gamma_node/package.xml`
3. Add `add_subdirectory(src/systems/gamma)` to main CMakeLists.txt
4. Create `launch/gamma_launch.py`, following `alpha_launch.py`'s
   pattern: declare its own `system_name` argument (default `gamma`) and pass
   it as a parameter to the single node executable it launches, so the new
   system's topics land under `/gamma/...` without colliding with Alpha's
5. Update README.md table

Each `src/localisation/` package (`ground_truth`, `inertial_odometry`,
`visual_odometry`, `wheel_odometry`) and `src/control/` package
(`ackermann_controller`) builds as a static library, not an executable,
specifically so a new system can pick its own stack: add its own composing
class (e.g. `gamma_node/objects/GammaNode.h`, following `alpha_node`'s
`AlphaNode.h` pattern), construct only the node classes that system wants,
add them to one owned executor, and `target_link_libraries()` just those
library targets — `src/localisation/CMakeLists.txt` and
`src/control/CMakeLists.txt` build all of them unconditionally, so any
subset is always available to link. A new system's own rover-specific
nodes (its own estimate node and Gazebo driver interface, following
`alpha_localisation`/`alpha_drivers`'s pattern)
are their own library subpackages the same way, composed by that one
entry point rather than run as separate executables. Give the new system
its own `parameters/systems/gamma/` tree (mirroring `src/` the same way
`parameters/systems/alpha/` does) for any package whose defaults it needs to
override.

A new system's EKF fusion works the same way but needs one more piece: the
generic filter engines under `src/localisation/kalman_filter/` (currently
`ekf_continuous_kalman_filter`) know nothing about any state layout, so the
new system needs its own model package (like `alpha_kalman_filter` under
`src/systems/alpha/alpha_localisation/`) that evaluates its own process/observation
model and hands the resulting matrices to whichever engine it links — see
the "System/node layout" section of `CLAUDE.md`.

To add new functionality to existing system:
- Add publishers/subscribers to the node .cpp file
- Update package.xml with new dependencies
- Update `config/alpha_ros_gz_bridge.yaml` if new topics needed
- Independent from other systems

## Post-Processing Reports

`post_processing/` is a source-tree Python tool (not a ROS executable) that
turns one captured test run's bag and node logs into a portable, interactive
Plotly HTML report site under that run's own `post_processing/` directory.
Source scripts live in the checkout at `post_processing/`; the generated
site lives with the run's other artifacts at
`test_runs/<run>/post_processing/` — the two `post_processing` paths are
different things with the same name.

Required environment: source both `/opt/ros/jazzy/setup.bash` and this
workspace's `install/setup.bash` first (the tool imports `rosbag2_py`,
`rclpy` and every recorded `*_msgs` package from there, not from pip), then
install the pip dependencies once:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pip install -r post_processing/requirements.txt
```

Generate the complete site (index plus all four subsystem pages) for one run:

```bash
python3 post_processing/post_processing.py --test-run test_runs/<run>
```

Each subsystem also has its own standalone script, generating only its own
page plus the shared assets, without shelling out to the aggregate command:

```bash
python3 post_processing/post_processing_kalman_filter.py    --test-run test_runs/<run>
python3 post_processing/post_processing_visual_odometry.py  --test-run test_runs/<run>
python3 post_processing/post_processing_inertial_odometry.py --test-run test_runs/<run>
python3 post_processing/post_processing_wheel_odometry.py   --test-run test_runs/<run>
```

Every script accepts `--output-dir` (default `<test-run>/post_processing`),
`--bag` (default: the run's own `ros/bags/localisation`),
`--maximum-alignment-gap-s`, and `--maximum-image-frames`; run any script
with `-h` for the full flag reference. Regeneration only overwrites this
tool's own fixed page/asset filenames — an analyst's own files already in
the output directory are left alone.

`--record-images` is opt-in because at 1024x1024/10 Hz the LocCam stereo and
annotated-feature image topics can add multiple gigabytes to a normal system
test; without it, the visual odometry page still renders fully, with a card
explaining how to capture a future run with images included. Sampled frames
are embedded as compressed PNG data URIs (bounded by `--maximum-image-frames`
per topic, default 24) rather than raw pixel data, keeping the visual
odometry page in the low tens of megabytes even with images included.

### Troubleshooting

- **"No default telemetry bag found"**: the run predates automatic
  recording, or its recorder failed to start — check
  `test_runs/<run>/logs/terminal.txt` and
  `test_runs/<run>/ros/bags/rosbag_record.log`. Pass `--bag` explicitly if
  the bag lives somewhere else.
- **"not a valid rosbag2 bag: no metadata.yaml"**: the recording did not
  shut down cleanly (metadata.yaml is only written on a finalized recorder
  exit); the bag's messages may still be readable with `ros2 bag info`, but
  this tool refuses to guess at an unfinalized bag's own metadata.
- **"ROS message type ... is not available"**: `/opt/ros/jazzy/setup.bash`
  and/or `install/setup.bash` were not sourced before running the script.
- **Missing images on the visual odometry page**: the run was captured
  without `--record-images`; the page says so and still renders every other
  section — re-run the capture with `--record-images` to include frames.

## Verification Steps

1. `colcon build --packages-select lunar_simulator`
2. `source install/setup.bash`
3. `ros2 launch lunar_simulator launch/alpha_launch.py`
4. Verify Gazebo launches with lunar world (low gravity = 1.62 m/s²)
5. Verify Alpha ground truth and estimate publish odometry and path topics
6. Test: publish an `actuator_msgs/msg/Actuators` command to `/alpha/control/cmd/wheel_joint_states`, then verify joint states and odometry

### Alpha wheel command

`/alpha/control/cmd/wheel_joint_states` accepts one `actuator_msgs/msg/Actuators`
message. Both arrays must contain exactly six values in this order:

1. front-left
2. front-right
3. centre-left
4. centre-right
5. rear-left
6. rear-right

`velocity` contains the six drive-joint targets in rad/s. `position` contains
the six steering-joint targets in rad. Supplying each steering angle
independently supports Ackermann, crab, and counter-phase steering geometries.

Example—all wheels at 1 rad/s with front-wheel Ackermann-like angles:

```bash
ros2 topic pub --once /alpha/control/cmd/wheel_joint_states actuator_msgs/msg/Actuators \
  "{velocity: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], position: [0.35, 0.28, 0.0, 0.0, 0.0, 0.0]}"
```

### Higher-level velocity command

`ackermann_controller` (`src/control/`) offers a simpler alternative to the
raw per-wheel command above: publish a `geometry_msgs/msg/Twist` to
`/alpha/control/cmd/velocity` and it computes and publishes all six wheels'
steering angles and speeds for you, covering pure Ackermann cornering
(`linear.y == 0`), pure crab translation (`angular.z == 0`), and any
combination:

```bash
ros2 topic pub --once /alpha/control/cmd/velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.015, y: 0.0}, angular: {z: 0.01}}"
```
