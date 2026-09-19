# Lunar Rover Simulator Systems

| System | Node | Topics | Description |
|--------|------|--------|-------------|
| **alpha** | `alpha_node` (single process; composes `continuous_ekf`, `alpha_driver_node`, and every other node below) | `/alpha/control/cmd/wheel_joint_states` (`actuator_msgs/Actuators`, public control input, perturbed with configured actuator noise and forwarded by `alpha_driver_node` to the raw Gazebo bridge) <br> `/alpha/localisation/ground_truth/{odometry,path}` <br> `/alpha/localisation/kalman_filter/{odometry,path}` <br> `/alpha/loccam/{left,right}` (images) | ExoMars-scale Alpha rover with triple-bogie, independently walking/steering/driven wheels and front/mast stereo cameras. Localisation owns ground truth while `continuous_ekf` publishes the estimated path and TF alongside its fused odometry. |

## How to Launch

```bash
# Alpha is the default; both commands are equivalent
./scripts/launch_simulator.sh
./scripts/launch_simulator.sh lunar_surface alpha

# Launch every node the Alpha system owns, plus Gazebo and the ROS bridge,
# standalone (scripts/launch_simulator.sh instead delegates just the node
# launching to this same file, after handling Gazebo spawn/pause/unpause
# itself)
ros2 launch lunar_simulator launch/alpha_launch.py
```

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
  "{linear: {x: 1.0, y: 0.0}, angular: {z: 0.3}}"
```
