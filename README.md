# Lunar Rover Simulator Systems

| System | Node | Topics | Description |
|--------|------|--------|-------------|
| **alpha_system** | `alpha_node` | `/alpha/cmd_wheel_joint_states` (`actuator_msgs/Actuators`, ROS->Gazebo) <br> `/localisation/ground_truth/{odometry,path}` <br> `/localisation/kalman_filter/{odometry,path}` <br> `/alpha/loccam/{left,right}` (images) | ExoMars-scale Alpha rover with triple-bogie, independently walking/steering/driven wheels and front/mast stereo cameras. Localisation owns ground truth while the Alpha node publishes only the estimated path and TF. |

## How to Launch

```bash
# Alpha is the default; both commands are equivalent
./scripts/launch_simulator
./scripts/launch_simulator --system alpha

# Launch alpha system only
ros2 launch lunar_simulator launch/alpha_system_launch.py
```

## Adding New Systems

To add a new rover system (e.g., gamma_system):
1. Create `src/systems/gamma_system/gamma_node/src/gamma_node.cpp`
2. Create `src/systems/gamma_system/gamma_node/package.xml`
3. Add `add_subdirectory(src/systems/gamma_system)` to main CMakeLists.txt
4. Create `launch/gamma_system_launch.py`
5. Update README.md table

To add new functionality to existing system:
- Add publishers/subscribers to the node .cpp file
- Update package.xml with new dependencies
- Update ros_gz_bridge.yaml if new topics needed
- Independent from other systems

## Verification Steps

1. `colcon build --packages-select lunar_simulator`
2. `source install/setup.bash`
3. `ros2 launch lunar_simulator launch/alpha_system_launch.py`
4. Verify Gazebo launches with lunar world (low gravity = 1.62 m/s²)
5. Verify Alpha ground truth and estimate publish odometry and path topics
6. Test: publish an `actuator_msgs/msg/Actuators` command to `/alpha/cmd_wheel_joint_states`, then verify joint states and odometry

### Alpha wheel command

`/alpha/cmd_wheel_joint_states` accepts one `actuator_msgs/msg/Actuators`
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
ros2 topic pub --once /alpha/cmd_wheel_joint_states actuator_msgs/msg/Actuators \
  "{velocity: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], position: [0.35, 0.28, 0.0, 0.0, 0.0, 0.0]}"
```
