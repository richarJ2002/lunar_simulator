# Alpha localisation

The localisation stack is split into independent measurement producers and a
continuous-discrete extended Kalman filter. Gazebo and localisation both use
`map` as the global frame. Estimated and ground-truth trajectories are separate
children of `map` and can therefore be compared directly. Linear and angular
twist components in `nav_msgs/Odometry` follow ROS convention and are expressed
in the child (base) frame.

Each package below (`ground_truth`, `inertial_odometry`, `visual_odometry`,
`wheel_odometry`) builds as a static library, not a standalone executable —
the owning system decides which ones it wants and constructs/spins them
itself. For Alpha, that composing class is `AlphaNode`
(`src/systems/alpha/alpha_node/objects/AlphaNode.h`, spun from
`alpha_node/main.cpp` — the single executable this whole system runs),
which links all four alongside its own estimate node and Gazebo driver
interface.
The fusing Kalman filter (`continuous_ekf` in the table below) follows the
same static-library pattern but is not one of these packages: its
rover-specific EKF model is `alpha_kalman_filter`
(`src/systems/alpha/alpha_localisation/alpha_kalman_filter/`), which wraps the
model-agnostic engine at `src/localisation/kalman_filter/
ekf_continuous_kalman_filter/` — see the "System/node layout" section of the
repo-root `CLAUDE.md` for the full split.

| Node | Input | Output |
|---|---|---|
| `alpha_driver_node` | `/alpha/raw/imu`, `/alpha/raw/joint_states` | `/alpha/imu`, `/alpha/joint_states` |
| `inertial_odometry` | `/alpha/imu` | `/alpha/localisation/inertial/odometry`, `/alpha/localisation/inertial/filtered_imu` |
| `visual_odometry` | `/alpha/loccam/left`, `/alpha/loccam/right` | `/alpha/localisation/visual/odometry`, `/alpha/localisation/visual/point_cloud`, `/alpha/localisation/visual/features` |
| `wheel_odometry` | `/alpha/joint_states`, `/alpha/localisation/visual/odometry`, `/alpha/localisation/kalman_filter/wheel_slip_ratio` | `/alpha/localisation/wheel/odometry`, `/alpha/localisation/wheel/slip_ratios`, `/alpha/localisation/wheel/slip_observation` |
| `continuous_ekf` | the three odometry topics above, plus `/alpha/localisation/wheel/slip_observation` | `/alpha/localisation/kalman_filter/{odometry,path,wheel_slip_ratio}` and the estimate TF |
| `ground_truth` | `/alpha/localisation/ground_truth/odometry` | `/alpha/localisation/ground_truth/path` and the truth TF |

Alpha's simulated sensor and actuator-command errors are configured in
`parameters/systems/alpha/alpha_drivers/alpha_drivers.yaml`, and the
fusing Kalman filter's tuning is in
`parameters/systems/alpha/alpha_localisation/alpha_kalman_filter.yaml`.
`alpha_driver_node` adds repeatable Gaussian noise and constant bias to the
IMU, adds Gaussian encoder noise to joint position and velocity, and updates
the IMU covariance fields; it also adds Gaussian noise to the outgoing wheel
command before forwarding it to the raw Gazebo actuator bridge. The random
sequence for all of this is controlled by one shared `random_seed`. LocCam
and NavCam Gaussian pixel noise is applied natively by
Gazebo to avoid relaying the large images through another ROS node;
`scripts/launch_simulator.sh` converts `camera_pixel_stddev` from 8-bit
intensity units to Gazebo's normalized intensity units when it creates the
temporary spawn SDF.
Camera-noise changes therefore take effect on the next simulator launch. Set
`noise_enabled: false` to disable all Alpha sensor noise. Simulator ground
truth bypasses this sensor path and remains noise-free.

The inertial estimator applies a first-order low-pass filter to acceleration
and angular rate, estimates stationary accelerometer/gyro bias during its
startup sample window, integrates the quaternion, removes lunar gravity, and
integrates velocity and position. Pure inertial integration will still drift;
the bias calibration assumes the rover is stationary at launch.

The visual estimator is metric stereo VO. It computes disparity with
`StereoBM`, tracks image corners between LocCam frames with pyramidal Lucas-
Kanade flow, reconstructs the previous 3-D feature positions, and estimates
the inter-frame transform with PnP RANSAC. The configured 800 px intrinsics,
0.15 m baseline, and camera-to-body transform match the Alpha SDF. PnP inlier
features are transformed into `map` and published as a sparse `PointCloud2`.
The cloud and annotated feature image publish for every synchronized stereo
frame, including empty clouds when depth cannot be recovered. Blue points in
the image are detections, green points and yellow tracks are temporal optical
flow, and magenta rings are stereo-valid motion correspondences. The estimator
still withholds odometry corrections when there are too few reliable matches.

Wheel odometry solves all six rolling constraints in least squares, so it
supports arbitrary per-wheel Ackermann steering angles; the solve's lateral
(vy) channel is unobservable whenever all six wheels are near-parallel
(steering angle 0), which `lateral_observability_threshold` correctly floors
to zero instead of amplifying steering-encoder noise into a spurious
velocity (see `CLAUDE.md`'s "Key invariants"). Each wheel speed is scaled by
`(1 - slip_ratio)` before integration. For every reliable visual odometry
update, the node projects the visual body twist onto each wheel's rolling
direction and compares it with that wheel's circumferential speed, and
publishes the resulting median-of-six observation on
`/alpha/localisation/wheel/slip_observation` for `continuous_ekf` to fuse
as an EKF state, rather than blending it into a local estimate itself.
`continuous_ekf` republishes the fused result on
`/alpha/localisation/kalman_filter/wheel_slip_ratio`, which `wheel_odometry`
reads back and applies to all six wheels before its next solve.
`/alpha/localisation/wheel/slip_ratios` publishes the currently-applied
values in front-left, front-right, centre-left, centre-right, rear-left,
rear-right order. The configured `slip_ratios` values are only the startup
prior used until the first fused estimate arrives.

The EKF state is `[position(3), roll/pitch/yaw, linear_velocity(3),
angular_rate(3), wheel_slip_ratio(1)]`. Its nonlinear Euler-rate process
model and covariance are propagated continuously in steps no larger than
20 ms; the slip state is the one exception, mean-reverting toward zero at
`slip_decay_time_constant_s` absent a fresh observation rather than
following pose/twist's kinematics. Inertial, visual, wheel, and slip
messages cause discrete Joseph-form covariance updates. The IMU observes
roll, pitch, and angular rates rather than its drift-prone integrated
position. Wheel updates observe planar velocity and yaw rate; visual
odometry provides the relative pose correction and, deliberately loosely
weighted (`visual_attitude_measurement_variance`), also corrects roll and
pitch: the IMU still dominates moment-to-moment, but its own open-loop
attitude integration drifts slowly and indefinitely absent an independent
check (see above), and stereo VO's per-frame PnP attitude estimate
provides exactly that, anchoring long-run drift without fighting the IMU's
better short-term estimate. This anchor works, but inertial_odometry's own
drift rate is inconsistent run to run and can occasionally outpace it (see
`CLAUDE.md`'s Key Invariants for the current suspected cause and open
follow-ups). All relative measurements are aligned to the configured
initial Alpha pose in `map`. Parameters are in
`parameters/systems/alpha/alpha_localisation/alpha_kalman_filter.yaml`.

The EKF output is a `nav_msgs/msg/Odometry`: `pose` carries position and
attitude, while `twist` carries their linear and angular derivatives. Every
published estimate is also validated and unconditionally re-exposed by the
same node as a retained path and TF (`AlphaKalmanFilterNode::
publishEstimatedPath()`). For TF comparison, `continuous_ekf` publishes the
estimated rover pose and `ground_truth` publishes the true pose beneath the
common `map` frame.
Ground-truth and estimated odometry and paths all retain that frame.
Both trajectories are sampled into bounded `nav_msgs/msg/Path` histories at
10 Hz.
Path publishers are transient-local so a newly opened RViz display receives
the current history immediately. The history length and sampling period are
configurable. Distinct child frames are required because a TF tree cannot
assign two poses to the same `alpha/base_link` at one time.
