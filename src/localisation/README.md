# Alpha localisation

The localisation stack is split into independent measurement producers and a
continuous-discrete extended Kalman filter. Gazebo and localisation both use
`map` as the global frame. Estimated and ground-truth trajectories are separate
children of `map` and can therefore be compared directly. Linear and angular
twist components in `nav_msgs/Odometry` follow ROS convention and are expressed
in the child (base) frame.

| Node | Input | Output |
|---|---|---|
| `alpha_sensor_node` | `/alpha/raw/imu`, `/alpha/raw/joint_states` | `/alpha/imu`, `/alpha/joint_states` |
| `inertial_odometry_node` | `/alpha/imu` | `/localisation/inertial/odometry`, `/localisation/inertial/filtered_imu` |
| `visual_odometry_node` | `/alpha/loccam/left`, `/alpha/loccam/right` | `/localisation/visual/odometry`, `/localisation/visual/point_cloud`, `/localisation/visual/features` |
| `wheel_odometry_node` | `/alpha/joint_states`, `/localisation/visual/odometry` | `/localisation/wheel/odometry`, `/localisation/wheel/slip_ratios` |
| `kalman_filter_node` | the three odometry topics above | `/localisation/kalman_filter/odometry` |
| `ground_truth_node` | `/localisation/ground_truth/odometry` | `/localisation/ground_truth/path` and the truth TF |
| `alpha_node` | EKF odometry | `/localisation/kalman_filter/path` and the estimate TF |

Alpha's simulated sensor errors are configured in `config/alpha.yaml`.
`alpha_sensor_node` adds repeatable Gaussian noise and constant bias to the
IMU, adds Gaussian encoder noise to joint position and velocity, and updates
the IMU covariance fields. The random sequence is controlled by
`random_seed`. LocCam and NavCam Gaussian pixel noise is applied natively by
Gazebo to avoid relaying the large images through another ROS node;
`launch_simulator` converts `camera_pixel_stddev` from 8-bit intensity units to
Gazebo's normalized intensity units when it creates the temporary spawn SDF.
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
supports arbitrary per-wheel Ackermann steering angles. Each wheel speed is
scaled by `(1 - slip_ratio)` before integration. For every reliable visual
odometry update, the node projects the visual body twist onto each wheel's
rolling direction and compares it with that wheel's circumferential speed.
The resulting six slip observations are bounded and low-pass filtered before
being applied to later wheel updates. `/localisation/wheel/slip_ratios`
publishes the current values in front-left, front-right, centre-left,
centre-right, rear-left, rear-right order. The configured values are the
startup priors used before reliable visual motion is available.

The EKF state is `[position(3), roll/pitch/yaw, linear_velocity(3),
angular_rate(3)]`. Its nonlinear Euler-rate process model and covariance are
propagated continuously in steps no larger than 20 ms. Inertial, visual, and
wheel messages cause discrete Joseph-form covariance updates. The IMU observes
roll, pitch, and angular rates rather than its drift-prone integrated position.
Wheel updates observe planar velocity and yaw rate; visual odometry provides
the relative pose correction. All relative measurements are aligned to the
configured initial Alpha pose in `map`. Parameters are in
`config/localisation.yaml`.

The EKF output is a `nav_msgs/msg/Odometry`: `pose` carries position and
attitude, while `twist` carries their linear and angular derivatives. For TF
comparison, `alpha_node` publishes the estimated rover pose and
`ground_truth_node` publishes the true pose beneath the common `map` frame.
Ground-truth and estimated odometry and paths all retain that frame.
Both trajectories are sampled into bounded `nav_msgs/msg/Path` histories at
10 Hz.
Path publishers are transient-local so a newly opened RViz display receives
the current history immediately. The history length and sampling period are
configurable. Distinct child frames are required because a TF tree cannot
assign two poses to the same `alpha/base_link` at one time.
