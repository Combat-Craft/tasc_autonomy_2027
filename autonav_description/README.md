# TASC rover description

ROS 2 Jazzy package for the six-wheel rover in `MAIN/THIS_MAIN.SLDASM`. It includes the chassis, three passive suspension pivots, six spin joints, three RGB cameras, the Orbbec Gemini 2, the Slamtec RPLIDAR A1M8, an IMU, and a GPS receiver.

## Build on Ubuntu 24.04

Clone the team repository into your ROS 2 workspace's `src` directory and run these commands from the workspace root. The SolidWorks files are only needed when referencing the original geometry.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui
colcon build --symlink-install --packages-select autonav_description
source install/setup.bash
```

## Inspect in RViz

```bash
ros2 launch autonav_description rover.launch.py
```

Move `left_front_rocker_joint`, `right_front_rocker_joint`, and `rear_bogie_joint` in the joint-state GUI. Each front rocker moves its front and middle wheel together. The rear beam rolls and moves both rear wheels. The six wheel joints spin independently. Enable **Frames** in RViz to inspect the sensor transforms.

The launch node publishes the expanded URDF on the transient-local `/robot_description` topic. RViz resolves the installed chassis and tread mesh assets from the package. Use `joint_gui:=false` when consuming joint states from hardware or another ROS node. Add `rviz:=false` to run the publisher alone.

## Test in Gazebo Harmonic

Close the RViz-only launch before running the simulation.

```bash
ros2 launch autonav_description rover.launch.py gazebo:=true
```

The world includes flat ground, a ramp in the left wheel track and a target for the camera and lidar. A six-wheel skid-steer plugin accepts `/cmd_vel`. All suspension joints move under contact forces. Gazebo publishes `/joint_states`, which drives the same model in RViz.

Open another terminal and source Jazzy and the workspace overlay before running these checks.

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /scan --field header
ros2 topic echo --once /imu/acc_gyro
ros2 topic echo --once /gps/fix
ros2 topic echo --once /orbbec/depth/image_raw --field header
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.25}}'
```

Watch the left front rocker move as the rover approaches the ramp. Stop the publisher with Ctrl+C and send a zero command with `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`. In RViz, enable **RPLIDAR** or add an Image display for any camera. Set Image reliability to **Best Effort** if needed.

If Gazebo opens a blank window in VMware, try the same XWayland and software-rendering settings that worked during initial VM testing.

```bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 launch autonav_description rover.launch.py gazebo:=true rviz:=false
```

Use `server_only:=true rviz:=true` to run Gazebo physics with RViz as the viewer.

| Simulated sensor | ROS topics | Model frame |
|---|---|---|
| Forward, rear and roof RGB cameras | `/front_camera`, `/rear_camera`, `/top_camera` with `image_raw` and `camera_info` | Each camera's `*_optical_frame` |
| Orbbec Gemini 2 | `/orbbec/color/image_raw`, `/orbbec/depth/image_raw`, `/orbbec/depth/camera_info`, `/orbbec/points` | `orbbec_optical_frame` |
| RPLIDAR A1M8 | `/scan` | `laser_frame` |
| IMU | `/imu/acc_gyro` | `gps_imu_link` |
| GPS | `/gps/fix` | `gps_link` |

The Orbbec simulation uses a single idealized optical centre for RGB and depth. Hardware camera calibration, hardware drive control, and localization belong to the rover's autonomy stack.

## Change the model

| File | What to change |
|---|---|
| `urdf/dimensions.xacro` | Chassis dimensions, axle and shaft coordinates, wheel size, mass estimates, and suspension travel |
| `urdf/chassis.xacro` | Chassis visuals, collisions, and inertia placement |
| `urdf/suspension.xacro` | Rocker profiles, rear beam, joints, and wheel geometry |
| `urdf/sensor_mounts.xacro` | Sensor parent link and mount pose in metres and radians |
| `urdf/sensors.xacro` | Sensor housings, optical frames, and simulated specs |
| `urdf/gazebo.xacro`, `config/bridge.yaml` | Gazebo model systems and ROS topic mapping |

The model uses +X forward, +Y left and +Z up. `base_link` is at the centre of the chassis footprint, on its bottom plane. Shaft axes, axle spacing, chassis envelope, and tire dimensions came from the supplied assembly transforms and part geometry. The saved rear-bogie pose was levelled around its longitudinal shaft. The assembled parts are represented by simplified visuals and primitive collision shapes. CAD gives the locations of the pivots, while the masses, damping, and stops are simulation estimates. Measure the physical sensor mounts and joint travel before using the transforms for hardware calibration.

The CAD shows two side pitch pivots and one shared rear roll pivot. This topology differs from the two serial rocker-and-bogie chains in NASA JPL's Open Source Rover. The package follows the supplied assembly. Its Xacro modules follow the team's Giskard reference structure and use Gazebo Sim systems for Harmonic.

- [Team Giskard description](https://github.com/Combat-Craft/tasc_autonomy/tree/artemis/giskard_description)
- [JPL Open Source Rover Xacro](https://github.com/nasa-jpl/osr-rover-code/blob/master/ROS/osr_gazebo/urdf/osr.urdf.xacro)
- [Sawppy-based Curio URDF and SDF](https://github.com/srmainwaring/curio/tree/master/curio_description/urdf)
- [Gazebo Harmonic sensor setup](https://gazebosim.org/docs/harmonic/sensors/)
- [Slamtec A1M8 datasheet](https://www.slamtec.ai/wp-content/uploads/2023/11/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf)
- [Orbbec Gemini 2 specifications](https://www.orbbec.com/products/structured-light-camera/gemini-2/)
