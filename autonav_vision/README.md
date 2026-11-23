
# Panorama

## Overview

This ROS 2 Jazzy package implements an automated panorama system using an **Arduino Nano and MG90D servo motor**. The current system commands 12 evenly spaced camera positions from **-120° to +120°**.

The main completed work is migrating the **motor controller and panorama capture nodes from ROS 2 Humble (Python) to ROS 2 Jazzy (C++)**, along with converting the standalone OpenCV stitcher to C++.

**Current status:** The panorama capture node runs in `test_mode: true`, which tests motor movement and the sweep state machine without requiring an IP camera. The IP camera functionality is included in the code but is incomplete and untested. Test mode does not generate image files.

The next major hardware upgrade is replacing the servo with a **NEMA 17 stepper motor**, using an external power supply and motor driver to achieve full 360° rotation.

### ROS 2 Topics

| Topic | Message Type | Description |
|---|---|---|
| `/motor_cmd` | `std_msgs/msg/Float64` | Camera angle command, currently -120° to +120° |
| `/panorama_trigger` | `std_msgs/msg/String` | Starts a panorama sweep using the message as the output folder name |

The panorama capture node publishes to `/motor_cmd` and subscribes to `/panorama_trigger`. The motor controller subscribes to `/motor_cmd`.

Manual commands can be published using Foxglove or the ROS 2 command line.

## 1. Build and Run

### Build the Package

Run from the ROS 2 workspace root:

```bash
cd ~/TASC/tasc_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select autonav_vision
source install/setup.bash
```

### Option #1 - With launch file

**Terminal 1 — Launch Motor Controller and Panorama Capture**

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 launch autonav_vision panorama.launch.py
```

Alternatively, specify the Arduino serial port:

```bash
ros2 launch autonav_vision panorama.launch.py serial_port:=<serial-port>
```

The default serial port is `/dev/nano`.

**Terminal 2 — Trigger Panorama Sweep**

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/String "{data: 'test_sweep_01'}"
```

### Option #2 - Without launch file

**Terminal 1 — Motor Controller**

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 run autonav_vision motor_controller
```

With a custom serial port:

```bash
ros2 run autonav_vision motor_controller --ros-args -p serial_port:=<serial-port>
```

**Terminal 2 — Panorama Capture (Test Mode)**

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 run autonav_vision panorama_capture --ros-args -p test_mode:=true
```

**Terminal 3 — Publish Commands**

Trigger a panorama sweep:

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/String "{data: 'test_sweep_01'}"
```

Move the motor manually:

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 0.0}"
```

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 30.0}"
```

Avoid publishing manual motor commands while a panorama sweep is running.

### Standalone Panorama Stitcher

The stitcher processes previously captured images and does not require ROS 2 communication.

```bash
source ~/TASC/tasc_ws/install/setup.bash
ros2 run autonav_vision panorama_opencv_stitcher ~/panorama_images/stitch_test
```

The input folder must contain at least two overlapping images named `frame_*.jpg`.

The stitched image is saved as `panorama.jpg` in the same folder. `metadata.csv` is optional.

**Note:** Test mode does not generate images. Use manually captured overlapping images to test the stitcher.

## 2. ROS 2 Nodes

### Motor Controller — `motor_controller.cpp`

Controls the Arduino Nano through USB serial communication.

**Subscribed Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/motor_cmd` | `std_msgs/msg/Float64` | Receives the requested camera angle |

**Published Topics:** None.

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `/dev/nano` | Arduino serial device path |
| `baud_rate` | `115200` | Serial communication baud rate |

**Functionality:**
- Opens the Arduino serial connection.
- Receives camera angle commands.
- Clamps commands to -120° through +120°.
- Maps the requested angle to the Arduino's 0–180 servo command range.
- Sends newline-terminated serial commands to the Arduino.

### Panorama Capture — `panorama_capture.cpp`

Controls the automated panorama sweep using a timer-driven state machine.

**Subscribed Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/panorama_trigger` | `std_msgs/msg/String` | Starts a named panorama sweep |

**Published Topics:**

| Topic | Message Type | Description |
|---|---|---|
| `/motor_cmd` | `std_msgs/msg/Float64` | Publishes the next camera angle |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `test_mode` | `false` | Simulates captures without requiring camera images |
| `cam_url` | IP camera snapshot URL | HTTP snapshot endpoint |
| `save_dir` | `~/panorama_images` | Directory for panorama sweep folders |
| `start_angle` | `-120.0` | First camera angle |
| `end_angle` | `120.0` | Last camera angle |
| `num_angles` | `12` | Number of capture positions |
| `move_time` | `1.0` | Motor movement delay in seconds |
| `settle_time` | `1.0` | Additional settling delay in seconds |
| `capture_timeout` | `3.0` | HTTP snapshot timeout in seconds |

The parameters can be configured through `config/panorama.yaml`.

**State Machine:**

1. `IDLE` — Waits for a panorama trigger.
2. `MOVING` — Publishes the next motor angle.
3. `WAITING` — Waits for the motor to move and settle.
4. `CAPTURING` — Downloads and saves an image when test mode is disabled.
5. After completing the sweep, saves metadata, returns the motor to center, and resets to `IDLE`.

**Output (Normal Mode):**

```text
~/panorama_images/
└── test_sweep_01/
    ├── frame_000.jpg
    ├── frame_001.jpg
    ├── ...
    ├── frame_011.jpg
    └── metadata.csv
```

The current metadata records the frame filename, servo angle, and image path. GPS and heading fields are reserved for future implementation.

In test mode, the sweep folder and CSV header are created, but no JPEG images or frame metadata rows are saved.

### Panorama Stitcher — `panorama_opencv_stitcher.cpp`

Stitches previously captured images into a panorama. It is a standalone program.

- Loads sorted `frame_*.jpg` images.
- Uses OpenCV's panorama stitching functionality.
- Optionally reads GPS and heading metadata.
- Saves the result as `panorama.jpg`.

## 3. Arduino Nano Connections

### MG90D Servo Wiring

| MG90D Wire | Arduino / Power Connection |
|---|---|
| Orange (Signal) | Arduino Nano D9 |
| Brown (Ground) | GND |
| Red (Power) | Regulated 5 V servo supply |

**Additional Notes:**
- Serial communication operates at 115200 baud.

## 4. Troubleshooting

**List connected USB devices:**

```bash
lsusb
```

**List available USB serial ports:**

```bash
ls -l /dev/ttyUSB*
```

**Find persistent USB serial device paths:**

```bash
ls -l /dev/serial/by-id/
```

**Inspect a serial device:**

```bash
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR=|ID_MODEL=|ID_SERIAL='
```

Replace `/dev/ttyUSB0` with the currently connected port.

**Monitor USB connection and disconnection events:**

```bash
sudo dmesg -w
```

**Verify ROS 2 panorama parameters:**

```bash
ros2 param get /panorama_capture test_mode
ros2 param get /panorama_capture num_angles
```

## 5. Dependencies

**Software:**
- ROS 2 Jazzy
- C++17
- CMake
- `colcon`
- `ament_cmake`
- `rclcpp`
- `std_msgs`
- `ros2launch` and `launch_ros`
- OpenCV (`libopencv-dev`)
- libcurl (`libcurl4-openssl-dev`)

**Hardware:**
- Arduino Nano
- MG90D servo motor

**Arduino Library:**
- `Servo.h`

## 6. Next Steps

- Replace the MG90D with a **NEMA 17 stepper motor**, compatible motor driver, and external power supply.
- Implement stepper control, homing, and full 360° rotation.
- Update the ROS 2 motor controller and panorama sweep range.
- Complete and test IP camera snapshot functionality.
- Add GPS latitude, longitude, rover heading, camera heading, and cardinal direction metadata.
- Verify end-to-end image capture and panorama stitching.

## 7. CH340/CH341 Driver (Jetson) [Completely ignore this section if the serial port shows up as specified in section 4]

The Arduino Nano uses a CH340/CH341 USB-to-serial converter.

First, try loading the built-in Linux driver:

```bash
sudo modprobe ch341
```

Check whether the Arduino serial device is available:

```bash
ls -l /dev/ttyUSB*
```

If the built-in driver is unavailable or does not work, the
following CH341SER driver was used during the original Jetson setup:

```bash
git clone https://github.com/juliagoda/CH341SER.git
cd CH341SER
make
sudo make load
```

### Serial Permissions

Check whether your user belongs to the `dialout` group:

```bash
groups
```

If `dialout` is missing:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in to apply the group change.