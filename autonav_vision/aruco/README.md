# aruco_task: ArUco Read & Store / Retrieve node

A ROS 2 Jazzy node written in C++ that detects ArUco markers in a camera image, remembers the IDs, and reports them on command. It implements the ArUco marker commands from the TASC / CIRC 2027 *Automated Demolition and Clearing* task for the Auto-Nav team.

> **Status:** base code for the Nov 24, 2026 milestone. It proves the publisher/subscriber plumbing and the detect, save and display logic. Network camera streaming and the "Press Marked Button" command are not included yet (see [Limitations and future work](#limitations-and-future-work)).

---

## Table of contents

1. [What it does](#what-it-does)
2. [Requirements](#requirements)
3. [Package layout](#package-layout)
4. [Build](#build)
5. [Run](#run)
6. [ROS 2 interface](#ros-2-interface)
7. [Behavior in detail](#behavior-in-detail)
8. [Testing](#testing)
9. [Design decisions](#design-decisions)
10. [Troubleshooting](#troubleshooting)
11. [Limitations and future work](#limitations-and-future-work)
12. [Adding this to the team repo](#adding-this-to-the-team-repo)
13. [References](#references)

---

## What it does

At the job site entrance there is a work assignment board with **5 ArUco markers in a single row**, read left to right. The rover must support two commands:

| Task command | Node behavior | Trigger topic |
|---|---|---|
| **Read and Store Markers** | Detect the 5 markers, display their IDs to an operator, and remember them | `/aruco/capture_cmd` |
| **Retrieve Markers** | Display the IDs stored by the most recent Read and Store | `/aruco/retrieve_cmd` |

Per the task notes, IDs are saved and shown **both as integers/text and as an image**:

- **Integers:** a log line (for example `Stored marker IDs (left to right): [7, 23, 42, 5, 88]`) and the `/aruco/ids` topic.
- **Image:** the camera frame with each marker outlined and labelled, shown in an OpenCV window and published on `/aruco/image`.

Both commands finish in the same function, so "display right after saving" and "display on command" behave identically.

---

## Requirements

| Item | Version / note |
|---|---|
| OS | Ubuntu 24.04 (the platform ROS Jazzy targets) |
| ROS 2 | Jazzy |
| Compiler | C++17 (the Ubuntu 24.04 default GCC is fine) |
| OpenCV | 4.6 or newer. The code compiles against either API (see below) |
| ROS packages | `rclcpp`, `std_msgs`, `sensor_msgs`, `cv_bridge` |
| Build tools | `colcon`, `ament_cmake` (from `ros-dev-tools`) |

Install the dependencies:

```bash
sudo apt update
sudo apt install -y ros-dev-tools ros-jazzy-cv-bridge libopencv-dev git
```

Confirm the ArUco headers are present and note your OpenCV version:

```bash
ls /usr/include/opencv4/opencv2/aruco.hpp
pkg-config --modversion opencv4
```

### OpenCV version handling

OpenCV changed its ArUco API in 4.7:

- **4.7 and newer:** `cv::aruco::ArucoDetector` class (`<opencv2/objdetect/aruco_detector.hpp>`).
- **4.6 and older:** free function `cv::aruco::detectMarkers` (`<opencv2/aruco.hpp>`).

Ubuntu 24.04's packaged OpenCV is 4.6.0, so code written from the current OpenCV tutorials would not compile there. This node selects the matching API at compile time with a preprocessor check on `CV_VERSION_MAJOR` / `CV_VERSION_MINOR` (the `ARUCO_NEW_API` switch). The old-API branch is the one tested; the new-API branch is written from the OpenCV documentation but has not been built here.

---

## Package layout

```
aruco_task/
├── CMakeLists.txt        build instructions
├── package.xml           package name and dependencies
├── include/aruco_task/   (empty; no headers needed yet)
├── src/
│   └── aruco_node.cpp    the whole node
└── README.md
```

The package name `aruco_task` is a placeholder. Rename it to match the team repo's conventions (see [Adding this to the team repo](#adding-this-to-the-team-repo)).

### CMakeLists.txt essentials

`ros2 pkg create` generates most of this. The OpenCV lines are the part to add by hand:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_node src/aruco_node.cpp)
ament_target_dependencies(aruco_node rclcpp std_msgs sensor_msgs cv_bridge)
target_include_directories(aruco_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_node ${OpenCV_LIBS})

install(TARGETS aruco_node DESTINATION lib/${PROJECT_NAME})
```

### package.xml dependencies

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>libopencv-dev</depend>
```

---

## Build

From the root of your workspace (the folder that contains `src/`):

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_task
source install/setup.bash
```

A successful build ends with `Summary: 1 package finished`. Remember to `source install/setup.bash` in every new terminal, and rebuild after editing the code.

> The generated folders `build/`, `install/` and `log/` must **not** be committed to git.

---

## Run

```bash
ros2 run aruco_task aruco_node --ros-args -p image_topic:=/image_raw
```

### Parameters

Set parameters at launch with `-p name:=value`.

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/image_raw` | Camera topic to subscribe to |
| `test_image` | `""` (unused) | Full path to a still image used as a fake camera frame. Use `$HOME/...`, not `~` |
| `dictionary` | `DICT_4X4_100` | Marker family. Supported names: `DICT_4X4_50`, `DICT_4X4_100`, `DICT_5X5_100`, `DICT_6X6_250`. Unknown names fall back to the default |
| `expected_markers` | `5` | Exact number of markers required to save |
| `show_window` | `true` | Set `false` for headless runs (no OpenCV window) |

> **Confirm the dictionary.** `DICT_4X4_100` comes from the old team Python code (`ArucoDetect.py`), not from the competition rules. A wrong dictionary means zero detections. The printed marker size is also still unconfirmed.

### Sending commands

```bash
# Read and Store
ros2 topic pub --once /aruco/capture_cmd  std_msgs/msg/String "{data: go}"

# Retrieve
ros2 topic pub --once /aruco/retrieve_cmd std_msgs/msg/String "{data: go}"

# Watch the IDs (start this first; it waits for messages)
ros2 topic echo /aruco/ids
```

The text in the trigger messages is ignored; only the arrival of a message matters. Triggers use `std_msgs/String` to match the team's existing `panorama_capture.py` convention. If a message seems lost, send it again: `--once` can fire before ROS discovery finishes.

---

## ROS 2 interface

### Subscribes

| Topic | Type | Purpose |
|---|---|---|
| `/aruco/capture_cmd` | `std_msgs/msg/String` | Trigger: Read and Store |
| `/aruco/retrieve_cmd` | `std_msgs/msg/String` | Trigger: Retrieve |
| `<image_topic>` | `sensor_msgs/msg/Image` | Live camera frames (QoS: `SensorDataQoS`, best-effort) |

### Publishes

| Topic | Type | Purpose |
|---|---|---|
| `/aruco/ids` | `std_msgs/msg/Int32MultiArray` | Stored IDs, left to right. **An empty array means nothing is saved** |
| `/aruco/image` | `sensor_msgs/msg/Image` | Stored frame (`bgr8`) with markers outlined and labelled |

`ros2 topic echo` prints `Int32MultiArray` with an empty `layout:` block followed by the `data:` line. Only `data:` carries the IDs.

---

## Behavior in detail

### Flow

```
camera topic --> on_image --> remembers only the NEWEST frame

/aruco/capture_cmd --> read_and_store
    1. take the newest frame
    2. detect ArUco markers
    3. require exactly expected_markers (otherwise warn and keep the old set)
    4. sort left to right by each marker's average corner x
    5. overwrite the stored IDs and the stored annotated image
    6. publish_stored

/aruco/retrieve_cmd --> retrieve --> publish_stored
```

`publish_stored` publishes the IDs and image, logs the IDs, and shows the window. If nothing is saved it logs `No markers saved yet` and publishes an empty IDs array.

### Rules worth knowing

- **Exactly N markers or nothing.** If the frame has more or fewer than `expected_markers`, the capture is rejected and the previously saved set is left untouched. A bad frame can never wipe good data.
- **Left to right.** The task reads markers left to right, so results are sorted by the average x of each marker's four corners. This assumes the camera is roughly level with the board.
- **Overwrite.** Each successful capture replaces the previous IDs and image.
- **Detection runs on command, not per frame.** The node caches the newest frame and detects only when triggered.
- **Storage is in memory.** Saved data is lost if the node restarts.

### Log messages

| Message | Meaning |
|---|---|
| `aruco_node ready (dictionary=..., expecting 5 markers, OpenCV ...)` | Startup, settings in use |
| `Saved new marker set` | A capture succeeded |
| `Stored marker IDs (left to right): [...]` | The display output |
| `No camera frame yet, cannot read markers` | No image has been received (check `image_topic`) |
| `Saw N markers, need exactly 5. Keeping previously saved set.` | Detection ran but the count was wrong |
| `No markers saved yet` | Retrieve requested before any successful capture |
| `Could not load test_image: ...` | The `test_image` path is wrong or unreadable |

---

## Testing

The following was tested on an Ubuntu 24.04 VM (VirtualBox) with ROS Jazzy and OpenCV 4.6.0: topics appearing in `ros2 topic list`, detection and left-to-right ID ordering from a generated test image, the empty "none saved" case, and live detection from a webcam.

### 1. Test with a still image (no camera needed)

Generate a PNG with five markers in a row, deliberately out of ID order so the sorting is exercised:

```bash
python3 - <<'EOF'
import cv2, numpy as np, os
A = cv2.aruco
if hasattr(A, "generateImageMarker"):
    d = A.getPredefinedDictionary(A.DICT_4X4_100)
    gen = lambda i: A.generateImageMarker(d, i, 200)
else:
    d = A.Dictionary_get(A.DICT_4X4_100)
    gen = lambda i: A.drawMarker(d, i, 200)
ids = [7, 23, 42, 5, 88]
img = np.full((250, 1250), 255, np.uint8)
for k, i in enumerate(ids):
    img[25:225, 25+250*k:225+250*k] = gen(i)
cv2.imwrite(os.path.expanduser("~/aruco_test.png"), img)
print("saved", ids)
EOF
```

Run the node against it:

```bash
ros2 run aruco_task aruco_node --ros-args -p test_image:=$HOME/aruco_test.png
```

In other terminals:

```bash
ros2 topic list                    # expect /aruco/capture_cmd, /aruco/retrieve_cmd, /aruco/ids, /aruco/image
ros2 topic echo /aruco/ids
ros2 topic pub --once /aruco/retrieve_cmd std_msgs/msg/String "{data: go}"   # nothing saved yet
ros2 topic pub --once /aruco/capture_cmd  std_msgs/msg/String "{data: go}"   # saves and displays
ros2 topic pub --once /aruco/retrieve_cmd std_msgs/msg/String "{data: go}"   # shows the same set
```

Expected result: the first retrieve logs `No markers saved yet` (`data: []`); capture opens a window with 5 outlined markers and logs `[7, 23, 42, 5, 88]`; the second retrieve repeats that set.

### 2. Test the failure path

```bash
ros2 run aruco_task aruco_node --ros-args -p test_image:=$HOME/aruco_test.png -p expected_markers:=4
```

A capture should log `Saw 5 markers, need exactly 4`, and a following retrieve should log `No markers saved yet`, showing that a rejected frame saves nothing.

### 3. Test the overwrite case

Make a second PNG with different IDs (edit the `ids` list and filename in the script above), capture the first image, restart the node with the second image, and capture again. Retrieve should return only the newer IDs. Also confirm that a failed capture leaves the previous set unchanged.

### 4. Test the real subscription path with a fake camera topic

This uses the camera subscriber rather than `test_image`. The package name and behavior are from memory, so verify them if the install fails.

```bash
sudo apt install ros-jazzy-image-publisher
ros2 run image_publisher image_publisher_node $HOME/aruco_test.png
```

```bash
ros2 run aruco_task aruco_node --ros-args -p image_topic:=/image_raw
```

### 5. Test with a webcam

In VirtualBox, attach the webcam from **Devices > Webcams**, then check `ls /dev/video*`.

The default `v4l2_camera` driver crashed on a webcam that delivers MJPEG (`Current pixel format is not supported yet: MJPG`). The `usb_cam` driver handled it:

```bash
sudo apt install ros-jazzy-usb-cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p pixel_format:=mjpeg2rgb
```

Check what the camera produces before trusting detection:

```bash
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view     # pick /image_raw
```

Then run the node with `-p image_topic:=/image_raw` and hold printed (preferably) markers in front of the camera with the white border visible. If `v4l2-ctl --list-formats-ext` (from `v4l-utils`) lists `YUYV`, the simpler fix is `v4l2_camera_node --ros-args -p pixel_format:=YUYV`.

### Not yet verified

- Viewing `/aruco/image` as a topic (for example in `rqt_image_view`).
- The OpenCV 4.7+ code branch.
- Behavior on the real rover camera.

---

## Design decisions

### One node, not several

All of the logic is in a single node (`aruco_node`):

- **Shared state.** The stored IDs and image are ordinary member variables, so "store" and "retrieve" read the same data with no extra topics or services to share it.
- **One display path.** A single `publish_stored()` serves both commands, which keeps them consistent.
- **Low load.** Detection on one frame is cheap, so there is no performance reason to split across processes.
- **Trade-off.** A crash in detection would also take down retrieval. If heavy work such as network camera streaming is added, that part can be split into its own node without changing this node's topic interface.

### Other choices

- **Topics over services.** The task describes subscribers acting as triggers, and the team's `panorama_capture.py` works the same way.
- **`String` triggers.** They match the existing convention and leave room to carry a label later.
- **Detection on a cached frame.** Commands are infrequent, so there is no reason to detect on every frame.
- **Best-effort camera QoS.** Most camera drivers publish best-effort. A best-effort publisher is only heard by a best-effort subscriber, so the node uses `SensorDataQoS` to avoid silently receiving nothing.
- **Single-threaded callbacks.** ROS runs callbacks one at a time by default, so `latest_frame_` needs no lock. If the node is moved to a multi-threaded executor, add a mutex.

---

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `ros2 topic echo` seems frozen | It is waiting for messages. That is normal. Press Ctrl+C to stop it |
| `ros2 topic list` does not show `/aruco/...` topics | Discovery timing or a stale cache. Wait a moment and retry, or use `ros2 topic list --no-daemon`, or `ros2 daemon stop` then retry |
| `No camera frame yet` | No image is arriving. Check the `image_topic` name (`ros2 topic list`, `ros2 topic hz`) and that the camera node is running |
| `Saw 0 markers` | Markers too small, blurry, glared or cut off, or the dictionary does not match the printed markers |
| `Saw 3 markers, need exactly 5` | Some markers were missed. Look at the image in `rqt_image_view`. Improve lighting and distance, keep the white borders |
| Build error: cannot find `cv_bridge/cv_bridge.hpp` | Use `#include <cv_bridge/cv_bridge.h>` instead |
| Build errors about `ArucoDetector` or `aruco` | OpenCV version mismatch. Check `pkg-config --modversion opencv4`; the version switch should handle 4.6 and 4.7+ |
| Camera driver aborts with `MJPG` / `Unrecognized image encoding []` | The driver does not support the camera's MJPEG format. Use `usb_cam` with `pixel_format:=mjpeg2rgb`, or force `YUYV` if the camera offers it |
| `ls /dev/video*` shows nothing in the VM | Attach the webcam under VirtualBox **Devices > Webcams**; check you are in the `video` group |
| No window appears | `cv::imshow` needs a desktop session (it will not work over plain SSH), or set `show_window:=false` |
| Command seems ignored | Re-send it; `ros2 topic pub --once` can fire before discovery completes |
| New terminal cannot find `aruco_task` | Run `source ~/ros2_ws/install/setup.bash` in that terminal |

---

## Limitations and future work

- **Single frame per command.** A blurred frame can miss a marker. Checking several frames and keeping consistent results would be more robust.
- **No stale-frame check.** If the camera stops publishing, the last frame is reused without a warning. A timestamp check would catch that.
- **Left-to-right ordering assumes a level camera.**
- **IP camera bridge.** The team's `panorama_capture.py` pulls JPEG snapshots over HTTP from an IP camera rather than using a ROS image topic. A bridge that republishes those frames as `sensor_msgs/Image` (or a C++ snapshot grabber) will be needed for the real rover. The team's camera also outputs small images (640x360 in that script), so marker size in pixels needs checking at the real competition distance.
- **Unconfirmed competition details.** The marker dictionary and printed marker size need confirming with the competition rules or the team.
- **Possible ambiguity in the task text.** "Display the 5 saved markers" is currently implemented as the full annotated frame. If the team means an image of the markers themselves (cropped and tiled), that is a small addition.
- **In-memory storage only.** Saved data does not survive a node restart. If persistence is required, write the IDs and image to disk and reload them at startup.
- **Header timestamps.** Published images have an empty header (no timestamp or frame ID).
- **Network display.** Streaming the image to another computer is planned work with the Camera Streaming effort, only if needed.
- **EFF Press Marked Button.** The third ArUco command in the task is not implemented; its details are still to come.
- **Unknown dictionary names** silently fall back to `DICT_4X4_100`. A warning would be an improvement.

---

## Adding this to the team repo

This section reflects the repo path in the task notes (`tasc_autonomy/autonomy_vision/autonomy_vision/panorama_capture.py`) plus general ROS conventions. It has not been checked against the repo itself, so confirm with the team.

The existing vision code appears to be a Python package (`ament_python`). As far as I know, a ROS package has a single build type, so a C++ (`ament_cmake`) node should go in a **sibling package** rather than inside the Python one:

```
tasc_autonomy/                 (git repo)
├── autonomy_vision/           (existing, Python)
└── <new_cpp_package>/         (this package: CMakeLists.txt, package.xml, src/, README.md)
```

Checklist before committing:

- [ ] Agree on the package name with the team and rename `aruco_task` everywhere (folder, `project()` in `CMakeLists.txt`, `<name>` in `package.xml`, run commands in this README).
- [ ] Set a real maintainer name and email and a license in `package.xml`.
- [ ] Make sure `build/`, `install/` and `log/` are not committed (add them to `.gitignore` if missing).
- [ ] Work on a branch rather than `main`, and open a pull request.
- [ ] Rebuild from a clean workspace and rerun the tests above.

---

## References

- OpenCV ArUco detection tutorial: <https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html> (the 4.13 page documents the `ArucoDetector` API, which needs OpenCV 4.7 or newer; the 4.6 docs show the older free-function API)
- Competition task description: <https://circ.cstag.ca/2027/tasks/#automated-demolition-and-clearing>
- Old team ArUco code (Python, ROS 2 Humble): <https://github.com/Combat-Craft/NOVA-AutoNav/tree/main/vision/Aruco>
- Team trigger-subscriber example: `panorama_capture.py` in `tasc_autonomy/autonomy_vision`
- ROS 2 Jazzy documentation: <https://docs.ros.org/en/jazzy/>

## Maintainer

Mohammad Alam, Auto-Nav team. Ask Artemis, Mina or Humaid for task and competition details.
