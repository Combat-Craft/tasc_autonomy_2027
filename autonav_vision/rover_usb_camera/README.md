# ROS 2 USB camera publisher

`rover_usb_camera` is a ROS 2 Jazzy C++ package that publishes one JPEG per frame on `image_raw/compressed` as `sensor_msgs/msg/CompressedImage`. Use `usb_camera` with a USB camera attached to Linux. Use `webcam_test` when Windows owns the laptop webcam and ROS runs in an Ubuntu VM.

## Build on Ubuntu

These commands assume ROS 2 Jazzy and its dependecies (eg. GStreamer) are installed and the extracted package is at `~/ros2_ws/src/rover_usb_camera/`, with `CMakeLists.txt` and `package.xml` directly inside it.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select rover_usb_camera
source install/setup.bash
```

## Run with a USB camera on Linux

```bash
ros2 run rover_usb_camera usb_camera --ros-args -r __ns:=/webcam \
  -p device:=/dev/video0 -p width:=640 -p height:=480 -p fps:=30 -p quality:=80
```

The source file is `src/usb_camera.cpp`. GStreamer opens the selected device with `v4l2src`, requests raw YUY2 frames, converts them to BGR, and compresses each frame with `jpegenc`. The node pulls JPEG bytes from `appsink` and publishes them on `/webcam/image_raw/compressed`. The appsink queue holds one frame and drops older frames if the publisher falls behind.

The example shows the default camera parameters. The device must support the requested raw format, size, and frame rate. To run a second camera, start another instance with its own device path and namespace, such as `-r __ns:=/rear`. Its topic will be `/rear/image_raw/compressed`.

**Compression settings.** Set `quality` from 0 to 100 at startup. Higher values typically produce larger JPEGs. The current code always uses `jpegenc`. It declares `encoder` and validates that `bitrate_bps` is positive, but neither parameter changes the encoded output. Target bitrate control is in the works.

## Test a Windows webcam from an Ubuntu VM

1. In Windows PowerShell, run `ipconfig`. Find the IPv4 address of the Windows VMware adapter on the same network as the guest. In the command below, replace `IP` with that address. Start the sender and leave it running. Windows needs GStreamer with the Media Foundation, JPEG, and TCP plugins.

   ```powershell
   gst-launch-1.0 -v mfvideosrc ! videoconvert ! jpegenc quality=80 ! tcpserversink host=IP port=5000
   ```

2. In the Ubuntu VM, run the test executable using the same Windows IP. The guest must be able to reach TCP port 5000 on that adapter.

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run rover_usb_camera webcam_test --ros-args -r __ns:=/webcam -p host:=IP
   ```

   The test node in `src/webcam_test.cpp` decodes the incoming JPEGs, adapts the raw frames to the requested resolution and frame rate, then encodes and publishes them through the same JPEG-to-ROS workflow. Its `host` parameter is required, and the TCP port is fixed at 5000.

## Verify and view the feed

While either publisher is running, open a second Ubuntu terminal. These commands use the `/webcam` namespace from the examples.

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic type /webcam/image_raw/compressed
ros2 topic hz /webcam/image_raw/compressed
```

Expect `sensor_msgs/msg/CompressedImage` and a recurring rate estimate. Press Ctrl+C to stop the rate check. 

## Phantom Bridge and testing status

The ROS topic is the handoff to Phantom Bridge. The node publishes JPEG frames, and the bridge handles H.264 encoding for the remote viewer. 

The Windows-to-VM test confirmed that ROS messages arrived. Direct USB capture on the Jetson, playback through Phantom Bridge, and CPU/RAM measurements on the rover still need verification.
