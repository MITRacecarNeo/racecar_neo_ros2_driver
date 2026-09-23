# RealSense D435i topic reference

ROS2 topics published by the Intel RealSense D435i through `realsense2_camera`,
launched by `launch/realsense.launch.py`. The launch includes the vendor
`rs_launch.py` (namespace `/`, camera name `camera`) after a 1 s delay for USB
enumeration, and remaps the three streams the rest of the stack reads onto
RACECAR names.

Hardware: Intel RealSense D435i (USB 3.x, `8086:0b3a`, firmware 5.17.0.9 or
later).

Rates below are the configured targets from the launch profiles. Measured
rates on a running car are reported by the dashboard and `racecar status`,
both of which read the camera's own `/diagnostics`.

## Contents

- [Remapped streams](#remapped-streams)
- [Other published topics](#other-published-topics)
- [Stream profiles](#stream-profiles)
- [Optional streams](#optional-streams)
- [TF frames](#tf-frames)
- [Known issues](#known-issues)

## Remapped streams

| Topic | Remapped from | Message type | Target rate | Content |
|---|---|---|---|---|
| `/camera/color` | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 60 Hz | Color image, 640x480 |
| `/camera/depth` | `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | Rectified depth, 640x480, 16UC1 in millimetres |
| `/imu/realsense` | `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz | Gyroscope plus accelerometer united at the gyro rate |

`edgetpu_node`, `imu_fusion_node`, the student library and the lab dashboards
read these names. The IMU is united with `unite_imu_method: 2` (linear
interpolation): the 63 Hz accelerometer is interpolated onto the 200 Hz
gyroscope timestamps. Depth post-processing is off (decimation, spatial and
temporal filters all disabled), so depth stays 640x480 to match the color
frame and the library's depth API.

## Other published topics

| Topic | Message type | Content |
|---|---|---|
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color intrinsics and distortion |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsics and distortion |
| `/camera/color/metadata`, `/camera/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | Per-frame exposure, gain, timestamp |
| `/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | Depth to color extrinsics |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Per-stream frequency, published every 1 s (`diagnostics_period`) |

## Stream profiles

| Launch argument | Default | Applies to |
|---|---|---|
| `depth_profile` | `640x480x30` | depth (and infrared, when enabled) |
| `color_profile` | `640x480x60` | color |
| `align_depth_enable` | `false` | aligned depth |
| `pointcloud_enable` | `false` | point cloud |

Fixed in the launch file: `gyro_fps` 200, `accel_fps` 63, `enable_sync` true,
`publish_tf` true with static transforms only (`tf_publish_rate` 0.0).

D435i profiles that fit the Pi 5:

| Resolution | Max depth fps | Max color fps |
|---|---|---|
| 1280x720 | 30 | 30 |
| 640x480 | 90 | 60 |
| 424x240 | 90 | 60 |

To change them, pass launch arguments:

```bash
ros2 launch racecar_neo_ros2_driver realsense.launch.py depth_profile:=424x240x60 color_profile:=424x240x60
```

Consumers that assume 640x480 (the student library's depth API, the dashboard
previews) need checking after a resolution change.

## Optional streams

All off by default to keep CPU load down on the Pi 5.

| Stream | Enable with | Topics |
|---|---|---|
| Aligned depth | `align_depth_enable:=true` | `/camera/aligned_depth_to_color/image_raw`, `.../camera_info` |
| Point cloud | `pointcloud_enable:=true` | `/camera/depth/color/points` (`sensor_msgs/msg/PointCloud2`) |
| Infrared | edit `enable_infra1` / `enable_infra2` in `launch/realsense.launch.py` | `/camera/infra1/image_rect_raw`, `/camera/infra2/image_rect_raw` |

Aligned depth is needed when color and depth are combined per pixel (detection
with distance, RGBD SLAM).

## TF frames

The RealSense node publishes static transforms between its sensor frames:

```
camera_link
├── camera_depth_frame
│   └── camera_depth_optical_frame
├── camera_color_frame
│   └── camera_color_optical_frame
├── camera_gyro_frame
│   └── camera_gyro_optical_frame
└── camera_accel_frame
    └── camera_accel_optical_frame
```

`camera_link` is the reference frame. Optical frames follow the ROS convention
(Z forward, X right, Y down). Infrared frames appear only when infrared is
enabled.

## Known issues

### IMU firmware requirement

Below firmware 5.17.0.9 the D435i IMU publishes nothing on the Pi 5's xHCI USB
controller and logs `Hardware Notification: Motion Module force pause`.
Firmware 5.17.0.9 or later fixes it. Camera firmware lives in the camera's own
flash, so every camera is flashed once, individually.

On a networked machine:

```bash
# Firmware index: https://dev.realsenseai.com/docs/firmware-releases-d400
wget -O /tmp/d400_fw.zip "https://realsenseai.com/wp-content/uploads/2025/07/d400_series_production_fw_5_17_0_9-4.zip"
unzip /tmp/d400_fw.zip -d /tmp/d400_fw
sudo env LD_LIBRARY_PATH=/opt/ros/jazzy/lib rs-fw-update -f /tmp/d400_fw/D4XX_FW_Image-5.17.0.9.bin
# If the camera enters DFU mode and access fails: add -r (recovery)
```

`rs-fw-update` needs `sudo` because the DFU-mode USB device (`8086:0adb`) is
root-only, and `sudo` strips `LD_*` even with `-E`, so the library path is
passed through `env`. Reasoning: docs/troubleshooting.md, "RealSense firmware
flash privileges".

### Airgapped fleet firmware flash

Cloning the OS image neither carries firmware to another camera nor breaks an
already-updated one. `rs-fw-update` never needs the network; only downloading
the `.bin` does.

Stage the firmware into the golden image once, on a networked machine, before
cloning:

```bash
sudo mkdir -p /opt/racecar/firmware
wget -O /tmp/d400_fw.zip "https://realsenseai.com/wp-content/uploads/2025/07/d400_series_production_fw_5_17_0_9-4.zip"
unzip /tmp/d400_fw.zip -d /tmp/d400_fw
sudo cp /tmp/d400_fw/D4XX_FW_Image-5.17.0.9.bin /opt/racecar/firmware/
```

Then on each airgapped car (the `.bin` and `rs-fw-update` both came along in
the clone):

```bash
racecar setup realsense                   # flash from /opt/racecar/firmware; skips if already 5.17.0.9
racecar setup realsense --check           # report current vs target only, no flash
racecar setup realsense --serial <serial> # pick one when several cameras are attached
```

`racecar setup realsense` runs `scripts/flash_realsense_offline.sh`. It is
idempotent (a camera already at the target version is a no-op), falls back to
DFU recovery mode on a failed normal-mode flash, and re-verifies the version
afterward. Override the target with `--version` / `RACECAR_RS_FW_VERSION` or
the staging directory with `--fw-dir` / `RACECAR_RS_FW_DIR`.

### IMU IIO permissions

The D435i IMU uses Linux HID-sensor IIO devices, whose sysfs attributes default
to root-only on the Pi 5; the camera node then fails to configure the gyroscope
and accelerometer with `Permission denied`. `scripts/setup_realsense.sh` fixes
this at the root level with a udev rule
(`/etc/udev/rules.d/99-realsense-imu.rules`, run on IIO device add) and the
`realsense-imu-permissions.service` boot unit. The launch file does not call
sudo.

If permission errors appear anyway, for example after replugging the camera
without a udev trigger, run:

```bash
sudo /usr/local/bin/fix-realsense-imu.sh
```
