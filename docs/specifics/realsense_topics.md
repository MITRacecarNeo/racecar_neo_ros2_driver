# RealSense D435i topic reference

ROS2 topics published by the Intel RealSense D435i using `realsense2_camera`, launched by `launch/realsense.launch.py`. The D435i is the only camera on RACECAR Neo v2: its color stream is remapped onto `/camera/forward` (the forward-camera contract consumed by `edgetpu_node` and the student library), depth publishes on `/camera/depth/image_rect_raw`, and the camera IMU on `/camera/imu`.

> **Hardware:** Intel RealSense D435i (USB 3.x, `8086:0b3a`, firmware 5.17.0.9)

Rates measured on RACECAR Neo hardware (Raspberry Pi 5, depth + color + IMU enabled at 640x480 @ 30 FPS with depth filters on, IR and alignment disabled).

---

## Table of Contents

- [Depth](#depth)
- [Color (RGB)](#color-rgb)
- [Infrared](#infrared)
- [IMU](#imu)
- [Aligned Depth](#aligned-depth)
- [Point Cloud](#point-cloud)
- [Camera Info and Metadata](#camera-info-and-metadata)
- [TF Frames](#tf-frames)
- [Configuration Notes](#configuration-notes)
- [Known Issues](#known-issues)

---

## Depth

| Topic | Message Type | Configured | Measured | Description |
|---|---|---|---|---|
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | ~30 Hz | Rectified depth image (16UC1, values in mm) |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | ~30 Hz | Depth camera intrinsics and distortion |

> With the optimized config (IR and alignment disabled), depth reaches near-full framerate even with filters enabled.

## Color (RGB)

The color image is remapped to `/camera/forward` so `edgetpu_node` and the student library keep the forward-camera contract.

| Topic | Message Type | Configured | Measured | Description |
|---|---|---|---|---|
| `/camera/forward` | `sensor_msgs/msg/Image` | 30 Hz | ~26 Hz | Color image (RGB8, 640x480), remapped from `/camera/color/image_raw` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | ~26 Hz | Color camera intrinsics and distortion |

## Infrared

| Topic | Message Type | Configured | Measured | Description |
|---|---|---|---|---|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | ~28 Hz | Left infrared camera (Y8, 640x480) |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz | ~28 Hz | Right infrared camera (Y8, 640x480) |

> **Disabled by default** to reduce CPU load. Enable with: `enable_infra1:=true enable_infra2:=true` in the RealSense launch args. Stereo infrared is useful for VIO and feature tracking in low-light conditions.

## IMU

| Topic | Message Type | Configured | Measured | Description |
|---|---|---|---|---|
| `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz | ~200 Hz | Fused gyroscope + accelerometer (linear interpolation) |
| `/camera/gyro/sample` | `sensor_msgs/msg/Imu` | 200 Hz | ~180 Hz | Raw gyroscope data only |
| `/camera/accel/sample` | `sensor_msgs/msg/Imu` | 63 Hz | ~63 Hz | Raw accelerometer data only |
| `/camera/gyro/imu_info` | `realsense2_camera_msgs/msg/IMUInfo` | Latched | Gyroscope noise and bias parameters |
| `/camera/accel/imu_info` | `realsense2_camera_msgs/msg/IMUInfo` | Latched | Accelerometer noise and bias parameters |

> The `unite_imu_method: 2` config interpolates accel data to match gyro timestamps, producing a unified `/camera/imu` topic at the gyro rate. This is the preferred input for VIO/SLAM pipelines.

> Firmware 5.17.0.9+ is required for IMU to work on Pi 5. See [Known Issues](#known-issues) for details.

## Aligned Depth

| Topic | Message Type | Configured | Measured | Description |
|---|---|---|---|---|
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | ~18 Hz | Depth image aligned to the color camera frame |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | ~18 Hz | Camera info matching the aligned depth |

> **Disabled by default** to reduce CPU load. Enable with: `ros2 launch racecar_neo_ros2_driver realsense.launch.py align_depth_enable:=true`. Alignment is CPU intensive (~10% additional CPU on Pi 5). Essential for tasks that combine color and depth (object detection with distance, RGBD SLAM).

## Point Cloud

| Topic | Message Type | Rate | Description |
|---|---|---|---|
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Up to 30 Hz | Colored 3D point cloud (XYZRGB) |

> **Disabled by default**; point cloud generation is CPU intensive on the Pi 5. Enable with: `ros2 launch racecar_neo_ros2_driver realsense.launch.py pointcloud_enable:=true`

## Camera Info and Metadata

| Topic | Message Type | Description |
|---|---|---|
| `/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | Extrinsic calibration between depth and color sensors |
| `/camera/extrinsics/depth_to_infra1` | `realsense2_camera_msgs/msg/Extrinsics` | Extrinsic calibration between depth and left infrared |
| `/camera/extrinsics/depth_to_infra2` | `realsense2_camera_msgs/msg/Extrinsics` | Extrinsic calibration between depth and right infrared |
| `/camera/extrinsics/depth_to_gyro` | `realsense2_camera_msgs/msg/Extrinsics` | Extrinsic calibration between depth and gyroscope |
| `/camera/extrinsics/depth_to_accel` | `realsense2_camera_msgs/msg/Extrinsics` | Extrinsic calibration between depth and accelerometer |
| `/camera/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | Per-frame metadata (exposure, gain, timestamp) |
| `/camera/color/metadata` | `realsense2_camera_msgs/msg/Metadata` | Per-frame metadata for color stream |

## TF Frames

The RealSense node publishes static transforms between all sensor frames:

```
camera_link
├── camera_depth_frame
│   └── camera_depth_optical_frame
├── camera_color_frame
│   └── camera_color_optical_frame
├── camera_infra1_frame
│   └── camera_infra1_optical_frame
├── camera_infra2_frame
│   └── camera_infra2_optical_frame
├── camera_gyro_frame
│   └── camera_gyro_optical_frame
└── camera_accel_frame
    └── camera_accel_optical_frame
```

`camera_link` is the reference frame. Optical frames follow the ROS convention (Z forward, X right, Y down).

---

## Configuration Notes

### Resolution and Framerate

The default config runs at 640x480 @ 15 FPS for depth and color streams. Available profiles for D435i:

| Resolution | Max FPS (Depth) | Max FPS (Color) | Notes |
|---|---|---|---|
| 1280x720 | 30 | 30 | Higher quality, more CPU load |
| 640x480 | 90 | 60 | Default — good balance for Pi 5 |
| 424x240 | 90 | 60 | Lowest latency |

To change resolution, pass launch arguments:

```bash
ros2 launch racecar_neo_ros2_driver realsense.launch.py depth_profile:=424x240x60 color_profile:=424x240x60
```

### Depth Filters

The following post-processing filters are enabled by default:

| Filter | Purpose |
|---|---|
| Decimation | Reduces depth resolution for faster processing |
| Spatial | Edge-preserving smoothing to reduce noise |
| Temporal | Smoothing across frames to fill holes |

Disabling filters will increase the depth framerate on the Pi 5 (~19 Hz with filters off vs ~17 Hz with filters on).

### Pi 5 Performance Considerations

- With the default config (depth + color + IMU + filters at 15 FPS, no IR or alignment), expect **~10-15 Hz** for depth/color publish rate under full teleop load
- **Infrared streams**, **aligned depth**, and **point cloud** are all disabled by default to minimize CPU load
- Enabling IR + alignment + all streams significantly increases CPU usage
- If CPU usage is too high, reduce to 424x240 or lower FPS
- The D435i is connected over **USB 3.2** which provides full bandwidth
- RealSense CPU usage: ~29-55% of one core depending on system load

---

## Known Issues

### IMU "Motion Module force pause" (Firmware < 5.17.0.9) — RESOLVED

The D435i IMU fails to publish data with `Hardware Notification: Motion Module force pause` on firmware 5.17.0.9 with the Pi 5's xHCI USB host controller.

**Fix:** Update firmware to 5.17.0.9+ (confirmed working on this unit, serial 943222073786):

```bash
# Download firmware from https://dev.realsenseai.com/docs/firmware-releases-d400
wget -O /tmp/d400_fw.zip "https://realsenseai.com/wp-content/uploads/2025/07/d400_series_production_fw_5_17_0_9-4.zip"
unzip /tmp/d400_fw.zip -d /tmp/d400_fw
sudo rs-fw-update -f /tmp/d400_fw/D4XX_FW_Image-5.17.0.9.bin
# If camera enters DFU mode and access fails, use: sudo rs-fw-update -r -f <path>
```

> **Note:** `rs-fw-update` needs `sudo` because the DFU-mode USB device (`8086:0adb`) requires root access. The ROS-packaged binary also needs its libs on the path, and `sudo` strips `LD_*` from the environment even with `-E` — pass it explicitly: `sudo env LD_LIBRARY_PATH=/opt/ros/jazzy/lib rs-fw-update ...`.

### Airgapped fleet firmware flash

Camera firmware lives in the camera's own flash, not on the disk. Cloning the OS image neither carries firmware to another camera nor breaks an already-updated one, so each unit must be flashed individually. `rs-fw-update` never needs the network; only downloading the `.bin` does.

Stage the firmware into the golden image **once** on a networked machine, before cloning:

```bash
sudo mkdir -p /opt/racecar/firmware
wget -O /tmp/d400_fw.zip "https://realsenseai.com/wp-content/uploads/2025/07/d400_series_production_fw_5_17_0_9-4.zip"
unzip /tmp/d400_fw.zip -d /tmp/d400_fw
sudo cp /tmp/d400_fw/D4XX_FW_Image-5.17.0.9.bin /opt/racecar/firmware/
```

Then on each airgapped car (the `.bin` and `rs-fw-update` both came along in the clone):

```bash
racecar setup realsense            # flash from /opt/racecar/firmware; skips if already 5.17.0.9
racecar setup realsense --check    # report current vs target only, no flash
racecar setup realsense --serial 943222070134   # pick one when several cameras are attached
```

`racecar setup realsense` runs `scripts/flash_realsense_offline.sh`. It is idempotent (a camera already at the target version is a no-op), falls back to DFU recovery mode on a failed normal-mode flash, and re-verifies the version afterward. Override the target with `--version` / `RACECAR_RS_FW_VERSION` or the staging directory with `--fw-dir` / `RACECAR_RS_FW_DIR`.

### IMU IIO Permissions (Raspberry Pi 5)

The D435i IMU uses Linux HID-sensor IIO devices. On the Pi 5, the sysfs attributes for these devices default to root-only, causing `Permission denied` errors when the RealSense node tries to configure the gyroscope and accelerometer.

The `realsense.launch.py` launch file automatically runs a permission fix script before starting the camera node. A udev rule (`/etc/udev/rules.d/99-realsense-imu.rules`) also attempts to fix permissions on device creation.

If you see IMU permission errors when launching manually (not through `realsense.launch.py`), run:

```bash
sudo /usr/local/bin/fix-realsense-imu.sh
```
