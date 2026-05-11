# Changelog

All notable changes to this project will be documented in this file. The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.2] — 2026-05-11

Sensor integration phase + setup automation + a 107-test pytest suite that covers software, hardware connectivity, and the setup scripts themselves.

### Added

**Sensors (Phase 2):**
- `imu_node` — LSM9DS1 9-DoF over I²C; timer-driven (100 Hz default), separate `/imu` and `/mag` topics, accel/gyro/mag calibration via `lsm9ds1_cal.yaml` and `lsm9ds1_mag_cal.yaml`
- `lidar.launch.py` — wraps the sllidar_ros2 driver with the racecar's defaults (`/dev/ttyUSB0`, 115200 baud, "Sensitivity" mode by default → 1080 points/rev at 0.33° resolution on an RPLIDAR A3-class device)
- `camera_forward.launch.py` — Logitech BRIO via gscam (MJPG 640×480 @ 30 fps → `/camera/forward`)
- `camera_backward.launch.py` — Arducam B0578 via gscam (MJPG 640×480 @ 30 fps → `/camera/backward`)
- Placeholder `sensor_msgs/CameraInfo` fields (`camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`, `distortion_model`, `image_width/height`) in each camera YAML — uncalibrated zeros for now, replace with `camera_calibration` output when ready
- gscam overlay build (`scripts/patch_gscam.sh`) — clones ros-drivers/gscam, applies the appsink memory-leak fix (`max-buffers=1, drop=true`), builds as a colcon overlay that shadows the apt package
- `sllidar_ros2` brought in as a sibling package; cloned from Slamtec upstream by `setup_workspace.sh`

**One-command setup (`scripts/setup_all.sh`):**
- 6-phase orchestrator: `setup_ros2.sh` → `setup_dev_tools.sh` → `setup_user_env.sh` → `setup_dotmatrix.sh` → `patch_gscam.sh` → `setup_workspace.sh`
- Adds the user to `dialout` / `i2c` / `spi` / `gpio` / `video` groups
- Installs ROS2 Jazzy + 18 ROS packages, the robotics dev apt set, GStreamer dev headers, Python hardware libs (smbus / serial / spidev), and `luma.led_matrix`
- Auto-sources ROS2 + workspace overlay in `~/.bashrc`
- Idempotent — re-runs are no-ops

**Shell aliases (installed by `setup_user_env.sh`):**
- `teleop` — `ros2 launch racecar_neo_ros2_driver teleop.launch.py`
- `racecar-source` — source the workspace overlay
- `racecar-build` — build the driver with `--symlink-install` and source the result
- `racecar-test` — run the full test suite with verbose results
- `racecar-clear-dmatrix` — quick MAX7219 sanity check (lights all pixels, then clears)

**Utility scripts:**
- `scripts/clear_dotmatrix.py` — single-shot MAX7219 sanity check using luma.led_matrix

**Test suite (`test/`):**
- `test_throttle.py`, `test_pwm.py`, `test_mux.py`, `test_imu.py` — unit tests against pure-math helpers extracted from the node classes
- `test_setup_scripts.py` — for each phase script: presence, `+x` bit, `bash -n` syntax, `set -e`, orchestrator references it; also catches stray `build/install/log` dirs inside the package source
- `test_hardware.py` — 9 classes covering Maestro, RPLIDAR, EasySMX, LSM9DS1, forward camera, Arducam, Coral EdgeTPU, MAX7219 dot matrix, Pi 5 RTC battery (`vcgencmd pmic_read_adc BATT_V` ≥ 3.0 V), and Python dependency imports
- ament_flake8 + ament_pep257 linters wired in; entire source tree compliant
- `setup.cfg` pytest config: custom `hardware` marker, filter for Python 3.12's `os.fork` deprecation warning emitted by flake8

### Changed

- Bumped `<version>` in `package.xml` and `setup.py` from 0.0.0 → 0.0.2
- Refactored `throttle_node`, `pwm_node`, `mux_node` to expose module-level pure functions (`scale_speed`, `scale_steering`, `command_to_pwm`, `select_mode`) so they can be unit-tested without rclpy
- Refactored `imu_node` from v1's `while rclpy.ok():` busy loop to a class-based timer-driven node, with `twos_complement` and `apply_mag_calibration` extracted as helpers
- `setup_user_env.sh` now adds the user to `video` (for `vcgencmd` / `/dev/vcio`) in addition to `dialout`, `i2c`, `spi`, `gpio`
- `maestro.py` `setRange(chan, min, max)` → `setRange(chan, min_target, max_target)` to stop shadowing Python builtins (A002)
- Imports across the package reordered to Google style (stdlib → third-party, alphabetic within each); multi-line docstrings switched to second-line-summary format (D213)

[Unreleased]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.2...HEAD
[0.0.2]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.1...v0.0.2

## [0.0.1] — 2026-05-11

Initial driver scaffolding and the control pipeline (gamepad → motor PWM). Sensor, ML, watchdog, and setup-automation layers are planned for later releases.

### Added

- `ament_python` package skeleton (`package.xml`, `setup.py`, `setup.cfg`, resource marker)
- `gamepad_node` — reads configured axes from `/joy` and publishes a normalized command in `[-1, 1]` to `/gamepad_drive`
- `mux_node` — timer-driven (50 Hz) command arbitration on `/mux_out`:
  - LB held → forwards `/gamepad_drive`
  - RB held → forwards `/drive` (autonomy)
  - Neither / both → publishes zero
  - 500 ms `/joy` disconnect timeout → publishes zero
  - 500 ms upstream command staleness → publishes zero
- `throttle_node` — single source of truth for per-direction speed and steering caps; clamps and rescales `/mux_out` → `/motor`
- `pwm_node` — two-parameter servo calibration per axis (`center_pwm` + `magnitude_pwm`); maps `[-1, 1]` commands to Pololu Maestro pulses
- `maestro.py` — Pololu serial protocol library (verbatim port from v1)
- Per-node launch files (`gamepad.launch.py`, `mux.launch.py`, `throttle.launch.py`, `pwm.launch.py`) so the future watchdog can restart any one in isolation
- Top-level `teleop.launch.py` composing all four with `joy_node`
- Parameter YAMLs: `config/gamepad.yaml`, `mux.yaml`, `throttle.yaml`, `pwm.yaml`
- Project files: `README.md`, `LICENSE` (GPLv3), `.gitignore`, `.gitattributes`

### Design notes & migration from v1

- **Normalized `[-1, 1]` command convention** on every intermediate topic. Autonomy code publishing to `/drive` should target this range; v1 expected `[-0.25, 0.25]`.
- **Single tuning surface for top speed.** `max_speed_forward / max_speed_backward / max_steering` in `throttle.yaml` are the only place the effective top speed is set. v1 spread this across three nodes with three duplicated constants.
- **Two-step servo calibration in `pwm.yaml`.** Per axis: (1) find `center_pwm` at command = 0, (2) raise `magnitude_pwm` at command = +1 until visible saturation. Replaces v1's six interdependent parameters per axis.
- **Mux is timer-driven** at 50 Hz, not event-driven on `/joy` callbacks. Keeps the Maestro continuously fed and gives the future watchdog an unambiguous "mux alive" signal.
- **Mux zeros on `/joy` disconnect and on upstream command staleness.** v1 had no such safety net.

[0.0.1]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/releases/tag/v0.0.1
