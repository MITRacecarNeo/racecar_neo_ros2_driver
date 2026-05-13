# Changelog

All notable changes to this project will be documented in this file. The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.8] — 2026-05-13

Ship the pip dependencies the v2 student library needs to run on Python 3.12 / Pi 5. No driver code changes — `setup_jupyter.sh` only.

### Added

- `scripts/setup_jupyter.sh` now installs three additional pip packages required by the v2 student library and the `labs/tests/test_async_core_real.ipynb` walkthrough: `ipywidgets` (live FPS / joystick / detection widgets — JupyterLab 4.x renders `ipywidgets >= 8` natively, no labextension step needed), `pandas` (backs `telemetry_real.visualize()` reading the recorded CSV), and `matplotlib-inline<0.2` (the IPython inline matplotlib backend). Idempotent: each dep is import-probed under the target user first, and `pip install` only runs for the missing ones — re-runs after a successful install are silent. The matplotlib-inline probe is version-aware so a system that already has a stale 0.2.x installation (e.g. pulled transitively by `ipykernel` before the pin landed) gets re-resolved to 0.1.x.
  - **Why `matplotlib-inline<0.2`:** the 0.2 line calls `matplotlib.rcParams._get(...)`, which only exists in matplotlib >= 3.10. Pi-OS bookworm/noble ship apt matplotlib 3.6.3, so on Python 3.12 (the Pi 5 default) the 0.2.x transitive blows up the first `plt.subplots()` call inside Jupyter with `AttributeError: 'RcParams' object has no attribute '_get'`. Pinning 0.1.x keeps the inline backend usable on the current apt matplotlib.
- **nptyping intentionally not added.** An earlier v0.0.8 draft pinned `nptyping<2` because the v1 student library used the deprecated `NDArray[(480, 640, 3), np.uint8]` generic form. On Python 3.12 both nptyping branches are broken: 2.x raises `InvalidArgumentsError` at class definition (incompatible `Shape["..."]` API), and 1.4.4 triggers a runaway recursion between `typing._type_repr` and nptyping's `__repr__` that adds ~30 s to a cold `import racecar_core`. MITUavNeo/uav-neo-library hit the same Py3.12 wall and resolved it by dropping nptyping entirely — every module that needed the `NDArray[...]` syntax got a 2-line inline stub instead. The racecar-neo v2 library v1.2.0 mirrors that pattern, so this driver doesn't need to ship nptyping at all.

## [0.0.7] — 2026-05-12

Functionality audit before tagging, plus the lidar/ModemManager hardening surfaced by the 2026-05-12 endurance test. Safety hardening, Pi 5 efficiency wins, and dead-code removal — no new features.

### Added

- `racecar_neo_ros2_driver/launch_common.py` — `single_node_launch()` helper that builds a one-node `LaunchDescription` from `(arg_name, default_yaml, package, executable)` plus optional `node_name` / `remappings`. Per-node launch files (pwm, mux, throttle, gamepad, lidar, dotmatrix, edgetpu, camera_forward, camera_backward) collapse onto it; IMU stays bespoke (two YAML params).
- `mux_node` boot-time arming gate: new `startup_grace_sec` (default 1.0) and `arm_axis_threshold` (default 0.2) parameters. After boot the mux publishes zero until the grace period has elapsed AND it has seen one `/joy` frame with every axis below threshold. Defends against stuck-stick-at-power-on. New `joy_is_centered(axes, threshold)` helper.
- `dashboard.MONITORED` entries gain a `supervised` boolean. EdgeTPU and dotmatrix are marked `supervised=False` since the watchdog doesn't restart them; their card status is `'unsupervised'` rather than `'dead'` when absent.
- `PGREP_FAIL_THRESHOLD = 5` in `watchdog.py`. `_is_running` now counts consecutive pgrep exceptions per-pattern and escalates from "assume alive" to "assume down" after the threshold, so a broken pgrep can't mask a real outage forever.
- `test/test_mux.py::TestJoyCentered` covering the new arming-gate helper.
- `test/test_setup_scripts.py::test_scripts_use_pipefail` extends the existing `set -e` check.
- `docs/post-audit-tests.md` — on-robot walk-through for verifying v0.0.7.

### Fixed

- **Maestro hardcoded `/dev/ttyACM0`** in `pwm_node.py` and `maestro.py`. Footgun for `ros2 run` invocations without `--params-file` (Coral / joystick passthrough can grab ACM0 transiently). Both now default to `/dev/maestro` — the udev symlink contract.
- **Gamepad node didn't clip to `[-1, 1]`** before publishing. A miscalibrated EasySMX or a typo'd `throttle_sign` could escape downstream. Throttle clamps already, but the contract per `[[project_conventions]]` is enforced at every boundary now.
- **`watchdog.py` shelled out to `ros2 topic list` every 5 s** — measurable Pi 5 CPU + DDS discovery pressure. Replaced with an in-process rclpy `Node` (`racecar_watchdog`) spun in a daemon thread, calling `node.get_topic_names_and_types()` directly. `_get_active_topics()` keeps the subprocess fallback for module-level helper tests.
- **`dashboard.py` fan-out of `ros2 topic hz` subprocesses** every 3 s, leaking zombies on `join(timeout)` mismatches. Replaced with a single long-lived rclpy node (`racecar_dashboard`) that subscribes BEST_EFFORT to each `RATE_TOPICS` entry, records monotonic arrival timestamps in a per-topic deque, and computes Hz over a 3 s window. Late-binding `attach_subscriptions()` picks up new publishers each tick.
- **`dashboard.py` read the full `watchdog.log` every refresh.** Replaced with seek-from-end so only the last 4 KB are read regardless of file size.
- **`dotmatrix_node` re-rendered text width every tick** — `PIL.Image` allocation at 15 Hz. Now memoized on `(message, id(font), height)`; `PIL` import hoisted to module top.
- **`watchdog.py` leaked the per-restart `log_fh`.** `_child_procs` now stores `(proc, log_fh)` tuples and closes the handle when the child is reaped.
- **`dashboard.py` dead `import os`** removed. `import re` hoisted out of `_read_battery_voltage` to module top per Google style.
- **`dotmatrix_node._latest_joy`** was assigned but never read. Dropped.
- **`launch/teleop.launch.py::_gated_include`** had two near-identical TimerAction branches. `TimerAction(period=0.0, condition=...)` honors the condition correctly, so the special-case was unnecessary. Collapsed.
- **`setup_networking.sh` destructive step ordering** — `nmcli connection delete` of prior Wi-Fi client connections ran before `netplan apply` on eth0. Any failure under `set -e` between them could strand an SSH-over-WiFi user. Reordered: dispatcher install → AP configure + bring up → eth0 netplan apply → delete prior Wi-Fi client. The user's existing WiFi survives any earlier failure now.
- **Bash `pipefail` missing across phase scripts** — `set -e` alone let `wget | dpkg -i` chains silently mask upstream failures. Standardized to `set -eo pipefail` across phase scripts + orchestrator; `test_scripts_use_pipefail` enforces it. `-u` not adopted yet (per-script audit of `${VAR:-default}` usage needed first).
- **Watchdog and dashboard module docstrings** trimmed to one-line summaries per `[[feedback_terse_comments]]`.
- **Lidar silently stopped publishing under ModemManager probe** — 2026-05-12 8h endurance: a snap-store refresh triggered `systemctl daemon-reload`, ModemManager re-probed every tty, and its probe of the lidar's CP2102 (`10c4:ea60`) desynced the sllidar SDK's binary frame reader. Process stayed alive, `/scan` stayed advertised, no scans came through. Two-part fix: (1) `scripts/udev/99-racecar.rules` adds `ENV{ID_MM_DEVICE_IGNORE}="1"` to the lidar rule so MM never opens that port, and (2) `scripts/watchdog.py` gains a `freshness_sec` field on NODES entries — if set, the watchdog subscribes via rclpy (BEST_EFFORT) and treats the topic as failed when no message arrives within the window, separately from process-presence. Only `lidar` opts in (`freshness_sec=5.0`), with a post-restart grace so cooldown can't trigger a self-restart loop. New tests cover the udev rule and the freshness monitor.
- **Mux arming gate never fired on the EasySMX** — `joy_is_centered` checked every axis under threshold 0.2, but axes[2] (LT) and axes[5] (RT) rest at +1.0 in Xbox-360 mode. Result: the v0.0.7 arming gate blocked driving entirely on the real controller. Added `arm_ignore_axes` parameter (default `[2, 5]`) that `joy_is_centered` skips during the centered check, leaving the stick axes (0, 1, 3, 4) fully gated. Three new tests cover the trigger-at-rest case, the ignore-axes path, and that ignoring triggers doesn't excuse a stuck stick.

### Changed

- Bumped `<version>` 0.0.6 → 0.0.7 in `package.xml` and `setup.py`.
- `config/mux.yaml` documents the new `startup_grace_sec` and `arm_axis_threshold` parameters.
- `Maestro.__init__` docstring updated to mention `/dev/maestro`.

### Deferred

- **EdgeTPU under watchdog supervision** — the `1a6e:089a → 18d1:9302` USB firmware enumeration needs its own retry-after-reset logic. v0.0.7 only marks edgetpu/dotmatrix as `supervised=False` on the dashboard.
- **Topic-name constants module** — topic strings repeat across nodes, watchdog, dashboard, launch files. Mechanical but large; punted.
- **PWM parameter nesting** — `motor.{channel,center_pwm,magnitude_pwm}` would read better than the flat parameters, but the YAML round-trip is non-trivial.
- **Watchdog failure-path test coverage** — cooldown, stale-child-kill, device-check-skip, volt-alarm-tripped branches are still mostly untested. Its own focused PR.
- **Shared `pi_health.py`** — RTC voltage classifier + BATT_V regex + rpi_volt hwmon walk live in watchdog, dashboard, and `test_hardware.py` with drift between copies.

## [0.0.6] — 2026-05-11

Phase 6: networking. eth0 dual-IP for predictable wired access, wlan0 isolated AP so anyone within range can reach the robot's dashboard / JupyterLab / SSH without needing existing WiFi infrastructure.

### Added

- `scripts/setup_networking.sh` — installs a NetworkManager dispatcher that blocks `FORWARD` on `wlan0` (AP isolation), removes prior WiFi-client connections on `wlan0`, creates the racecar AP via `nmcli` (WPA2 / 2.4 GHz / channel 6 / 10.42.0.1/24), and writes `/etc/netplan/99-racecar-eth0.yaml` with both static (default `192.168.52.200/24`) and DHCP on eth0. Idempotent — re-running only writes files that changed. **Not invoked by `setup_all.sh`** since reconfiguring wlan0 can drop SSH-over-WiFi sessions during a fresh install.
- All tunables parameterized via env vars (`RACECAR_AP_SSID`, `RACECAR_AP_PSK`, `RACECAR_AP_CHANNEL`, `RACECAR_AP_ADDR`, `RACECAR_ETH_STATIC`) AND via `~/.config/racecar/networking.env` (persisted overrides loaded on every run, current-shell env vars take precedence).
- `racecar setup <phase>` shell-tool subcommand. Phases:
  - `all` — runs the 11-phase orchestrator (`scripts/setup_all.sh`)
  - `networking` — runs `scripts/setup_networking.sh` after persisting any `--flag=value` to `~/.config/racecar/networking.env`. Flags: `--ssid`, `--psk`, `--channel`, `--ap-addr`, `--eth-static`. Plus `--show` (print persisted overrides), `--reset` (delete the persisted file), `--help`.
- Tab completion: `racecar setup <TAB>` offers `all` / `networking`; `racecar setup networking <TAB>` offers the flag set.
- `test/test_setup_scripts.py::TestNetworkingScript` — verifies the script exists, executable, `bash -n` clean, references all five `RACECAR_*` env vars, loads the persisted config, has the iptables AP-isolation dispatcher wired up, AND is intentionally **not** referenced from `setup_all.sh`.
- `test/test_racecar_tool.py::TestSetup` — flag parsing (`--help`, `--show` with/without persisted file, `--reset`), unknown phase / unknown flag error paths.
- `test/test_setup_scripts.py` gains a `STANDALONE_SCRIPTS` list separating "scripts the orchestrator calls" from "scripts the user runs manually" (currently just `setup_networking.sh`).
- `docs/networking_test_checklist.md` — walk-through checklist for verifying v0.0.6 networking end-to-end (pre-flight, persistence, the destructive reconfiguration, AP-client connectivity, isolation, idempotency, reboot persistence).
- `dotmatrix_node` splash screen: new `splash_message` parameter (default `>>> Welcome to RACECAR Neo! >>>`) scrolls once on node startup, then yields to the normal glyph / label / pixels / text render path. New `splash_period_sec` parameter (default 8.0 s) controls the scroll speed independently of the regular `scroll_period_sec`. Empty `splash_message` disables. `/dotmatrix/pixels` and `/dotmatrix/text` interrupt the splash immediately (they sit higher in the priority cascade).
- `scripts/modprobe.d/blacklist-hid-nintendo.conf` — blacklists `hid_nintendo` so the EasySMX KC-8236 gamepad downgrades to Xbox 360 mode on Pi 5. Installed by `setup_udev.sh`, which also `cmp`-gates an `update-initramfs -u` (needed because `hid_nintendo` can auto-load from initramfs before `/etc/modprobe.d/` is read).

### Fixed

- `racecar setup networking --ssid=foo --show` now persists `foo` BEFORE printing the file contents. The first cut made `--show` short-circuit before the persist step, so flags combined with `--show` were silently lost. Two-pass parse: collect every flag first, then act. Same fix path rejects `--reset` combined with override flags (those would be deleted immediately — almost certainly a user error). Regression covered by `test_networking_flag_persists_when_combined_with_show` and `test_networking_reset_with_overrides_errors`.
- `scripts/setup_networking.sh` idempotency: previously a no-op re-run still reported "Dispatcher installed" / "Connection already exists — reapplying settings" / "Applying netplan..." (touching the live AP and bouncing eth0 for nothing). Now the script:
  - Writes the dispatcher only if its content differs (`cmp` vs the live file).
  - Probes `NetworkManager-dispatcher.service` with `is-enabled --quiet`, not `is-active --quiet` (the service is `Type=simple`, so it's `inactive` between events even though it'll fire correctly).
  - Diffs each AP-connection setting against `nmcli -g` output before calling `nmcli connection modify`.
  - Only `nmcli connection up`s the AP when settings changed or the connection isn't currently activated (avoids dropping AP clients during clean re-runs).
  - Only `netplan apply`s when the netplan YAML actually changed (eliminates the noisy `systemd-networkd is not running` warning on no-op re-runs).
- `scripts/setup_networking.sh` enables `NetworkManager-dispatcher.service` if it's not already enabled. Without it, the AP-isolation dispatcher script gets installed but never invoked, so the `iptables FORWARD REJECT` rules silently never apply. On Ubuntu Server the service is enabled by default; on Raspberry Pi OS / Ubuntu Desktop it ships disabled, which is what bit us on first install.
- EasySMX KC-8236 wrong-button-mapping on cold boot: the controller spoofs Nintendo Switch Pro VID:PID (`057e:2009`), and kernels ≥ 5.16 (= every Pi 5 image) ship `hid_nintendo` which claims it and binds it as a Switch controller — A/B/X/Y swapped from Xbox, no `/dev/input/js0`, no force feedback. **Worked fine on Pi 4 only because its older kernel image lacked `hid_nintendo`.** Fix: blacklist `hid_nintendo` system-wide. With the driver out of the way, the controller's firmware times out waiting for a HID handler and downgrades itself to Xbox 360 mode (`2f24:016d`), which binds cleanly to `xpad`. Tradeoff acknowledged: a real Nintendo Switch Pro Controller wouldn't work on the racecar either — not a use case we support. Verified on-robot: `lsusb` reports `2f24:016d`, `/dev/input/js0` present, mux mode-switch via LB/RB works.
  - **Two earlier attempts were tried and removed:** (a) udev rule unbinding from `usbhid` — `hid-nintendo` re-grabbed instantly; (b) udev rule unbinding from the `nintendo` HID driver directly — same problem, the kernel re-runs match logic and `nintendo` is the only driver willing to claim `057e:2009`. The mode-switch only triggers when no driver responds at all, which requires the system-wide blacklist.

### Changed

- Bumped `<version>` 0.0.5 → 0.0.6 in `package.xml` and `setup.py`.
- README: new top-level Networking section documenting the workflow, defaults, persistence file location, and verification commands. `racecar` shell-tool list expanded to include `setup`.

## [0.0.5] — 2026-05-11

Phase 5 polish: log noise eliminated, raspi-config consolidated, real Coral latency test, README brought current with v0.0.3 + v0.0.4 features.

### Added

- `scripts/setup_raspi_config.sh` — consolidates the raspi-config flags the racecar stack depends on: `do_i2c 0` (IMU), `do_spi 0` (dot matrix), `do_serial_cons 1` + `do_serial_hw 0` (frees `/dev/serial0` for future modules; getty no longer holds the UART). Idempotent. Wired as phase 4 of `setup_all.sh` (now 11 phases).
- `config/camera_forward_info.yaml` and `config/camera_backward_info.yaml` — `sensor_msgs/CameraInfo` placeholder files referenced by `camera_info_url` in each camera config. Stops gscam from logging an ERROR + WARN on every boot trying to read a missing file. Replace via `ros2 run camera_calibration cameracalibrator …` when real calibration is needed.
- `test/test_hardware.py::TestCoral::test_inference_within_latency_budget` — loads the bundled efficientdet-lite0 model, runs 10 timed inferences with a synthetic frame, asserts mean latency < 100 ms. Skips cleanly when `edgetpu_node` is already holding the USB device. Catches Coral firmware regressions / wrong-model-format issues the existing import tests miss.

### Changed

- Bumped `<version>` 0.0.4 → 0.0.5 in `package.xml` and `setup.py`.
- `launch/teleop.launch.py` migrates `LaunchConfigurationEquals` → `IfCondition(EqualsSubstitution(...))` (the deprecated `LaunchConfigurationEquals` was logging a DeprecationWarning every teleop start).
- `config/camera_forward.yaml` and `config/camera_backward.yaml` set `camera_info_url: "package://racecar_neo_ros2_driver/config/camera_*_info.yaml"` and drop the inline placeholder fields (those have moved into the sibling `*_info.yaml` files).
- `scripts/setup_dotmatrix.sh` no longer enables SPI — that responsibility moved to `setup_raspi_config.sh`. The dotmatrix script is now just `pip install luma.led_matrix`.
- `README.md` documented Phase 3 + 4 features: 11-phase setup, the full `racecar` subcommand list (`cd`, `watchdog`, `service`, `selftest`, `cleanup`), the dashboard at `:8080`, and JupyterLab at `:8888`.

### Fixed

- gscam camera calibration noise on every boot — `Unable to open camera calibration file` ERROR + `Camera calibration file not found` WARN are gone now that both cameras have a real (placeholder) `camera_info` YAML loaded via `package://` URLs.
- `LaunchConfigurationEquals` deprecation warning on every teleop start.

## [0.0.4] — 2026-05-11

Safety + recovery infrastructure: full-stack launch wrapper, restart-on-failure watchdog, four systemd services (teleop / watchdog / dashboard / jupyter), a real-time web dashboard, and quality-of-life additions to the `racecar` tool.

### Added

**Phase 4A — Full-stack launch + wrapper:**
- `launch/teleop.launch.py` promoted to a full-stack aggregator. Always launches the control pipeline (joy / gamepad / mux / throttle / pwm) and conditionally includes each sensor / ML / display subsystem via `<name>_enable` launch arguments (default `true`). EdgeTPU is delayed 10 s for Coral USB firmware enumeration; the backward camera is delayed 5 s to stagger USB bandwidth contention.
- `scripts/launch_teleop.sh` — runtime wrapper that creates `~/logs/<YYYYMMDD_HHMMSS>/`, updates `~/logs/latest` symlink atomically, sweeps FastRTPS SHM orphans (`/dev/shm/fastrtps_port*`), exports `ROS_LOG_DIR` / `ROS_HOME`, tees stdout/stderr into `teleop.log`, and `exec`s `ros2 launch` so systemd tracks the launch PID directly.

**Phase 4B — Node watchdog:**
- `scripts/watchdog.py` — supervises 8 nodes (pwm, throttle, mux, gamepad, imu, lidar, camera_forward, camera_backward). Two-signal liveness (`ros2 topic list` + `pgrep` on the entry-point path), 30 s restart cooldown, SIGTERM → SIGKILL escalation with `pkill -f`, hardware-aware skip when the device is missing (e.g. unplugged Maestro), FastRTPS SHM orphan sweep every 60 s, Pi 5 PMIC under-voltage sticky-alarm watch. Each restart spawns `ros2 launch racecar_neo_ros2_driver <node>.launch.py` with its own log under `~/logs/latest/restart_<node>_<ts>.log`. EdgeTPU + dot matrix are intentionally out of scope (USB firmware re-load risk, non-safety-critical).
- `racecar watchdog` shell-tool entry point.

**Phase 4C — systemd services:**
- `scripts/racecar-teleop.service` — Type=exec, User=racecar, Restart=on-failure, KillMode=control-group; `ExecStart=launch_teleop.sh`. `Wants=racecar-watchdog.service` pulls the watchdog along on manual start; the watchdog's `BindsTo=racecar-teleop.service` stops it again when teleop stops.
- `scripts/racecar-watchdog.service` — `ExecStartPre=/bin/sleep 15` lets teleop settle before the watchdog first samples liveness.
- `scripts/racecar-dashboard.service` — port 8080 status page (see Phase 4E).
- `scripts/racecar-jupyter.service` — JupyterLab on port 8888 with PYTHONPATH / AMENT_PREFIX_PATH / LD_LIBRARY_PATH pre-set so `import rclpy` and `import racecar_neo_ros2_driver` work inside notebooks.
- `scripts/setup_services.sh` — idempotent installer: drops unit files in `/etc/systemd/system/`, runs `systemctl daemon-reload`, and enables each unit. Does not start them — user controls when the stack first comes up.
- `scripts/setup_jupyter.sh` — `pip install --user jupyterlab` and creates `~/jupyter_ws/` with a starter README.
- `setup_all.sh` now orchestrates 10 phases (added `setup_jupyter.sh` and `setup_services.sh`).

**Phase 4E — Web dashboard:**
- `scripts/dashboard.py` — stdlib-only HTTP server on `0.0.0.0:8080`. Background thread polls `ros2 node list` / `ros2 topic list` and measures `ros2 topic hz` for 7 key topics in parallel; cached snapshot served as JSON at `/api/status` and rendered as a single-page dashboard at `/`.
- 10 node-status cards (one per monitored subsystem including edgetpu + dotmatrix); 7 topic-rate rows (`/motor`, `/mux_out`, `/imu`, `/scan`, `/camera/forward`, `/camera/backward`, `/edgetpu/inference`); System Health cards (RTC backup-battery voltage via `vcgencmd pmic_read_adc BATT_V` with green/yellow/red thresholds at 3.0 V / 2.7 V, and the Pi 5 PMIC sticky under-voltage alarm); live tail of `~/logs/latest/watchdog.log`.
- System Health diagnostics refresh on a separate 60 s cadence (slow-changing — avoids hammering vcgencmd / hwmon every 3 s).
- `scripts/dashboard.html` — HTML template lives in a separate file (so flake8 doesn't drown in long-line warnings on embedded CSS / JS). Auto-refreshes every 3 s.

**`racecar` tool additions:**
- `racecar cd` — chdir to the package source root (function, not subprocess, so the cd sticks in the user's shell).
- `racecar service <action>` — `install`, `start`, `stop`, `restart`, `enable`, `disable`, `logs <unit>`, `status`, `help`. Default action `status` lists `active=` and `enabled=` per unit. Tab-completion offers the action set and (for relevant actions) the unit list `teleop / watchdog / dashboard / jupyter`.
- `racecar cleanup [--dry-run | --force]` — list / kill stale racecar processes and FastRTPS SHM orphans. Uses sudo for root-owned PIDs when forced. Dry-run by default so it's safe to alias to a keybind.
- `racecar watchdog` — runs the supervisor in the foreground (logs to `~/logs/latest/watchdog.log`).
- `racecar teleop` now invokes `launch_teleop.sh` instead of `ros2 launch` directly so users get the same log dir / SHM cleanup as systemd-managed runs.

**Tests (now 327 total):**
- `test/test_watchdog.py` — NODES dict schema for all 8 nodes (required keys, topics start with `/`, launch files exist, callable checks); camera kill-pattern disambiguation; restart cooldown sanity; helpers (`_clean_fastrtps_orphans`, `_is_running`, `_find_rpi_volt_alarm`).
- `test/test_dashboard.py` — `MONITORED` covers all 10 subsystems; `RATE_TOPICS` are a subset of monitored publishers; `get_status()` returns JSON-serializable snapshot with required keys; HTML template is present and references `/api/status`; title says RACECAR Neo (regression guard against UAV Neo leftover); `_classify_rtc` thresholds (3.0 V healthy, 2.7 V stale, below 2.7 V dead); `_collect_system_health` returns both `rtc` and `under_voltage` entries with valid statuses; port is 8080.
- `test/test_setup_scripts.py::TestSystemdServices` — all four `.service` files exist, contain `[Unit] [Service] [Install]`, `WantedBy=multi-user.target`, `User=racecar`, `BindsTo` + `After` on watchdog, `Wants=racecar-watchdog.service` on teleop, correct ExecStart referents.
- `test/test_setup_scripts.py::TestLaunchWrapper` — `launch_teleop.sh` exists + executable, `bash -n` clean, creates log dir + symlink, sweeps FastRTPS SHM, `exec`s ros2 launch.
- `test/test_racecar_tool.py` — new tests for `cd`, `cleanup`, `service` (status/help/error paths).
- `test/test_hardware.py::TestGamepad` — replaces `TestEasySMX`; accepts both joydev (`/dev/input/jsN`) and evdev (`/dev/input/eventN`) so the test passes against the user's Switch Pro Controller (which only exposes evdev).

### Changed

- Bumped `<version>` 0.0.3 → 0.0.4 in `package.xml` and `setup.py`.
- `scripts/setup_all.sh` orchestrator: 8 → 10 phases.
- `scripts/setup_dotmatrix.sh` adds SPI enable via `raspi-config nonint do_spi 0` (no-op on machines without raspi-config).

### Skipped

- **Phase 4D — image_relay.py** — UAV Neo's QoS-matched relay shim is a 30-line stdlib script worth porting only when something actually needs a QoS-adapted republish. Nothing in the racecar stack currently does (gscam publishes directly to `/camera/forward` with sensor_data QoS, which `edgetpu_node` subscribes to with matching QoS). Deferred; will land if and when a consumer needs it.

## [0.0.3] — 2026-05-11

ML inference, dot matrix display, stable device paths, and a unified `racecar` developer CLI.

### Added

**Phase 3A — Coral EdgeTPU object detection:**
- `edgetpu_node` — subscribes to `/camera/forward`, runs object detection on the USB Coral, and publishes `vision_msgs/Detection2DArray` on `/edgetpu/inference` plus a heartbeat `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
- Numpy-only image path — no `cv_bridge` / `cv2` dependency. PIL bilinear resize keeps the package opencv-free
- SSD-style output-tensor auto-detection (`map_output_tensors`) — works with any 4-output EfficientDet-Lite / SSD-MobileNet variant; no hardcoded output indices
- Retry-once `make_interpreter` to absorb the cold-boot Coral firmware load (USB ID flip from `1a6e:089a` to `18d1:9302`)
- `config/edgetpu.yaml` — model + labels paths, score threshold, max detections, image topic, diagnostics period
- `launch/edgetpu.launch.py` — standalone launch (watchdog restart target)
- `scripts/setup_coral.sh` — idempotent userspace install: `libedgetpu1-std.deb` + `tflite_runtime` and `pycoral` wheels (all vendored under `depend/`)
- Bundled model: `models/efficientdet_lite0_generic_edgetpu.tflite` + `models/labels.txt`

**Phase 3B — MAX7219 dot matrix driver:**
- `dotmatrix_node` — three input topics with priority (highest first):
  - `/dotmatrix/pixels` (`std_msgs/UInt8MultiArray`) — 8×24 row-major pixel array for arbitrary frames. Non-zero is on; values stale after `pixels_timeout_sec` (default 5 s) revert to the next priority.
  - `/dotmatrix/text` (`std_msgs/String`) — renders in `proportional(TINY_FONT)` with a patched diagonal `N` glyph; auto-scrolls when wider than the 24 px viewport (uses true `rendered_text_width`, not the over-counting `text_pixel_width`, so short messages render static).
  - Fallback — 8×8 pictographic mode glyph on the leftmost module (IDLE = pause bars, GAMEPAD = steering wheel, AUTONOMY = play triangle) plus a centered `IDLE` / `MAN` / `AUTO` text label on the right two modules. `MAN` is centered (per-mode origin precomputed) since `MANUAL` is 23 px (too wide for the 16 px label region).
- `config/dotmatrix.yaml` + `launch/dotmatrix.launch.py` — defaults to 3 cascaded modules (24×8 viewport on this robot)
- Module-level helpers `mode_glyph`, `mode_label`, `draw_glyph`, `decode_pixel_array`, `text_pixel_width`, `rendered_text_width`, `scroll_offset` for unit testing
- `scripts/dmatrix_patterns.py` — self-test pattern publisher (checkerboard, all-on, sweep, module-id, font A-Z 0-9 in static 6-char chunks)

**Phase 5B (pulled forward) — udev rules:**
- `scripts/udev/99-racecar.rules` — stable symlinks `/dev/maestro`, `/dev/lidar`, `/dev/cam_forward`, `/dev/cam_backward`. Maestro rule pins `ID_USB_INTERFACE_NUM=00` so the symlink always binds the command CDC port (not the auxiliary TTL one). Arducam autosuspend disabled. Coral pre/post-init USB IDs get `GROUP="plugdev"`.
- `scripts/setup_udev.sh` — installs the rules and triggers a reload. Wired in as phase 4 of `setup_all.sh` (now 8 phases).
- `config/pwm.yaml`, `config/lidar.yaml`, `config/camera_forward.yaml`, `config/camera_backward.yaml` updated to reference the stable symlinks instead of `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/video{0,4}`.

**`racecar` developer shell tool:**
- `scripts/racecar-tool.sh` — single `racecar` shell function exposing: `build`, `test`, `source`, `teleop`, `launch <name>`, `clear --dmatrix`, `udev`, `selftest --dmatrix[=<pattern>]`, `status`, `help`
- `selftest --dmatrix` runs hardware patterns through the live `dotmatrix_node` via `/dotmatrix/pixels` (checkerboard / all-on / sweep / module-id / font / all)
- Tab completion for subcommands; `racecar launch <TAB>` discovers launch files dynamically; `racecar clear --<TAB>` and `racecar selftest --<TAB>` offer their flags
- Extra args forward through (e.g. `racecar build --cmake-args ...`, `racecar launch dotmatrix dotmatrix_config:=/tmp/x.yaml`)
- `scripts/setup_user_env.sh` now sources `racecar-tool.sh` from `~/.bashrc` instead of installing five `racecar-*` aliases; cleans up legacy aliases on re-run

**Tests (now 206 total):**
- `test/test_dotmatrix.py` — glyph shapes, mode→bitmap mapping, label mapping (with rendered-width check), patched-N glyph integrity, `decode_pixel_array` (rgb8 + truncate + pad + bytes input), text width helpers, scroll math
- `test/test_edgetpu.py` — `image_msg_to_rgb` (rgb8 + bgr8), `resize_rgb` (PIL bilinear), `load_labels`, `map_output_tensors`
- `test/test_dmatrix_patterns.py` — pure-helper tests for checkerboard / all-on / sweep / module-id pattern generators
- `test/test_racecar_tool.py` — `bash -n` syntax, function definition, help rendering, error paths (`unknown command`, missing args, unknown flag), completion installation, `selftest` flag validation
- `test/test_setup_scripts.py::TestUdevRules` — rules file existence, symlink declarations, known VID:PID matches, Maestro `ID_USB_INTERFACE_NUM=00` pinning

### Changed

- Bumped `<version>` 0.0.2 → 0.0.3 in `package.xml` and `setup.py`
- `scripts/setup_all.sh` now orchestrates 8 phases (added `setup_udev.sh` + `setup_coral.sh`)
- `scripts/setup_dotmatrix.sh` now also runs `raspi-config nonint do_spi 0` for fresh installs
- `scripts/clear_dotmatrix.py` default `--cascaded` 4 → 3 to match physical hardware
- `dotmatrix_node` text path uses `TINY_FONT` (was `CP437_FONT`) so /dotmatrix/text fits more chars static; uses true `rendered_text_width` to decide scroll vs static (was the over-counting `text_pixel_width`)
- `setup.py` `data_files` ships `models/` to the install share so `model_path: "models/..."` resolves correctly via `get_package_share_directory`
- `test/test_hardware.py` — Maestro, RPLIDAR, BRIO, and Arducam classes now check the udev symlinks instead of raw `/dev/tty*` / `/dev/video*` paths

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

[Unreleased]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.6...HEAD
[0.0.6]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.5...v0.0.6
[0.0.5]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.4...v0.0.5
[0.0.4]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.3...v0.0.4
[0.0.3]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.2...v0.0.3
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
