# v0.0.7 Functionality Audit — On-Robot Test Checklist

Walk through this on the actual robot after the v0.0.7 build lands. The audit
touched safety-relevant code (mux arming gate, Maestro device path, gamepad
clipping), the watchdog + dashboard internals (rclpy rewrite), all 9 launch
files (helper consolidation), and the networking setup script (step reorder).
Most regressions would surface here.

**Run order matters**: phases are listed in increasing risk-to-the-robot
order. Stop and root-cause if any earlier phase fails — don't paper over a
boot-arming-gate bug by skipping ahead to the on-road tests.

Per-section checkboxes are pre-checked (`[X]`) when copy-paste verified;
flip to `[ ]` if you spot a regression, and **stop**.

## 0. Pre-flight (no hardware risk)

- [X] Build is clean
  ```sh
  cd ~/ros2_ws && colcon build --packages-select racecar_neo_ros2_driver
  ```
  Expected: `Finished` with no errors. Pi 5 ≈ 5 s.

- [X] Unit tests all pass
  ```sh
  racecar test
  ```
  Expected: `364 passed, 2 skipped`. The two skipped are EdgeTPU/Coral
  hardware tests that only run when the device is free.

- [X] Version bump landed
  ```sh
  grep version setup.py package.xml
  ```
  Expected: `0.0.7` in both files.

## 1. Launch helper sanity — all 9 launch files still parse

The audit collapsed 9 launch files behind `single_node_launch()` in
`racecar_neo_ros2_driver/launch_common.py`. A regression here would prevent
the watchdog from restarting any node.

- [X] Every per-node launch file shows its config arg
  ```sh
  for lf in pwm mux throttle gamepad lidar dotmatrix edgetpu camera_forward camera_backward; do
      echo "=== $lf ==="
      ros2 launch racecar_neo_ros2_driver "${lf}.launch.py" --show-args | head -3
  done
  ```
  Expected: each prints `'<name>_config':` as the first arg. No tracebacks.

- [X] IMU launch (only outlier — two YAML param files) still works
  ```sh
  ros2 launch racecar_neo_ros2_driver imu.launch.py --show-args
  ```
  Expected: shows both `imu_cal` and `imu_mag_cal` args.

- [X] Teleop aggregator still resolves every subsystem
  ```sh
  ros2 launch racecar_neo_ros2_driver teleop.launch.py --show-args | grep _enable
  ```
  Expected: 6 lines — `imu_enable`, `lidar_enable`, `camera_forward_enable`,
  `camera_backward_enable`, `edgetpu_enable`, `dotmatrix_enable`, all
  defaulting to `'true'`.

## 2. Maestro device path — `/dev/maestro` not `/dev/ttyACM0`

The pwm_node default was changed from `/dev/ttyACM0` → `/dev/maestro`. The
YAML always wins, but anyone running `ros2 run` without `--params-file`
would previously have grabbed whatever ACM0 enumerated to first.

- [X] udev symlink is alive
  ```sh
  ls -l /dev/maestro
  ```
  Expected: symlink → `ttyACM*`. If absent, `racecar setup` Phase 5 (udev) was
  never run on this robot.

- [X] Direct invocation now uses the right device
  Pull the Maestro power for a moment if you can do it safely, then:
  ```sh
  ros2 run racecar_neo_ros2_driver pwm_node 2>&1 | head -3
  ```
  Expected: a `SerialException: could not open port /dev/maestro` —
  confirming it's looking at the symlink, not at whatever ACM0 happens to be.
  Plug Maestro back in.

- [X] YAML-driven launch unchanged
  ```sh
  ros2 launch racecar_neo_ros2_driver pwm.launch.py
  ```
  Expected: `PWM ready: motor ch=0 ... steering ch=1 ...` log line. Ctrl-C.

## 3. Mux boot-time arming gate — the headline safety fix

The mux now publishes zero until (a) `startup_grace_sec` (1 s) has elapsed AND
(b) it has seen one `/joy` frame whose every axis magnitude is below
`arm_axis_threshold` (0.2). Defends against a stuck stick on power-on.

**Test with the EasySMX deflected on purpose.** If you can't reproduce a
stuck-stick condition, hold the throttle stick all the way forward when you
power-cycle the bench.

- [X] Centered controller: arms within ~1 s, normal operation
  ```sh
  # Bench. Wheels off ground. EasySMX powered ON, sticks released.
  racecar service start teleop
  ros2 topic echo /mux_out --once   # wait ~2 s after service start
  ```
  Expected: drive speed/steering both 0.0 (mux is publishing zeros until LB
  or RB is pressed — but is armed). Press LB and gently flick throttle:
  motors respond. Log line `Mux armed` appears in `journalctl -u racecar-teleop`.

- [X] **Deflected stick at boot: stays in IDLE**
  ```sh
  # 1. Stop the stack
  racecar service stop teleop
  # 2. Hold the throttle stick fully forward (or wedge it)
  # 3. With stick still deflected, restart:
  racecar service start teleop
  # 4. Watch the mux_out stream
  ros2 topic echo /mux_out
  ```
  Expected: `drive.speed: 0.0` indefinitely. **Wheels do not move.** No
  `Mux armed` log line appears. Release the stick → within ~50 ms (one timer
  tick at 50 Hz) the mux logs `Mux armed`. Subsequent LB-throttle works.

- [X] Cooldown after deflected stick: still armed after release
  ```sh
  # Continuing from previous: stick now released, mux just armed.
  # Press LB, gently flick stick. Then release. Then deflect again.
  ```
  Expected: arming is one-shot. Once armed in a session, deflecting the
  stick again is treated as a normal command, not as a re-arm requirement.

- [X] Tunable threshold works
  ```sh
  ros2 param set /mux_node arm_axis_threshold 0.01
  ```
  Expected: returns `Successful: true`. (Won't take effect until next
  process start — declared in `__init__`.)

## 4. Gamepad input clipping

The gamepad node now clips to `[-1, 1]` before publishing. Hard to trigger
from a sane controller, but verify the contract holds.

- [X] Normal range passes through
  ```sh
  ros2 topic echo /gamepad_drive --field drive.speed
  ```
  With teleop running, max-throttle the stick. Expected: values in `[-1.0, 1.0]`,
  never exceeding bounds.

- [X] Misconfigured sign multiplier doesn't escape
  ```sh
  ros2 param set /gamepad_node throttle_sign 2
  ```
  (Param won't take effect until next process start, but if you want to
  prove the clip works:)
  ```sh
  # In a fresh shell — bypasses YAML
  ros2 run racecar_neo_ros2_driver gamepad_node --ros-args -p throttle_sign:=2
  # In another shell:
  ros2 topic echo /gamepad_drive --field drive.speed
  ```
  Press the stick fully forward. Expected: speed = `1.0`, not `2.0`. (The
  full-stick raw axis is ~1.0; sign=2 would naively produce 2.0.)
  Ctrl-C, set `throttle_sign` back to `1` in `config/gamepad.yaml`.

## 5. Watchdog — rclpy introspection replaces `ros2 topic list` subprocess

Internal: the watchdog now spins an rclpy `Node` (`racecar_watchdog`) and
calls `node.get_topic_names_and_types()` in-process instead of forking
`ros2 topic list` every 5 s. Behavior should be identical.

- [X] Watchdog starts cleanly
  ```sh
  racecar service start teleop   # pulls watchdog along
  sleep 20                        # 15s startup grace + first poll
  journalctl -u racecar-watchdog --since "1 minute ago" --no-pager | head -10
  ```
  Expected: `Watchdog started — monitoring: pwm, throttle, mux, ...`. Then
  a log line per node that's alive. **No `Failed to query ros2 topic list`
  warnings.** No tracebacks.

- [X] `racecar_watchdog` node is visible
  ```sh
  ros2 node list | grep watchdog
  ```
  Expected: `/racecar_watchdog`.

- [X] Kill-and-restart still works (regression of v0.0.4 contract)
  ```sh
  pkill -f "racecar_neo_ros2_driver/imu_node"
  sleep 35  # cooldown 30s + poll 5s
  ros2 topic hz /imu --window 3
  ```
  Expected: IMU topic recovers. `journalctl -u racecar-watchdog` shows
  `imu: process not running — device LSM9DS1 @ I2C bus 1 addr 0x6B
  connected, attempting restart` followed by `imu: launched PID NNN`.

- [X] pgrep failure counter — ensure the new escalation logic doesn't
      false-positive
  ```sh
  journalctl -u racecar-watchdog --since "10 minutes ago" --no-pager | \
      grep -E "pgrep.*failed"
  ```
  Expected: no output (under normal operation, pgrep just works). If you
  see `pgrep(...) failed (N/5)` warnings, investigate before deploying.

- [X] Per-restart log handles close cleanly — check the parent's open FD count
      doesn't grow over a long run
  ```sh
  WD_PID=$(pgrep -f scripts/watchdog.py)
  # Force a few restarts:
  for i in 1 2 3; do
      pkill -f "racecar_neo_ros2_driver/imu_node"
      sleep 35
  done
  ls /proc/$WD_PID/fd | wc -l
  ```
  Expected: the FD count is the same (±1) before and after the restart
  burst. Prior versions would leak one FD per restart.

## 6. Dashboard — rclpy subscriptions replace `ros2 topic hz` thread storm

Internal: dashboard now spins an rclpy `_RateSampler` node (`racecar_dashboard`)
and uses BEST_EFFORT subscriptions + a deque to compute Hz, instead of
spawning 7 parallel `ros2 topic hz` subprocesses every 3 s.

- [X] Dashboard process is alone — no orphaned `ros2 topic hz` children
  ```sh
  racecar service start dashboard
  sleep 10
  pgrep -af "ros2 topic hz"
  ```
  Expected: **no output**. The old code accumulated these as zombies.

- [X] `racecar_dashboard` node is visible
  ```sh
  ros2 node list | grep racecar_dashboard
  ```
  Expected: `/racecar_dashboard`.

- [X] Status page renders rates correctly
  Browse to `http://racecar-neo.local:8080`. Expected: every topic in the
  rate table has a real Hz number (matching `ros2 topic hz` if you check
  in parallel). Wait a full minute on first load — the rolling window
  takes one window-length (3 s) to fill but late-binding to new
  publishers can take a tick or two.

- [ ] Unsupervised nodes render distinctly
  Stop edgetpu manually:
  ```sh
  pkill -f "racecar_neo_ros2_driver/edgetpu_node"
  ```
  Refresh the dashboard. Expected: EdgeTPU card shows status
  `'unsupervised'` (NOT `'dead'`). Dotmatrix likewise if you kill it. The
  CSS may need a frontend update to colorize the new status — open a
  followup if so; the JSON contract is in place.

- [ ] Watchdog log tail still works after the bounded-read change
  Dashboard `Last 10 log lines` panel should match
  `tail -n 10 ~/logs/latest/watchdog.log`. Should NOT lag, even after a
  multi-hour session when `watchdog.log` is large (run the dashboard
  service across a full bench session if you want to confirm).

## 7. Dot matrix — text width caching

`rendered_text_width()` is now memoized. The matrix should still look identical.

- [ ] Splash + mode labels render correctly
  ```sh
  racecar service restart teleop
  ```
  Watch the matrix:
  1. Splash `>>> Welcome to RACECAR Neo! >>>` scrolls once.
  2. Then idle glyph + `IDLE` label, centered.
  3. Press LB on the gamepad: matrix switches to teleop glyph + `MAN` label.
  4. Press RB: switches to play-triangle glyph + `AUTO` label.

- [ ] Custom text via `/dotmatrix/text` still renders
  ```sh
  ros2 topic pub --once /dotmatrix/text std_msgs/String "data: 'hello'"
  ```
  Expected: `hello` shown static (short enough to fit). Then:
  ```sh
  ros2 topic pub --once /dotmatrix/text std_msgs/String \
      "data: 'this message is long enough to need scrolling'"
  ```
  Expected: scrolls horizontally with smooth pacing (cache key includes
  the message string, so the new message gets a fresh width measurement).

- [ ] Clear it back out
  ```sh
  ros2 run racecar_neo_ros2_driver clear_dotmatrix
  ```

## 8. Networking script — destructive step reorder

`setup_networking.sh` now does AP setup + eth0 netplan BEFORE deleting the
prior Wi-Fi client connection. **This test is destructive to network
state — run it from a wired (eth0) connection or the console.**

- [ ] Step labels match new order
  ```sh
  grep -E '^echo "\[' scripts/setup_networking.sh
  ```
  Expected:
  ```
  [1/4] Installing AP isolation dispatcher ...
  [2/4] Configuring AP connection ...
  [3/4] Configuring eth0 dual-IP ...
  [4/4] Removing any prior Wi-Fi client connection on wlan0 ...
  ```

- [ ] Re-running is still idempotent
  ```sh
  racecar setup networking
  ```
  Expected: every step says "already up to date" / "already matches" / "No
  prior Wi-Fi client connections found", and exit code 0. NO `netplan apply`
  on a no-op re-run.

- [ ] Recovery-from-failure scenario (only if you can spare a reboot)
  Edit `scripts/setup_networking.sh` to inject a `false` after step 3 (eth0
  netplan apply, before the new step 4). Run `racecar setup networking`.
  Expected: script exits non-zero AFTER eth0 netplan succeeds but BEFORE
  the prior Wi-Fi client is deleted. Verify the prior client connection is
  still in `nmcli connection show`. Revert the edit.

## 9. Bash hardening — `pipefail` is now set

All 12 phase scripts + the orchestrator should have `set -eo pipefail`.

- [ ] Quick verify
  ```sh
  grep -L pipefail scripts/setup_*.sh scripts/launch_teleop.sh \
      scripts/patch_gscam.sh
  ```
  Expected: **no output** (every script has it).

- [ ] Phase scripts still re-run cleanly under the stricter setting
  ```sh
  racecar setup       # alias for setup_all.sh, 11 phases
  ```
  Expected: each phase reports "skipped" / "already installed" / "no
  change" and exits 0. If any phase trips on a previously-tolerated pipe
  failure, that's a real bug surfaced by pipefail — root-cause it.

## 9b. Lidar/ModemManager hardening (regression from 2026-05-12 endurance)

The lidar's CP2102 is now `ID_MM_DEVICE_IGNORE`'d in udev, and the watchdog
has a per-topic freshness check (5 s for `/scan`) that catches silent
desync — process up + topic advertised but no messages.

- [ ] udev rule installed and applied
  ```sh
  grep ID_MM_DEVICE_IGNORE /etc/udev/rules.d/99-racecar.rules
  ```
  Expected: one line matching the lidar entry. If absent, re-run
  `racecar udev` and unplug+replug the lidar.

- [ ] ModemManager no longer probes the lidar
  ```sh
  sudo udevadm info /dev/lidar | grep ID_MM_DEVICE_IGNORE
  ```
  Expected: `E: ID_MM_DEVICE_IGNORE=1`. Trigger the failure mode that started
  the incident:
  ```sh
  sudo systemctl restart ModemManager
  sleep 5
  journalctl -u ModemManager --since "10s ago" | grep usb4
  ```
  Expected: **no** "couldn't check support for device .../usb4/4-2" line
  (the lidar's USB path). Confirm `/scan` is still publishing afterward:
  ```sh
  ros2 topic hz /scan
  ```
  Expected: ~10 Hz, sustained.

- [ ] Watchdog freshness check fires on a stalled lidar
  Simulate the stall by suspending the sllidar process so its FD stays open
  but it stops reading:
  ```sh
  pkill -STOP -f sllidar_node
  # Wait ~10s (5s freshness + one poll interval).
  journalctl -u racecar-watchdog --since "20s ago" --no-pager | tail -10
  ```
  Expected: `lidar: topic stale (N.Ns) — device /dev/lidar (RPLIDAR)
  connected, attempting restart`, then the restart pipeline. (If sllidar
  doesn't recover after `pkill -CONT`, the watchdog's own SIGTERM+SIGKILL
  pass should still bring up a fresh node.) Continue once `/scan` is back.

  Cleanup if needed: `pkill -CONT -f sllidar_node` then restart teleop.

## 10. End-to-end road test (after all above pass)

Wheels back on, robot on the ground, in a clear space.

- [ ] Power on with sticks centered: arms and drives normally.
- [ ] Power off, deflect throttle stick, power on: stays idle. Release →
      arms → drives.
- [ ] Drive for ~30 s in a figure-8. Mux mode-switch between LB/RB works
      as expected. No mid-drive watchdog restarts.
- [ ] Stop, check `~/logs/latest/watchdog.log` — should be quiet
      (informational lines only, no restart attempts).
- [ ] Dashboard at `http://racecar-neo.local:8080` showed live rates the
      whole time, no `dead`-status glitches for supervised nodes.

## Rollback

If anything in §3 (mux arming gate) or §2 (Maestro path) misbehaves on the
real robot:

```sh
racecar service stop teleop
cd ~/ros2_ws/src/racecar_neo_ros2_driver
git checkout main           # or git checkout v0.0.6
cd ~/ros2_ws
colcon build --packages-select racecar_neo_ros2_driver
racecar service start teleop
```

File an issue with the failing step, the log line(s), and which test
exposed it.
