# Lab dashboards

Internals of the lab dashboards in `scripts/dashboards/`: webteleop
(8081), linefollow (8082), and wallfollow (8083). camlabel (8085) is
described in its own README. Install and use: README,
"Lab dashboards". Unit install: `scripts/setup_dashboards.sh`.

## Contents

- [Shared structure](#shared-structure)
- [Tuning and logs](#tuning-and-logs)
- [Lidar convention](#lidar-convention)
- [webteleop](#webteleop)
- [linefollow](#linefollow)
- [wallfollow](#wallfollow)
- [Tests](#tests)

## Shared structure

Each dashboard is one Python process: a ROS node plus a stdlib HTTP server
(`ThreadingHTTPServer`), with the whole browser side in one HTML file.

| Piece | Responsibility |
|---|---|
| `<name>.py` | The node (subscriptions, the `/drive` timer), the HTTP handler, and pure helpers the tests reach without a node |
| `<name>.html` | The page: controls, canvases, charts; polls `GET /state` |
| `<name>.yaml` | Shipped defaults for every tunable |
| `racecar-<name>.service.in` | Unit template; `@DIR@` is substituted at install |

`/drive` comes from a timer (15 Hz), never from a browser post. A command older
than `cmd_timeout` reads as released, so a closed tab, a dropped link, or a
page that lost focus with a key down coasts to a stop instead of driving the
last command forever.

Steering is negated on publish: positive on the wire turns this car left
(verified on the car).

Three threads share the state: the ROS executor, the HTTP request threads, and
the main thread in `serve_forever`. Everything shared sits behind one `_lock`,
never held across an encode or a network write. `rclpy.init()` installs signal
handlers that stop ROS but leave `serve_forever` blocked, so `main` takes the
signals back and calls `shutdown` off the serving thread.

## Tuning and logs

Tuning loads `<name>.yaml`, then `<name>.local.yaml` on top when it exists.
Save writes only `<name>.local.yaml` (seeded from the defaults on the first
save) and rewrites its numbers in place so the comments survive. The local
file is gitignored, like `config/*.local.yaml`, so a car's tuning never dirties
the tree.

CSV logs go to `~/data/dashboards/<name>/` and are listed and downloaded
through the page.

## Lidar convention

`LIDAR_MOUNT_YAW_DEG` is the raw scan angle that points at the car's nose:

```
student_deg = LIDAR_MOUNT_YAW_DEG - raw_deg
```

The RPLIDAR faces aft over 360 degrees at 1080 points, so the nose sits at the
raw 180 edge and the constant is `180.0`. An object in front reports at raw
180, one off the car's right at raw +90: a yaw, not a handedness flip, so the
order around the circle is kept and only the origin moves. Indices and angles
come from each message's `angle_min` and `angle_increment` and wrap modulo the
point count, so nothing assumes a sweep width. webteleop and wallfollow carry
the constant; linefollow does not read the lidar.

## webteleop

Drive from buttons or the keyboard, with every sensor in view: color and
depth previews with the object detector's boxes, the lidar, encoder, IMU,
battery, and compass, plus dot matrix and LED strip editors. The cards match
the simulator's sensor dashboard, read from ROS topics instead of
`racecar_core`.

```
/camera/color ─┐
               ├─▶ _image_cb ──▶ resize ──▶ JPEG ──▶ _preview ──────▶ GET /frame
/camera/depth ─┴─▶ _depth_cb ──▶ resize ──▶ _colorize_depth ──▶ JPEG ─▶ GET /depth

/edgetpu/inference ─▶ _det_cb ──▶ normalize against _state['res'] ─┐
/scan ─────────────▶ _scan_cb ─▶ _student_deg, thin by 3 ─────────┤
/odom ─────────────▶ _odom_cb ────────────────────────────────────┼─▶ _state ─▶ GET /state
                                                                   │
browser buttons/keys ─▶ POST /cmd ─▶ _cmd ─┐                       │
                                            ├─▶ _actuate (15 Hz) ──┘
                            tuning ─────────┘         │
                                                      ▼
                                              /drive  AckermannDriveStamped
```

- Each preview has its own loop, so a stalled depth stream costs only its own
  frame rate. Depth resizes nearest neighbor: averaging a reading against a
  zero would invent a surface halfway to a hole.
- Detections leave as fractions of the frame and are scaled in the browser to
  the drawn preview. They expire after `DET_TIMEOUT_S`, so a stale box never
  sits on a live frame.
- `/imu/fused`, `/mag`, `/dotmatrix/frame`, and `/led/pixels` are raw
  subscriptions that keep only the newest message; `_actuate` decodes them at
  15 Hz into `_sense` (10 s of acceleration, angular velocity, and field) and
  the state. `/battery/state` arrives at 1 Hz.
- POST /matrix, /text, and /leds record a frame in `_display`; `_actuate`
  re-sends it inside the driver's timeouts (`MATRIX_RESEND_S`,
  `LED_RESEND_S`) until POST /display/release, or until the page has not
  polled /state for `DISPLAY_HOLD_S`.
- Charts are drawn by the shared `/ui/charts.js`.

## linefollow

Follow a colored line with the camera; HSV thresholds tuned in the page.

```
/camera/color ─▶ _image_cb ─▶ resize to process_width ─▶ HSV ─▶ inRange(low, high)
                                                                     │
                                       ┌─────────────────────────────┤
                                       ▼                             ▼
                              crop to the band              mask preview JPEG
                                       │                             │
                        largest contour above min_area               ▼
                                       │                        GET /mask
                              its center column
                                       │
                         error = (center - width/2) / (width/2)   -1 .. +1
                                       │
                    ┌──────────────────┴───────────────────┐
                    ▼                                      ▼
        steer = angle_kp * error                 annotated color JPEG
              + angle_kd * d(error)                        │
        speed = speed - speed_kp * |error|                 ▼
                    │                                 GET /frame
                    ▼
          _actuate (15 Hz) ─▶ /drive  AckermannDriveStamped

/odom ─▶ _odom_cb ─▶ measured speed ─▶ _state ─▶ GET /state
```

- With no line in the band the last command is held.
- The mask preview shows the whole frame, not only the band: a threshold can
  only be judged against everything the camera sees.
- The offset chart's axis is fixed at -1 to 1, so a small wobble looks small.

## wallfollow

Follow walls with the lidar; two controllers, chosen by `mode` in the YAML (no
switch in the page, since they read the tune panel differently).

```
                          tuning  mode: 0 static | 1 dynamic
                                    │
/scan ─▶ _scan_cb ─▶ _student_deg ──┤
                                    │
      ┌─────────────────────────────┴───────────────────────────┐
      ▼                                                         ▼
   STATIC                                                    DYNAMIC
   _mean_at(+look_angle, window)          candidate headings every SEARCH_STEP
   _mean_at(-look_angle, window)          across +/- width of the nose
      │                                              │
   error = right - left  (setpoint 0)      obstacles inflated by HALF_CLEAR
      │                                    score = clearance
      │                                          - CENTER_BIAS * |deg|
      │                                          + side_weight * deg
      │                                              │
      │                                    error = best heading
      └──────────────────────┬───────────────────────┘
                             ▼
        steer = kp * error + kd * d(error)          overlay ─▶ _state
        speed = slider * max_mps
              - speed_kp * |error| (when enabled)          │
                             │                             ▼
                             ▼                        GET /state
                  /drive  AckermannDriveStamped

/odom ─▶ _odom_cb ─▶ measured speed ─▶ _state
```

- Distances are capped at `lookahead`, so an open doorway reads as `lookahead`
  rather than 12 m of empty room and does not dominate the score.
- The lidar view draws the rays the controller used and the heading it chose,
  so a wrong turn is diagnosed by seeing which ray it believed.

## Tests

`test/test_dashboard_teleop.py`, `test_dashboard_linefollow.py`, and
`test_dashboard_wallfollow.py` load each module by path and need no ROS graph,
camera, or browser.
