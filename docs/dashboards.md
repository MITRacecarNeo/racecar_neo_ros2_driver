# Lab dashboards

Internals of the lab dashboards in `scripts/dashboards/`: webteleop
(8081), linefollow (8082), and wallfollow (8083). camlabel (8085) is
described in its own README. Install and use: README,
"Lab dashboards". Unit install: `scripts/setup_dashboards.sh`.

## Contents

- [Shared structure](#shared-structure)
- [Tuning and logs](#tuning-and-logs)
- [Lidar convention](#lidar-convention)
- [New app checklist](#new-app-checklist)
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

Each dashboard is an entry in `config/apps.yaml` with its `dir` and `script`;
its unit is rendered from `scripts/dashboards/racecar-app.service.in`.

`/drive` comes from a timer (15 Hz), never from a browser post. A command older
than `cmd_timeout` reads as released, so a closed tab, a dropped link, or a
page that lost focus with a key down coasts to a stop instead of driving the
last command forever.

Steering is negated on publish: positive on the wire turns this car left
(verified on the car).

The handler is a `webui.AppHandler` and `main` ends in `webui.serve()`
(`scripts/webui.py`, the same for every app): HTTP/1.1 keep-alive, the
request body read before `do_POST`, `/ui/` assets, pages with `{{asset}}`
and `{{home}}` filled in, a threaded server, and a ROS spin thread that
survives an error. Three threads share the state: the ROS executor, the HTTP
request threads, and the main thread in `serve_forever`. Everything shared
sits behind one `_lock`, never held across an encode or a network write.

The page side is shared too (`scripts/web/`): the app-page section of
`rcn.css` (`rcn-main`, `rcn-section`, `rcn-tools`, `rcn-state`, `rcn-zoom`,
`rcn-slider`, `rcn-toast`, `data-tip`), `ui.js` (`RcnUi`: element, escape,
pills, canvas colors from the CSS tokens, live camera frames, toasts),
`api.js` (`RcnApi`: POST JSON, and a `/state` poll with one request in flight
that stops while the tab is hidden), `tune.js` (`RcnTune.sliders`: live
sliders that send at most one `POST /params` per 60 ms and follow what the
app clamped) and `logs.js` (`RcnLogs.mount`: the Logs section). A page keeps
its grid columns and what only it draws.

## Tuning and logs

Both come from `scripts/labkit.py`; each dashboard makes a `Tuning` (its
tunables, a clamp function, whole-number and file-only keys) and a `RunLog`
(its CSV header) and passes them its `_lock`.

Tuning loads the shipped `<name>.yaml`, then `dashboards/<name>.yaml` in the
car profile (`~/.config/racecar`, `racecar_neo_ros2_driver/car_profile.py`) on
top when it exists. Save writes only the profile file (seeded from the
defaults on the first save): it rewrites the numbers in place so the comments
survive, and appends any tunable the file lacks, so a file saved before a
setting existed gains it. Wall follow's `mode` is never written; it is chosen
by editing the file. The file it replaces goes to the profile's `history/`.
The shipped file stays unchanged, so a driver update keeps a car's tuning.
Slider changes become marks on the charts; a drag of one slider within 1 s is
one mark.

CSV logs go to `dashboards/<name>/` under the data root
(`racecar_neo_ros2_driver/data_root.py`) as `LogN.csv`: the header, the marks
inside the saved window as `# mark` lines, then the rows behind the live
chart. `GET /logs`, `GET /logs/<name>` and `POST /logs/save` list, send and
save them.

## Lidar convention

Lab angles are 0 at the nose and positive to the car's right. A ray at raw
scan angle r points r + yaw counterclockwise from the nose, where yaw is the
lidar's mount from the car's frames (`laser_yaw_deg` in `config/frames.yaml`,
with the car profile's calibrated value on top), so:

```
lab_deg = -(raw_deg + laser_yaw_deg)      wrapped to -180..180
```

The RPLIDAR faces aft over 360 degrees at 1080 points: the shipped yaw is
180, the calibrated one close to it (179.619 on the reference car). An object
in front reports at raw 180, one off the car's right at raw +90. Helpers in
`racecar_neo_ros2_driver/frames.py`: `laser_mount()`, `student_deg()`,
`raw_rad()` and `student_scan()`; the map viewer and 8080 Overview read the
same mount. Indices and angles come from each message's `angle_min` and
`angle_increment`, so nothing assumes a sweep width. webteleop and wallfollow
read the lidar; linefollow does not. The student library's `LidarReal` turns
its samples by the same yaw, read from the base_link to laser transform on
`/tf_static`.

## New app checklist

A new browser app is assembled from the shared pieces, not copied from a
sibling. Port numbers, unit names and the one-at-a-time rule come from the
registry; a hard-coded port or unit list is a bug.

1. Registry: an entry in `config/apps.yaml` (id, name, icon from
   `scripts/web/icons.svg`, port, group, unit `racecar-<id>`, `exclusive`
   when it publishes `/drive`, `startable`, `needs`). A lab dashboard adds
   `dir` and `script`, and `setup_dashboards.sh` renders its unit from
   `racecar-app.service.in`; another app ships its own unit in `scripts/`.
2. Start from 8080: a startable app's unit goes in
   `scripts/polkit/49-racecar-dashboards.rules` (`test/test_apps.py` fails
   until it does).
3. Server: `PORT = apps.port('<id>')`; a handler on `webui.AppHandler`
   (`serve_asset`, `page`, `send`, `send_json`, `read_json`); `main` ends in
   `webui.serve(PORT, Handler, spin=webui.ros_spinner(node),
   cleanup=webui.ros_shutdown)`. Subscriptions that cost CPU go behind a
   `webui.IdleGate`.
4. Sensors: camera frames through `imaging` (`bgr`, `depth_m`, `fit_width`,
   `jpeg`, `depth_preview`); lidar angles through `frames` (`laser_mount`,
   `student_deg`, `raw_rad`, `student_scan`).
5. A lab dashboard: `labkit.Tuning` and `labkit.RunLog` with the app's lock,
   `labkit.drive_msg` for every `/drive` message.
6. Page: link `rcn.css`, `ui.js`, `api.js` and what it uses of `charts.js`,
   `tune.js`, `logs.js`, `lidar.js`, `lost.js`, `battery.js` and
   `override.js` (a driving app) as `/ui/<file>?v={{asset}}`; set
   `data-app` on `<html>`; lay out with `rcn-main` and `rcn-section`; poll
   with `RcnApi.poll`; draw canvas colors from `RcnUi.token`. Keep in the
   page only its grid and what only it draws.
7. Tests: a `test/test_dashboard_<id>.py` for the app's own logic; the
   registry, asset and page-kit tests in `test_apps.py` and `test_webui.py`
   cover the rest.

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
                              crop to the crop area         mask preview JPEG
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

- With no contour in the crop area the last command is held.
- The mask preview shows the whole frame, not only the crop area: a threshold can
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
