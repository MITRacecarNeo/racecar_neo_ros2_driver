# Web Teleop Dashboard

Manual driving for the RACECAR Neo from a browser, on port 8081. Four buttons, bound to WASD and the arrow keys, drive the car forward, back, left, and right, two at once for a turn on the move, with both camera streams, the lidar, and the wheel encoder in view. Two sliders set how hard the buttons push.

The service name is `racecar-webteleop`, not `racecar-teleop`: that name belongs to the driver's core stack, which this dashboard rides on.

## Contents

- [Service control](#service-control)
- [Driving](#driving)
- [Dashboard](#dashboard)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar setup dashboards`), stopped and disabled.
Dashboard: `http://<car-ip>:8081`.

| Command | Does |
|---|---|
| `racecar service start webteleop` | start now; stops any other lab dashboard first |
| `racecar service stop webteleop` | stop |
| `racecar service enable webteleop` | start at every boot |
| `racecar service logs webteleop` | follow the log |

Saved tuning goes to `teleop.local.yaml` beside `teleop.yaml`; the shipped file
keeps the defaults. Internals: `docs/dashboards.md` in the driver.

## Driving

The browser keeps a set of what is held: each pad button under a pointer or finger, each of the eight keys that is down. That set collapses to two axes, forward/back and left/right, each -1, 0, or +1, and the pair is posted to the service as one command on every change and every 100 ms while anything is held. Forward and left pressed together are therefore one command, applied at the same instant; forward and back together cancel to 0.

```
buttons / keys --> held set --> {fwd, turn} --> POST /cmd (on change, 10 Hz while held)
                                                        |
        throttle = fwd  * speed                         v
        steering = turn * angle          /drive at 15 Hz from a timer
                                                        |
                              no command for cmd_timeout -> drive zero
```

`/drive` comes from a timer, not from the browser's posts, so the mux never sees a gap. A command older than `cmd_timeout` counts as released: a closed tab, a dropped link, or a page that lost focus with a key down all coast to zero instead of driving the last command forever. The browser also releases everything on its own when the window loses focus or the tab is hidden, so the timeout is the backstop rather than the first line.

Keys: W, A, S, D or the arrow keys. Key auto-repeat is ignored; a key drives from its keydown to its keyup. Keys are read anywhere on the page except inside a slider or the log picker.

## Dashboard

Top row, the camera stack, the lidar, then the encoder and the logs down the rest of the width:

- Camera card: the live color frame with the depth frame under it, same width and same framing, so a feature lines up vertically between the two. The Detections switch on the card's heading overlays the object detector's boxes on the color frame.
- Depth view: bright is near, dark is `depth_max_m` away, black is a pixel the camera got no return from, which is what glass, a mirror, and anything past the far end all look like. The ramp is inferno, one ordered scale from dark to bright, so a step in color is a step in distance. A car with no `depth_topic` set drops the pane; a car that has one and receives nothing keeps it, so a dead stream stays visible as a fault.
- Detections: each box carries its class and confidence, drawn in brand orange. The switch is disabled on a car with no `detections_topic`, and the label reads `no signal` when a configured detector has stopped publishing; boxes older than 1.5 s are cleared rather than left frozen on a live frame. What the boxes say depends on the model the driver has loaded: the shipped default is EfficientDet-Lite0 on COCO, so 90 everyday classes.
- Lidar view: scan points around the car, the nose up, range rings at 1, 2, and 3 m, scroll to zoom. 0 degrees is the nose. The Neo's RPLIDAR faces aft, so the raw scan is yawed 180 degrees onto that convention by `LIDAR_MOUNT_YAW_DEG`; see `docs/dashboards.md`, "Lidar convention", in the driver.
- Encoder card: a state box, STOPPED, FORWARD, FORWARD LEFT, and so on, ember while moving, crimson on TIMED OUT, and under it the live chart: measured speed from `/odom` in white on its own scale, the commanded throttle in orange on a fixed -1 to 1, dashed rules at every slider change. The gap between the two traces is the car's lag and the difference between a commanded fraction and real meters per second.

Second row: the drive pad on the left, the two sliders on the right. The page shows only names and numbers; every explanation is a tooltip. Hover a view, the chart, the state box, a pad button, or a slider to read what it does.

Save and Load write and read teleop.yaml on the car. Reset (top bar) re-reads the yaml. STOP (top bar) releases every button and sets speed to 0.

Save log snapshots what is on the live chart to LogN.csv: the two axes, the throttle and steer sent, and the measured speed. Load log defaults to the latest save and draws measured speed, throttle, and steer together.

## Parameters

teleop.yaml:

| Key | Notes |
| --- | --- |
| speed | 0..1 throttle while forward or back is held; ships at 0.0 |
| angle | 0..1 steering while left or right is held; 1 is full lock |
| cmd_timeout | seconds without a command before the service drives zero; not on the dashboard |
| camera_topic, preview_width, preview_quality | `/camera/color` on the latest driver, `/camera` on older cars; live view size and jpeg quality, shared by both views |
| depth_topic | `/camera/depth` on a D435i. Empty on a car whose camera is color only, which drops the depth pane rather than subscribing to a topic nobody publishes |
| depth_max_m | far end of the depth ramp, metres; anything past it clamps to the dark end instead of dropping out |
| detections_topic | `vision_msgs/Detection2DArray` for the Detections switch, `/edgetpu/inference` on this driver. Empty on a car with no detector, which disables the switch |

Save rewrites the numbers in place, so the comments in the yaml survive.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, pursuit, eps, smartfollow, and linefollow dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: webteleop).

The camera and the lidar are read directly, so the driver's inference node does not need to be running. camlabel reads the same camera topic and does not publish /drive, so it can run alongside.

## Tests

`test/test_dashboard_teleop.py` in the driver; no ROS graph, camera, or browser
needed.

## Safety

The RACECAR Neo mux gates /drive on the RB bumper and zeroes output when /joy or the active source goes stale; a FlySky transmitter can hold the gate instead when rc_authority_enable is set. The shipped yaml has speed at 0.0, so the buttons steer but the car cannot move until the slider is raised. The speed command is hard capped at 1.0 either way in code, and the service drives zero after `cmd_timeout` without a command.

## Car specifics

This package is calibrated for the RACECAR Neo: JPEG frames on /camera/color, D435i depth on /camera/depth, RPLIDAR angle mapping, steering sign, speed feedback from /odom, ROS Jazzy paths. The steering sign was verified physically on the sibling dashboards and is the same here.
