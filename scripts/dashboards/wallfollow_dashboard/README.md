# Wall Follow Dashboard

Wall following as a service for the RACECAR Neo, with a live tuning dashboard on port 8083.

One process subscribes to /scan, computes PD steering, publishes AckermannDriveStamped on /drive, and serves the dashboard. Runs as a systemd service that starts on boot, alongside the stock racecar services.

## Contents

- [Service control](#service-control)
- [Dashboard](#dashboard)
- [Modes](#modes)
- [Speed](#speed)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar setup dashboards`), stopped and disabled.
Dashboard: `http://<car-ip>:8083`.

| Command | Does |
|---|---|
| `racecar service start wallfollow` | start now; stops any other lab dashboard first |
| `racecar service stop wallfollow` | stop |
| `racecar service enable wallfollow` | start at every boot |
| `racecar service logs wallfollow` | follow the log |

Saved tuning goes to `wallfollow.local.yaml` beside `wallfollow.yaml`; the shipped file
keeps the defaults. Internals: `docs/dashboards.md` in the driver.

## Dashboard

- Lidar view: scan points, the averaging windows or search arc in gray, and the measured rays in orange. Scroll or use + and - to zoom.
- Error chart: the last 30 s of error against the setpoint (cm in static mode, -1 to 1 in dynamic), with dashed lines at parameter changes.
- Tune panel: speed is a 0 to 1 slider that applies while dragging; the other values apply on Apply.
- Save and Load write and read wallfollow.yaml on the car. Reset (top bar) re-reads it. Stop (top bar) sets speed to 0.
- Save log snapshots the error chart to LogN.csv. Load log draws a saved one with the error and the measured speed.

## Modes

`mode` in wallfollow.yaml selects the controller: 0 = static, 1 = dynamic. It is switched only by editing the file, then pressing Reset or Load. There is no UI toggle.

- `static`: mirrored side windows at look_angle. Error = right minus left in meters, setpoint 0 keeps the car centered.
- `dynamic`: steers toward the most open heading in a front arc, obstacles inflated by the car's half width. width opens and closes the arc, side_weight biases one side.

All parameters are documented in wallfollow.yaml.

## Speed

The speed slider is a throttle cap, not a constant command. The target speed drops when the wall ahead is inside lookahead and when steering hard, and speed_kp / speed_kd ramp the throttle toward that target. The live throttle shows in the top bar.

## Parameters

wallfollow.yaml. Save rewrites the numbers in place, so the comments in the file survive.

| Key | Notes |
| --- | --- |
| mode | 0 static, 1 dynamic. File edit only; there is no UI toggle, because the two controllers take different parameters and mean different things by `error` |
| speed | 0..1 of `max_mps`; ships at 0.0, so the car cannot drive until the slider is raised |
| kp, kd | steering strength and its damper. Raise `kd` if the car wobbles |
| speed_kp, speed_kd | slow down in proportion to \|error\|. 0 leaves speed constant |
| window | averaging window per ray, degrees; smooths a noisy scan |
| max_mps | the car's top speed; the slider is a fraction of this |
| lookahead | meters. Distances are capped here in both modes, so an open side reads as `lookahead` rather than as 12 m of empty room |
| look_angle | static mode: the side windows sit this many degrees off the nose. 90 is straight sideways |
| width, side_weight | dynamic mode: how far either side of the nose to search, and a bias toward one side |

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter. The webteleop and linefollow dashboards both publish it; stop them before starting this one:

```
racecar service stop webteleop
racecar service stop linefollow
```

Check with `ros2 topic info -v /drive`: there should be exactly one publisher.

A leftover Jupyter kernel that ever created a racecar object also keeps publishing /drive. Restart the jupyter service to clear them.

## Tests

`test/test_dashboard_wallfollow.py` in the driver; no ROS graph, camera, or browser
needed.

## Safety

The RACECAR Neo mux gates /drive on the RB bumper and zeroes output when /joy or the active source goes stale; a FlySky transmitter can hold the gate instead when rc_authority_enable is set. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. The speed command is hard capped at 1.0 in code.

## Car specifics

This package is calibrated for the RACECAR Neo: RPLIDAR angle mapping via LIDAR_MOUNT_YAW_DEG (the lidar faces aft over 360 degrees at 1080 points, so the nose sits at raw 180), steering sign, speed feedback from /odom, ROS Jazzy paths. Both signs were verified physically on the car.
