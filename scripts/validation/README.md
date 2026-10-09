# Validation scripts

Test tooling for development on the car. Run from the source tree; nothing
here is installed or exposed through `racecar`.

## Contents

- [SLAM session](#slam-session)
- [Single measurements](#single-measurements)
- [Storage](#storage)

## SLAM session

```sh
cd ~/ros2_ws/src/racecar_neo_ros2_driver/scripts/validation
./slam_session.py map NAME                    # mapping; --camera off to drop the RealSense
./slam_session.py localize MAP                # AMCL; --localizer slam_toolbox, --pose x,y,yaw
./slam_session.py analyze [SESSION_DIR]       # rewrite report.md; default the newest session
./slam_session.py replay SESSION_DIR TAG --set angle_variance_penalty=0.03   # offline tuning
./slam_session.py replay LOC_SESSION_DIR TAG --localizer slam_toolbox       # other localizer, same bag
```

One terminal runs the whole session. Mapping sessions cap the gamepad speed at the firmware's floor (`throttle_mapping.yaml`: press the trigger past halfway to move); the RC transmitter is not capped. The script stops the teleop service
(sudo may ask for a password), starts teleop with EdgeTPU off, the autonomy
launch, `foxglove_bridge` with `foxglove_validation.yaml`, a bag, and the load
recorder, then prints the Lichtblick address (`ws://<car>:8765`).

Keys while driving, read without Enter:

| Key | Action |
|---|---|
| `m` | Record the map pose with the car stopped on a mark; asks for a label |
| `s` | Save the map now (mapping) |
| `t` | Timed save: the longest gap in `/pose` while slam_toolbox serializes (mapping; drive while pressing it) |
| `q` | Stop everything in order, save a mapping run, write the report |

Each session folder in `~/data/slam_eval/sessions/` holds `session.yaml`
(settings, commit, start and end storage), `calibration/` (the car's
`.local.yaml` files), `load.csv`, `marks.csv`, `stalls.csv`, `bag/`, `logs/`,
`report.md`, and `map.png`. A mapping run's map goes to `~/data/maps/NAME/`.
Taped distances between marks go in `~/data/slam_eval/taped.txt`, one pair per
line: `A B 2.35`.

`replay` runs a session's bag through a fresh mapper or localizer with the
`--set` overrides (repeatable, YAML values) applied to slam_toolbox or AMCL,
and writes `replay/TAG/` in the session folder: `overrides.yaml`,
`report.md`, `logs/`, and for a mapping run `map.pgm`, `map.yaml`, and
`map.png`. A mapping run replays through slam_toolbox mapping
(`config/slam_mapping.yaml`). A localization run replays through its own
localizer, or the one `--localizer` names, on the run's map from its start
pose (AMCL with `config/amcl.yaml`, the slam_toolbox localizer with
`config/slam_localization.yaml`), so both localizers can be scored on the
same drive. It runs on ROS
domain 77 and drops the bag's `map -> odom`, so it neither sees nor disturbs
a running car and the recorded solution does not leak in. It takes about as
long as the session. Compare variants by the rotation spread of `map -> odom`
(how often the scan matcher overrides the odometry) and by occupied cells per
free square meter (thinner walls give fewer).

Session and replay reports score the scan-to-map fit (`scan_fit.py`): each
scan is placed in the map at the localizer's pose and every point's distance
to the nearest wall cell is measured. The report gives the median distance,
the share of points within 100 mm, and the 90th percentile of each scan's
median (the bad stretches), with the time of the worst scan. A mapping run
scored against its own map gives the best fit the map allows.

## Single measurements

`slam_eval.py` runs one measurement beside `racecar launch autonomy`: `save`,
`load`, `mark`, `report`, `quality`. See its docstring.

## Storage

A session uses about 15 to 30 MB for a 5 minute run, plus about 7 MB per timed
save. The session script warns below 10 GB free or above 80% used, lists the
largest folders under `~/logs`, the sessions, and the maps, and refuses to
start below 3 GB.
