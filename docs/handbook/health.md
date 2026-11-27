# Health checks

These checks show that every part of the car works. Run them on a new car
after the installation, and whenever the car behaves oddly.

## Contents

- [Status check](#status-check)
- [Parked checks](#parked-checks)
- [Health run](#health-run)
- [Loose contact checks](#loose-contact-checks)
- [Reports](#reports)

## Status check {#status-check}

`racecar status` takes one reading of every part and prints a row each.

```sh
racecar status
```

Each row starts with a mark:

| Mark | Meaning |
| ---- | ------- |
| `[ OK ]` | The part works. |
| `[WARN]` | The part works, with a problem to look at. The detail names it. |
| `[FAIL]` | The part does not work. The detail names the fix. |

Pass: no `[FAIL]` row.

Fail: follow the detail on the row. The
[troubleshooting guide](../troubleshooting.md) covers most rows by name.

## Parked checks {#parked-checks}

`racecar check` tests that the car is fit to map and drive, with the car
still. It takes about 5 seconds.

1. Put the car on a stand or on a flat floor.
2. Keep your hands off the car.
3. Run `racecar check`.

```sh
racecar check              # calibration, IMU, odometry, lidar, rates, teleop, battery
racecar check --all        # the same, then the power, link and drive checks
racecar check imu lidar    # only the named checks
```

The Checks page of the Car Profile app runs the same checks, each alone or
with Run all, and the shake and poke checks with their own buttons.

Pass: `RESULT OK`, or `WARN` with something to look at.

Fail: each row that is not OK names its repair. Most repairs are a
calibration step on the [Calibration](calibration.md#sensor-calibration)
page.

## Health run {#health-run}

`racecar health` tests the power supply and the link to the NEO-PIT PCB
for 30 seconds each, then the data drive. Teleop must be running.

1. Put the car on a stand so all four wheels are off the ground.
2. Connect a charged battery.
3. Run `racecar health`.
4. Leave the car still until it finishes.

```sh
racecar health          # power, link, then nvme
racecar health power    # the Pi's 5 V input, throttling, the battery
racecar health link     # the link to the NEO-PIT PCB
racecar health nvme     # the data drive
```

| Check | Fails when | Fix |
| ----- | ---------- | --- |
| `power` | the 5 V input drops under 4.75 V, the Pi slows down to protect itself (throttling), or the battery is under 3.55 V per cell | Charge the battery. Check the 5 V regulator wiring. |
| `link` | a gap over 50 ms, a damaged message (CRC failure), a reconnect, or a Teensy reset | Check the cable between the Pi and the PCB. |
| `nvme` | no drive, not mounted, a write that does not read back, or drive errors | See [NVMe data drive](installation.md#nvme). |

Pass: the run ends with `OK`. A `WARN` row is a pass with something to look
at.

## Loose contact checks {#loose-contact-checks}

`shake` and `poke` look for a loose contact. Each asks for one movement at a
time. Press Enter to start each movement. Keep the car still for the
first 5 seconds.

Shake:

1. Hold the car in your hands with the wheels off the ground.
2. Run `racecar health shake`.
3. Shake the car along each axis when the step asks: forward and back, side
   to side, up and down.
4. Tap the top deck when the step asks.

Poke:

1. Put the car on a stand.
2. Take off the top deck.
3. Run `racecar health poke`.
4. Press and wiggle each connector the step names, one at a time.

Pass: no step reports a fault.

Fail: the step names the fault and the connector or part. Reseat that
connector and run the check again.

## Reports {#reports}

Each run saves a report to `health/<time>/` on the data drive:
`report.md` to read, `report.json` for tools, and `samples.csv` with every
5 V reading.
