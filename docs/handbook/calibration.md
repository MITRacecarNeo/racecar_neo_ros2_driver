# Calibration

Every car in the fleet uses the same ESC program settings and the same ESC
throttle calibration. The Teensy firmware's speed control is tuned for them.

## Contents

- [ESC program settings](#esc-program-settings)
- [ESC throttle calibration](#esc-throttle-calibration)
- [Dynamics after an ESC calibration](#dynamics-after-an-esc-calibration)
- [Per-car speed constants](#per-car-speed-constants)
- [Car record](#car-record)

Set up the ESC on a new car, after replacing the ESC, and after an ESC
factory reset. Do the three ESC steps in order.

## ESC program settings

The ESC is a Hobbywing QuicRun 10BL60 G2 Sensored. It stores twelve settings,
called items. Each item holds an option number. The Hobbywing LED Program
Card reads and sets them.

| Item | Setting | Option |
| ---- | ------- | ------ |
| 1 | Running mode | 3 (forward and reverse) |
| 2 | Cutoff voltage | 4 (3.0 V per cell) |
| 3 | Punch | 3 (level 3) |
| 4 | Drag brake force | 5 (40%) |
| 5 | Maximum brake force | 3 (75%) |
| 6 | Maximum reverse force | 2 (50%) |
| 7 | Neutral range | 1 (6%) |
| 8 | Boost timing | 2 (4 degrees) |
| 9 | Turbo timing | 1 (0 degrees) |
| 10 | Turbo delay | 9 (1.0 s) |
| 11 | BEC voltage | 1 (6.0 V) |
| 12 | Motor rotation | 1 (CCW) |

The program card has four buttons:

- ITEM: moves to the next item.
- VALUE: changes the option of the item shown.
- OK: saves the option shown.
- RESET: restores every item to its factory default.

Connect the card:

1. Turn the ESC switch off.
2. Plug the program card into the ESC's fan port with the three-wire cable.
   White is data, red is VCC, and black is ground.
3. Turn the ESC switch on. The ESC enters program mode, and the card's LEDs
   show the item and its option.

Set the items:

1. Press ITEM to step through all twelve items. Write down the option of
   each.
2. Press RESET to restore the factory defaults.
3. For each item in the table, press ITEM until the card shows that item.
4. Press VALUE until the card shows the option in the table.
5. Press OK to save it.
6. Step through all twelve items again with ITEM. Each one matches the
   table.
7. Turn the ESC switch off.
8. Unplug the program card.

A factory reset sets every item back to its default. Maximum reverse force,
for example, goes back to 25%.

## ESC throttle calibration

The ESC learns three throttle positions: neutral at 1500 us, full forward at
2000 us, and full reverse at 1000 us. The Teensy holds each position while
you press SET on the ESC. A special calibration firmware does this. The car
does not drive while it is on the Teensy.

Setup:

1. Put the car on a stand so all four wheels are off the ground.
2. Plug the Teensy USB cable into the Pi.
3. Stop any lab app (web teleop, line follow, wall follow) from the 8080
   page.
4. Open a terminal on the car in the `racecar-pit-firmware` folder.

Steps:

1. Flash the calibration firmware: `pio run -e teensy41_esc_cal -t upload`.
2. Run `python3 scripts/esc_cal.py`.
3. Follow the steps the script prints. It first asks you to put the ESC in
   calibration mode:
   1. Turn the transmitter off.
   2. Turn the ESC switch off.
   3. Press and hold the SET button on the ESC.
   4. Turn the ESC switch on while you hold SET.
   5. Release SET as soon as the ESC LED starts flashing red.
4. The script then holds each position in turn. The LED strip shows it as a
   purple bar: half the strip at neutral, the whole strip at full forward, one
   LED at each end at full reverse.
   1. At neutral, press SET once. The motor beeps once.
   2. At full forward, press SET once. The motor beeps twice.
   3. At full reverse, press SET once. The motor beeps three times.
5. Wait for the script to print "Calibration sent." The ESC plays no
   separate tone. It goes back to normal operation.
6. Check out the firmware release tag: `git checkout v0.9.2`.
7. Flash the normal firmware: `pio run -e teensy41 -t upload`.

At idle on the normal firmware the ESC LED is solid red, because boost
timing is set. It flashes green while the car drives in reverse.

Check: run `racecar status`. The `teensy firmware` row shows the firmware
version, with no warning for a test build.

If the script prints "No reply", the calibration firmware is not on the
Teensy. Flash it again (step 1).

## Dynamics after an ESC calibration

Run `racecar calibrate dynamics` after every ESC throttle calibration. The
car's speeds change with the calibration.

Setup:

1. Use a clear straight path of about 4 meters with walls or furniture
   nearby.
2. Use a clear area of about 2.3 by 1.6 meters for driving circles.
3. Connect the controller.

Hold RB while the car drives each run. The car stops when you release RB.
The results go to the car profile.

## Per-car speed constants

The Teensy keeps one set of speed-loop constants for the car. A car with no
set of its own uses the fleet values. Read which one is in use from the
firmware info:

```
ros2 topic echo --once --full-length /pit/firmware
```

| Key | Meaning |
| --- | ------- |
| `cal.source` | `builtin` for the fleet values, `eeprom` for the car's own set |
| `cal.eeprom` | State of the stored set: `empty`, or why it did not load |
| `cal.crc` | Checksum of the stored set |
| `cal.written` | Date the stored set was saved |
| `pid.*` | The constants in use |

A car that matches the fleet in `racecar calibrate dynamics` keeps the fleet
values.

### Refitting a car

TODO: the measurement runs that decide new constants for a car that does
not match the fleet. Until then, ask the firmware maintainer.

The firmware's `scripts/car_constants.py` changes and stores the
constants over the Teensy's USB port. The car must be parked; changes are
refused while it moves.

| Command | Action |
| ------- | ------ |
| `python3 scripts/car_constants.py get` | Print the values in use and where they came from |
| `python3 scripts/car_constants.py set pid.ff_rev_us=110` | Change values now; lost at power-off |
| `python3 scripts/car_constants.py save` | Store the values in use, dated today |
| `python3 scripts/car_constants.py revert` | Go back to the stored values |
| `python3 scripts/car_constants.py erase` | Remove the stored values; the fleet values load at power-on |

## Car record

Record these for each car. They show how the car was set up, so a car that
drives differently from the fleet can be compared item by item.

| Field | Where to read it | Reference car |
| ----- | ---------------- | ------------- |
| Firmware version and commit | `racecar status`, `teensy firmware` row | 0.9.2, 0e94b5d |
| Firmware release tag flashed | The tag checked out before flashing | v0.9.2 |
| Speed constants | `cal.source`, `cal.crc`, `cal.written` on `/pit/firmware` | builtin (fleet values) |
| ESC model | ESC label | Hobbywing QuicRun 10BL60 G2 Sensored |
| ESC program settings | Program card | 3,4,3,5,3,2,1,2,1,9,1,1 |
| ESC calibration date | Date of the throttle calibration | 2026-11-02 |
| ESC calibration endpoints | Fixed by the calibration firmware | 1000 / 1500 / 2000 us |
| Motor | Motor label | LCE019 DPM 2845 sensored |
| Pinion | Gear label | L6200, 17T |
| Spur gear | Kit part | LC Racing L6005 slipper spur, 60T, 0.5 module (EMB-SC stock) |
| Servo | Servo label | L6094 Batan D135F |
| Boost converter | Board label | Pololu U3V70F9 (9 V, 7 A) |
| Pi regulator | Board label | Pololu D24V90F5 (5 V, 9 A) |
| Battery | Battery label | Gens ace 5000 mAh 2S 7.4 V 60C short-size |
| Idle battery current | 8080 Current chip, car on a stand, teleop running | 1.93 A at 7.83 V |
| Dynamics date | Date of `racecar calibrate dynamics` | 2026-11-07 |
