# Installation

This page takes a new car from a blank SD card to a finished car profile.
It needs a second computer with an SD card reader and the car's battery.

## Contents

- [Operating system](#os-image)
- [Driver install](#driver-install)
- [Hardware fitting](#hardware-fitting)
- [Guided car setup](#car-setup)

## Operating system {#os-image}

The car runs Ubuntu Desktop 24.04 LTS (64-bit) on a Raspberry Pi 5.

1. Install Raspberry Pi Imager on the second computer.
2. Put the SD card into the second computer.
3. In Imager, choose Raspberry Pi 5 as the device.
4. Choose *Other general-purpose OS*, then *Ubuntu*, then *Ubuntu Desktop
   24.04 LTS (64-bit)*.
5. Choose the SD card as the storage.
6. Open the OS customization settings.
7. Set the hostname to `racecar-neo`.
8. Set the username to `racecar`.
9. Set a password.
10. Enter your lab's WiFi network name and password.
11. Turn on SSH with password login.
12. Write the card.
13. Put the SD card into the Pi.
14. Connect the battery and turn on the power switch.

Ubuntu Desktop can ignore the Imager settings. If the Pi shows a setup
screen on first boot, connect a monitor, keyboard and mouse, and enter the
same user name, password and WiFi network there.

Find the car's address on your network from your router, or run
`ip neigh` on another computer on the same network. Log in from the second
computer:

```sh
ssh racecar@<address>
```

## Driver install {#driver-install}

Run every command on the car, as the `racecar` user.

Set needrestart to restart services without asking.

```sh
sudo apt update && sudo apt -y install needrestart git
sudo sed -i "s/^#\$nrconf{restart} =.*/\$nrconf{restart} = 'a';/" /etc/needrestart/needrestart.conf
sudo sed -i "s/^#\$nrconf{kernelhints} =.*/\$nrconf{kernelhints} = -1;/" /etc/needrestart/needrestart.conf
```

Upgrade the system. This takes 8 to 15 minutes.

```sh
sudo apt -y full-upgrade
```

Clone the driver and run the setup:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MITRacecarNeo/racecar_neo_ros2_driver.git
bash racecar_neo_ros2_driver/scripts/setup_all.sh
```

The setup asks for the password once, then runs for about 45 minutes. Do
not run it with `sudo`.

Pass: the setup ends with a request to reboot.

Fail: the setup stops at the failed part and prints the command that
resumes there, for example `bash scripts/setup_all.sh --from 5`. Fix the
problem it names, then run that command.

## Hardware fitting {#hardware-fitting}

1. Shut down the Pi: `sudo poweroff`.
2. Turn off the power switch.
3. Unplug the battery.
4. Connect the NEO-PIT PCB. The motor, the steering servo, the IMU and the
   dot matrix connect to it.
5. Connect the RealSense camera to a blue USB 3 port.
6. Connect the lidar.
7. Fit the Coral Edge TPU M.2 card.
8. Fit the NVMe drive.
9. Plug in the ALFA WiFi dongle. The car's access point runs on it.
10. Plug in the gamepad's USB receiver.
11. Fit the RTC backup cell on the Pi, if the car has one.
12. Connect the battery.
13. Turn on the power switch.

Log in again over SSH.

## Guided car setup {#car-setup}

`racecar setup car` takes the car the rest of the way. It shows a checklist,
then runs the next open step.

The Setup page of the Car Profile app (`http://<car>.local/profile/setup`)
runs the same steps from a browser. Tap a tile to run a step or fill in its
answers. When a step asks for the password, type the racecar password in
the box that opens.

```sh
racecar setup car             # the checklist, then the next open step
racecar setup car status      # the checklist only
racecar setup car STEP        # run one step again
racecar setup car done STEP   # mark a step done by hand
```

At each step, press Enter to run or confirm the step, d to mark it done, s
to skip it, or q to stop. Every step checks the car first, so a step already
done shows `[DONE]`. Stop at any point and run `racecar setup car` again
later to continue.

Run it over the WiFi network (`wlan0`) or at the car's own screen. The
access point step changes the car's own network, and the eth0 change can
drop a session that arrives over a cable.

Until every step is done, the 8080 home page keeps the utility apps closed.

### Car identity {#identity}

The step asks three things:

- The car name.
- A nickname, optional. The pages show it beside the hostname.
- The car number, from 0 to 54. Each car in the fleet has its own number.

It sets the hostname to `racecar-neo-<number>`. The new hostname applies
after the reboot step.

Pass: the checklist shows the name, the number and the hostname.

### Hardware connection check {#hardware}

The step checks that every device answers: the NEO-PIT PCB, the lidar, the
camera, the gamepad, the Coral card and the WiFi dongle.

Pass: no device is missing.

Fail: the detail names each missing device. Shut down the car, turn off the
power switch, and check that part's cable. If the gamepad is missing,
unplug its receiver and plug it back in.

### NVMe data drive {#nvme}

The step runs `racecar disk setup` when an NVMe drive is fitted. It makes
the drive the car's data store at `/mnt/nvme`. Recordings, maps and reports
go there.

Pass: the detail shows the data folder on the drive.

Fail: `no drive found` means the drive is not seen. Shut down the car, turn
off the power switch, and reseat the drive and its cable. A drive that
holds other data is refused; `racecar disk format` erases it after you type
its serial number.

### Access point {#access_point}

The step runs `racecar setup networking --id=<number>`. The car then makes
its own WiFi network named `racecar-neo-<number>` on the ALFA dongle. The
eth0 port gets the address `192.168.52.(200 + number)`.

Pass: the access point runs with this car's number.

Fail: `not running` means the dongle is missing or the network did not
start. Check the dongle, then run the step again.

### Reboot {#reboot}

The step asks for one reboot when the new hostname or the drive waits for
it.

1. Run `sudo reboot`.
2. Log in again. Use the new name: `ssh racecar@racecar-neo-<number>.local`.
3. Run `racecar setup car`.

Pass: the hostname matches the identity step.

### Teensy firmware {#firmware}

The step checks the firmware on the NEO-PIT PCB against the version this
driver needs (`config/firmware.yaml`).

1. Connect the car to the internet.
2. Connect the Teensy USB cable to the Pi.
3. Run the step. It downloads the firmware release, checks the file, and
   flashes it. The first try often fails. The step tries up to three times.

Pass: the step prints `Firmware 0.9.2 is running.`, and the checklist shows
the installed version.

Fail: `no version read` means teleop is not running. Run
`racecar service restart teleop` and wait 10 seconds. A failed download
needs the internet. A failed flash needs the Teensy USB cable.

### ESC setup {#esc}

Set up the ESC first, as the calibration page shows:
[ESC program settings](calibration.md#esc-program-settings) and
[ESC throttle calibration](calibration.md#esc-throttle-calibration). Then
the step asks for the ESC model, the program settings and the calibration
date.

Pass: the three values are recorded.

### Student library and camera firmware {#extras}

The step installs the student library and labs if they are missing. It
flashes the camera firmware when the version differs from
`config/firmware.yaml`.

Pass: the library is installed and the camera firmware matches.

Fail: the library install needs the internet. Connect the car to WiFi and
run the step again.

### Car geometry {#measurements}

The step shows the stock positions of the lidar, the camera and the IMU.
Keep them for a car built to the fleet design. If your car differs, enter
your own measurements in millimeters. Measure from the floor and from the
center of the rear axle.

Pass: you kept the stock values or entered your own.

### Sensor calibration {#calibration}

The step runs `racecar calibrate`, then `racecar calibrate check`. Follow
the [sensor calibration](calibration.md#sensor-calibration) section of the
calibration page.

Pass: every required calibration is stored.

### Hardware health check {#health}

The step runs `racecar health`. See [Health checks](health.md#health-run).

Pass: the last report passed and has power and link checks.

### Car record {#record}

The step asks for the labels of the car's parts: the motor, the gears, the
servo, the regulators and the battery. The fleet parts are the defaults.
Press Enter to keep a default.

The idle battery current is optional. Read it from the Current chip on the
8080 home page with the car on a stand.

Pass: every field but the idle current is filled.

### Profile backup {#finish}

The step saves the car profile to the data drive and prints
`racecar status`. Keep this backup. `racecar profile import FILE` restores
it after an SD card swap.

Pass: the backup file exists. The checklist shows `Setup complete.`
