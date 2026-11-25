# Upgrade

This page moves a car that is already set up onto a new driver release.
The car profile (settings and calibration) stays as it is.

Each release's changelog lists which of the steps below it needs. Steps
the changelog does not name can be left out.

## Before the upgrade {#upgrade-before}

1. Connect the car to a WiFi network with internet.
2. Put the car on a stand, or park it.
3. Save the car profile: `racecar profile export ~/profile-backup.tar.gz`.

## Upgrade steps {#upgrade-steps}

Get the release. Replace `vX.Y.Z` with the release tag:

```sh
cd ~/ros2_ws/src/racecar_neo_ros2_driver
git fetch --tags
git checkout vX.Y.Z
```

Then:

1. Build the driver: `racecar build`.
2. When the changelog says the permissions changed, run
   `bash scripts/setup_user_env.sh`.
3. When the changelog says the services changed, run
   `sudo racecar setup dashboards`. This step asks for the password.
4. Reboot the car: `sudo reboot`. Wait until the car is on the network again.
5. When the changelog names a new student library, run
   `racecar library install`.
6. Run `racecar setup car status`.

Pass: every setup step shows `[DONE]`, and `racecar status` has no
`[FAIL]` row.

Fail: a step shows `[OPEN]` when the release added a step or changed what
it checks. Run `racecar setup car` to finish it.

## Firmware upgrade {#upgrade-firmware}

When the release needs new Teensy firmware, the `firmware` step shows
`[OPEN]`. Flash it as the [Teensy firmware](installation.md#firmware) step
shows.

## Rollback {#upgrade-back}

1. Check out the previous tag: `git checkout vX.Y.Z`.
2. Run `racecar build`.
3. Reboot the car: `sudo reboot`.
4. If a setting is lost, restore the profile:
   `racecar profile import ~/profile-backup.tar.gz`.
