#!/bin/bash
# setup_realsense.sh: install the RealSense D435i ROS2 driver and Pi 5 IMU fix.
#
# This script:
#   1. Installs the realsense2_camera ROS2 packages
#   2. Creates the IMU permission fix script (required on Pi 5)
#   3. Creates a udev rule to auto-fix permissions on device plug
#   4. Creates a systemd service as a boot-time backup
# then runs the fix once and checks that the camera enumerates.
#
# No reboot required. The udev rule takes effect immediately.

set -eo pipefail

echo "=== RealSense D435i Setup ==="

echo "[1/4] Installing RealSense ROS2 packages..."
sudo apt-get install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs ros-jazzy-realsense2-description

echo "[2/4] Installing IMU permission fix script..."
sudo tee /usr/local/bin/fix-realsense-imu.sh > /dev/null << 'FIXSCRIPT'
#!/bin/bash
for dev in /sys/bus/iio/devices/iio:device*; do
    [ -d "$dev" ] || continue
    chmod 666 "$dev"/scan_elements/in_*_en 2>/dev/null
    chmod 666 "$dev"/buffer/enable 2>/dev/null
    chmod 666 "$dev"/buffer/length 2>/dev/null
    chmod 666 "$dev"/trigger/current_trigger 2>/dev/null
    chmod 666 "$dev"/in_*_sampling_frequency 2>/dev/null
    chmod 666 "$dev"/in_*_hysteresis 2>/dev/null
    [ -e /dev/"$(basename "$dev")" ] && chmod 666 /dev/"$(basename "$dev")"
done
# The motion module's accelerometer needs enable_sensor writable too; it lives
# at the HID-SENSOR-*.auto node, not under iio:device*. Without this the accel
# fails with "Failed to enable_sensor ... Permission denied" and reads zeros.
for f in $(find /sys/devices -path '*HID-SENSOR*' -name enable_sensor 2>/dev/null); do
    chmod 666 "$f" 2>/dev/null
done
FIXSCRIPT
sudo chmod +x /usr/local/bin/fix-realsense-imu.sh
echo "  Created /usr/local/bin/fix-realsense-imu.sh"

echo "[3/4] Installing udev rule for IMU permissions..."
UDEV_RULE="/etc/udev/rules.d/99-realsense-imu.rules"
echo 'SUBSYSTEM=="iio", KERNEL=="iio:device*", ACTION=="add", RUN+="/usr/local/bin/fix-realsense-imu.sh"' | sudo tee "$UDEV_RULE" > /dev/null
sudo udevadm control --reload-rules
echo "  Created $UDEV_RULE"

echo "[4/4] Installing systemd boot-time IMU fix service..."
sudo tee /etc/systemd/system/realsense-imu-permissions.service > /dev/null << 'SVCFILE'
[Unit]
Description=Fix RealSense IMU IIO permissions on Pi 5
After=systemd-udevd.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/fix-realsense-imu.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SVCFILE
sudo systemctl daemon-reload
sudo systemctl enable realsense-imu-permissions.service
echo "  Enabled realsense-imu-permissions.service"

sudo /usr/local/bin/fix-realsense-imu.sh

echo ""
echo "Verifying RealSense detection..."
if command -v rs-enumerate-devices &>/dev/null; then
    rs-enumerate-devices --compact || echo "  WARNING: Camera not detected. Is it plugged in via USB 3.0?"
else
    echo "  rs-enumerate-devices not found; install may need 'source /opt/ros/jazzy/setup.bash'"
fi

echo ""
echo "=== RealSense D435i setup complete ==="
echo "No reboot required."
echo "Verify: racecar launch realsense"
