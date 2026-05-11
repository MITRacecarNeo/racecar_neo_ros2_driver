"""
Pre-flight hardware connectivity tests for RACECAR Neo v2.

Each assertion's failure message includes a one-line fix hint. Tests for
hardware that isn't connected will fail loudly — that's intentional. Run
selectively with `pytest -m hardware` or skip with `pytest -m 'not hardware'`.
"""

import grp
import importlib
import importlib.util
import os
import pwd
import re
import subprocess

import pytest


def _lsusb_match(vid_pid):
    """Return True if `lsusb` lists a USB device matching vid:pid."""
    try:
        out = subprocess.run(
            ['lsusb'], capture_output=True, text=True, timeout=5
        ).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return vid_pid.lower() in out.lower()


def _i2c_probe(bus, address):
    """Return True if i2cdetect reports a device at `address` on `bus`."""
    try:
        out = subprocess.run(
            ['i2cdetect', '-y', str(bus)],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    detected = set()
    for line in out.splitlines()[1:]:
        parts = line.split()
        if not parts or not parts[0].endswith(':'):
            continue
        row_base = int(parts[0].rstrip(':'), 16)
        for col, val in enumerate(parts[1:]):
            if re.fullmatch(r'[0-9a-fA-F]{2}', val):
                detected.add(row_base + col)
    return address in detected


def _user_in_group(name):
    """
    Return True if the current user is a member of `name` per /etc/group.

    Reads /etc/group rather than os.getgroups() so usermod -aG additions are
    visible without requiring the user to log out and back in.
    """
    user = pwd.getpwuid(os.getuid()).pw_name
    try:
        target = grp.getgrnam(name)
    except KeyError:
        return False
    if user in target.gr_mem:
        return True
    return pwd.getpwnam(user).pw_gid == target.gr_gid


# ---------------------------------------------------------------------------
# Pololu Maestro — drive ESC (ch 0) + steering servo (ch 1)
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestMaestro:
    DEVICE = '/dev/ttyACM0'

    def test_device_node_exists(self):
        assert os.path.exists(self.DEVICE), (
            f'{self.DEVICE} not present. Plug in the Pololu Maestro USB cable; '
            f'verify with `lsusb | grep -i pololu`.'
        )

    def test_user_in_dialout(self):
        assert _user_in_group('dialout'), (
            'User not in dialout group. Fix: '
            'sudo usermod -aG dialout $USER and log out + back in.'
        )


# ---------------------------------------------------------------------------
# RPLIDAR — 2D LIDAR
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestRPLIDAR:
    DEVICE = '/dev/ttyUSB0'
    CP210X_USB_ID = '10c4:ea60'

    def test_device_node_exists(self):
        assert os.path.exists(self.DEVICE), (
            f'{self.DEVICE} not present. Plug in the RPLIDAR USB cable, '
            f'or check the CP210x bridge with `lsusb | grep -i silicon`.'
        )

    def test_cp210x_bridge_enumerated(self):
        assert _lsusb_match(self.CP210X_USB_ID), (
            'CP210x USB-serial bridge not detected. The LIDAR USB cable is '
            'likely not connected; also verify the green motor cable.'
        )


# ---------------------------------------------------------------------------
# EasySMX gamepad
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestEasySMX:
    JS_DEVICE = '/dev/input/js0'

    def test_joystick_device_present(self):
        assert os.path.exists(self.JS_DEVICE), (
            f'{self.JS_DEVICE} not present. Plug in the EasySMX USB dongle '
            f'and power the controller; check `ls /dev/input/js*`.'
        )


# ---------------------------------------------------------------------------
# LSM9DS1 IMU — accel/gyro at 0x6B, magnetometer at 0x1E on I²C bus 1
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestLSM9DS1:
    BUS = 1
    ACCEL_GYRO_ADDR = 0x6B
    MAGNETOMETER_ADDR = 0x1E

    def test_i2c_bus_present(self):
        path = f'/dev/i2c-{self.BUS}'
        assert os.path.exists(path), (
            f'{path} not present. Enable I²C: '
            'sudo raspi-config nonint do_i2c 0 && sudo reboot.'
        )

    def test_user_in_i2c_group(self):
        assert _user_in_group('i2c'), (
            'User not in i2c group. Fix: '
            'sudo usermod -aG i2c $USER and log out + back in.'
        )

    def test_accel_gyro_responds(self):
        assert _i2c_probe(self.BUS, self.ACCEL_GYRO_ADDR), (
            f'No device at I²C bus {self.BUS} address '
            f'0x{self.ACCEL_GYRO_ADDR:02x} (LSM9DS1 accel/gyro). Check '
            f'wiring and that the IMU board is powered.'
        )

    def test_magnetometer_responds(self):
        assert _i2c_probe(self.BUS, self.MAGNETOMETER_ADDR), (
            f'No device at I²C bus {self.BUS} address '
            f'0x{self.MAGNETOMETER_ADDR:02x} (LSM9DS1 magnetometer). Check '
            f'wiring.'
        )


# ---------------------------------------------------------------------------
# Forward camera — Logitech BRIO over V4L2
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestForwardCamera:
    def test_v4l2_device_enumerated(self):
        try:
            out = subprocess.run(
                ['v4l2-ctl', '--list-devices'],
                capture_output=True, text=True, timeout=5,
            ).stdout
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pytest.fail('v4l2-ctl missing — `sudo apt install v4l-utils`.')
        assert '/dev/video' in out, (
            'No V4L2 video devices enumerated. Plug in the forward camera; '
            'check `lsusb | grep -i logitech`.'
        )


# ---------------------------------------------------------------------------
# Backward camera — Arducam B0578
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestArducam:
    USB_ID = '0c45:0578'

    def test_usb_present(self):
        assert _lsusb_match(self.USB_ID), (
            f'Arducam B0578 (USB {self.USB_ID}) not detected. Check the '
            f'rear camera USB cable.'
        )


# ---------------------------------------------------------------------------
# Coral EdgeTPU — USB accelerator (Phase 3A)
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestCoral:
    # USB ID flips after the first firmware load; either is acceptable.
    PRE_INIT_ID = '1a6e:089a'
    POST_INIT_ID = '18d1:9302'

    def test_usb_present(self):
        assert _lsusb_match(self.PRE_INIT_ID) or _lsusb_match(self.POST_INIT_ID), (
            f'Coral EdgeTPU not detected (looking for USB {self.PRE_INIT_ID} '
            f'or {self.POST_INIT_ID}). Plug in the USB accelerator.'
        )

    @pytest.mark.skipif(
        importlib.util.find_spec('tflite_runtime') is None,
        reason='tflite_runtime not yet installed (Phase 3A)',
    )
    def test_tflite_runtime_importable(self):
        import tflite_runtime  # noqa: F401

    @pytest.mark.skipif(
        importlib.util.find_spec('pycoral') is None,
        reason='pycoral not yet installed (Phase 3A)',
    )
    def test_pycoral_importable(self):
        import pycoral.utils.edgetpu  # noqa: F401


# ---------------------------------------------------------------------------
# MAX7219 dot matrix — SPI (Phase 3B)
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestDotMatrix:
    SPI_DEVICE = '/dev/spidev0.0'

    def test_spi_device_present(self):
        assert os.path.exists(self.SPI_DEVICE), (
            f'{self.SPI_DEVICE} not present. Enable SPI: '
            'sudo raspi-config nonint do_spi 0 && sudo reboot.'
        )

    def test_user_in_spi_group(self):
        try:
            grp.getgrnam('spi')
        except KeyError:
            pytest.skip(
                'spi group not present yet; enable SPI with '
                '`sudo raspi-config nonint do_spi 0 && sudo reboot` first.'
            )
        assert _user_in_group('spi'), (
            'User not in spi group. Fix: '
            'sudo usermod -aG spi $USER and log out + back in.'
        )

    @pytest.mark.skipif(
        importlib.util.find_spec('luma') is None,
        reason='luma.led_matrix not yet installed (Phase 3B)',
    )
    def test_luma_importable(self):
        import luma.led_matrix  # noqa: F401


# ---------------------------------------------------------------------------
# RTC backup battery — vcgencmd pmic_read_adc BATT_V on Pi 5
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestRTC:
    BATT_MIN_VOLTS = 3.0

    def _read_batt_volts(self):
        """Return RTC battery voltage in volts, or None if vcgencmd is unusable."""
        try:
            r = subprocess.run(
                ['vcgencmd', 'pmic_read_adc', 'BATT_V'],
                capture_output=True, text=True, timeout=5,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return None
        if r.returncode != 0 or 'BATT_V' not in r.stdout:
            return None
        m = re.search(r'BATT_V\s+volt\(\d+\)=([0-9.]+)V', r.stdout)
        return float(m.group(1)) if m else None

    def test_user_in_video_group(self):
        assert _user_in_group('video'), (
            'User not in video group (needed for /dev/vcio → vcgencmd). Fix: '
            'sudo usermod -aG video $USER and log out + back in.'
        )

    def test_battery_above_threshold(self):
        volts = self._read_batt_volts()
        if volts is None:
            pytest.skip(
                'vcgencmd pmic_read_adc BATT_V unavailable; either /dev/vcio '
                'access is missing (relog after adding video group) or this '
                "isn't a Pi 5."
            )
        assert volts >= self.BATT_MIN_VOLTS, (
            f'RTC backup battery at {volts:.2f}V (threshold '
            f'{self.BATT_MIN_VOLTS}V). Replace the CR2032 cell on the Pi 5 '
            f'RTC connector — without it the clock resets on every power-off.'
        )


# ---------------------------------------------------------------------------
# Python runtime dependencies for the driver
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestDependencies:
    @pytest.mark.parametrize('module', [
        'ackermann_msgs',
        'cv2',
        'numpy',
        'rclpy',
        'sensor_msgs',
        'serial',
        'smbus',
        'spidev',
    ])
    def test_module_importable(self, module):
        try:
            importlib.import_module(module)
        except ImportError as e:
            pytest.fail(f'{module} not importable: {e}. Run scripts/setup_all.sh.')
