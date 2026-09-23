"""
Pre-flight hardware connectivity tests for RACECAR Neo v2.

Each assertion's failure message includes a one-line fix hint. Tests for
absent hardware fail. Run them with `pytest -m hardware`; skip them with
`pytest -m 'not hardware'`.
"""

import glob
import grp
import importlib
import importlib.util
import os
from pathlib import Path
import pwd
import subprocess
import time

from conftest import load_script
import pytest

sysinfo = load_script('sysinfo')


def _lsusb_match(vid_pid):
    try:
        out = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return vid_pid.lower() in out.lower()


def _lspci_match(vid_pid):
    try:
        out = subprocess.run(['lspci', '-nn'], capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return vid_pid.lower() in out.lower()


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
# RPLIDAR 2D LIDAR
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestRPLIDAR:
    DEVICE = '/dev/lidar'
    CP210X_USB_ID = '10c4:ea60'

    def test_device_symlink_exists(self):
        assert os.path.exists(self.DEVICE), (
            f'{self.DEVICE} symlink not present. Run `bash scripts/setup_udev.sh` '
            f'and unplug+replug the LIDAR USB cable. '
            f'Check the CP210x bridge with `lsusb | grep -i silicon`.'
        )

    def test_cp210x_bridge_enumerated(self):
        assert _lsusb_match(self.CP210X_USB_ID), (
            'CP210x USB-serial bridge not detected. The LIDAR USB cable is '
            'likely not connected; also verify the green motor cable.'
        )


# ---------------------------------------------------------------------------
# Gamepad: any USB HID joystick (EasySMX, Switch Pro, Xbox, etc.)
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestGamepad:
    def test_gamepad_present(self):
        # joy_node reads joydev (/dev/input/jsN) or evdev (/dev/input/eventN);
        # some controllers, such as the Switch Pro, expose only evdev.
        js_devices = glob.glob('/dev/input/js*')
        joystick_event = False
        try:
            with open('/proc/bus/input/devices') as f:
                blob = f.read()
            for block in blob.split('\n\n'):
                if 'EV=' in block and ('ABS' in block or 'js' in block.lower()):
                    if 'Handlers=' in block and 'event' in block:
                        joystick_event = True
                        break
        except OSError:
            pass
        assert js_devices or joystick_event, (
            'No joystick detected. Plug in the gamepad and ensure the USB '
            'dongle is seated; check `ls /dev/input/` and '
            '`cat /proc/bus/input/devices`.'
        )


# ---------------------------------------------------------------------------
# RealSense D435i
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestRealSense:
    USB_ID = '8086:0b3a'

    def test_usb_present(self):
        assert _lsusb_match(self.USB_ID), (
            f'RealSense D435i (USB {self.USB_ID}) not detected on the USB bus. '
            f'Check the USB 3.0 cable and port.'
        )

    def test_v4l2_devices_exist(self):
        out = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
        assert 'RealSense' in out, (
            'No RealSense V4L2 devices found. '
            'Check the USB connection and try: rs-enumerate-devices --compact'
        )

    def test_rs_enumerate(self):
        result = subprocess.run(
            ['rs-enumerate-devices', '--compact'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert (
            result.returncode == 0
        ), 'rs-enumerate-devices failed. Install: bash scripts/setup_realsense.sh'
        assert (
            'D435I' in result.stdout or 'D435i' in result.stdout
        ), f'D435i not found in rs-enumerate-devices output:\n{result.stdout}'

    def test_usb3_connection(self):
        out = subprocess.run(
            ['rs-enumerate-devices', '--compact'],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout
        if 'Usb Type Descriptor' not in out:
            pytest.skip('rs-enumerate-devices reported no USB type descriptor')
        assert '3.' in out.split('Usb Type Descriptor')[1].split('\n')[0], (
            'RealSense is not on a USB 3.x port. '
            'Depth + color + IMU at full rate requires USB 3.0+.'
        )

    def test_imu_permissions(self):
        iio_base = '/sys/bus/iio/devices'
        if not os.path.isdir(iio_base):
            pytest.skip('No IIO subsystem (not running on Pi 5?)')
        iio_devices = [
            os.path.join(iio_base, d) for d in os.listdir(iio_base) if d.startswith('iio:device')
        ]
        if not iio_devices:
            pytest.skip('No IIO devices found (RealSense IMU may not be enumerated yet)')
        bad = []
        for dev in iio_devices:
            buf_enable = os.path.join(dev, 'buffer', 'enable')
            if os.path.exists(buf_enable) and not os.access(buf_enable, os.W_OK):
                bad.append(buf_enable)
        assert not bad, (
            f'IMU IIO permissions not fixed ({len(bad)} file(s) not writable). '
            'Fix: sudo /usr/local/bin/fix-realsense-imu.sh\n'
            'Or run: bash scripts/setup_realsense.sh'
        )

    def test_imu_fix_script_installed(self):
        script = '/usr/local/bin/fix-realsense-imu.sh'
        assert os.path.isfile(script), f'{script} not found. Run: bash scripts/setup_realsense.sh'
        assert os.access(
            script, os.X_OK
        ), f'{script} is not executable. Fix: sudo chmod +x {script}'


# ---------------------------------------------------------------------------
# Coral EdgeTPU (M.2 primary, USB fallback)
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestCoral:
    M2_PCI_ID = '1ac1:089a'
    APEX_DEV = '/dev/apex_0'
    # The USB accelerator's ID changes after its first firmware load.
    PRE_INIT_ID = '1a6e:089a'
    POST_INIT_ID = '18d1:9302'

    def test_device_present(self):
        m2 = _lspci_match(self.M2_PCI_ID) or os.path.exists(self.APEX_DEV)
        usb = _lsusb_match(self.PRE_INIT_ID) or _lsusb_match(self.POST_INIT_ID)
        assert m2 or usb, (
            f'Coral EdgeTPU not detected. Expected the M.2 Apex '
            f'(PCI {self.M2_PCI_ID} or {self.APEX_DEV}) or the USB accelerator '
            f'(USB {self.PRE_INIT_ID}/{self.POST_INIT_ID}). Run setup_coral.sh; '
            f'the M.2 needs a reboot after setup.'
        )

    @pytest.mark.skipif(
        importlib.util.find_spec('tflite_runtime') is None,
        reason='tflite_runtime not installed',
    )
    def test_tflite_runtime_importable(self):
        import tflite_runtime  # noqa: F401

    @pytest.mark.skipif(
        importlib.util.find_spec('pycoral') is None,
        reason='pycoral not installed',
    )
    def test_pycoral_importable(self):
        import pycoral.utils.edgetpu  # noqa: F401

    # Steady state is single-digit ms on the M.2 card; 100 ms absorbs warmup
    # and bus contention with the camera streams.
    INFERENCE_BUDGET_MS = 100.0

    @pytest.mark.skipif(
        importlib.util.find_spec('pycoral') is None,
        reason='pycoral not installed',
    )
    def test_inference_within_latency_budget(self):
        import numpy as np
        from pycoral.utils.edgetpu import list_edge_tpus, make_interpreter

        if not list_edge_tpus():
            pytest.skip('No EdgeTPU device; cannot run inference')

        # The EdgeTPU delegate is single-user; a running edgetpu_node holds it.
        running = subprocess.run(
            ['pgrep', '-f', 'lib/racecar_neo_ros2_driver/edgetpu_node'],
            capture_output=True,
        )
        if running.returncode == 0:
            pytest.skip('edgetpu_node is running; cannot test in isolation')

        model = (
            Path(__file__).parent.parent / 'models' / 'efficientdet_lite0_generic_edgetpu.tflite'
        )
        if not model.exists():
            pytest.skip(f'Model file missing: {model}')

        # Retry once on first call (cold-boot Coral firmware reload).
        try:
            interpreter = make_interpreter(str(model))
        except ValueError:
            time.sleep(1.5)
            interpreter = make_interpreter(str(model))

        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()[0]
        _, h, w, _ = input_details['shape']
        frame = np.full((1, h, w, 3), 128, dtype=np.uint8)

        # The first invocation pays ~30 ms of one-time allocation.
        interpreter.set_tensor(input_details['index'], frame)
        interpreter.invoke()

        n = 10
        t0 = time.monotonic()
        for _ in range(n):
            interpreter.set_tensor(input_details['index'], frame)
            interpreter.invoke()
        mean_ms = (time.monotonic() - t0) / n * 1000.0

        assert mean_ms < self.INFERENCE_BUDGET_MS, (
            f'Mean inference latency {mean_ms:.1f} ms exceeds '
            f'{self.INFERENCE_BUDGET_MS:.0f} ms budget. Possible causes: '
            f'USB-2 hub instead of direct USB-3 port, model not '
            f'edgetpu_compiler-compiled, or libedgetpu version mismatch.'
        )


# ---------------------------------------------------------------------------
# RTC backup battery: vcgencmd pmic_read_adc BATT_V on Pi 5
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestRTC:
    BATT_MIN_VOLTS = sysinfo.RTC_LOW_VOLTS

    def test_user_in_video_group(self):
        assert _user_in_group('video'), (
            'User not in video group (needed for /dev/vcio for vcgencmd). Fix: '
            'sudo usermod -aG video $USER and log out + back in.'
        )

    def test_battery_above_threshold(self):
        volts = sysinfo.read_rtc_voltage()
        if volts is None:
            pytest.skip(
                'vcgencmd pmic_read_adc BATT_V unavailable; either /dev/vcio '
                'access is missing (relog after adding video group) or this '
                "isn't a Pi 5."
            )
        assert volts >= self.BATT_MIN_VOLTS, (
            f'RTC backup battery at {volts:.2f}V (floor '
            f'{self.BATT_MIN_VOLTS}V). Recharge the Pi 5 RTC backup cell '
            f'(usable 2.7-3.0 V); below 2.7 V the clock resets on every '
            f'power-off.'
        )


# ---------------------------------------------------------------------------
# Python runtime dependencies for the driver
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestDependencies:
    @pytest.mark.parametrize(
        'module',
        [
            'ackermann_msgs',
            'diagnostic_msgs',
            'luma.core',
            'nav_msgs',
            'numpy',
            'PIL',
            'rclpy',
            'rosbag2_py',
            'sensor_msgs',
            'serial',
            'vision_msgs',
            'yaml',
        ],
    )
    def test_module_importable(self, module):
        try:
            importlib.import_module(module)
        except ImportError as e:
            pytest.fail(f'{module} not importable: {e}. Run scripts/setup_all.sh.')
