"""
Sanity tests for scripts/setup_*.sh.

Catches the most common breakages: missing files, missing exec bit, bash
syntax errors, and the orchestrator forgetting to call a phase script.
"""

import os
from pathlib import Path
import subprocess

import pytest

SCRIPTS_DIR = Path(__file__).parent.parent / 'scripts'

PHASE_SCRIPTS = [
    'setup_ros2.sh',
    'setup_dev_tools.sh',
    'setup_user_env.sh',
    'setup_raspi_config.sh',
    'setup_udev.sh',
    'setup_dotmatrix.sh',
    'setup_coral.sh',
    'patch_gscam.sh',
    'setup_workspace.sh',
    'setup_jupyter.sh',
    'setup_services.sh',
]
ORCHESTRATOR = 'setup_all.sh'
ALL_SCRIPTS = PHASE_SCRIPTS + [ORCHESTRATOR]


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_exists(name):
    assert (SCRIPTS_DIR / name).is_file(), f'{name} missing from scripts/'


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_is_executable(name):
    assert os.access(SCRIPTS_DIR / name, os.X_OK), f'{name} missing +x bit'


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_has_bash_hashbang(name):
    first = (SCRIPTS_DIR / name).read_text().splitlines()[0]
    assert first.startswith('#!'), f'{name} missing shebang'
    assert 'bash' in first, f'{name} should use bash (got: {first!r})'


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_passes_bash_syntax(name):
    """`bash -n` parses without executing — catches typos and unclosed quotes."""
    result = subprocess.run(
        ['bash', '-n', str(SCRIPTS_DIR / name)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, (
        f'{name} fails bash -n:\n{result.stderr}'
    )


def test_orchestrator_calls_every_phase_script():
    """setup_all.sh must invoke every phase script we ship."""
    text = (SCRIPTS_DIR / ORCHESTRATOR).read_text()
    for phase in PHASE_SCRIPTS:
        assert phase in text, f'{ORCHESTRATOR} does not reference {phase}'


def test_scripts_use_set_dash_e():
    """Phase scripts should exit on first error so partial setup is loud."""
    for name in PHASE_SCRIPTS + [ORCHESTRATOR]:
        text = (SCRIPTS_DIR / name).read_text()
        assert 'set -e' in text, f'{name} should `set -e` for fail-fast'


def test_no_stray_colcon_dirs_in_package():
    """build/, install/, log/ must live in the workspace root, not the package."""
    pkg_root = SCRIPTS_DIR.parent
    for d in ('build', 'install', 'log'):
        stray = pkg_root / d
        assert not stray.exists(), (
            f'{stray} exists; colcon was invoked from the wrong CWD. '
            f'Always run `colcon build` from $HOME/ros2_ws, not the package dir.'
        )


class TestLaunchWrapper:
    """launch_teleop.sh is the runtime wrapper systemd / racecar-tool calls."""

    WRAPPER = SCRIPTS_DIR / 'launch_teleop.sh'

    def test_exists_and_executable(self):
        assert self.WRAPPER.is_file()
        assert os.access(self.WRAPPER, os.X_OK)

    def test_bash_syntax_clean(self):
        result = subprocess.run(
            ['bash', '-n', str(self.WRAPPER)],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0, result.stderr

    def test_creates_log_dir_and_symlink(self):
        text = self.WRAPPER.read_text()
        # Two-part contract: timestamped subdir + atomic 'latest' symlink.
        assert 'mkdir -p "$LOG_DIR"' in text
        assert 'ln -sfn "$LOG_DIR" "$HOME/logs/latest"' in text

    def test_sweeps_fastrtps_shm_orphans(self):
        text = self.WRAPPER.read_text()
        assert '/dev/shm/fastrtps_port' in text

    def test_execs_ros2_launch(self):
        # The final `exec ros2 launch` is what lets systemd track the launch PID.
        text = self.WRAPPER.read_text()
        assert 'exec ros2 launch racecar_neo_ros2_driver teleop.launch.py' in text


class TestSystemdServices:
    """The four racecar-*.service files ship with the package."""

    SERVICES = (
        'racecar-teleop.service',
        'racecar-watchdog.service',
        'racecar-dashboard.service',
        'racecar-jupyter.service',
    )

    @pytest.mark.parametrize('name', SERVICES)
    def test_service_file_exists(self, name):
        assert (SCRIPTS_DIR / name).is_file()

    @pytest.mark.parametrize('name', SERVICES)
    def test_has_required_sections(self, name):
        text = (SCRIPTS_DIR / name).read_text()
        for section in ('[Unit]', '[Service]', '[Install]'):
            assert section in text, f'{name} missing {section}'

    @pytest.mark.parametrize('name', SERVICES)
    def test_wantedby_multi_user(self, name):
        text = (SCRIPTS_DIR / name).read_text()
        assert 'WantedBy=multi-user.target' in text

    @pytest.mark.parametrize('name', SERVICES)
    def test_runs_as_racecar_user(self, name):
        text = (SCRIPTS_DIR / name).read_text()
        assert 'User=racecar' in text
        assert 'Group=racecar' in text

    def test_watchdog_bindsto_teleop(self):
        # BindsTo means watchdog stops when teleop stops — exactly what we want.
        text = (SCRIPTS_DIR / 'racecar-watchdog.service').read_text()
        assert 'BindsTo=racecar-teleop.service' in text
        assert 'After=racecar-teleop.service' in text

    def test_teleop_wants_watchdog(self):
        # Wants= pulls watchdog along whenever teleop starts (manual or boot).
        # Without this, `systemctl start racecar-teleop` only starts teleop.
        text = (SCRIPTS_DIR / 'racecar-teleop.service').read_text()
        assert 'Wants=racecar-watchdog.service' in text

    def test_teleop_calls_launch_wrapper(self):
        text = (SCRIPTS_DIR / 'racecar-teleop.service').read_text()
        assert 'launch_teleop.sh' in text

    def test_watchdog_invokes_watchdog_py(self):
        text = (SCRIPTS_DIR / 'racecar-watchdog.service').read_text()
        assert 'watchdog.py' in text


class TestUdevRules:
    """The 99-racecar.rules file ships with the package and binds each peripheral."""

    RULES_FILE = SCRIPTS_DIR / 'udev' / '99-racecar.rules'

    def test_rules_file_exists(self):
        assert self.RULES_FILE.is_file(), f'{self.RULES_FILE} missing'

    @pytest.mark.parametrize('symlink', [
        'maestro', 'lidar', 'cam_forward', 'cam_backward',
    ])
    def test_rules_define_symlink(self, symlink):
        text = self.RULES_FILE.read_text()
        assert f'SYMLINK+="{symlink}"' in text, (
            f'No rule defines /dev/{symlink}'
        )

    @pytest.mark.parametrize('vid_pid', [
        ('10c4', 'ea60'),  # CP2102 (RPLIDAR)
        ('046d', '085e'),  # Logitech BRIO
        ('0c45', '0578'),  # Arducam B0578
        ('1a6e', '089a'),  # Coral pre-init
        ('18d1', '9302'),  # Coral post-init
    ])
    def test_rules_match_known_usb_ids(self, vid_pid):
        # Maestro uses ENV-style matching (see test below) — exempted.
        vid, pid = vid_pid
        text = self.RULES_FILE.read_text()
        assert f'ATTRS{{idVendor}}=="{vid}"' in text, f'VID {vid} not matched'
        assert f'ATTRS{{idProduct}}=="{pid}"' in text, f'PID {pid} not matched'

    def test_maestro_rule_pins_command_interface(self):
        # The Maestro exposes two CDC ACM interfaces (00 = command, 02 = aux TTL).
        # The rule must pin interface 00 or /dev/maestro races between the two.
        text = self.RULES_FILE.read_text()
        assert 'ENV{ID_VENDOR_ID}=="1ffb"' in text, 'Maestro VID not matched via ENV'
        assert 'ENV{ID_USB_INTERFACE_NUM}=="00"' in text, (
            'Maestro rule must pin ID_USB_INTERFACE_NUM=00 (command port). '
            'Without this, /dev/maestro may bind to the wrong CDC interface.'
        )
