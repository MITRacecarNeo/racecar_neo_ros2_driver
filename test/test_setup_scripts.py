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
    'setup_realsense.sh',
    'setup_workspace.sh',
    'setup_jupyter.sh',
    'setup_services.sh',
    'setup_dashboards.sh',
]
ORCHESTRATOR = 'setup_all.sh'

# Scripts that ship with the package but are NOT called by setup_all.sh —
# the user runs them manually (or via `racecar setup <phase>`) because their
# side-effects are too disruptive to include in a one-shot install.
STANDALONE_SCRIPTS = [
    'setup_networking.sh',  # reconfigures wlan0; can drop SSH-over-WiFi sessions
    'setup_eth.sh',  # switches eth0 addressing mode; drops SSH sessions on eth0
    'flash_realsense_offline.sh',  # per-machine camera firmware flash (airgapped)
    'setup_nvme.sh',  # erases the target disk; must be an explicit, typed choice
]

ALL_SCRIPTS = PHASE_SCRIPTS + [ORCHESTRATOR] + STANDALONE_SCRIPTS


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


def test_scripts_use_pipefail():
    """Pipefail catches `wget | dpkg -i` style chains where the upstream silently fails."""
    for name in PHASE_SCRIPTS + [ORCHESTRATOR]:
        text = (SCRIPTS_DIR / name).read_text()
        assert 'pipefail' in text, f'{name} should set `pipefail` (use `set -eo pipefail`)'


def test_no_stray_colcon_dirs_in_package():
    """build/, install/, log/ must live in the workspace root, not the package."""
    pkg_root = SCRIPTS_DIR.parent
    for d in ('build', 'install', 'log'):
        stray = pkg_root / d
        assert not stray.exists(), (
            f'{stray} exists; colcon was invoked from the wrong CWD. '
            f'Always run `colcon build` from $HOME/ros2_ws, not the package dir.'
        )


class TestNetworkingScript:
    """setup_networking.sh: eth0 addressing + wlan0 isolated AP (standalone)."""

    SCRIPT = SCRIPTS_DIR / 'setup_networking.sh'

    def test_exists_and_executable(self):
        assert self.SCRIPT.is_file()
        assert os.access(self.SCRIPT, os.X_OK)

    def test_bash_syntax_clean(self):
        result = subprocess.run(
            ['bash', '-n', str(self.SCRIPT)],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0, result.stderr

    def test_not_in_orchestrator(self):
        # setup_networking.sh must NOT be in setup_all.sh — it reconfigures
        # wlan0 and would drop SSH-over-WiFi sessions during a fresh install.
        text = (SCRIPTS_DIR / 'setup_all.sh').read_text()
        assert 'setup_networking.sh' not in text, (
            'setup_networking.sh should be standalone; running it from '
            'setup_all.sh can drop SSH-over-WiFi sessions during install.'
        )

    def test_parameterized_via_env_vars(self):
        # Each tunable should be readable from an environment variable so
        # the racecar-tool can pass overrides without editing the script.
        text = self.SCRIPT.read_text()
        for var in ('RACECAR_AP_SSID', 'RACECAR_AP_PSK', 'RACECAR_AP_CHANNEL',
                    'RACECAR_AP_ADDR', 'RACECAR_AP_IFACE', 'RACECAR_ETH_STATIC'):
            assert var in text, f'{var} not referenced in setup_networking.sh'

    def test_ap_on_alfa_dongle_not_wlan0(self):
        # v0.7.0 moved the AP onto the ALFA dongle (default wlan1) and returns
        # wlan0 to default client mode. The AP interface must be parameterized,
        # and the script must reset wlan0 (set it managed).
        text = self.SCRIPT.read_text()
        assert 'AP_IFACE' in text, 'AP interface should be parameterized'
        assert 'wlan1' in text, 'default AP interface (wlan1) not referenced'
        assert 'device set wlan0 managed' in text, (
            'setup_networking.sh must reset the Pi built-in wlan0 to managed/client'
        )

    def test_reset_mode_disables_ap_only(self):
        # RACECAR_AP_RESET=1 tears down the AP connection and exits before the
        # eth0 section (imaging: ship a generic clone with no active AP).
        text = self.SCRIPT.read_text()
        assert 'RACECAR_AP_RESET' in text, 'reset mode not handled'
        assert 'connection delete' in text, 'reset mode must delete the AP connection'

    def test_ssid_composed_from_car_id(self):
        # SSID is a fixed base plus a per-car ID so multiple cars differ.
        text = self.SCRIPT.read_text()
        assert 'RACECAR_AP_ID' in text, 'per-car SSID id not referenced'
        assert 'racecar-neo' in text, 'SSID base not referenced'

    def test_eth0_delegated_to_setup_eth(self):
        # v0.7.4: this script no longer renders its own eth0 netplan block.
        # It used to emit a dual-IP stanza (static AND dhcp4 together), which
        # is what made the static drop. setup_eth.sh is the only writer now,
        # so the two paths cannot disagree about the file.
        text = self.SCRIPT.read_text()
        assert 'setup_eth.sh' in text, 'eth0 config must delegate to setup_eth.sh'
        assert 'network:\n  version: 2' not in text, (
            'setup_networking.sh must not render netplan YAML itself'
        )
        assert 'RACECAR_ETH_MODE' in text, 'eth0 mode must be parameterized'

    def test_loads_persisted_config(self):
        # The script must source the ~/.config/racecar/networking.env file
        # so the user's persisted overrides apply on every run.
        text = self.SCRIPT.read_text()
        assert 'networking.env' in text

    def test_ap_isolation_dispatcher_configured(self):
        # The whole point of "isolated AP" is the iptables FORWARD reject
        # rules. Make sure the dispatcher script body is wired up.
        text = self.SCRIPT.read_text()
        assert 'iptables' in text
        assert 'FORWARD' in text
        assert '99-racecar-ap-isolate' in text

    def test_enables_networkmanager_dispatcher_service(self):
        # On Ubuntu Server the dispatcher service is enabled by default, but
        # on Desktop / Raspberry Pi OS it's typically inactive. Without it
        # the dispatcher script never gets invoked and the isolation rules
        # silently never apply — exactly the bug v0.0.6 hit on first install.
        text = self.SCRIPT.read_text()
        assert 'NetworkManager-dispatcher.service' in text
        assert 'systemctl enable' in text


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
        'neo-pit-pcb', 'lidar',
    ])
    def test_rules_define_symlink(self, symlink):
        text = self.RULES_FILE.read_text()
        assert f'SYMLINK+="{symlink}"' in text, (
            f'No rule defines /dev/{symlink}'
        )

    @pytest.mark.parametrize('vid_pid', [
        ('10c4', 'ea60'),  # CP2102 (RPLIDAR)
        ('1a6e', '089a'),  # Coral pre-init
        ('18d1', '9302'),  # Coral post-init
    ])
    def test_rules_match_known_usb_ids(self, vid_pid):
        # Maestro uses ENV-style matching (see test below) — exempted.
        vid, pid = vid_pid
        text = self.RULES_FILE.read_text()
        assert f'ATTRS{{idVendor}}=="{vid}"' in text, f'VID {vid} not matched'
        assert f'ATTRS{{idProduct}}=="{pid}"' in text, f'PID {pid} not matched'

    def test_alfa_ap_dongle_renamed_to_wlan1(self):
        # v0.7.0: the ALFA MT7612U (0e8d:7612) hosts the WiFi AP and must be
        # renamed to a stable wlan1 so setup_networking.sh binds a fixed name
        # instead of the per-unit MAC-derived wlx<mac>.
        text = self.RULES_FILE.read_text()
        alfa = [ln for ln in text.splitlines()
                if 'ATTRS{idVendor}=="0e8d"' in ln and 'ATTRS{idProduct}=="7612"' in ln]
        assert alfa, 'no rule matches the ALFA MT7612U (0e8d:7612)'
        assert any('NAME="wlan1"' in ln for ln in alfa), (
            'ALFA rule must rename the dongle to wlan1'
        )

    def test_realsense_autosuspend_rule_present(self):
        # RealSense D435i (USB 8086:0b3a). The autosuspend rule matches the usb
        # device node directly, so it uses ATTR (singular), not ATTRS.
        text = self.RULES_FILE.read_text()
        assert 'ATTR{idVendor}=="8086"' in text, 'RealSense VID not matched'
        assert 'ATTR{idProduct}=="0b3a"' in text, 'RealSense PID not matched'

    def test_lidar_rule_ignores_modemmanager(self):
        # ModemManager will probe any tty unless told otherwise. For the lidar's
        # CP2102, that probe desynced the sllidar SDK's binary frame reader
        # mid-stream during the 2026-05-12 endurance test and /scan went silent
        # without the process dying. ID_MM_DEVICE_IGNORE=1 prevents recurrence.
        text = self.RULES_FILE.read_text()
        lidar_lines = [ln for ln in text.splitlines() if 'SYMLINK+="lidar"' in ln]
        assert lidar_lines, 'lidar rule missing'
        assert any('ID_MM_DEVICE_IGNORE' in ln for ln in lidar_lines), (
            'lidar rule must set ID_MM_DEVICE_IGNORE=1 to block ModemManager probes'
        )

    def test_neo_pit_rule_matches_gpio_uart(self):
        # The NEO-PIT PCB is on the Pi's GPIO UART, which enumerates as ttyAMA0
        # on Pi 5 / Ubuntu (there is no /dev/serial0). Pin the symlink to that
        # kernel name so ttyAMA10 (the SoC debug UART) is never matched.
        text = self.RULES_FILE.read_text()
        assert 'KERNEL=="ttyAMA0"' in text, 'neo-pit-pcb rule must match ttyAMA0'
        assert 'SYMLINK+="neo-pit-pcb"' in text, 'neo-pit-pcb symlink rule missing'


class TestHidNintendoBlacklist:
    """The kernel blacklist that unbreaks the EasySMX KC-8236 on Pi 5."""

    CONF = SCRIPTS_DIR / 'modprobe.d' / 'blacklist-hid-nintendo.conf'

    def test_blacklist_file_exists(self):
        assert self.CONF.is_file()

    def test_blacklists_hid_nintendo(self):
        # Must blacklist with the underscore-form module name (`hid_nintendo`,
        # not `hid-nintendo`); modprobe accepts either, but the underscore
        # form matches what `lsmod` reports and what the kernel uses internally.
        text = self.CONF.read_text()
        assert 'blacklist hid_nintendo' in text

    def test_setup_udev_installs_blacklist(self):
        # The setup script must copy the .conf to /etc/modprobe.d/ AND
        # regenerate the initramfs (since hid_nintendo can be loaded from
        # initramfs before /etc/modprobe.d/ is read).
        text = (SCRIPTS_DIR / 'setup_udev.sh').read_text()
        assert 'blacklist-hid-nintendo.conf' in text
        assert '/etc/modprobe.d/' in text
        # initramfs regen MUST be conditional on a content change — a fresh
        # update-initramfs takes ~30s and we'd run it on every setup_all.sh.
        assert 'update-initramfs' in text
        # And we should unload the running module so the change applies in
        # this boot (otherwise it only takes effect on the next reboot).
        assert 'modprobe -r hid_nintendo' in text


class TestEthScript:
    """setup_eth.sh: eth0 in exactly one IPv4 addressing mode."""

    SCRIPT = SCRIPTS_DIR / 'setup_eth.sh'

    def _run(self, tmp_path, *args):
        """Run the script in dry-run mode, isolated from the real system files."""
        env = {k: v for k, v in os.environ.items() if not k.startswith('RACECAR_ETH')}
        env.update({
            'RACECAR_ETH_DRY_RUN': '1',
            'RACECAR_ETH_NETPLAN': str(tmp_path / '99-racecar-eth0.yaml'),
            'RACECAR_ETH_CONFIG': str(tmp_path / 'networking.conf'),
        })
        return subprocess.run(
            ['bash', str(self.SCRIPT), *args],
            capture_output=True, text=True, timeout=20, env=env,
        )

    def _render(self, tmp_path, mode, *extra):
        result = self._run(tmp_path, mode, *extra)
        assert result.returncode == 0, result.stderr
        return (tmp_path / '99-racecar-eth0.yaml').read_text()

    def test_exists_and_executable(self):
        assert self.SCRIPT.is_file()
        assert os.access(self.SCRIPT, os.X_OK)

    def test_bash_syntax_clean(self):
        result = subprocess.run(
            ['bash', '-n', str(self.SCRIPT)],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0, result.stderr

    def test_static_render(self, tmp_path):
        yaml = self._render(tmp_path, 'static')
        assert 'dhcp4: false' in yaml
        assert '"192.168.52.200/24"' in yaml
        assert 'addresses:' in yaml

    def test_dynamic_render(self, tmp_path):
        yaml = self._render(tmp_path, 'dynamic')
        assert 'dhcp4: true' in yaml
        assert 'route-metric: 100' in yaml

    def test_modes_are_mutually_exclusive(self, tmp_path):
        # The point of v0.7.4: eth0 never carries a static address and a DHCP
        # lease at the same time. Static must not enable dhcp4, and dynamic
        # must not declare a fixed address.
        static = self._render(tmp_path, 'static')
        assert 'dhcp4: true' not in static, 'static mode must not enable DHCP'

        dynamic = self._render(tmp_path, 'dynamic')
        assert 'addresses:' not in dynamic, 'dynamic mode must not declare a static address'
        assert 'dhcp4: false' not in dynamic

    def test_ipv6_never_default_is_static_only(self, tmp_path):
        # Router advertisements would hand eth0 a v6 default route even with no
        # v4 gateway, so static suppresses it. Dynamic is a normally-connected
        # mode and keeps it.
        assert 'ipv6.never-default: "true"' in self._render(tmp_path, 'static')
        assert 'ipv6.never-default' not in self._render(tmp_path, 'dynamic')

    def test_custom_static_address(self, tmp_path):
        yaml = self._render(tmp_path, 'static', '--addr=10.9.9.9/24')
        assert '"10.9.9.9/24"' in yaml
        assert '192.168.52.200' not in yaml

    def test_persists_mode(self, tmp_path):
        self._render(tmp_path, 'dynamic')
        cfg = (tmp_path / 'networking.conf').read_text()
        assert 'RACECAR_ETH_MODE="dynamic"' in cfg

    def test_persisted_static_address_round_trips(self, tmp_path):
        self._render(tmp_path, 'static', '--addr=172.16.4.4/24')
        # A later bare `static` reuses the persisted address.
        yaml = self._render(tmp_path, 'static')
        assert '"172.16.4.4/24"' in yaml

    def test_unknown_argument_errors(self, tmp_path):
        result = self._run(tmp_path, '--bogus')
        assert result.returncode == 2
        assert 'unknown argument' in result.stderr

    def test_default_action_is_status(self, tmp_path):
        # status is read-only; it exits 1 on a car that currently has an
        # address conflict, so either code is a valid outcome here.
        result = self._run(tmp_path)
        assert result.returncode in (0, 1), result.stderr
        assert 'addressing' in result.stdout

    def test_status_does_not_prompt_for_sudo(self, tmp_path):
        # netplan files are root-only. A read-only status must degrade to
        # "unreadable" rather than blocking on a password prompt.
        result = self._run(tmp_path, 'status')
        assert result.returncode in (0, 1)
        assert 'password' not in result.stderr.lower()


class TestDashboardScript:
    """setup_dashboards.sh renders our own units from the upstream templates."""

    SCRIPT = SCRIPTS_DIR / 'setup_dashboards.sh'

    @pytest.fixture(scope='class')
    def text(self):
        return self.SCRIPT.read_text()

    def test_pulls_are_fast_forward_only(self, text):
        # A diverged checkout means someone edited it; losing that is worse
        # than skipping the update.
        assert '--ff-only' in text
        assert 'reset --hard' not in text

    def test_never_enables_or_starts(self, text):
        # Six of seven publish /drive; enabling them all would put six
        # publishers on the mux at boot.
        assert 'systemctl enable' not in text
        assert 'systemctl start' not in text

    def test_renders_jazzy_not_humble(self, text):
        assert '/opt/ros/humble|/opt/ros/jazzy' in text

    def test_injects_the_discovery_scope(self, text):
        assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in text

    def test_renames_units_to_the_racecar_prefix(self, text):
        assert 'racecar-${unit#neoracer-}' in text

    def test_verifies_every_substitution_token(self, text):
        # A template that stops carrying a token has changed shape upstream;
        # installing the result anyway would point the unit at the wrong ROS.
        for token in ('@DIR@', '/opt/ros/humble', 'Environment=HOME='):
            assert token in text

    def test_clone_failure_is_not_fatal(self, text):
        assert 'fetch_repo "$repo" || true' in text

    def test_units_only_skips_the_network(self, text):
        assert '--units-only' in text
        assert 'MODE="units"' in text

    def test_env_var_skips_the_phase(self, text):
        assert 'RACECAR_DASHBOARDS' in text

    def test_all_seven_repositories(self, text):
        for repo in ('wallfollow', 'camlabel', 'pursuit', 'eps',
                     'smartfollow', 'linefollow', 'teleop'):
            assert f'{repo}_dashboard' in text

    def test_checkouts_are_gitignored(self):
        gitignore = (SCRIPTS_DIR.parent / '.gitignore').read_text()
        assert 'scripts/dashboards/' in gitignore

    def test_linters_exclude_the_checkouts(self):
        flake8 = (SCRIPTS_DIR.parent / 'test' / 'test_flake8.py').read_text()
        pep257 = (SCRIPTS_DIR.parent / 'test' / 'test_pep257.py').read_text()
        assert 'dashboards' in flake8
        assert 'dashboards' in pep257

    def test_pytest_does_not_collect_the_checkouts(self):
        cfg = (SCRIPTS_DIR.parent / 'setup.cfg').read_text()
        assert 'norecursedirs' in cfg
        assert 'scripts/dashboards' in cfg
