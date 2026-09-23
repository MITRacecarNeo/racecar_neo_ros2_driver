"""
Static and dry-run checks for the files under scripts/.

Covers the setup and runtime shell scripts, the Python entry points, and the
systemd, udev, polkit and modprobe files they install.
"""

import os
from pathlib import Path
import re
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

# Scripts that ship with the package but are NOT called by setup_all.sh;
# the user runs them manually (or via `racecar setup <phase>`) because their
# side-effects are too disruptive to include in a one-shot install.
STANDALONE_SCRIPTS = [
    'setup_networking.sh',  # reconfigures wlan0; can drop SSH-over-WiFi sessions
    'setup_eth.sh',  # switches eth0 addressing mode; drops SSH sessions on eth0
    'flash_realsense_offline.sh',  # per-machine camera firmware flash (airgapped)
    'setup_nvme.sh',  # erases the target disk; must be an explicit, typed choice
]

RUNTIME_SCRIPTS = ['launch_teleop.sh']

ALL_SCRIPTS = PHASE_SCRIPTS + [ORCHESTRATOR] + STANDALONE_SCRIPTS + RUNTIME_SCRIPTS

PY_SCRIPTS = sorted(p.name for p in SCRIPTS_DIR.glob('*.py'))
# Entry points run by path or by `ros2 run`; the rest are imported siblings.
PY_ENTRY_POINTS = [
    name for name in PY_SCRIPTS if "if __name__ == '__main__':" in (SCRIPTS_DIR / name).read_text()
]


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
    result = subprocess.run(
        ['bash', '-n', str(SCRIPTS_DIR / name)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, f'{name} fails bash -n:\n{result.stderr}'


@pytest.mark.parametrize('name', PY_SCRIPTS)
def test_python_script_compiles(name):
    result = subprocess.run(
        ['python3', '-m', 'py_compile', str(SCRIPTS_DIR / name)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('name', PY_ENTRY_POINTS)
def test_python_entry_point_is_executable(name):
    path = SCRIPTS_DIR / name
    assert path.read_text().startswith('#!/usr/bin/env python3\n'), f'{name} missing shebang'
    assert os.access(path, os.X_OK), f'{name} missing +x bit'


def test_python_entry_points_found():
    assert {'dashboard.py', 'diagnose.py', 'watchdog.py'} <= set(PY_ENTRY_POINTS)


def test_orchestrator_calls_every_phase_script():
    text = (SCRIPTS_DIR / ORCHESTRATOR).read_text()
    for phase in PHASE_SCRIPTS:
        assert phase in text, f'{ORCHESTRATOR} does not reference {phase}'


def test_scripts_use_set_dash_e():
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


@pytest.mark.parametrize('name', STANDALONE_SCRIPTS)
def test_standalone_scripts_stay_out_of_the_orchestrator(name):
    text = (SCRIPTS_DIR / ORCHESTRATOR).read_text()
    assert name not in text, f'{name} is disruptive and must stay out of {ORCHESTRATOR}'


class TestNetworkingScript:
    """setup_networking.sh: eth0 addressing + wlan1 isolated AP (standalone)."""

    SCRIPT = SCRIPTS_DIR / 'setup_networking.sh'

    def test_parameterized_via_env_vars(self):
        # racecar-tool passes overrides through the environment.
        text = self.SCRIPT.read_text()
        for var in (
            'RACECAR_AP_SSID',
            'RACECAR_AP_PSK',
            'RACECAR_AP_CHANNEL',
            'RACECAR_AP_ADDR',
            'RACECAR_AP_IFACE',
            'RACECAR_ETH_STATIC',
        ):
            assert var in text, f'{var} not referenced in setup_networking.sh'

    def test_ap_on_alfa_dongle_not_wlan0(self):
        # The AP runs on the ALFA dongle (wlan1); wlan0 is reset to a managed client.
        text = self.SCRIPT.read_text()
        assert 'AP_IFACE' in text, 'AP interface should be parameterized'
        assert 'wlan1' in text, 'default AP interface (wlan1) not referenced'
        assert (
            'device set wlan0 managed' in text
        ), 'setup_networking.sh must reset the Pi built-in wlan0 to managed/client'

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
        # setup_eth.sh is the only writer of the eth0 netplan file.
        text = self.SCRIPT.read_text()
        assert 'setup_eth.sh' in text, 'eth0 config must delegate to setup_eth.sh'
        assert (
            'network:\n  version: 2' not in text
        ), 'setup_networking.sh must not render netplan YAML itself'
        assert 'RACECAR_ETH_MODE' in text, 'eth0 mode must be parameterized'

    def test_eth_guard_is_not_bypassed(self):
        # setup_eth.sh asks before cutting off an eth0 SSH session; --force
        # would skip that.
        call = [ln for ln in self.SCRIPT.read_text().splitlines() if 'bash "$SETUP_ETH"' in ln]
        assert call, 'setup_eth.sh is not invoked'
        assert all('--force' not in ln and '-y' not in ln.split() for ln in call)

    def test_advice_is_console_or_wlan0(self):
        text = self.SCRIPT.read_text()
        assert 'console or over wlan0' in text
        assert 'wired (eth0)' not in text

    def test_loads_persisted_config(self):
        text = self.SCRIPT.read_text()
        assert 'networking.env' in text

    def test_ap_isolation_dispatcher_configured(self):
        # Isolation is the iptables FORWARD reject in the dispatcher script.
        text = self.SCRIPT.read_text()
        assert 'iptables' in text
        assert 'FORWARD' in text
        assert '99-racecar-ap-isolate' in text

    def test_enables_networkmanager_dispatcher_service(self):
        # See docs/troubleshooting.md, "AP isolation dispatcher".
        text = self.SCRIPT.read_text()
        assert 'NetworkManager-dispatcher.service' in text
        assert 'systemctl enable' in text


class TestLaunchWrapper:
    """launch_teleop.sh is the runtime wrapper systemd / racecar-tool calls."""

    WRAPPER = SCRIPTS_DIR / 'launch_teleop.sh'

    def test_creates_log_dir_and_symlink(self):
        # Timestamped session dir; 'latest' is swapped in with a rename.
        text = self.WRAPPER.read_text()
        assert 'mkdir -p "$LOG_DIR"' in text
        assert 'ln -sfn "$LOG_DIR" "$HOME/logs/latest.tmp"' in text
        assert 'mv -Tf "$HOME/logs/latest.tmp" "$HOME/logs/latest"' in text

    def test_log_mirror_starts_before_any_output(self):
        # Every echo, the SHM sweep included, must reach teleop.log.
        text = self.WRAPPER.read_text()
        tee = text.index('exec &> >(tee -a "$LOG_DIR/teleop.log")')
        assert tee < text.index('echo ')

    def test_sweeps_fastrtps_shm_orphans(self):
        text = self.WRAPPER.read_text()
        assert '/dev/shm/fastrtps_port' in text

    def test_execs_ros2_launch(self):
        # The final `exec ros2 launch` is what lets systemd track the launch PID.
        text = self.WRAPPER.read_text()
        assert 'exec ros2 launch racecar_neo_ros2_driver teleop.launch.py' in text


class TestSystemdServices:
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
        # BindsTo means watchdog stops when teleop stops.
        text = (SCRIPTS_DIR / 'racecar-watchdog.service').read_text()
        assert 'BindsTo=racecar-teleop.service' in text
        assert 'After=racecar-teleop.service' in text

    def test_teleop_wants_watchdog(self):
        # Starting teleop, by hand or at boot, pulls the watchdog along.
        text = (SCRIPTS_DIR / 'racecar-teleop.service').read_text()
        assert 'Wants=racecar-watchdog.service' in text

    def test_teleop_calls_launch_wrapper(self):
        text = (SCRIPTS_DIR / 'racecar-teleop.service').read_text()
        assert 'launch_teleop.sh' in text

    def test_watchdog_invokes_watchdog_py(self):
        text = (SCRIPTS_DIR / 'racecar-watchdog.service').read_text()
        assert 'watchdog.py' in text


class TestNetworkPolkitRule:
    """The polkit rule that lets `racecar wifi connect` work over SSH."""

    RULE_FILE = SCRIPTS_DIR / 'polkit' / '49-racecar-network.rules'
    INSTALLER = SCRIPTS_DIR / 'setup_user_env.sh'

    @pytest.fixture
    def text(self):
        return self.RULE_FILE.read_text()

    def test_rule_file_exists(self):
        assert self.RULE_FILE.is_file(), f'{self.RULE_FILE} missing'

    def test_sorts_ahead_of_the_polkit_default(self):
        # 50-default.rules answers "auth" for network-control. A rule that
        # sorts after it never gets asked.
        prefix = int(self.RULE_FILE.name.split('-')[0])
        assert prefix < 50, 'rule must sort before polkit 50-default.rules'

    @pytest.mark.parametrize(
        'action_id',
        [
            'org.freedesktop.NetworkManager.network-control',
            'org.freedesktop.NetworkManager.enable-disable-wifi',
            'org.freedesktop.NetworkManager.wifi.scan',
            'org.freedesktop.NetworkManager.settings.modify.own',
            'org.freedesktop.NetworkManager.settings.modify.system',
        ],
    )
    def test_grants_the_actions_the_wifi_command_needs(self, text, action_id):
        assert f'"{action_id}"' in text, f'{action_id} not granted'

    def test_grant_is_scoped_to_a_group_and_an_action_list(self, text):
        # An unconditional YES would hand every polkit action on the car to
        # anyone, NetworkManager or not.
        assert 'subject.isInGroup("sudo")' in text
        assert 'indexOf(action.id)' in text

    def test_installed_by_setup_user_env(self):
        text = self.INSTALLER.read_text()
        assert 'polkit/49-racecar-network.rules' in text
        assert '/etc/polkit-1/rules.d/49-racecar-network.rules' in text


class TestUdevRules:
    RULES_FILE = SCRIPTS_DIR / 'udev' / '99-racecar.rules'

    def test_rules_file_exists(self):
        assert self.RULES_FILE.is_file(), f'{self.RULES_FILE} missing'

    @pytest.mark.parametrize(
        'symlink',
        [
            'neo-pit-pcb',
            'lidar',
        ],
    )
    def test_rules_define_symlink(self, symlink):
        text = self.RULES_FILE.read_text()
        assert f'SYMLINK+="{symlink}"' in text, f'No rule defines /dev/{symlink}'

    @pytest.mark.parametrize(
        'vid_pid',
        [
            ('10c4', 'ea60'),  # CP2102 (RPLIDAR)
            ('1a6e', '089a'),  # Coral pre-init
            ('18d1', '9302'),  # Coral post-init
        ],
    )
    def test_rules_match_known_usb_ids(self, vid_pid):
        vid, pid = vid_pid
        text = self.RULES_FILE.read_text()
        assert f'ATTRS{{idVendor}}=="{vid}"' in text, f'VID {vid} not matched'
        assert f'ATTRS{{idProduct}}=="{pid}"' in text, f'PID {pid} not matched'

    def test_alfa_ap_dongle_renamed_to_wlan1(self):
        # A stable name for setup_networking.sh instead of the MAC-derived wlx<mac>.
        text = self.RULES_FILE.read_text()
        alfa = [
            ln
            for ln in text.splitlines()
            if 'ATTRS{idVendor}=="0e8d"' in ln and 'ATTRS{idProduct}=="7612"' in ln
        ]
        assert alfa, 'no rule matches the ALFA MT7612U (0e8d:7612)'
        assert any(
            'NAME="wlan1"' in ln for ln in alfa
        ), 'ALFA rule must rename the dongle to wlan1'

    def test_realsense_autosuspend_rule_present(self):
        # The autosuspend rule matches the usb device itself: ATTR, not ATTRS.
        text = self.RULES_FILE.read_text()
        assert 'ATTR{idVendor}=="8086"' in text, 'RealSense VID not matched'
        assert 'ATTR{idProduct}=="0b3a"' in text, 'RealSense PID not matched'

    def test_lidar_rule_ignores_modemmanager(self):
        # A ModemManager probe desyncs the sllidar frame reader and /scan goes silent.
        text = self.RULES_FILE.read_text()
        lidar_lines = [ln for ln in text.splitlines() if 'SYMLINK+="lidar"' in ln]
        assert lidar_lines, 'lidar rule missing'
        assert any(
            'ID_MM_DEVICE_IGNORE' in ln for ln in lidar_lines
        ), 'lidar rule must set ID_MM_DEVICE_IGNORE=1 to block ModemManager probes'

    def test_neo_pit_rule_matches_gpio_uart(self):
        # GPIO UART is ttyAMA0 on Pi 5 / Ubuntu; ttyAMA10 is the SoC debug UART.
        text = self.RULES_FILE.read_text()
        assert 'KERNEL=="ttyAMA0"' in text, 'neo-pit-pcb rule must match ttyAMA0'
        assert 'SYMLINK+="neo-pit-pcb"' in text, 'neo-pit-pcb symlink rule missing'


class TestHidNintendoBlacklist:
    """Kernel blacklist; see docs/troubleshooting.md, "Gamepad hid_nintendo blacklist"."""

    CONF = SCRIPTS_DIR / 'modprobe.d' / 'blacklist-hid-nintendo.conf'

    def test_blacklist_file_exists(self):
        assert self.CONF.is_file()

    def test_blacklists_hid_nintendo(self):
        # The underscore form matches lsmod.
        text = self.CONF.read_text()
        assert 'blacklist hid_nintendo' in text

    def test_setup_udev_installs_blacklist(self):
        # hid_nintendo can load from the initramfs before /etc/modprobe.d is
        # read, so the initramfs is rebuilt too; the running module is unloaded
        # so the change applies this boot.
        text = (SCRIPTS_DIR / 'setup_udev.sh').read_text()
        assert 'blacklist-hid-nintendo.conf' in text
        assert '/etc/modprobe.d/' in text
        assert 'update-initramfs' in text
        assert 'modprobe -r hid_nintendo' in text


class TestEthScript:
    """setup_eth.sh: eth0 in exactly one IPv4 addressing mode."""

    SCRIPT = SCRIPTS_DIR / 'setup_eth.sh'

    def _run(self, tmp_path, *args):
        """Run the script in dry-run mode, isolated from the real system files."""
        env = {k: v for k, v in os.environ.items() if not k.startswith('RACECAR_ETH')}
        env.update(
            {
                'RACECAR_ETH_DRY_RUN': '1',
                'RACECAR_ETH_NETPLAN': str(tmp_path / '99-racecar-eth0.yaml'),
                'RACECAR_ETH_CONFIG': str(tmp_path / 'networking.conf'),
            }
        )
        return subprocess.run(
            ['bash', str(self.SCRIPT), *args],
            capture_output=True,
            text=True,
            timeout=20,
            env=env,
        )

    def _render(self, tmp_path, mode, *extra):
        result = self._run(tmp_path, mode, *extra)
        assert result.returncode == 0, result.stderr
        return (tmp_path / '99-racecar-eth0.yaml').read_text()

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
        # eth0 never carries a static address and a DHCP lease together.
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
        # All three publish /drive; enabling them all would put three
        # publishers on the mux at boot.
        assert 'systemctl enable' not in text
        assert 'systemctl start' not in text

    def test_renders_jazzy_not_humble(self, text):
        assert '/opt/ros/humble|/opt/ros/jazzy' in text

    def test_injects_the_discovery_scope(self, text):
        assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in text

    def test_renames_units_to_the_racecar_prefix(self, text):
        # Either project's prefix comes off, so a fork and a fork synced from
        # Neobotics upstream land on the same racecar-<name>.service.
        assert 'unit_name()' in text
        assert 'base="${base#neoracer-}"' in text
        assert 'base="${base#racecar-}"' in text

    def test_verifies_every_substitution_token(self, text):
        # A template that stops carrying a token has changed shape; installing
        # the result anyway would point the unit at the wrong directory.
        for token in ('@DIR@', 'Environment=HOME='):
            assert token in text
        # The ROS overlay may read humble (upstream) or jazzy (the forks), but
        # one of them has to be there to rewrite.
        assert '/opt/ros/(humble|jazzy)' in text

    def test_clone_failure_is_not_fatal(self, text):
        assert 'fetch_repo "$repo" || true' in text

    def test_units_only_skips_the_network(self, text):
        assert '--units-only' in text
        assert 'MODE="units"' in text

    def test_env_var_skips_the_phase(self, text):
        assert 'RACECAR_DASHBOARDS' in text

    def test_the_three_forked_repositories(self, text):
        for repo in ('teleop', 'linefollow', 'wallfollow'):
            assert f'{repo}_dashboard' in text
        for gone in ('camlabel', 'pursuit', 'eps', 'smartfollow'):
            assert f'{gone}_dashboard' not in text

    def test_clones_the_platform_branch(self, text):
        # The forks' default branch is the Neobotics original: upstream
        # ports, neoracer unit names, Humble paths, forward-facing lidar.
        # Cloning it would install the wrong dashboards on a fresh car.
        assert 'BRANCH="${RACECAR_DASHBOARD_BRANCH:-racecar-neo}"' in text
        assert '--branch "$BRANCH"' in text

    def test_pull_names_the_remote_branch(self, text):
        # A checkout made before the clone set tracking has no upstream, so
        # a bare `git pull` fails with 'no tracking information'.
        assert 'pull --ff-only --quiet origin "$BRANCH"' in text

    def test_a_checkout_on_another_branch_is_left_alone(self, text):
        assert "not '$BRANCH'; left alone" in text

    def test_pins_the_dashboard_version(self, text):
        # The dashboards track the driver's release rather than a count of
        # their own, the same way the RealSense firmware target is pinned.
        assert 'DASHBOARD_VERSION="${RACECAR_DASHBOARD_VERSION:-' in text

    def test_the_pin_matches_the_driver_version(self, text):
        pinned = re.search(r'DASHBOARD_VERSION="\$\{RACECAR_DASHBOARD_VERSION:-([^}]+)\}"', text)
        assert pinned, 'no pinned dashboard version'
        setup_py = (SCRIPTS_DIR.parent / 'setup.py').read_text()
        driver = re.search(r"version='([^']+)'", setup_py)
        assert driver, 'no version in setup.py'
        assert pinned.group(1) == driver.group(1)

    def test_a_version_mismatch_is_not_fatal(self, text):
        # A car mid-upgrade should still end up with working units; which
        # release to run is the operator's call, not the script's.
        assert 'mismatched+=' in text
        assert 'exit 1' not in text.split('check_version()')[1].split('}')[0]

    def test_a_failed_install_is_not_reported_as_success(self, text):
        # install_unit runs under `|| true`, which suppresses errexit for its
        # whole body, so the install has to be tested explicitly.
        assert 'elif $SUDO install -m 0644' in text
        assert 'install failed' in text

    def test_a_failed_daemon_reload_still_reaches_the_summary(self, text):
        assert 'if $SYSTEMCTL daemon-reload; then' in text

    def _check_version(self, tmp_path, contents, pinned='0.8.1'):
        """Run the shipped check_version() against a throwaway checkout."""
        text = self.SCRIPT.read_text()
        body = text.split('check_version() {', 1)[1].split('\n}\n', 1)[0]
        repo = tmp_path / 'teleop_dashboard'
        repo.mkdir()
        if contents is not None:
            (repo / 'VERSION').write_text(contents)
        script = (
            f'DASH_DIR={tmp_path}\n'
            f'DASHBOARD_VERSION={pinned}\n'
            'mismatched=()\n'
            'check_version() {' + body + '\n}\n'
            'check_version teleop_dashboard\n'
            'printf "MISMATCHED:%s\\n" "${mismatched[*]}"\n'
        )
        return subprocess.run(['bash', '-c', script], capture_output=True, text=True)

    def test_a_matching_checkout_passes(self, tmp_path):
        r = self._check_version(tmp_path, '0.8.1\n')
        assert 'teleop_dashboard: 0.8.1' in r.stdout
        assert 'MISMATCHED:\n' in r.stdout

    def test_a_stale_checkout_is_named(self, tmp_path):
        r = self._check_version(tmp_path, '0.8.0\n')
        assert 'driver pins 0.8.1' in r.stderr
        assert '0.8.0' in r.stdout.split('MISMATCHED:')[1]

    def test_a_checkout_with_no_version_is_caught(self, tmp_path):
        # A checkout with no VERSION file must not pass.
        r = self._check_version(tmp_path, None)
        assert 'no VERSION' in r.stderr
        assert 'no VERSION' in r.stdout.split('MISMATCHED:')[1]

    def test_trailing_whitespace_in_version_is_tolerated(self, tmp_path):
        r = self._check_version(tmp_path, '  0.8.1  \n\n')
        assert 'MISMATCHED:\n' in r.stdout

    def test_clones_from_the_racecar_org(self, text):
        # The forks carry this platform's lidar convention, ports and
        # branding; the Neobotics originals do not.
        assert 'github.com/MITRacecarNeo' in text
        assert 'Neobotics-Foundation-Inc' not in text

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

    def _retired(self, tmp_path, installed):
        """Run the shipped remove_retired_unit() against a throwaway unit dir."""
        text = self.SCRIPT.read_text()
        body = text.split('remove_retired_unit() {', 1)[1].split('\n}\n', 1)[0]
        retired = text.split('RETIRED_UNITS=(', 1)[1].split(')', 1)[0].split()
        units = tmp_path / 'units'
        units.mkdir()
        for name in installed:
            (units / f'{name}.service').write_text('[Unit]\n')
        log = tmp_path / 'systemctl.log'
        stub = tmp_path / 'systemctl'
        stub.write_text(f'#!/bin/bash\necho "$*" >> "{log}"\n')
        stub.chmod(0o755)
        script = (
            f'SYSTEMD_DIR={units}\n'
            'SUDO=\n'
            f'SYSTEMCTL={stub}\n'
            'changed=0\n'
            'failed=()\n'
            'remove_retired_unit() {' + body + '\n}\n'
            f'for unit in {" ".join(retired)}; do remove_retired_unit "$unit"; done\n'
            'printf "CHANGED:%s\\n" "$changed"\n'
        )
        result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        calls = log.read_text().splitlines() if log.exists() else []
        return result, units, calls, retired

    def test_retired_dashboards_are_listed(self, text):
        retired = text.split('RETIRED_UNITS=(', 1)[1].split(')', 1)[0].split()
        assert sorted(retired) == [
            'racecar-camlabel',
            'racecar-eps',
            'racecar-pursuit',
            'racecar-smartfollow',
        ]

    def test_retired_units_are_stopped_disabled_and_removed(self, tmp_path):
        result, units, calls, _ = self._retired(
            tmp_path, ['racecar-camlabel', 'racecar-eps', 'racecar-webteleop']
        )
        assert result.returncode == 0, result.stderr
        assert not (units / 'racecar-camlabel.service').exists()
        assert not (units / 'racecar-eps.service').exists()
        assert (units / 'racecar-webteleop.service').exists(), 'a live dashboard was removed'
        for unit in ('racecar-camlabel.service', 'racecar-eps.service'):
            assert f'stop {unit}' in calls
            assert f'disable {unit}' in calls
        assert 'CHANGED:1' in result.stdout

    def test_nothing_retired_means_no_change(self, tmp_path):
        result, _, calls, _ = self._retired(tmp_path, ['racecar-webteleop'])
        assert result.returncode == 0, result.stderr
        assert calls == []
        assert 'CHANGED:0' in result.stdout

    def test_removal_is_followed_by_daemon_reload(self, text):
        removal = text.index('remove_retired_unit "$unit"')
        reload_block = text.index('if [[ $changed -eq 1 ]]; then')
        assert removal < reload_block


class TestRaspiConfig:
    """setup_raspi_config.sh: config.txt edits for the UART and the RTC cell."""

    SCRIPT = SCRIPTS_DIR / 'setup_raspi_config.sh'

    FIXTURE = """# Ubuntu Pi config.txt
[pi4]
max_framebuffers=2
enable_uart=1

[all]
kernel=vmlinuz
enable_uart=0
dtparam=i2c_arm=on
enable_uart=1
dtparam=rtc_bbat_vchg=3000000

[cm4]
otg_mode=1
"""

    def _call(self, tmp_path, func, config, **env):
        """Run one shipped function from the script against a temp config.txt."""
        text = self.SCRIPT.read_text()
        body = text.split(f'{func}() {{', 1)[1].split('\n}\n', 1)[0]
        cfg = tmp_path / 'config.txt'
        if not cfg.exists():
            cfg.write_text(config)
        assigns = ''.join(f'{k}={v}\n' for k, v in env.items())
        script = f'CONFIG_TXT={cfg}\nSUDO=\n{assigns}{func}() {{{body}\n}}\n{func}\n'
        result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        return cfg.read_text()

    @staticmethod
    def _all_scope(text):
        """Lines in the [all] scope: before any header, or under [all]."""
        in_all = True
        lines = []
        for line in text.splitlines():
            if line.startswith('['):
                in_all = line.startswith('[all]')
                continue
            if in_all:
                lines.append(line.strip())
        return lines

    def test_exactly_one_enable_uart_in_all(self, tmp_path):
        out = self._call(tmp_path, 'ensure_enable_uart', self.FIXTURE)
        uart = [ln for ln in self._all_scope(out) if ln.startswith('enable_uart=')]
        assert uart == ['enable_uart=1']

    def test_other_sections_are_untouched(self, tmp_path):
        out = self._call(tmp_path, 'ensure_enable_uart', self.FIXTURE)
        pi4 = out.split('[pi4]')[1].split('[all]')[0]
        assert 'enable_uart=1' in pi4
        assert 'otg_mode=1' in out
        assert 'dtparam=i2c_arm=on' in out

    def test_enable_uart_is_idempotent(self, tmp_path):
        first = self._call(tmp_path, 'ensure_enable_uart', self.FIXTURE)
        second = self._call(tmp_path, 'ensure_enable_uart', first)
        assert first == second

    def test_enable_uart_added_when_missing(self, tmp_path):
        out = self._call(tmp_path, 'ensure_enable_uart', '[all]\nkernel=vmlinuz\n')
        assert self._all_scope(out) == ['kernel=vmlinuz', 'enable_uart=1']

    def test_header_names_the_neo_pit_link(self):
        head = self.SCRIPT.read_text().split('set -eo pipefail')[0]
        assert 'NEO-PIT' in head
        assert 'future modules' not in head

    def test_rtc_zero_turns_charging_off(self, tmp_path):
        out = self._call(tmp_path, 'apply_rtc_charge', self.FIXTURE, RTC_VCHG_UV='0')
        assert 'rtc_bbat_vchg' not in out

    def test_rtc_zero_without_a_line_is_a_no_op(self, tmp_path):
        cfg = '[all]\nkernel=vmlinuz\n'
        assert self._call(tmp_path, 'apply_rtc_charge', cfg, RTC_VCHG_UV='0') == cfg

    def test_rtc_value_is_updated_in_place(self, tmp_path):
        out = self._call(tmp_path, 'apply_rtc_charge', self.FIXTURE, RTC_VCHG_UV='2900000')
        assert out.count('dtparam=rtc_bbat_vchg=') == 1
        assert 'dtparam=rtc_bbat_vchg=2900000' in out

    def test_rtc_value_is_added_when_missing(self, tmp_path):
        out = self._call(
            tmp_path, 'apply_rtc_charge', '[all]\nkernel=vmlinuz\n', RTC_VCHG_UV='3000000'
        )
        assert out.endswith('dtparam=rtc_bbat_vchg=3000000\n')


class TestLinterInstall:
    """setup_dev_tools.sh pins the linters behind `racecar lint`."""

    def test_linters_are_pinned(self):
        text = (SCRIPTS_DIR / 'setup_dev_tools.sh').read_text()
        for pin in ('ruff==0.16.8', 'black==26.5.1', 'mypy==2.3.1'):
            assert pin in text
        assert 'pip3 install --user --break-system-packages' in text

    def test_user_bin_is_on_path(self):
        # pip --user puts ruff/black/mypy in ~/.local/bin; ~/.profile adds it
        # only for login shells.
        text = (SCRIPTS_DIR / 'setup_user_env.sh').read_text()
        assert 'export PATH="$HOME/.local/bin:$PATH"' in text

    def test_path_block_is_a_single_line(self):
        # replace_block deletes from the marker to the next blank line.
        text = (SCRIPTS_DIR / 'setup_user_env.sh').read_text()
        block = text.split('replace_block "$PATH_MARKER" <<\'EOF\'\n', 1)[1].split('\nEOF\n')[0]
        assert block.strip() and '\n\n' not in block
