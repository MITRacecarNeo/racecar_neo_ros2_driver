"""
Static and dry-run checks for the files under scripts/.

Covers the setup and runtime shell scripts, the Python entry points, and the
systemd, udev, polkit and modprobe files they install.
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

# Scripts that ship with the package but are NOT called by setup_all.sh;
# the user runs them manually (or via `racecar setup <phase>`) because their
# side-effects are too disruptive to include in a one-shot install.
STANDALONE_SCRIPTS = [
    'setup_networking.sh',  # reconfigures wlan0; can drop SSH-over-WiFi sessions
    'setup_eth.sh',  # switches eth0 addressing mode; drops SSH sessions on eth0
    'flash_realsense_offline.sh',  # per-machine camera firmware flash (airgapped)
]

RUNTIME_SCRIPTS = ['launch_teleop.sh', 'launch_autonomy.sh']

ALL_SCRIPTS = PHASE_SCRIPTS + [ORCHESTRATOR] + STANDALONE_SCRIPTS + RUNTIME_SCRIPTS

# Relative to scripts/, so the calibration/ tools are covered too.
PY_SCRIPTS = sorted(
    str(p.relative_to(SCRIPTS_DIR))
    for p in [*SCRIPTS_DIR.glob('*.py'), *SCRIPTS_DIR.glob('calibration/*.py')]
)
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
        'racecar-data.service',
        'racecar-teleop.service',
        'racecar-watchdog.service',
        'racecar-dashboard.service',
        'racecar-jupyter.service',
        'racecar-mapview.service',
        'racecar-battery-guard.service',
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

    def test_mapview_independent_of_teleop(self):
        # The page offers "Start mapping" while teleop and autonomy are down.
        text = (SCRIPTS_DIR / 'racecar-mapview.service').read_text()
        assert 'teleop' not in text.split('[Service]')[0]
        assert 'scripts/mapview.py' in text

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
        rules = (
            '49-racecar-network.rules 49-racecar-autonomy.rules '
            '49-racecar-dashboards.rules 49-racecar-teleop.rules 49-racecar-power.rules'
        )
        assert f'for rule in {rules}; do' in text
        assert 'POLKIT_SRC="${SCRIPT_DIR}/polkit/${rule}"' in text
        assert 'POLKIT_DST="/etc/polkit-1/rules.d/${rule}"' in text


def test_battery_guard_independent_of_teleop():
    # The guard must keep watching across teleop restarts and stop teleop itself.
    text = (SCRIPTS_DIR / 'racecar-battery-guard.service').read_text()
    assert 'teleop' not in text.split('[Service]')[0]
    assert 'scripts/battery_guard.py' in text
    assert 'Restart=always' in text


def test_battery_guard_enabled_by_setup_services():
    text = (SCRIPTS_DIR / 'setup_services.sh').read_text()
    services = text.split('SERVICES=(')[1].split(')')[0]
    assert 'racecar-battery-guard.service' in services


class TestPowerPolkitRule:
    """The polkit rule that lets the battery guard stop teleop and power the car off."""

    RULE_FILE = SCRIPTS_DIR / 'polkit' / '49-racecar-power.rules'

    @pytest.fixture
    def text(self):
        return self.RULE_FILE.read_text()

    def test_sorts_ahead_of_the_polkit_default(self):
        assert int(self.RULE_FILE.name.split('-')[0]) < 50

    @pytest.mark.parametrize(
        'action_id',
        [
            'org.freedesktop.login1.power-off',
            'org.freedesktop.login1.power-off-multiple-sessions',
            'org.freedesktop.login1.power-off-ignore-inhibit',
        ],
    )
    def test_grants_power_off(self, text, action_id):
        assert f'"{action_id}"' in text

    def test_scoped_to_racecar_and_teleop_stop(self, text):
        assert 'subject.user !== "racecar"' in text
        assert 'action.lookup("unit") === "racecar-teleop.service"' in text
        assert 'action.lookup("verb") === "stop"' in text
        assert 'reboot' not in text.split('polkit.addRule')[1]


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
    """setup_dashboards.sh installs units for the dashboards shipped in scripts/dashboards/."""

    SCRIPT = SCRIPTS_DIR / 'setup_dashboards.sh'
    DASHBOARDS = {
        'teleop_dashboard': ('racecar-webteleop', '8081'),
        'linefollow_dashboard': ('racecar-linefollow', '8082'),
        'wallfollow_dashboard': ('racecar-wallfollow', '8083'),
        'camlabel_dashboard': ('racecar-camlabel', '8085'),
    }

    @pytest.fixture(scope='class')
    def text(self):
        return self.SCRIPT.read_text()

    def test_never_enables_or_starts(self, text):
        # All three publish /drive; enabling them all would put three
        # publishers on the mux at boot.
        assert 'systemctl enable' not in text
        assert 'systemctl start' not in text

    def test_no_network(self, text):
        assert 'git clone' not in text and 'git -C' not in text

    def test_injects_the_discovery_scope(self, text):
        assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in text

    def test_verifies_every_substitution_token(self, text):
        # A template that stops carrying a token has changed shape; installing
        # the result anyway would point the unit at the wrong directory.
        for token in ('@DIR@', 'Environment=HOME='):
            assert token in text

    def test_env_var_skips_the_phase(self, text):
        assert 'RACECAR_DASHBOARDS' in text

    def test_ships_the_four_dashboards(self, text):
        listed = text.split('DASHBOARDS=(', 1)[1].split(')', 1)[0].split()
        assert listed == list(self.DASHBOARDS)

    @pytest.mark.parametrize(
        'name',
        ['teleop_dashboard', 'linefollow_dashboard', 'wallfollow_dashboard', 'camlabel_dashboard'],
    )
    def test_templates_ship_in_tree(self, name):
        unit, port = self.DASHBOARDS[name]
        body = (SCRIPTS_DIR / 'dashboards' / name / f'{unit}.service.in').read_text()
        assert '/opt/ros/jazzy' in body and '/opt/ros/humble' not in body
        assert f'(port {port})' in body
        assert body.count('@DIR@') == 2

    def test_installs_rendered_units(self, tmp_path, monkeypatch):
        units = tmp_path / 'units'
        units.mkdir()
        monkeypatch.setenv('RACECAR_SYSTEMD_DIR', str(units))
        monkeypatch.setenv('RACECAR_SUDO', '')
        monkeypatch.setenv('RACECAR_SYSTEMCTL', 'true')
        r = subprocess.run(['bash', str(self.SCRIPT)], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        for name, (unit, _) in self.DASHBOARDS.items():
            body = (units / f'{unit}.service').read_text()
            assert f'WorkingDirectory={SCRIPTS_DIR / "dashboards" / name}' in body
            assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in body
        again = subprocess.run(['bash', str(self.SCRIPT)], capture_output=True, text=True)
        assert again.stdout.count('already up to date') == len(self.DASHBOARDS)

    def test_a_failed_install_is_not_reported_as_success(self, text):
        # install_unit runs under `|| true`, which suppresses errexit for its
        # whole body, so the install has to be tested explicitly.
        assert 'elif $SUDO install -m 0644' in text
        assert 'install failed' in text

    def test_a_failed_daemon_reload_still_reaches_the_summary(self, text):
        assert 'if $SYSTEMCTL daemon-reload; then' in text

    def test_gitignore_tracks_the_dashboards(self):
        gitignore = (SCRIPTS_DIR.parent / '.gitignore').read_text().splitlines()
        assert 'scripts/dashboards/' not in gitignore
        for kept_out in ('eps', 'pursuit', 'smartfollow'):
            assert f'scripts/dashboards/{kept_out}_dashboard/' in gitignore
        assert 'scripts/dashboards/*/*.local.yaml' in gitignore

    def test_linters_exclude_only_the_retired_dashboards(self):
        root = SCRIPTS_DIR.parent
        flake8 = (root / 'test' / 'test_flake8.py').read_text()
        pep257 = (root / 'test' / 'test_pep257.py').read_text()
        pyproject = (root / 'pyproject.toml').read_text()
        for retired in ('eps_dashboard', 'pursuit_dashboard', 'smartfollow_dashboard'):
            assert retired in flake8
            assert retired in pep257
            assert pyproject.count(retired) == 3  # black, ruff, mypy
        for shipped in ('teleop', 'linefollow', 'wallfollow', 'camlabel'):
            assert f'{shipped}_dashboard' not in flake8 + pep257 + pyproject

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
            'racecar-eps',
            'racecar-pursuit',
            'racecar-smartfollow',
        ]

    def test_retired_units_are_stopped_disabled_and_removed(self, tmp_path):
        result, units, calls, _ = self._retired(
            tmp_path, ['racecar-pursuit', 'racecar-eps', 'racecar-webteleop']
        )
        assert result.returncode == 0, result.stderr
        assert not (units / 'racecar-pursuit.service').exists()
        assert not (units / 'racecar-eps.service').exists()
        assert (units / 'racecar-webteleop.service').exists(), 'a live dashboard was removed'
        for unit in ('racecar-pursuit.service', 'racecar-eps.service'):
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
        helper = text.split('ensure_all_setting() {', 1)[1].split('\n}\n', 1)[0]
        cfg = tmp_path / 'config.txt'
        if not cfg.exists():
            cfg.write_text(config)
        assigns = ''.join(f'{k}={v}\n' for k, v in env.items())
        script = (
            f'CONFIG_TXT={cfg}\nSUDO=\n{assigns}'
            f'ensure_all_setting() {{{helper}\n}}\n{func}() {{{body}\n}}\n{func}\n'
        )
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

    def test_usb_max_power_and_dtparams_once_in_all(self, tmp_path):
        cfg = tmp_path / 'config.txt'
        cfg.write_text(self.FIXTURE)
        text = self.SCRIPT.read_text()
        helper = text.split('ensure_all_setting() {', 1)[1].split('\n}\n', 1)[0]
        script = (
            f'CONFIG_TXT={cfg}\nSUDO=\nensure_all_setting() {{{helper}\n}}\n'
            'ensure_all_setting usb_max_power_enable 1\n'
            'ensure_all_setting usb_max_power_enable 1\n'
            'ensure_all_setting dtparam=i2c_arm on\n'
        )
        result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        scope = self._all_scope(cfg.read_text())
        assert scope.count('usb_max_power_enable=1') == 1
        assert scope.count('dtparam=i2c_arm=on') == 1

    def test_runs_without_raspi_config(self):
        text = self.SCRIPT.read_text()
        assert 'raspi-config not found; skipping' not in text
        assert 'if command -v raspi-config >/dev/null; then' in text
        assert text.index('ensure_enable_uart\n') > text.index('if command -v raspi-config')

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


def test_terminal_app_packages_are_installed_by_setup():
    # A fresh car must get what scripts/terminal.py runs: ttyd, PAM for Python, tmux.
    text = (SCRIPTS_DIR / 'setup_dev_tools.sh').read_text()
    for package in ('ttyd', 'python3-pampy', 'tmux'):
        assert f' {package}' in text, package


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


class TestOneShotSetup:
    """What a new car needs from setup_all.sh, checked against the scripts' text."""

    def _all_setup_text(self):
        return '\n'.join((SCRIPTS_DIR / s).read_text() for s in [*PHASE_SCRIPTS, ORCHESTRATOR])

    @pytest.mark.parametrize(
        'package',
        [
            'raspi-config',
            'rpi-eeprom',
            'libraspberrypi-bin',
            'network-manager',
            'avahi-daemon',
            'polkitd',
            'python3-matplotlib',
            'ros-jazzy-xacro',
            'ros-jazzy-robot-state-publisher',
            'nvme-cli',
            'gdisk',
            'ros-jazzy-slam-toolbox',
            'ros-jazzy-nav2-amcl',
            'ros-jazzy-foxglove-bridge',
        ],
    )
    def test_runtime_packages_are_installed(self, package):
        assert package in self._all_setup_text(), package

    def test_refuses_the_wrong_user_root_or_folder(self):
        text = (SCRIPTS_DIR / ORCHESTRATOR).read_text()
        guards = text.index('EXPECTED_DIR=')
        assert guards < text.index('sudo -v')
        assert '"$EUID" -eq 0' in text and '"$(id -un)" != "racecar"' in text

    def test_ends_with_a_reboot_and_reports_dashboard_failures(self):
        text = (SCRIPTS_DIR / ORCHESTRATOR).read_text()
        assert 'setup_dashboards.sh" || DASHBOARDS_OK=0' in text
        assert 'sudo reboot' in text.split('setup_dashboards.sh')[-1]
        dash = (SCRIPTS_DIR / 'setup_dashboards.sh').read_text()
        assert 'exit 1' in dash.split('Incomplete:')[1]

    def test_ros_apt_source_lookup_has_a_fallback(self):
        text = (SCRIPTS_DIR / 'setup_ros2.sh').read_text()
        assert 'curl -fsS --retry 3' in text and 'ROS_APT_SOURCE_FALLBACK=' in text
        assert text.index('apt-get update') < text.index('apt-get install')

    def test_network_installs_are_pinned(self):
        assert 'SLLIDAR_COMMIT=' in (SCRIPTS_DIR / 'setup_workspace.sh').read_text()
        assert 'jupyterlab==' in (SCRIPTS_DIR / 'setup_jupyter.sh').read_text()
        assert 'luma.led_matrix==' in (SCRIPTS_DIR / 'setup_dotmatrix.sh').read_text()

    def test_coral_m2_driver_on_every_pi5(self):
        text = (SCRIPTS_DIR / 'setup_coral.sh').read_text()
        assert "grep -qa 'Raspberry Pi 5' /proc/device-tree/model" in text
        assert 'if [ "$IS_PI5" = "1" ] ||' in text

    def test_udev_trigger_is_scoped(self):
        text = (SCRIPTS_DIR / 'setup_udev.sh').read_text()
        assert 'sudo udevadm trigger\n' not in text
        assert '--subsystem-match' in text

    def test_every_unit_in_a_polkit_rule_is_installed(self):
        import re

        rules = '\n'.join(p.read_text() for p in (SCRIPTS_DIR / 'polkit').glob('*.rules'))
        named = set(re.findall(r'racecar-[a-z]+\.service', rules))
        installed = {p.name for p in SCRIPTS_DIR.glob('racecar-*.service')}
        templates = (SCRIPTS_DIR / 'dashboards').glob('*/racecar-*.service.in')
        installed |= {p.name[: -len('.in')] for p in templates}
        assert named and named <= installed, named - installed
        user_env = (SCRIPTS_DIR / 'setup_user_env.sh').read_text()
        for rule in (SCRIPTS_DIR / 'polkit').glob('*.rules'):
            assert rule.name in user_env, rule.name

    def test_every_unit_the_tools_manage_is_installed(self):
        services = (SCRIPTS_DIR / 'setup_services.sh').read_text()
        for unit in (
            'racecar-data.service',
            'racecar-teleop.service',
            'racecar-watchdog.service',
            'racecar-dashboard.service',
            'racecar-jupyter.service',
            'racecar-mapview.service',
            'racecar-autonomy.service',
        ):
            assert unit in services, unit


class TestRerunOnASetUpCar:
    """A second setup_all run, after the car profile moved the per-car files out."""

    def test_build_clears_dangling_links_first(self):
        text = (SCRIPTS_DIR / 'setup_workspace.sh').read_text()
        assert 'find "$tree" -xtype l -print0' in text
        assert text.index('-xtype l') < text.rindex('colcon build')

    def test_local_yaml_is_not_installed(self):
        setup_py = (SCRIPTS_DIR.parent / 'setup.py').read_text()
        assert "if not f.endswith('.local.yaml')" in setup_py

    @pytest.mark.parametrize('script', [*PHASE_SCRIPTS, ORCHESTRATOR])
    def test_no_grep_q_after_a_pipe(self, script):
        # Under pipefail, grep -q exiting at the first match can SIGPIPE the
        # writer and fail the test even though it matched.
        assert '| grep -q' not in (SCRIPTS_DIR / script).read_text()
