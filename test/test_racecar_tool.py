"""Tests for scripts/racecar-tool.sh (the `racecar` shell function)."""

import os
from pathlib import Path
import subprocess

import pytest

TOOL = Path(__file__).parent.parent / 'scripts' / 'racecar-tool.sh'


def _run(*args):
    """Source the tool in a non-interactive bash and invoke `racecar <args>`."""
    script = f'set +u; source "{TOOL}"; racecar {" ".join(args)}'
    return subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=15,
    )


def test_tool_file_exists():
    assert TOOL.is_file()


def test_bash_syntax_clean():
    result = subprocess.run(
        ['bash', '-n', str(TOOL)],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, f'bash -n failed:\n{result.stderr}'


def test_sourcing_defines_racecar_function():
    script = f'source "{TOOL}" && type -t racecar'
    result = subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.strip() == 'function'


@pytest.mark.parametrize('args', [[], ['help'], ['--help'], ['-h']])
def test_help_renders(args):
    result = _run(*args)
    assert result.returncode == 0
    assert 'racecar' in result.stdout
    assert 'Commands' in result.stdout
    expected = ('build', 'test', 'source', 'cd', 'teleop', 'launch',
                'clear', 'udev', 'watchdog', 'service', 'setup', 'library',
                'cleanup', 'status')
    for sub in expected:
        assert sub in result.stdout, f'help missing "{sub}"'


def test_unknown_command_errors():
    result = _run('bogus_subcommand')
    assert result.returncode == 2
    assert 'unknown command' in result.stderr


def test_launch_without_name_errors():
    result = _run('launch')
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_clear_without_target_errors():
    result = _run('clear')
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_clear_rejects_unknown_flag():
    result = _run('clear', '--cosmic-rays')
    assert result.returncode == 2
    assert 'unknown flag' in result.stderr


def test_selftest_is_gone():
    # Removed in v0.7.4. Falls through to the unknown-command branch rather
    # than silently doing nothing.
    result = _run('selftest')
    assert result.returncode == 2
    assert 'unknown command' in result.stderr


def test_cd_changes_pwd_to_package_root():
    # `cd` must run in the user's shell context (no subshell), so a single
    # bash session that sources the tool, runs `racecar cd`, then echoes PWD
    # should print the package root.
    script = (
        f'set +u; source "{TOOL}"; '
        'racecar cd && pwd'
    )
    result = subprocess.run(
        ['bash', '-c', script],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.strip().endswith('racecar_neo_ros2_driver')


def test_status_runs_without_error():
    # status is read-only and idempotent; it should always succeed even with
    # no ros2 daemon / no peripherals.
    result = _run('status')
    assert result.returncode == 0
    assert 'USB peripherals' in result.stdout
    assert 'Stable device symlinks' in result.stdout


class TestService:
    """`racecar service` covers install/start/stop/restart/enable/disable/logs/status."""

    def test_status_action_runs(self):
        # Default action is `status`, which just calls `systemctl is-active`
        # for each unit. No sudo required, no side effects.
        result = _run('service', 'status')
        assert result.returncode == 0
        # status output enumerates each unit name.
        for unit in ('racecar-teleop', 'racecar-watchdog',
                     'racecar-dashboard', 'racecar-jupyter'):
            assert unit in result.stdout, f'status missing {unit}'

    def test_default_action_is_status(self):
        # `racecar service` with no action should fall through to status.
        result = _run('service')
        assert result.returncode == 0
        assert 'racecar-teleop' in result.stdout

    def test_help_action(self):
        result = _run('service', 'help')
        assert result.returncode == 0
        for action in ('install', 'start', 'stop', 'status', 'logs'):
            assert action in result.stdout

    def test_rejects_unknown_action(self):
        result = _run('service', 'flambé')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr


class TestSetup:
    """`racecar setup` dispatches to setup_all.sh / setup_networking.sh."""

    def test_no_phase_errors(self):
        result = _run('setup')
        assert result.returncode == 2
        assert 'phases:' in result.stderr

    def test_unknown_phase_errors(self):
        result = _run('setup', 'whatever')
        assert result.returncode == 2
        assert 'unknown phase' in result.stderr

    def test_networking_help(self):
        result = _run('setup', 'networking', '--help')
        assert result.returncode == 0
        for flag in ('--ssid', '--psk', '--channel', '--ap-addr',
                     '--ap-iface', '--eth-static', '--show', '--reset'):
            assert flag in result.stdout

    def test_networking_unknown_flag_errors(self):
        result = _run('setup', 'networking', '--gloryhole')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_realsense_listed_as_phase(self):
        # Unknown-phase error and no-phase usage should both advertise realsense.
        assert 'realsense' in _run('setup').stderr
        assert 'realsense' in _run('setup', 'nope').stderr

    def test_realsense_help_dispatches_to_flash_script(self):
        # --help hits the flash script's own usage (no hardware needed) and
        # lists its flags. Confirms `racecar setup realsense` wires through.
        result = _run('setup', 'realsense', '--help')
        assert result.returncode == 0
        for flag in ('--check', '--force', '--version', '--serial', '--fw-dir'):
            assert flag in result.stdout

    def test_networking_show_with_no_persisted_file(self, tmp_path, monkeypatch):
        # --show with no $HOME/.config/racecar/networking.env should report
        # "No persisted networking config" and not invoke the script.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking --show'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert 'No persisted networking config' in result.stdout

    def test_networking_reset_removes_persisted_file(self, tmp_path, monkeypatch):
        # Stub the (destructive, sudo-using) networking script so the test only
        # exercises the tool's file-clear + dispatch, never the real teardown.
        monkeypatch.setenv('HOME', str(tmp_path))
        monkeypatch.setenv('RACECAR_NETWORKING_SCRIPT', '/dev/null')
        cfg_dir = tmp_path / '.config' / 'racecar'
        cfg_dir.mkdir(parents=True)
        cfg_file = cfg_dir / 'networking.env'
        cfg_file.write_text('RACECAR_AP_SSID="dummy"\n')
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking --reset'],
            capture_output=True, text=True, timeout=10,
        )
        assert result.returncode == 0
        assert not cfg_file.exists()
        assert 'Cleared' in result.stdout

    def test_networking_reset_disables_ap(self, tmp_path, monkeypatch):
        # --reset must invoke the networking script in reset mode
        # (RACECAR_AP_RESET=1) so it tears down the wlan1 AP.
        monkeypatch.setenv('HOME', str(tmp_path))
        marker = tmp_path / 'reset_marker'
        stub = tmp_path / 'stub.sh'
        stub.write_text(f'#!/bin/bash\necho "reset=${{RACECAR_AP_RESET:-0}}" > "{marker}"\n')
        monkeypatch.setenv('RACECAR_NETWORKING_SCRIPT', str(stub))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking --reset'],
            capture_output=True, text=True, timeout=10,
        )
        assert result.returncode == 0
        assert marker.read_text().strip() == 'reset=1'

    def test_networking_apply_no_prompt_noninteractive(self, tmp_path, monkeypatch):
        # Plain apply with no TTY must not block on the car-ID prompt; it runs
        # the (stubbed) script and returns.
        monkeypatch.setenv('HOME', str(tmp_path))
        marker = tmp_path / 'apply_marker'
        stub = tmp_path / 'stub.sh'
        stub.write_text(f'#!/bin/bash\ntouch "{marker}"\n')
        monkeypatch.setenv('RACECAR_NETWORKING_SCRIPT', str(stub))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; racecar setup networking'],
            capture_output=True, text=True, timeout=10, stdin=subprocess.DEVNULL,
        )
        assert result.returncode == 0
        assert marker.exists()

    def test_networking_flag_persists_when_combined_with_show(self, tmp_path, monkeypatch):
        # Regression: an earlier impl treated --show as a short-circuit BEFORE
        # writing vals[] to the file. The two-pass parse fixes that: --ssid
        # gathered, --show registered as action, persist runs, then --show
        # prints the (now-up-to-date) file.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; '
             'racecar setup networking --ssid=test-ssid --psk=test-pass --show'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        cfg_file = tmp_path / '.config' / 'racecar' / 'networking.env'
        assert cfg_file.exists(), '--show should NOT short-circuit before persistence'
        text = cfg_file.read_text()
        assert 'RACECAR_AP_SSID="test-ssid"' in text
        assert 'RACECAR_AP_PSK="test-pass"' in text
        # And --show output should reflect what was just written.
        assert 'Persisted networking config' in result.stdout
        assert 'test-ssid' in result.stdout

    def test_networking_reset_with_overrides_errors(self, tmp_path, monkeypatch):
        # --reset + --ssid=foo is nonsense; the new value would be lost
        # immediately. Reject rather than do something surprising.
        monkeypatch.setenv('HOME', str(tmp_path))
        result = subprocess.run(
            ['bash', '-c',
             f'set +u; source "{TOOL}"; '
             'racecar setup networking --ssid=foo --reset'],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 2
        assert 'cannot be combined' in result.stderr


class TestLibrary:
    """`racecar library` manages racecar_student.pth in user site-packages."""

    @staticmethod
    def _run_isolated(home, *args):
        # Override HOME so site.getusersitepackages() resolves to a tmp dir
        # and ~/jupyter_ws probes a tmp tree. PYTHONUSERBASE pins the user-site
        # path under HOME on systems where it would otherwise resolve elsewhere.
        env_setup = (
            f'export HOME="{home}"; '
            f'export PYTHONUSERBASE="{home}/.local"; '
        )
        return subprocess.run(
            ['bash', '-c',
             f'set +u; {env_setup} source "{TOOL}"; '
             f'racecar library {" ".join(args)}'],
            capture_output=True, text=True, timeout=10,
        )

    def test_no_action_errors(self):
        result = _run('library')
        assert result.returncode == 2
        assert 'usage:' in result.stderr

    def test_help_lists_actions(self):
        result = _run('library', '--help')
        assert result.returncode == 0
        for flag in ('--select', '--list', '--reset', '--status'):
            assert flag in result.stdout

    def test_unknown_flag_errors(self):
        result = _run('library', '--vaporize')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_status_with_no_pth(self, tmp_path):
        # Fresh HOME → no .pth file → friendly hint, exit 0.
        result = self._run_isolated(tmp_path, '--status')
        assert result.returncode == 0
        assert 'No racecar library is currently selected' in result.stdout
        assert '--select' in result.stdout

    def test_list_with_no_jupyter_ws(self, tmp_path):
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        assert 'No ~/jupyter_ws/ directory' in result.stdout

    def test_list_skips_folders_without_racecar_core(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        # Valid candidate
        (jws / 'goodlib' / 'library').mkdir(parents=True)
        (jws / 'goodlib' / 'library' / 'racecar_core.py').write_text('')
        # Bogus: no library/ at all
        (jws / 'badlib').mkdir(parents=True)
        # Bogus: library/ exists but no racecar_core.py
        (jws / 'emptylib' / 'library').mkdir(parents=True)
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        assert 'goodlib' in result.stdout
        assert 'badlib' not in result.stdout
        assert 'emptylib' not in result.stdout

    def test_select_writes_pth_file(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        result = self._run_isolated(tmp_path, '--select', 'mylib')
        assert result.returncode == 0, result.stderr
        assert 'Selected library' in result.stdout
        # The .pth file should land somewhere under HOME/.local and contain libdir.
        pth_files = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_files) == 1, f'expected one .pth, found {pth_files}'
        assert pth_files[0].read_text().strip() == str(libdir)

    def test_select_with_equals_form(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        result = self._run_isolated(tmp_path, '--select=mylib')
        assert result.returncode == 0, result.stderr
        pth_files = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_files) == 1
        assert pth_files[0].read_text().strip() == str(libdir)

    def test_select_rejects_missing_folder(self, tmp_path):
        (tmp_path / 'jupyter_ws').mkdir()
        result = self._run_isolated(tmp_path, '--select', 'ghost')
        assert result.returncode == 2
        assert 'not a folder' in result.stderr

    def test_select_rejects_folder_without_racecar_core(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        (jws / 'shell' / 'library').mkdir(parents=True)
        # Note: no racecar_core.py
        result = self._run_isolated(tmp_path, '--select', 'shell')
        assert result.returncode == 2
        assert 'racecar_core.py' in result.stderr

    def test_select_requires_target(self):
        # `--select` with no following arg.
        result = _run('library', '--select')
        assert result.returncode == 2
        assert 'requires a folder name' in result.stderr

    def test_reset_removes_pth(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        # Select, then reset.
        self._run_isolated(tmp_path, '--select', 'mylib')
        pth_before = list(tmp_path.rglob('racecar_student.pth'))
        assert len(pth_before) == 1
        result = self._run_isolated(tmp_path, '--reset')
        assert result.returncode == 0
        assert 'removed' in result.stdout
        pth_after = list(tmp_path.rglob('racecar_student.pth'))
        assert pth_after == []

    def test_reset_when_nothing_to_remove(self, tmp_path):
        result = self._run_isolated(tmp_path, '--reset')
        assert result.returncode == 0
        assert 'no .pth file to remove' in result.stdout

    def test_status_after_select_reports_path(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        libdir = jws / 'mylib' / 'library'
        libdir.mkdir(parents=True)
        (libdir / 'racecar_core.py').write_text('')
        self._run_isolated(tmp_path, '--select', 'mylib')
        result = self._run_isolated(tmp_path, '--status')
        assert result.returncode == 0
        assert 'Current library' in result.stdout
        assert str(libdir) in result.stdout

    def test_list_marks_current_with_asterisk(self, tmp_path):
        jws = tmp_path / 'jupyter_ws'
        for name in ('alpha', 'beta'):
            libdir = jws / name / 'library'
            libdir.mkdir(parents=True)
            (libdir / 'racecar_core.py').write_text('')
        self._run_isolated(tmp_path, '--select', 'beta')
        result = self._run_isolated(tmp_path, '--list')
        assert result.returncode == 0
        # Find the line for beta and check it has a '*' marker.
        lines = [ln for ln in result.stdout.splitlines() if 'beta' in ln]
        assert lines, 'beta missing from --list output'
        assert '*' in lines[0]
        # alpha line should NOT have a star (just leading whitespace).
        alpha_lines = [ln for ln in result.stdout.splitlines()
                       if 'alpha' in ln]
        assert alpha_lines
        assert '*' not in alpha_lines[0]


class TestCleanup:
    def test_dry_run_default_is_safe(self):
        # Dry-run default: must always exit 0 and never invoke kill/rm.
        result = _run('cleanup')
        assert result.returncode == 0
        # Either the process inventory or the SHM section should appear; both
        # have predictable headings or 'No ...' fallback.
        assert 'racecar processes' in result.stdout.lower() or \
               'no racecar processes' in result.stdout.lower()
        assert 'fastrtps shm' in result.stdout.lower() or \
               'no fastrtps' in result.stdout.lower()

    def test_dry_run_marker_appears_when_things_found(self):
        # If the test environment has any racecar process or SHM orphan, the
        # output should label the action as dry-run (i.e. nothing was killed).
        # If nothing is found, the "No ..." messages stand alone — both fine.
        result = _run('cleanup')
        assert result.returncode == 0
        # The "(dry-run; pass --force to ...)" hint appears once per category
        # that found matches. We don't assert it must appear (clean system),
        # but if anything appeared, --force must not have been silently invoked.
        if 'pid=' in result.stdout:
            assert '(dry-run' in result.stdout

    def test_help_flag_describes_behavior(self):
        result = _run('cleanup', '--help')
        assert result.returncode == 0
        assert 'dry-run' in result.stdout
        assert '--force' in result.stdout

    def test_rejects_unknown_flag(self):
        result = _run('cleanup', '--burn-it-all')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr


class TestCompletionInstalled:
    def test_completion_function_defined(self):
        script = f'source "{TOOL}" && type -t _racecar_complete'
        result = subprocess.run(
            ['bash', '-c', script],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert result.stdout.strip() == 'function'

    def test_complete_command_registered(self):
        script = f'source "{TOOL}" && complete -p racecar'
        result = subprocess.run(
            ['bash', '-c', script],
            capture_output=True, text=True, timeout=5,
        )
        assert result.returncode == 0
        assert '_racecar_complete' in result.stdout


class TestWifiCommand:
    """`racecar wifi`: client radio on wlan0; the AP on wlan1 stays untouched."""

    STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$NMCLI_LOG"
case "$*" in
    "radio wifi")                         echo "${STUB_RADIO:-enabled}" ;;
    "-t -f DEVICE device status")         echo "${STUB_DEVICES:-wlan0}" ;;
    "-t -f NAME con show")                echo "${STUB_SAVED:-}" ;;
    "-t -f NAME,TYPE con show") echo "${STUB_SAVED:+$STUB_SAVED:802-11-wireless}" ;;
    *"device wifi list ifname wlan0"*) printf 'EntNet:WPA2 802.1X\\nPskNet:WPA2\\nOpenNet:\\n' ;;
    *) : ;;
esac
"""

    def _wifi(self, tmp_path, *args, stdin=None, **env_extra):
        stub = tmp_path / 'nmcli'
        stub.write_text(self.STUB)
        stub.chmod(0o755)
        log = tmp_path / 'nmcli.log'
        log.touch()
        env = dict(os.environ)
        env.update({'NMCLI_LOG': str(log), 'RACECAR_NMCLI': str(stub)})
        env.update(env_extra)
        script = f'set +u; source "{TOOL}"; racecar wifi {" ".join(args)}'
        result = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True,
            timeout=20, env=env, input=stdin,
        )
        return result, log.read_text()

    def test_help_lists_actions(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'help')
        assert result.returncode == 0
        for action in ('status', 'list', 'connect', 'disconnect'):
            assert action in result.stdout

    def test_rejects_unknown_action(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'bogus')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr

    def test_connect_without_ssid_errors(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'connect')
        assert result.returncode == 2
        assert 'usage:' in result.stderr

    def test_reports_disabled_radio(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'list', STUB_RADIO='disabled')
        assert result.returncode == 3
        assert 'rfkill' in result.stderr

    def test_reports_missing_interface(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'list', STUB_DEVICES='eth0')
        assert result.returncode == 3
        assert 'not present' in result.stderr

    def test_list_is_scoped_to_wlan0(self, tmp_path):
        _, log = self._wifi(tmp_path, 'list')
        scan = [ln for ln in log.splitlines() if 'device wifi list' in ln]
        assert scan, 'no scan issued'
        assert all('ifname wlan0' in ln for ln in scan)

    def test_disconnect_targets_the_device_not_a_connection(self, tmp_path):
        # `nmcli connection down` could match the AP profile by name and drop
        # the link the operator is using. Disconnect must address the device.
        result, log = self._wifi(tmp_path, 'disconnect')
        assert result.returncode == 0, result.stderr
        assert 'device disconnect wlan0' in log
        assert 'connection down' not in log

    def test_saved_profile_brought_up_on_wlan0(self, tmp_path):
        _, log = self._wifi(tmp_path, 'connect', 'HomeNet', STUB_SAVED='HomeNet')
        assert 'connection up HomeNet ifname wlan0' in log

    def test_every_nmcli_call_that_names_an_interface_names_wlan0(self, tmp_path):
        # The guard for the whole command: wlan1 carries the AP, so it must
        # never appear in anything this command generates.
        for args in (['list'], ['status'], ['disconnect'],
                     ['connect', 'HomeNet']):
            _, log = self._wifi(tmp_path, *args, STUB_SAVED='HomeNet')
            assert 'wlan1' not in log, f'`racecar wifi {args[0]}` touched wlan1'

    def test_enterprise_requires_a_verifiable_server(self, tmp_path):
        # An identity with no realm gives nothing to match the RADIUS server
        # against. Refuse rather than hand credentials to any access point
        # broadcasting the SSID.
        result, log = self._wifi(
            tmp_path, 'connect', 'EntNet', '--identity=plainuser', stdin='pw\n')
        assert result.returncode == 2
        assert 'domain-suffix-match' in result.stderr
        assert 'connection add' not in log, 'must not create an unverified profile'

    def test_enterprise_profile_always_validates_the_server(self, tmp_path):
        _, log = self._wifi(
            tmp_path, 'connect', 'EntNet', '--identity=someone@school.edu',
            stdin='hunter2\n')
        add = next((ln for ln in log.splitlines() if 'connection add' in ln), '')
        assert add, 'no profile created'
        assert '802-1x.domain-suffix-match school.edu' in add
        assert '802-1x.system-ca-certs yes' in add
        assert 'ifname wlan0' in add

    def test_enterprise_ca_cert_override(self, tmp_path):
        _, log = self._wifi(
            tmp_path, 'connect', 'EntNet', '--identity=someone@school.edu',
            '--ca-cert=/etc/ssl/certs/ca.pem', stdin='hunter2\n')
        add = next((ln for ln in log.splitlines() if 'connection add' in ln), '')
        assert '802-1x.ca-cert /etc/ssl/certs/ca.pem' in add
        assert '802-1x.domain-suffix-match school.edu' in add

    def test_connect_rejects_unknown_flag(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'connect', 'PskNet', '--turbo')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_connect_to_invisible_network_errors(self, tmp_path):
        result, _ = self._wifi(tmp_path, 'connect', 'NotBroadcasting')
        assert result.returncode == 4
        assert 'not visible' in result.stderr


class TestDesktopCommand:
    """`racecar desktop`: reboot-scoped GNOME toggle, enabled by default."""

    STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$SCTL_LOG"
case "$*" in
    "get-default")   echo "${STUB_DEFAULT:-graphical.target}" ;;
    "is-active graphical.target")
        [ "${STUB_ACTIVE:-graphical}" = "graphical" ] && exit 0 || exit 3 ;;
    "show display-manager.service -p Id --value") echo "${STUB_DM-gdm.service}" ;;
    "is-active gdm.service") echo active ;;
    *) : ;;
esac
"""

    def _desktop(self, tmp_path, *args, **env_extra):
        stub = tmp_path / 'systemctl'
        stub.write_text(self.STUB)
        stub.chmod(0o755)
        log = tmp_path / 'systemctl.log'
        log.touch()
        env = dict(os.environ)
        env.update({
            'SCTL_LOG': str(log),
            'RACECAR_SYSTEMCTL': str(stub),
            'RACECAR_SUDO': '',  # enable/disable are sudo-gated in real use
        })
        env.update(env_extra)
        script = f'set +u; source "{TOOL}"; racecar desktop {" ".join(args)}'
        result = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True,
            timeout=20, env=env,
        )
        return result, log.read_text()

    def test_help_lists_actions(self, tmp_path):
        result, _ = self._desktop(tmp_path, 'help')
        assert result.returncode == 0
        for action in ('status', 'enable', 'disable'):
            assert action in result.stdout

    def test_rejects_unknown_action(self, tmp_path):
        result, _ = self._desktop(tmp_path, 'sideways')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr

    def test_status_is_the_default_action(self, tmp_path):
        result, _ = self._desktop(tmp_path)
        assert result.returncode == 0, result.stderr
        assert 'default target' in result.stdout

    def test_status_reports_enabled(self, tmp_path):
        result, _ = self._desktop(tmp_path, 'status', STUB_DEFAULT='graphical.target')
        assert 'Desktop is enabled' in result.stdout

    def test_status_reports_disabled(self, tmp_path):
        result, _ = self._desktop(
            tmp_path, 'status', STUB_DEFAULT='multi-user.target', STUB_ACTIVE='multi')
        assert 'Desktop is disabled' in result.stdout

    def test_status_reports_pending_reboot(self, tmp_path):
        # Default says headless but the graphical target is still running:
        # the change is real but has not taken effect yet.
        result, _ = self._desktop(
            tmp_path, 'status', STUB_DEFAULT='multi-user.target', STUB_ACTIVE='graphical')
        assert 'Pending' in result.stdout
        assert 'reboot' in result.stdout.lower()

    def test_status_without_a_display_manager(self, tmp_path):
        result, _ = self._desktop(tmp_path, 'status', STUB_DM='')
        assert 'none installed' in result.stdout

    def test_disable_sets_multi_user_target(self, tmp_path):
        result, log = self._desktop(
            tmp_path, 'disable', STUB_DEFAULT='graphical.target')
        assert result.returncode == 0, result.stderr
        assert 'set-default multi-user.target' in log

    def test_enable_sets_graphical_target(self, tmp_path):
        result, log = self._desktop(
            tmp_path, 'enable', STUB_DEFAULT='multi-user.target')
        assert result.returncode == 0, result.stderr
        assert 'set-default graphical.target' in log

    def test_toggle_announces_the_reboot_requirement(self, tmp_path):
        result, _ = self._desktop(
            tmp_path, 'disable', STUB_DEFAULT='graphical.target')
        assert 'next boot' in result.stdout or 'after a reboot' in result.stdout

    def test_toggle_is_idempotent(self, tmp_path):
        result, log = self._desktop(
            tmp_path, 'enable', STUB_DEFAULT='graphical.target')
        assert result.returncode == 0
        assert 'already' in result.stdout
        assert 'set-default' not in log, 'no write when already in the target state'

    def test_never_isolates_a_target(self, tmp_path):
        # The toggle is reboot-scoped on purpose: isolating would tear down
        # the session of whoever is sitting at the machine.
        for args, env in (
            (['enable'], {'STUB_DEFAULT': 'multi-user.target'}),
            (['disable'], {'STUB_DEFAULT': 'graphical.target'}),
        ):
            _, log = self._desktop(tmp_path, *args, **env)
            assert 'isolate' not in log

    def test_never_tries_to_enable_the_display_manager(self, tmp_path):
        # gdm.service is `static` (no [Install] section), so enable/disable on
        # it always fails. graphical.target is what pulls it in.
        for args, env in (
            (['enable'], {'STUB_DEFAULT': 'multi-user.target'}),
            (['disable'], {'STUB_DEFAULT': 'graphical.target'}),
        ):
            _, log = self._desktop(tmp_path, *args, **env)
            assert 'enable gdm' not in log
            assert 'disable gdm' not in log


class TestEthCommand:
    """`racecar eth`: eth0 addressing mode; static or DHCP, never both."""

    def test_help_lists_actions(self):
        result = _run('eth', 'help')
        assert result.returncode == 0
        for action in ('status', 'static', 'dynamic'):
            assert action in result.stdout

    def test_rejects_unknown_action(self):
        result = _run('eth', 'sideways')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr

    def test_dispatches_to_setup_eth_script(self, tmp_path):
        # Stub the script so the test never touches netplan.
        stub = tmp_path / 'stub_eth.sh'
        stub.write_text('#!/bin/bash\necho "STUB called with: $*"\n')
        stub.chmod(0o755)
        script = (
            f'set +u; export RACECAR_ETH_SCRIPT="{stub}"; '
            f'source "{TOOL}"; racecar eth static --addr=10.0.0.5/24'
        )
        result = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True, timeout=15,
        )
        assert result.returncode == 0, result.stderr
        assert 'STUB called with: static --addr=10.0.0.5/24' in result.stdout

    def test_monitor_dispatches_to_the_logger(self):
        # The mutual-exclusion fix is unproven, so the logger is how it gets
        # confirmed; --once samples and exits without looping.
        result = _run('eth', 'monitor', '--once', '--log', '/dev/null')
        assert result.returncode == 0, result.stderr
        assert 'carrier=' in result.stdout

    def test_monitor_listed_in_help(self):
        assert 'monitor' in _run('eth', 'help').stdout

    def test_default_action_is_status(self, tmp_path):
        stub = tmp_path / 'stub_eth.sh'
        stub.write_text('#!/bin/bash\necho "STUB called with: $*"\n')
        stub.chmod(0o755)
        script = (
            f'set +u; export RACECAR_ETH_SCRIPT="{stub}"; '
            f'source "{TOOL}"; racecar eth'
        )
        result = subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True, timeout=15,
        )
        assert result.returncode == 0, result.stderr
        assert 'STUB called with: status' in result.stdout
