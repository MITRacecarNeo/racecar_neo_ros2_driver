"""The autonomy unit: racecar-autonomy.service, its launcher, mode file, and polkit rule."""

from configparser import ConfigParser
from datetime import datetime
import os
from pathlib import Path

from conftest import load_script
import pytest
from racecar_neo_ros2_driver import autonomy as au

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'
MODE_FILE = 'autonomy.conf'


@pytest.fixture(scope='module')
def args():
    return load_script('autonomy_args')


@pytest.fixture(scope='module')
def unit():
    parser = ConfigParser(strict=False, interpolation=None)
    parser.optionxform = str
    parser.read(SCRIPTS / 'racecar-autonomy.service')
    return parser


class TestUnit:
    def test_rides_on_teleop(self, unit):
        u = unit['Unit']
        assert u['After'] == u['BindsTo'] == u['PartOf'] == 'racecar-teleop.service'

    def test_restarts_after_a_crash_but_not_a_stop(self, unit):
        # ros2 launch exits 0 when slam_toolbox dies; on-failure would never restart it.
        assert unit['Service']['Restart'] == 'always'
        assert int(unit['Unit']['StartLimitBurst']) == 3

    def test_not_enabled_at_boot(self, unit):
        assert 'Install' not in unit

    def test_mode_file_and_stop_sequence(self, unit):
        s = unit['Service']
        assert s['EnvironmentFile'].startswith('-') and s['EnvironmentFile'].endswith(MODE_FILE)
        assert s['ExecStart'].endswith('scripts/launch_autonomy.sh')
        assert 'autonomy_stop.py $MAINPID' in s['ExecStop']
        assert s['User'] == 'racecar' and s['KillSignal'] == 'SIGINT'
        assert int(s['TimeoutStopSec']) >= 30

    def test_installed_but_not_enabled(self):
        text = (SCRIPTS / 'setup_services.sh').read_text()
        assert 'INSTALL_ONLY=(\n    racecar-autonomy.service\n)' in text
        assert 'for svc in "${SERVICES[@]}"; do\n    if systemctl is-enabled' in text

    def test_launcher_uses_the_mode_file_arguments(self):
        text = (SCRIPTS / 'launch_autonomy.sh').read_text()
        assert 'autonomy_args.py' in text
        # tee must outlive the stop signal, or launch exits 120 on a broken pipe.
        assert "exec &> >(trap '' INT TERM; exec tee -a" in text
        assert 'exec ros2 launch racecar_neo_ros2_driver autonomy.launch.py "${ARGS[@]}"' in text


class TestModeFile:
    def test_mapping(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        got = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's1'})
        assert got == ['mode:=mapping', 'map:=room', 'session:=s1']

    def test_localization(self, args):
        env = {
            'AUTONOMY_MODE': 'localization',
            'AUTONOMY_MAP': 'room',
            'AUTONOMY_POSE': '-1.5,2,0.3',
            'AUTONOMY_POSE_SOURCE': 'last_pose',
            'AUTONOMY_LOCALIZER': 'slam_toolbox',
            'AUTONOMY_FOXGLOVE': '1',
        }
        assert args.launch_args(env) == [
            'mode:=localization',
            'map:=room',
            'session:=',
            'localizer:=slam_toolbox',
            'pose:=-1.5,2,0.3',
            'pose_source:=last_pose',
            'foxglove:=true',
        ]

    def test_resume_from_the_same_session(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        auto = tmp_path / 'room' / au.AUTOSAVE_DIR
        auto.mkdir(parents=True)
        (auto / 'room.posegraph').write_text('g')
        au.write_yaml_atomic(
            auto / au.AUTOSAVE_FILE,
            au.pose_record((1.0, -2.0, 0.5), datetime(2026, 10, 9), session='s1'),
        )
        got = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's1'})
        assert got[-2:] == [f'resume:={auto / "room"}', 'pose:=1.0,-2.0,0.5']
        fresh = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's2'})
        assert not any(a.startswith('resume:=') for a in fresh)

    def test_extend_a_saved_map(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room', 'room')
        env = {
            'AUTONOMY_MAP': 'hall',
            'AUTONOMY_SESSION': 's1',
            'AUTONOMY_FROM': 'room',
            'AUTONOMY_POSE': '-1.5,2,0.3',
            'AUTONOMY_POSE_SOURCE': 'last_pose',
        }
        assert args.launch_args(env) == [
            'mode:=mapping',
            'map:=hall',
            'session:=s1',
            f'extend:={tmp_path / "room" / "room"}',
            'pose:=-1.5,2,0.3',
            'pose_source:=last_pose',
        ]

    def test_extend_an_unsaved_map_from_its_autosave(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room' / au.AUTOSAVE_DIR, 'room')
        got = args.launch_args({'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': 'room'})
        assert f'extend:={tmp_path / "room" / au.AUTOSAVE_DIR / "room"}' in got
        assert 'pose:=0,0,0' in got

    def test_a_crashed_extension_resumes_its_own_autosave(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room', 'room')
        auto = tmp_path / 'hall' / au.AUTOSAVE_DIR
        _files(auto, 'hall')
        au.write_yaml_atomic(
            auto / au.AUTOSAVE_FILE,
            au.pose_record((1.0, 0.0, 0.0), datetime(2026, 10, 9), session='s1'),
        )
        env = {'AUTONOMY_MAP': 'hall', 'AUTONOMY_SESSION': 's1', 'AUTONOMY_FROM': 'room'}
        got = args.launch_args(env)
        assert f'resume:={auto / "hall"}' in got
        assert not any(a.startswith('extend:=') for a in got)

    @pytest.mark.parametrize(
        'env',
        [
            {'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': 'missing'},
            {'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': '../x'},
            {},
            {'AUTONOMY_MAP': '../x'},
            {'AUTONOMY_MAP': 'room', 'AUTONOMY_MODE': 'drive'},
            {'AUTONOMY_MAP': 'room', 'AUTONOMY_LOCALIZER': 'gmapping'},
        ],
    )
    def test_bad_values(self, args, env):
        with pytest.raises(ValueError):
            args.launch_args(env)


def _files(folder, name):
    folder.mkdir(parents=True, exist_ok=True)
    for suffix in au.MAP_SUFFIXES:
        (folder / f'{name}{suffix}').write_text('m')


def test_polkit_rule_scope():
    text = (SCRIPTS / 'polkit' / '49-racecar-autonomy.rules').read_text()
    assert '"racecar-autonomy.service"' in text and 'subject.user === "racecar"' in text
    assert '["start", "stop", "restart"]' in text
    assert 'org.freedesktop.systemd1.manage-units' in text
    installer = (SCRIPTS / 'setup_user_env.sh').read_text()
    assert '49-racecar-autonomy.rules' in installer


def test_stop_waits_only_while_the_launch_runs():
    stop = load_script('autonomy_stop')
    assert stop.running(str(os.getpid()))
    assert not stop.running('0') and not stop.running('') and not stop.running('x')
    assert stop.SERVICE_WAIT_SEC >= 8 and stop.GONE_WAIT_SEC <= 1
