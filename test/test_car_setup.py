"""scripts/car_setup.py: the guided first setup (`racecar setup car`)."""

import json
from pathlib import Path
import subprocess

from conftest import load_script
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture
def cs():
    return load_script('car_setup')


def write_car(path, data):
    path.mkdir(parents=True, exist_ok=True)
    (path / 'car.yaml').write_text(yaml.safe_dump(data))


class FakeCar:
    """The car state a check reads, set per test."""

    def __init__(self, cs, **kw):
        self.real = cs.Car()
        self.values = kw

    def __getattr__(self, name):
        if name in self.values:
            value = self.values[name]
            return lambda: value
        return getattr(self.real, name)


class TestIdentity:
    def test_needs_name_number_and_hostname(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill'})
        assert not cs.check_identity(cs.Car()).done
        write_car(car_profile_dir, {'name': 'bill', 'id': 0})
        car = FakeCar(cs, static_hostname='racecar-neo')
        assert 'needs racecar-neo-0' in cs.check_identity(car).detail
        car = FakeCar(cs, static_hostname='racecar-neo-0')
        assert cs.check_identity(car).done
        write_car(car_profile_dir, {'name': 'bill', 'nickname': 'Bill', 'id': 0})
        assert cs.check_identity(FakeCar(cs, static_hostname='racecar-neo-0')).detail == (
            'bill (Bill), number 0, racecar-neo-0'
        )

    def test_run_writes_name_number_and_sets_the_hostname(self, cs, car_profile_dir, monkeypatch):
        answers = iter(['racecar-neo-10', 'Diana', '99', '10'])
        monkeypatch.setattr('builtins.input', lambda prompt='': next(answers))
        calls = []

        def run(argv, **kw):
            calls.append(argv)
            return subprocess.CompletedProcess(argv, 0, '', '')

        monkeypatch.setattr(cs.subprocess, 'run', run)
        monkeypatch.setattr(cs, 'read_text', lambda path: '127.0.0.1\tlocalhost\n')
        assert cs.run_identity(cs.Car()) == 0
        info = yaml.safe_load((car_profile_dir / 'car.yaml').read_text())
        assert info['name'] == 'racecar-neo-10' and info['nickname'] == 'Diana'
        assert info['id'] == 10
        assert ['sudo', 'hostnamectl', 'set-hostname', 'racecar-neo-10'] in calls
        assert ['sudo', 'tee', '/etc/hosts'] in calls


class TestChecks:
    def test_access_point_needs_this_cars_number_and_the_connection(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill', 'id': 0})
        car = FakeCar(cs, networking={'RACECAR_AP_ID': '3'}, active_connections=[])
        assert cs.check_access_point(car).detail == 'set for car 3'
        car = FakeCar(cs, networking={'RACECAR_AP_ID': '0'}, active_connections=[])
        assert cs.check_access_point(car).detail == 'not running'
        car = FakeCar(cs, networking={'RACECAR_AP_ID': '0'}, active_connections=['racecar-neo-ap'])
        assert cs.check_access_point(car).done

    def test_reboot_waits_for_a_new_hostname(self, cs, car_profile_dir, monkeypatch):
        monkeypatch.setattr(cs, 'read_text', lambda path: '')
        car = FakeCar(cs, static_hostname='racecar-neo-10', hostname='racecar-neo', data_root='')
        assert cs.check_reboot(car).detail == 'needed for hostname racecar-neo-10'
        car = FakeCar(cs, static_hostname='racecar-neo-10', hostname='racecar-neo-10')
        assert cs.check_reboot(car).done

    def test_firmware_against_the_pin(self, cs, car_profile_dir, monkeypatch):
        monkeypatch.setattr(cs, 'pinned', lambda: {'teensy': {'version': '0.9.2'}})
        assert cs.check_firmware(FakeCar(cs, about={'firmware': '0.9.2'})).done
        assert cs.check_firmware(FakeCar(cs, about={'firmware': '0.9.1'})).detail == (
            '0.9.1 installed, 0.9.2 needed'
        )
        assert 'teleop' in cs.check_firmware(FakeCar(cs, about={})).detail
        monkeypatch.setattr(cs, 'pinned', lambda: {})
        assert 'racecar build' in cs.check_firmware(FakeCar(cs, about={})).detail

    def test_esc_and_record_need_their_fields(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'record': {'esc_model': 'x'}})
        assert cs.check_esc(cs.Car()).detail == 'not recorded'
        full = {key: 'v' for key, _, _ in cs.RECORD_FIELDS}
        write_car(car_profile_dir, {'record': full})
        assert cs.check_esc(cs.Car()).done and cs.check_record(cs.Car()).done

    def test_health_needs_power_and_link(self, cs, car_profile_dir, tmp_path, monkeypatch):
        from racecar_neo_ros2_driver import data_root

        health = tmp_path / 'health'
        monkeypatch.setattr(data_root, 'data_dir', lambda sub=None: health)
        assert cs.check_health(cs.Car()).detail == 'no report'
        run = health / '20261109_120000'
        run.mkdir(parents=True)
        report = {'time': 't', 'result': 'OK', 'checks': [{'group': 'power'}]}
        (run / 'report.json').write_text(json.dumps(report))
        assert cs.check_health(cs.Car()).detail.startswith('no link check in the last report')
        report['checks'].append({'group': 'link'})
        (run / 'report.json').write_text(json.dumps(report))
        assert cs.check_health(cs.Car()).done


class TestState:
    def test_passing_check_records_and_a_failing_one_clears(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill'})
        step = cs.Step('nvme', 'NVMe', lambda car: cs.Result(True, 'ok'), lambda car: 0)
        assert cs.evaluate(cs.Car(), step)[0]
        assert yaml.safe_load((car_profile_dir / 'car.yaml').read_text())['setup']['nvme']
        step = cs.Step('nvme', 'NVMe', lambda car: cs.Result(False, 'gone'), lambda car: 0)
        assert not cs.evaluate(cs.Car(), step)[0]
        assert 'nvme' not in (yaml.safe_load((car_profile_dir / 'car.yaml').read_text())['setup'])

    def test_marked_done_stays_done(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill'})
        cs.mark(cs.Car(), 'esc', 'user')
        step = cs.Step('esc', 'ESC', lambda car: cs.Result(False, 'not recorded'), lambda car: 0)
        done, by, detail = cs.evaluate(cs.Car(), step)
        assert done and by == 'user' and detail == 'marked done'

    def test_complete_only_with_every_step(self, cs):
        setup = {k: {'done': 'now', 'by': 'user'} for k in cs.STEP_KEYS}
        assert cs.setup_complete({'setup': setup})
        setup.pop('record')
        assert not cs.setup_complete({'setup': setup})
        assert cs.open_steps({'setup': setup}) == ['Car record']

    def test_other_car_yaml_keys_survive(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill', 'terminal': {'allow_lan': True}})
        cs.update_car({'record': {'motor': 'm'}})
        cs.update_car({'record': {'servo': 's'}})
        info = yaml.safe_load((car_profile_dir / 'car.yaml').read_text())
        assert info['terminal'] == {'allow_lan': True}
        assert info['record'] == {'motor': 'm', 'servo': 's'}


class TestMeasurements:
    def test_stock_kept_writes_nothing(self, cs, car_profile_dir, monkeypatch):
        write_car(car_profile_dir, {'name': 'bill'})
        monkeypatch.setattr('builtins.input', lambda prompt='': '')
        assert cs.run_measurements(cs.Car()) == 0
        assert not (car_profile_dir / 'params' / 'frames.yaml').exists()
        assert (
            yaml.safe_load((car_profile_dir / 'car.yaml').read_text())['setup']['measurements'][
                'note'
            ]
            == 'stock'
        )

    def test_changed_values_written(self, cs, car_profile_dir, monkeypatch):
        write_car(car_profile_dir, {'name': 'bill'})
        answers = iter(['n', '240', '', '', '', '', ''])
        monkeypatch.setattr('builtins.input', lambda prompt='': next(answers))
        cs.run_measurements(cs.Car())
        frames = yaml.safe_load((car_profile_dir / 'params' / 'frames.yaml').read_text())
        assert frames == {'laser_x_mm': 240.0}


def test_racecar_setup_car_dispatches():
    tool = (SCRIPTS / 'racecar-tool.sh').read_text()
    assert 'python3 "$pkg_dir/scripts/car_setup.py" "$@"' in tool
    assert 'compgen -W "all car services' in tool


@pytest.mark.parametrize(
    'car_id, address',
    [('0', '192.168.52.200/24'), ('7', '192.168.52.207/24'), ('54', '192.168.52.254/24'),
     ('55', '192.168.52.200/24'), ('', '192.168.52.200/24')],
)  # fmt: skip
def test_eth_address_from_the_car_number(car_id, address):
    script = f'source "{SCRIPTS / "setup_lib.sh"}"; eth_static_default "{car_id}"'
    out = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert out.stdout.strip() == address


def test_both_networking_scripts_use_it():
    for name in ('setup_networking.sh', 'setup_eth.sh'):
        text = (SCRIPTS / name).read_text()
        assert 'eth_static_default "${RACECAR_AP_ID:-}"' in text, name


class TestOutput:
    def test_checklist_rows_like_racecar_status(self, cs, car_profile_dir, capsys, monkeypatch):
        write_car(car_profile_dir, {'name': 'bill'})
        steps = (
            cs.Step('a', 'Hardware connection check', lambda car: cs.Result(True, ''), None),
            cs.Step('b', 'Car record', lambda car: cs.Result(False, '4 fields empty'), None),
        )
        monkeypatch.setattr(cs, 'STEPS', steps)
        monkeypatch.setattr(cs, 'use_color', lambda: False)
        cs.checklist(cs.Car())
        assert capsys.readouterr().out.splitlines() == [
            '  [DONE]  Hardware connection check',
            '  [OPEN]  Car record                 4 fields empty',
        ]

    def test_prompt_uses_the_calibration_wording(self, cs):
        calibrate = load_script('calibrate')
        assert calibrate.RUN_PROMPT == 'Press Enter to run, s to skip, and q to stop: '
        assert cs.STEP_PROMPT == (
            'Press Enter to run or confirm a step, d to mark done, s to skip, and q to stop'
        )

    def test_no_detail_repeats_its_step_name(self, cs):
        names = {s.title.lower() for s in cs.STEPS}
        assert 'hardware connection check' in names
        assert all('every device answers' not in n for n in names)


class TestUnknownAndOptional:
    def test_unreadable_check_keeps_a_done_step(self, cs, car_profile_dir):
        write_car(car_profile_dir, {'name': 'bill'})
        cs.mark(cs.Car(), 'firmware', 'check')
        step = cs.Step(
            'firmware',
            'Teensy firmware',
            lambda car: cs.Result(False, 'no read', unknown=True),
            None,
        )
        assert not cs.evaluate(cs.Car(), step)[0]
        assert 'firmware' in yaml.safe_load((car_profile_dir / 'car.yaml').read_text())['setup']

    def test_idle_current_optional_and_count_wording(self, cs, car_profile_dir):
        record = {key: 'v' for key, _, _ in cs.RECORD_FIELDS}
        record['idle_current'] = ''
        write_car(car_profile_dir, {'record': record})
        assert cs.check_record(cs.Car()).done
        record['motor'] = ''
        write_car(car_profile_dir, {'record': record})
        assert cs.check_record(cs.Car()).detail == '1 field empty'
