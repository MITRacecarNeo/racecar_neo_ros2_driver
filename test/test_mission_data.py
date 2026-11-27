"""racecar_neo_ros2_driver.mission_data and scripts/mission_recorder.py: what a mission records."""

import json
from pathlib import Path
from types import SimpleNamespace

from conftest import load_script
import pytest
from racecar_neo_ros2_driver import mission_data, missions

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def config():
    return mission_data.load_config(ROOT / 'config')


def event(t, name, waypoint=1, loop=1, dist=0.0, **fields):
    return {'t': t, 'event': name, 'loop': loop, 'waypoint': waypoint, 'dist_m': dist, **fields}


class TestGroups:
    def test_shipped_groups(self, config):
        ids = [g['id'] for g in mission_data.groups_view(config)]
        assert ids == ['state', 'commands', 'lidar', 'navigation', 'camera']
        assert set(missions.RECORD_GROUPS) == set(ids) - {'camera'}
        topics = mission_data.topics(config, ['lidar', 'state', 'lidar', 'gone'])
        assert topics[0] == '/scan' and len(topics) == len(set(topics))
        everything = mission_data.topics(config, ids)
        assert not {'/camera/color', '/camera/depth'} & set(everything)
        assert mission_data.min_free_bytes(config) == 2e9

    def test_the_car_profile_adds_and_replaces(self, tmp_path, monkeypatch):
        own = tmp_path / 'mission_data.yaml'
        own.write_text('mission_data: {min_free_gb: 5, groups: {lidar: {name: L, topics: [/l]}, '
                       'mine: {name: Mine, topics: [/m]}}}')  # fmt: skip
        monkeypatch.setattr(mission_data.car_profile, 'param_override', lambda name: own)
        config = mission_data.load_config(ROOT / 'config')
        assert mission_data.topics(config, ['lidar', 'mine']) == ['/l', '/m']
        assert 'state' in config['groups'] and mission_data.min_free_bytes(config) == 5e9

    def test_bag_name(self):
        assert mission_data.bag_name('loop5', 0).endswith('_mission_loop5')
        assert mission_data.bag_name('', 0).endswith('_mission')


class TestEventLog:
    def test_waits_for_the_folder_rosbag2_makes(self, tmp_path):
        folder = tmp_path / 'bag'
        log = mission_data.EventLog(folder, {'loops': 1})
        log.add('start', 10.0, loop=1)
        assert not folder.exists()
        folder.mkdir()
        log.add('reached', 11.0, loop=1)
        got = mission_data.read_events(folder)
        assert [e['event'] for e in got] == ['start', 'reached'] and got[0]['t'] == 10.0
        assert (folder / 'mission.yaml').read_text() == 'loops: 1\n'

    def test_an_unfinished_line_is_left_out(self, tmp_path):
        line = json.dumps({'t': 1, 'event': 'start'})
        (tmp_path / 'events.jsonl').write_text(f'{line}\n[1]\n{{"t": 2, "ev')
        assert len(mission_data.read_events(tmp_path)) == 1
        assert mission_data.read_events(tmp_path / 'none') == []


class TestSummary:
    EVENTS = [
        event(0, 'start', mission={'waypoints': [1, 2, 3]}),
        event(0, 'sent'),
        event(1, 'sent', resend=True),
        event(5, 'reached', dist=2.0),
        event(5, 'sent', 2, dist=2.0),
        event(7, 'paused', 2, reason='stalled', dist=3.0),
        event(17, 'resumed', 2, dist=3.0),
        event(17, 'sent', 2, dist=3.0),
        event(18, 'relocalizing', 2, dist=3.5),
        event(20, 'localized', 2, dist=3.5),
        event(20, 'sent', 2, dist=3.5),
        event(22, 'reached', 2, dist=4.5),
        event(22, 'sent', 3, dist=4.5),
        event(24, 'skipped', 3, dist=5.0),
        event(24, 'sent', 1, 2, dist=5.0),
        event(30, 'stopped', 1, 2, dist=6.25),
    ]

    def test_legs_pauses_and_what_was_missed(self):
        s = mission_data.summarize(self.EVENTS)
        assert (s['result'], s['duration_sec'], s['distance_m']) == ('stopped', 30, 6.25)
        legs = [(g['loop'], g['waypoint'], g['result'], g['sec'], g['resends']) for g in s['legs']]
        assert legs == [(1, 1, 'reached', 5, 1), (1, 2, 'reached', 17, 0),
                        (1, 3, 'skipped', 2, 0), (2, 1, 'stopped', 6, 0)]  # fmt: skip
        assert s['legs'][1]['dist_m'] == 2.5
        pauses = [(p['cause'], p['waypoint'], p['sec']) for p in s['pauses']]
        assert pauses == [('stalled', 2, 10), ('localization lost', 2, 2)]
        assert s['not_reached'] == [3]
        assert [g['sec'] for g in s['loops']] == [24, 6]
        text = '\n'.join(mission_data.summary_lines(s))
        assert 'car stalled, 10.0 s' in text and 'not reached: 3' in text and 'loop 2' in text

    def test_a_log_that_just_ends(self):
        s = mission_data.summarize(self.EVENTS[:6])
        assert s['result'] == 'recording ended' and s['pauses'][0]['sec'] == 0
        assert mission_data.summarize([])['legs'] == []


class TestAnalyze:
    def test_reads_the_bag_and_the_events(self, tmp_path, monkeypatch, capsys):
        log = load_script('racecar_log')
        topic = SimpleNamespace(topic_metadata=SimpleNamespace(name='/scan'), message_count=20)
        meta = SimpleNamespace(duration=SimpleNamespace(nanoseconds=2e9), message_count=20,
                               topics_with_message_count=[topic])  # fmt: skip
        monkeypatch.setattr(log, 'find_bag', lambda name: tmp_path)
        monkeypatch.setattr(log, '_read_metadata', lambda bag: meta)
        assert log.main(['analyze']) == 0 and 'mission' not in capsys.readouterr().out
        lines = ''.join(json.dumps(e) + '\n' for e in TestSummary.EVENTS)
        (tmp_path / 'events.jsonl').write_text(lines)
        assert log.main(['analyze']) == 0
        out = capsys.readouterr().out
        assert '/scan' in out and 'mission   stopped, 30.0 s, 6.25 m' in out
        assert log.main(['analyze', '--json']) == 0
        assert json.loads(capsys.readouterr().out)['mission']['not_reached'] == [3]


class TestRecorder:
    @pytest.fixture
    def rec(self, tmp_path, monkeypatch):
        log = load_script('racecar_log')
        mod = load_script('mission_recorder')
        state = {}
        calls = []

        def run(argv, **k):
            calls.append(argv[2])
            if argv[2] == 'start':
                state.update(bag=str(tmp_path / argv[4]), stdout=str(tmp_path / 'out.log'), pid=1)
            else:
                state.clear()
            return SimpleNamespace(returncode=0)

        monkeypatch.setattr(log, 'active_state', lambda: dict(state) or None)
        r = mod.Recorder(run)
        r.state, r.calls, r.mod = state, calls, mod
        return r

    def test_records_and_stops_its_own(self, rec, tmp_path):
        assert rec.start('a', ['/scan'], 0) and rec.running()
        assert rec.folder == tmp_path / 'a'
        rec.stop()
        assert rec.calls == ['start', 'stop'] and not rec.running()

    def test_status_while_recording(self, rec, tmp_path, monkeypatch):
        assert rec.status(0) is None
        rec.start('a', ['/scan', '/odom'], 0)
        (tmp_path / 'out.log').write_text("Subscribed to topic '/scan'\n")
        rec.state['started'] = rec.mod.time.time() - 60
        (tmp_path / 'a').mkdir()
        (tmp_path / 'a' / 'a_0.mcap').write_bytes(bytes(6 * 1024 * 1024))
        s = rec.status(0)
        assert (s['time'], s['size'], s['rate']) == ('1:00', '6.0 MB', '102.4 KB/s')
        assert s['topics'] == [1, 2] and s['free']
        monkeypatch.setattr(rec.mod, 'DAY_SEC', 1e12)
        assert rec.status(0)['left']
        monkeypatch.setattr(rec.mod, 'DAY_SEC', 1)
        assert rec.status(0)['left'] is None
        assert rec.status(1e30)['left'] == '0:00'
        rec.stop()
        assert rec.status(0) is None

    def test_analyze_is_what_the_command_prints(self, rec, tmp_path):
        seen = []

        def run(argv, **k):
            seen.append(argv[2:])
            return SimpleNamespace(returncode=0, stdout='name\n  mission   arrived\n', stderr='')

        rec._run = run
        assert rec.analyze(tmp_path / 'a') == 'name\n  mission   arrived'
        assert seen == [['analyze', str(tmp_path / 'a')]]
        failed = SimpleNamespace(returncode=1, stdout='', stderr='no such bag\n')
        rec._run = lambda argv, **k: failed
        assert rec.analyze(tmp_path / 'a') == 'no such bag'

    def test_leaves_the_users_recording_alone(self, rec, tmp_path):
        rec.state.update(bag=str(tmp_path / 'users'), stdout='', pid=1)
        assert not rec.start('a', ['/scan'], 0)
        rec.stop()
        assert rec.calls == []

    def test_does_not_stop_one_started_after_its_own_ended(self, rec, tmp_path):
        rec.start('a', ['/scan'], 0)
        rec.state.update(bag=str(tmp_path / 'users'))
        rec.stop()
        assert rec.calls == ['start'] and not rec.running()

    def test_no_space_or_no_topics_is_no_recording(self, rec):
        assert not rec.start('a', [], 0) and rec.calls == []
        assert not rec.start('a', ['/scan'], 1e30) and rec.calls == ['start', 'stop']

    def test_ready_once_every_topic_is_subscribed(self, rec, tmp_path):
        rec.start('a', ['/scan', '/odom'], 0)
        (tmp_path / 'out.log').write_text("Subscribed to topic '/scan'\n")
        assert not rec.wait_ready(['/scan', '/odom'], lambda: False, 0.1)
        assert not rec.wait_ready(['/scan', '/odom'], lambda: True, 60)
        with (tmp_path / 'out.log').open('a') as f:
            f.write("Subscribed to topic '/odom'\n")
        assert rec.wait_ready(['/scan', '/odom'], lambda: False, 0.1)


def test_the_planner_unit_lets_the_recording_finish():
    text = (ROOT / 'scripts' / 'racecar-planner.service').read_text()
    assert 'KillMode=mixed' in text
