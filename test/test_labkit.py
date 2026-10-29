"""Tests for scripts/labkit.py: marks, tuning load and save, run logs."""

import threading

from conftest import load_script
import pytest

labkit = load_script('labkit')

SHIPPED = """# Lab parameters.
speed: 0.0            # 0..1 throttle
kp: 0.5               # steering strength
count: 3              # a whole number
mode: 1               # chosen by editing the file
topic: /scan
"""


@pytest.fixture
def clock(monkeypatch):
    t = [10.0]
    monkeypatch.setattr(labkit, 'now', lambda: t[0])
    return t


@pytest.fixture
def tuning(tmp_path):
    shipped = tmp_path / 'lab.yaml'
    shipped.write_text(SHIPPED)

    def clean(p):
        p['speed'] = max(0.0, min(1.0, float(p['speed'])))

    t = labkit.Tuning(
        'lab',
        shipped,
        ('speed', 'kp', 'count'),
        threading.Lock(),
        labkit.Marks(),
        clean=clean,
        int_keys=('count',),
        fixed=('mode',),
    )
    t.path = tmp_path / 'profile' / 'lab.yaml'
    t.load()
    return t


class TestMarks:
    def test_a_drag_is_one_mark(self, clock):
        marks = labkit.Marks()
        marks.add('speed 0.1')
        clock[0] += 0.5
        marks.add('speed 0.2')
        assert list(marks) == [[10.5, 'speed 0.2']]

    def test_other_keys_and_later_changes_add_marks(self, clock):
        marks = labkit.Marks()
        marks.add('speed 0.1')
        marks.add('kp 1')
        clock[0] += 2.0
        marks.add('kp 2')
        assert [m[1] for m in marks] == ['speed 0.1', 'kp 1', 'kp 2']

    def test_keeps_the_last_forty(self, clock):
        marks = labkit.Marks()
        for i in range(50):
            clock[0] += 2.0
            marks.add(f'k {i}')
        assert len(marks) == 40 and marks[-1][1] == 'k 49'


class TestRewriteNumbers:
    def test_keeps_comments_and_spacing(self):
        text = labkit.rewrite_numbers(SHIPPED, {'speed': 0.4, 'kp': 2.0})
        assert 'speed: 0.4            # 0..1 throttle\n' in text
        assert 'kp: 2.0               # steering strength\n' in text
        assert text.startswith('# Lab parameters.\n') and 'topic: /scan\n' in text

    def test_appends_tunables_the_file_lacks(self):
        text = labkit.rewrite_numbers('speed: 0.0\n', {'speed': 0.2, 'gain': 3}, add=('gain',))
        assert text == 'speed: 0.2\ngain: 3\n'

    def test_never_appends_other_keys_or_text(self):
        text = labkit.rewrite_numbers('speed: 0.0', {'speed': 0.2, 'x': 1, 'y': 'a'}, add=('y',))
        assert text == 'speed: 0.2\n'

    def test_booleans_are_left_alone(self):
        assert labkit.rewrite_numbers('on: 1\n', {'on': True}) == 'on: 1\n'


class TestTuning:
    def test_load_puts_the_profile_on_top(self, tuning):
        tuning.path.parent.mkdir(parents=True)
        tuning.path.write_text('kp: 3.0\n')
        tuning.load()
        assert tuning.params['kp'] == 3.0 and tuning.params['speed'] == 0.0
        assert [m[1] for m in tuning._marks] == ['yaml load']

    def test_first_load_adds_no_mark(self, tuning):
        assert list(tuning._marks) == []

    def test_update_cleans_and_labels(self, tuning):
        assert tuning.update({'speed': 4.0, 'count': '7', 'other': 1}) == ['speed 1', 'count 7']
        assert tuning.params['count'] == 7 and isinstance(tuning.params['count'], int)
        assert 'other' not in tuning.params
        assert tuning.update({'speed': 1.0}) == []

    def test_save_writes_only_the_profile_file(self, tuning):
        tuning.update({'speed': 0.3, 'count': 5})
        tuning.save()
        text = tuning.path.read_text()
        assert 'speed: 0.3            # 0..1 throttle\n' in text and 'count: 5 ' in text
        assert tuning.defaults.read_text() == SHIPPED

    def test_save_keeps_the_fixed_keys(self, tuning):
        tuning.params['mode'] = 0
        tuning.save()
        assert 'mode: 1 ' in tuning.path.read_text()

    def test_an_older_saved_file_gains_new_settings(self, tuning):
        # A file saved before `kp` existed, then the setting was tuned.
        tuning.path.parent.mkdir(parents=True)
        tuning.path.write_text('speed: 0.2\n')
        tuning.load()
        tuning.update({'kp': 1.5})
        tuning.save()
        assert tuning.path.read_text() == 'speed: 0.2\nkp: 1.5\ncount: 3\n'


class TestRunLog:
    @pytest.fixture
    def log(self, tmp_path, clock):
        marks = labkit.Marks()
        log = labkit.RunLog('lab', 't,error', marks, maxlen=5)
        log.dir = tmp_path / 'logs'
        return log

    def test_thin_keeps_the_newest_row(self, log):
        log.hist = labkit.deque(([i, 0.0] for i in range(1000)), maxlen=1000)
        sent = log.thin()
        assert len(sent) < 1000 and sent[-1] is log.hist[-1]
        log.hist = labkit.deque(([i, 0.0] for i in range(50)), maxlen=50)
        assert log.thin() == list(log.hist)

    def test_save_numbers_the_logs_and_keeps_marks_in_range(self, log, clock):
        log._marks.add('kp 1')
        clock[0] = 20.0
        log.rows.extend(['15.0,0.1\n', '20.0,0.2\n'])
        log._marks.add('speed 0.2')
        assert log.save() == 'Log1.csv'
        assert log.save() == 'Log2.csv'
        text = (log.dir / 'Log1.csv').read_text()
        assert text == 't,error\n# mark,20.0,speed 0.2\n15.0,0.1\n20.0,0.2\n'
        (log.dir / 'notes.csv').write_text('')
        assert log.names() == ['notes.csv', 'Log1.csv', 'Log2.csv']

    def test_read_stays_in_the_folder(self, log):
        log.dir.mkdir()
        (log.dir / 'Log1.csv').write_text('t\n')
        assert log.read('/logs/Log1.csv') == b't\n'
        assert log.read('/logs/../secret.csv') is None
        assert log.read('/logs/Log9.csv') is None

    def test_rows_hold_the_window(self, log):
        log.rows.extend(f'{i},0\n' for i in range(9))
        assert len(log.rows) == 5 and log.rows[0] == '4,0\n'
