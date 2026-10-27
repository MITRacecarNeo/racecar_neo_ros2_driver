"""Unit tests for scripts/battery_alert.py: messages, schedule, terminal and tmux delivery."""

import os
from types import SimpleNamespace

from conftest import load_script

ba = load_script('battery_alert')


class TestMessages:
    def test_warn(self):
        assert (
            ba.warn_message(14.4).text == 'Low battery: 14 min left. Charge or swap the battery.'
        )
        assert ba.warn_message(None).text == 'Low battery: Charge or swap the battery.'

    def test_countdown(self):
        start = ba.countdown_message(60, start=True)
        assert start.text == 'Low battery: shutting down in 60 s.' and start.bell
        tick = ba.countdown_message(7)
        assert tick.text == 'Shutting down in 7 s.' and not tick.bell

    def test_final(self):
        assert ba.final_message().text == 'Turning the car off now. Turn off the power switch.'

    def test_tty_bytes_one_red_line(self):
        data = ba.tty_bytes(ba.canceled_message()).decode()
        assert data == f'\r\n{ba.RED}Shutdown canceled.{ba.RESET}\r\n\a'
        assert not ba.tty_bytes(ba.countdown_message(5)).decode().endswith('\a')


def _no_tmux(argv, **_kwargs):
    return SimpleNamespace(returncode=1, stdout='', stderr='no server')


class _Recorder:
    def __init__(self):
        self.writes = []

    def __call__(self, path, msg):
        self.writes.append((path, msg))
        return True


def _announcer(tmp_path, run=_no_tmux):
    for name in ('0', '1'):
        (tmp_path / name).write_text('')
    rec = _Recorder()
    ann = ba.Announcer(run=run, write=rec, uid=os.getuid(), globs=[str(tmp_path / '[0-9]')])
    return ann, rec


class TestSchedule:
    def test_countdown_marks(self, tmp_path):
        ann, rec = _announcer(tmp_path)
        shown = []
        shown += ann.update(['countdown floor'], 'countdown', 60.0, 2.0, 0.0)
        for t in range(1, 60):
            shown += ann.update([], 'countdown', 60.0, 0.0, float(t))
        shown += ann.update(['shutdown'], 'off', 60.0, 0.0, 60.0)
        seconds = [m.text.split()[-2] for m in shown if m.kind == 'countdown']
        assert seconds == ['60', '30', '20', '10', '9', '8', '7', '6', '5', '4', '3', '2', '1']
        assert shown[-1].kind == 'final'
        # Two terminals, one write each per message.
        assert len(rec.writes) == 2 * len(shown)

    def test_a_late_tick_announces_the_current_second(self, tmp_path):
        ann, _ = _announcer(tmp_path)
        ann.update(['countdown floor'], 'countdown', 60.0, 0.0, 0.0)
        ann.update([], 'countdown', 60.0, 0.0, 50.0)
        shown = ann.update([], 'countdown', 60.0, 0.0, 52.0)
        assert shown[0].text == 'Shutting down in 8 s.'

    def test_nothing_between_marks(self, tmp_path):
        ann, _ = _announcer(tmp_path)
        ann.update(['countdown floor'], 'countdown', 60.0, 0.0, 0.0)
        assert ann.update([], 'countdown', 60.0, 0.0, 15.0) == []

    def test_warn_and_cancel(self, tmp_path):
        ann, _ = _announcer(tmp_path)
        assert ann.update(['warn'], 'warn', None, 14.0, 0.0)[0].kind == 'warn'
        ann.update(['countdown estimate'], 'countdown', 60.0, 4.0, 0.0)
        assert ann.update(['canceled'], 'warn', None, 4.0, 5.0)[0].kind == 'canceled'
        assert ann.update([], 'warn', None, 4.0, 6.0) == []


class TestDelivery:
    def _tmux(self, clients, panes):
        calls = []

        def run(argv, **kwargs):
            calls.append(argv)
            socket = argv[2] if argv[1] == '-L' else None
            if 'list-clients' in argv:
                return SimpleNamespace(returncode=0, stdout='\n'.join(clients.get(socket, [])))
            if 'list-panes' in argv:
                return SimpleNamespace(returncode=0, stdout='\n'.join(panes.get(socket, [])))
            return SimpleNamespace(returncode=0, stdout='')

        return run, calls

    def test_tmux_ttys_get_the_status_line_not_writes(self, tmp_path):
        client, pane = str(tmp_path / '0'), str(tmp_path / '1')
        run, calls = self._tmux({None: [client]}, {'racecar-terminal': [pane]})
        ann, rec = _announcer(tmp_path, run=run)
        ann.update(['warn'], 'warn', None, 14.0, 0.0)
        assert rec.writes == []
        shown = [c for c in calls if 'display-message' in c]
        assert shown == [
            [
                'tmux',
                'display-message',
                '-c',
                client,
                '-d',
                str(ba.TMUX_SHOW_MS),
                '#[fg=red,bold]Low battery: 14 min left. Charge or swap the battery.',
            ]
        ]

    def test_terminal_app_socket(self, tmp_path):
        run, calls = self._tmux({'racecar-terminal': ['/dev/pts/99']}, {})
        ann, _ = _announcer(tmp_path, run=run)
        ann.update(['warn'], 'warn', None, 14.0, 0.0)
        shown = [c for c in calls if 'display-message' in c]
        assert shown[0][:3] == ['tmux', '-L', 'racecar-terminal']

    def test_every_countdown_second_reaches_tmux(self, tmp_path):
        run, calls = self._tmux({None: ['/dev/pts/99']}, {})
        ann, _ = _announcer(tmp_path, run=run)
        ann.update(['countdown floor'], 'countdown', 60.0, 0.0, 0.0)
        for t in range(1, 60):
            ann.update([], 'countdown', 60.0, 0.0, float(t))
        assert len([c for c in calls if 'display-message' in c]) == 13

    def test_hash_is_escaped_for_tmux(self):
        argv = ba.tmux_message_argv(None, '/dev/pts/9', ba.Message('x', 'car #2'))
        assert argv[-1] == '#[fg=red,bold]car ##2'

    def test_tmux_errors_are_ignored(self, tmp_path):
        def run(argv, **kwargs):
            raise OSError('no tmux')

        ann, rec = _announcer(tmp_path, run=run)
        ann.update(['warn'], 'warn', None, 14.0, 0.0)
        assert len(rec.writes) == 2


class TestTerminals:
    def test_own_ttys(self, tmp_path):
        (tmp_path / '3').write_text('')
        globs = [str(tmp_path / '[0-9]')]
        assert ba.own_ttys(os.getuid(), globs) == [str(tmp_path / '3')]
        assert ba.own_ttys(os.getuid() + 1, globs) == []

    def test_write_tty(self, tmp_path):
        path = tmp_path / '3'
        path.write_text('')
        assert ba.write_tty(str(path), ba.canceled_message())
        assert path.read_bytes() == ba.tty_bytes(ba.canceled_message())

    def test_write_tty_missing(self, tmp_path):
        assert not ba.write_tty(str(tmp_path / 'gone'), ba.canceled_message())


def _proc(tmp_path, entries):
    """Fake /proc: entries is [(pid, comm, tty_nr, tpgid)]."""
    proc = tmp_path / 'proc'
    for pid, comm, tty_nr, tpgid in entries:
        d = proc / str(pid)
        d.mkdir(parents=True)
        (d / 'stat').write_text(f'{pid} ({comm}) S 1 {pid} {pid} {tty_nr} {tpgid} 4194304 0\n')
        (d / 'comm').write_text(comm + '\n')
    return proc


class TestRedraw:
    PTS = '/dev/pts/5'

    def _stat(self, monkeypatch, rdev, mode=0o20620):
        real = os.stat
        fake = SimpleNamespace(st_rdev=rdev, st_mode=mode)
        monkeypatch.setattr(
            ba.os, 'stat', lambda p, *a, **k: fake if p == self.PTS else real(p, *a, **k)
        )

    def test_foreground_group_of_a_pts(self, tmp_path, monkeypatch):
        rdev = os.makedev(136, 5)
        self._stat(monkeypatch, rdev)
        tty_nr = (5 & 0xFF) | (136 << 8)
        proc = _proc(
            tmp_path, [(10, 'sshd', 0, -1), (20, 'bash', tty_nr, 30), (30, 'vim', tty_nr, 30)]
        )
        assert ba.foreground_group(self.PTS, proc) == 30
        assert ba.foreground_command(self.PTS, proc) == 'vim'

    def test_command_name_with_spaces(self, tmp_path, monkeypatch):
        self._stat(monkeypatch, os.makedev(136, 5))
        tty_nr = (5 & 0xFF) | (136 << 8)
        proc = _proc(tmp_path, [(40, 'my prog) x', tty_nr, 40)])
        assert ba.foreground_group(self.PTS, proc) == 40

    def test_not_a_character_device(self, tmp_path, monkeypatch):
        self._stat(monkeypatch, 0, mode=0o100644)
        assert ba.foreground_group(self.PTS, _proc(tmp_path, [])) is None

    def test_no_foreground_group(self, tmp_path, monkeypatch):
        self._stat(monkeypatch, os.makedev(136, 5))
        assert ba.foreground_group(self.PTS, _proc(tmp_path, [(10, 'sshd', 0, -1)])) is None

    def test_only_shells_are_nudged(self, monkeypatch):
        opened = []
        monkeypatch.setattr(ba, 'foreground_command', lambda path: 'vim')
        monkeypatch.setattr(ba.os, 'open', lambda *a: opened.append(a) or 3)
        ba.redraw(self.PTS)
        assert opened == []

    def test_shell_width_nudged_and_restored(self, monkeypatch):
        sizes = []
        monkeypatch.setattr(ba, 'foreground_command', lambda path: 'bash')
        monkeypatch.setattr(ba.os, 'open', lambda *a: 3)
        monkeypatch.setattr(ba.os, 'close', lambda fd: None)
        monkeypatch.setattr(ba.time, 'sleep', lambda s: None)
        start = ba.struct.pack('HHHH', 24, 80, 0, 0)

        def ioctl(fd, op, arg):
            if op == ba.termios.TIOCGWINSZ:
                return start
            sizes.append(ba.struct.unpack('HHHH', arg)[:2])
            return arg

        monkeypatch.setattr(ba.fcntl, 'ioctl', ioctl)
        ba.redraw(self.PTS)
        assert sizes == [(24, 81), (24, 80)]
