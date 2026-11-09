"""scripts/student_library.py: pinned library in /opt, labs from main, backups, selection."""

import argparse
import importlib.util
from pathlib import Path
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'


@pytest.fixture
def sl():
    spec = importlib.util.spec_from_file_location(
        'student_library', SCRIPTS / 'student_library.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeRun:
    """Stands in for git and sudo: clones make the folders a real clone would."""

    def __init__(self, reachable=True, describe='v0.9.0-bot'):
        self.calls = []
        self.reachable = reachable
        self.describe = describe

    def __call__(self, argv, **kwargs):
        self.calls.append(list(argv))
        args = argv[1:] if argv[0] == 'sudo' else argv
        rc, out = 0, ''
        if args[:2] == ['git', 'ls-remote']:
            rc = 0 if self.reachable else 2
        elif args[:2] == ['git', 'clone']:
            dest = Path(args[-1])
            (dest / '.git').mkdir(parents=True)
            if 'library' in args[-2]:
                (dest / 'library').mkdir()
                (dest / 'library' / 'racecar_core.py').write_text('')
            else:
                (dest / 'labs').mkdir()
        elif args[:2] == ['git', '-C']:
            out = self.describe + '\n'
        elif args[0] == 'mkdir':
            Path(args[-1]).mkdir(parents=True, exist_ok=True)
        elif args[0] == 'mv':
            Path(args[-2]).rename(args[-1])
        return subprocess.CompletedProcess(argv, rc, out, '')


PIN = {
    'library_repo': 'https://github.com/MITRacecarNeo/racecar-neo-library',
    'tag': 'v0.9.2-bot',
    'labs_repo': 'https://github.com/MITRacecarNeo/racecar-neo-outreach-labs',
    'branch': 'main',
}


def quiet(_line):
    pass


class TestPin:
    def test_shipped_pin(self, sl):
        pin = sl.load_pin()
        assert pin['tag'].endswith('-bot') and pin['branch'] == 'main'
        assert pin['library_repo'].endswith('racecar-neo-library')
        assert pin['labs_repo'].endswith('racecar-neo-outreach-labs')

    def test_installed_with_the_config(self):
        assert "glob.glob('config/*.yaml')" in (ROOT / 'setup.py').read_text()


class TestBackupName:
    def test_format(self, sl):
        now = time.mktime((2026, 11, 8, 19, 30, 5, 0, 0, -1))
        assert sl.backup_name('labs', 'v0.9.0-bot', now) == (
            '20261108_193005_racecar-neo-labs-v0.9.0-bot'
        )

    def test_odd_tag_is_made_safe(self, sl):
        assert sl.backup_name('library', 'a/b c', 0).endswith('racecar-neo-library-a-b-c')

    def test_plain_copy_is_untracked(self, sl, tmp_path):
        assert sl.describe(tmp_path, FakeRun()) == 'untracked'


class TestInstall:
    def test_library_cloned_read_only_and_linked(self, sl, tmp_path):
        run, root, jws = FakeRun(), tmp_path / 'opt', tmp_path / 'jws'
        lib = sl.install_library(PIN, root, run, quiet)
        assert lib == root / 'library-v0.9.2-bot' / 'library'
        clone = [c for c in run.calls if 'clone' in c][0]
        assert clone[0] == 'sudo' and '--branch' in clone and 'v0.9.2-bot' in clone
        assert any('compileall' in c for c in run.calls)
        sl.link_library(lib, jws, tmp_path / 'backups', 0, run, quiet)
        assert (jws / 'library').is_symlink() and (jws / 'library').resolve() == lib

    def test_installed_tag_is_not_cloned_again(self, sl, tmp_path):
        run, root = FakeRun(), tmp_path / 'opt'
        sl.install_library(PIN, root, run, quiet)
        run.calls.clear()
        sl.install_library(PIN, root, run, quiet)
        assert run.calls == []

    def test_leftover_temp_is_refused_not_deleted(self, sl, tmp_path):
        root = tmp_path / 'opt'
        (root / '.library-v0.9.2-bot.new').mkdir(parents=True)
        with pytest.raises(RuntimeError, match='left from an earlier run'):
            sl.install_library(PIN, root, FakeRun(), quiet)

    def test_real_library_folder_moves_to_backups(self, sl, tmp_path):
        jws, backups = tmp_path / 'jws', tmp_path / 'backups'
        (jws / 'library').mkdir(parents=True)
        (jws / 'library' / 'mine.py').write_text('x')
        target = tmp_path / 'opt' / 'library'
        target.mkdir(parents=True)
        sl.link_library(target, jws, backups, 0, FakeRun(), quiet)
        moved = list(backups.iterdir())
        assert len(moved) == 1 and moved[0].name.endswith('racecar-neo-library-untracked')
        assert (moved[0] / 'mine.py').read_text() == 'x'

    def test_existing_labs_move_to_backups_then_main_is_cloned(self, sl, tmp_path):
        jws, backups = tmp_path / 'jws', tmp_path / 'backups'
        (jws / 'labs' / '.git').mkdir(parents=True)
        (jws / 'labs' / 'grand_prix.py').write_text('student work')
        run = FakeRun(describe='v0.9.0-bot-3-gabc123')
        sl.install_labs(PIN, jws, backups, 0, run, quiet)
        moved = list(backups.iterdir())
        assert moved[0].name.endswith('racecar-neo-labs-v0.9.0-bot-3-gabc123')
        assert (moved[0] / 'grand_prix.py').read_text() == 'student work'
        assert (jws / 'labs' / 'labs').is_dir()
        clone = [c for c in run.calls if 'clone' in c][0]
        assert clone[0] == 'git' and clone[clone.index('--branch') + 1] == 'main'

    def test_failed_labs_clone_keeps_the_old_folder(self, sl, tmp_path):
        jws = tmp_path / 'jws'
        (jws / 'labs').mkdir(parents=True)

        def failing(argv, **kwargs):
            return subprocess.CompletedProcess(argv, 128, '', '')

        with pytest.raises(RuntimeError):
            sl.install_labs(PIN, jws, tmp_path / 'backups', 0, failing, quiet)
        assert (jws / 'labs').is_dir() and not (tmp_path / 'backups').exists()

    def test_selects_racecar_neo_only_when_nothing_is_selected(self, sl, tmp_path):
        pth = tmp_path / 'site' / 'racecar_student.pth'
        sl.select_if_unset(tmp_path / 'jws' / 'library', pth, quiet)
        assert pth.read_text().strip() == str(tmp_path / 'jws' / 'library')
        pth.write_text('/home/racecar/jupyter_ws/other/library\n')
        sl.select_if_unset(tmp_path / 'jws' / 'library', pth, quiet)
        assert 'other' in pth.read_text()


class TestCommand:
    def _args(self, **kw):
        return argparse.Namespace(
            **{'tag': None, 'no_labs': False, 'labs_if_missing': False, **kw}
        )

    def _home(self, sl, tmp_path, monkeypatch):
        monkeypatch.setattr(sl.Path, 'home', classmethod(lambda cls: tmp_path))
        monkeypatch.setattr(sl, 'OPT_ROOT', tmp_path / 'opt')
        monkeypatch.setattr(sl.site, 'getusersitepackages', lambda: str(tmp_path / 'site'))

    def test_offline_names_the_command_and_changes_nothing(self, sl, tmp_path, monkeypatch):
        self._home(sl, tmp_path, monkeypatch)
        run = FakeRun(reachable=False)
        assert sl.cmd_install(self._args(), run) == 3
        assert not (tmp_path / 'opt').exists() and not (tmp_path / 'jupyter_ws').exists()

    def test_first_setup_keeps_existing_labs(self, sl, tmp_path, monkeypatch):
        self._home(sl, tmp_path, monkeypatch)
        labs = tmp_path / 'jupyter_ws' / 'racecar-neo' / 'labs'
        labs.mkdir(parents=True)
        run = FakeRun()
        assert sl.cmd_install(self._args(labs_if_missing=True), run) == 0
        assert not (tmp_path / 'backups').exists()
        assert not any('outreach-labs' in ' '.join(c) for c in run.calls)

    def test_install_then_status(self, sl, tmp_path, monkeypatch):
        self._home(sl, tmp_path, monkeypatch)
        assert sl.cmd_install(self._args(), FakeRun()) == 0
        jws = tmp_path / 'jupyter_ws' / 'racecar-neo'
        lines = sl.status_lines(sl.load_pin(), tmp_path / 'opt', jws, FakeRun())
        assert 'installed v0.9.2-bot' in lines[1]
        assert lines[2].endswith('library-v0.9.2-bot/library')
        assert (tmp_path / 'site' / 'racecar_student.pth').is_file()


class TestDriverLookup:
    def test_pinned_library_found(self, tmp_path, monkeypatch):
        from racecar_neo_ros2_driver import paths

        monkeypatch.setattr(paths, 'config_dir', lambda: ROOT / 'config')

        folder = tmp_path / 'library-v0.9.2-bot' / 'library'
        folder.mkdir(parents=True)
        assert paths.student_library(tmp_path) is None
        (folder / 'racecar_core.py').write_text('')
        assert paths.student_library(tmp_path) == folder


def test_setup_runs_it_without_replacing_labs():
    text = (SCRIPTS / 'setup_jupyter.sh').read_text()
    assert 'student_library.py" install --labs-if-missing' in text
    assert 'racecar library install' in text


def test_racecar_library_install_dispatches():
    text = (SCRIPTS / 'racecar-tool.sh').read_text()
    assert 'python3 "$pkg_dir/scripts/student_library.py" install "$@"' in text
