"""Tests for racecar_neo_ros2_driver.data_root: where the car's data goes."""

from pathlib import Path

import pytest
from racecar_neo_ros2_driver import data_root


@pytest.fixture
def fresh(monkeypatch):
    data_root.reset()
    yield
    data_root.reset()


class TestResolve:
    def test_nvme_when_mounted_and_writable(self, tmp_path, monkeypatch):
        mount = tmp_path / 'nvme'
        (mount / 'data').mkdir(parents=True)
        monkeypatch.setattr(data_root.os.path, 'ismount', lambda p: Path(p) == mount)
        assert data_root.resolve(mount, tmp_path / 'old') == (mount / 'data', '')

    def test_read_only_falls_back(self, tmp_path, monkeypatch):
        mount = tmp_path / 'nvme'
        (mount / 'data').mkdir(parents=True)
        monkeypatch.setattr(data_root.os.path, 'ismount', lambda p: Path(p) == mount)
        monkeypatch.setattr(data_root, 'writable', lambda p: False)
        root, warning = data_root.resolve(mount, tmp_path / 'old')
        assert root == data_root.fallback() and 'read-only' in warning

    def test_old_mount_names_the_fix(self, tmp_path, monkeypatch):
        old = tmp_path / 'data'
        monkeypatch.setattr(data_root.os.path, 'ismount', lambda p: Path(p) == old)
        root, warning = data_root.resolve(tmp_path / 'nvme', old)
        assert root == data_root.fallback() and 'racecar disk setup' in warning

    def test_no_drive(self, tmp_path):
        root, warning = data_root.resolve(tmp_path / 'nvme', tmp_path / 'old')
        assert root == Path.home() / 'data' and 'SD card' in warning

    def test_writable_checks_by_writing(self, tmp_path):
        assert data_root.writable(tmp_path)
        assert not data_root.writable(tmp_path / 'missing')


class TestCurrent:
    def test_env_first(self, fresh, monkeypatch, tmp_path):
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path))
        assert data_root.current() == (tmp_path, '')

    def test_run_file_next(self, fresh, monkeypatch, tmp_path):
        monkeypatch.delenv(data_root.DATA_ENV)
        run_file = tmp_path / 'data_root'
        run_file.write_text('/mnt/nvme/data\n')
        monkeypatch.setattr(data_root, 'RUN_FILE', run_file)
        assert data_root.current()[0] == Path('/mnt/nvme/data')

    def test_worked_out_once(self, fresh, monkeypatch, tmp_path):
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path / 'a'))
        first = data_root.current()
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path / 'b'))
        assert data_root.current() == first

    def test_data_dir_creates_the_folder(self, data_root_dir):
        assert data_root.data_dir('bags') == data_root_dir / 'bags'
        assert (data_root_dir / 'bags').is_dir()

    def test_on_nvme(self):
        assert data_root.on_nvme(Path('/mnt/nvme/data/bags'))
        assert not data_root.on_nvme(Path.home() / 'data')


def test_boot_service_writes_both_files(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(data_root, 'RUN_FILE', tmp_path / 'data_root')
    monkeypatch.setattr(data_root, 'RUN_ENV_FILE', tmp_path / 'environment')
    root = tmp_path / 'home_data'
    monkeypatch.setattr(data_root, 'resolve', lambda: (root, 'on the SD card'))
    monkeypatch.setattr('sys.argv', ['data_root.py', 'write'])
    assert data_root.main() == 0
    assert (tmp_path / 'data_root').read_text() == f'{root}\n'
    assert (tmp_path / 'environment').read_text() == f'RACECAR_DATA_DIR={root}\n'
    assert root.is_dir()
    assert 'SD card' in capsys.readouterr().out


def test_units_and_shells_get_the_variable():
    scripts = Path(__file__).resolve().parent.parent / 'scripts'
    for unit in ('teleop', 'dashboard', 'jupyter', 'mapview', 'autonomy'):
        text = (scripts / f'racecar-{unit}.service').read_text()
        assert 'EnvironmentFile=-/run/racecar/environment' in text, unit
    service = (scripts / 'racecar-data.service').read_text()
    assert 'data_root.py write' in service and 'RuntimeDirectoryPreserve=yes' in service
    assert 'RACECAR_DATA_DIR' in (scripts / 'profile.d-racecar-data.sh').read_text()
    assert 'racecar-data.service' in (scripts / 'setup_services.sh').read_text()
    assert 'export RACECAR_DATA_DIR' in (scripts / 'racecar-tool.sh').read_text()


def test_session_problem(data_root_dir, monkeypatch):
    assert data_root.session_problem() == ''
    monkeypatch.setattr(data_root, 'writable', lambda p: False)
    assert 'dropped out' in data_root.session_problem()
