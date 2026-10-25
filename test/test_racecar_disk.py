"""Tests for scripts/racecar_disk.py: refusals, fstab, and the setup flows, with fake commands."""

from datetime import datetime
import json
from pathlib import Path
import subprocess

from conftest import load_script
import pytest

UUID = '99315696-4294-4362-a0ad-1a28cbfcb761'
NOW = datetime(2026, 10, 24, 13, 0, 0)
FSTAB = (
    '# /etc/fstab\n'
    'LABEL=writable  /  ext4  discard,errors=remount-ro  0  1\n'
    'LABEL=system-boot  /boot/firmware  vfat  defaults  0  1\n'
    f'UUID={UUID}  /data  ext4  defaults,noatime,nofail,x-systemd.device-timeout=10  0  2\n'
)


@pytest.fixture(scope='module')
def disk():
    return load_script('racecar_disk')


def lsblk(children=(), serial='25475480ECE6'):
    return json.dumps(
        {
            'blockdevices': [
                {
                    'name': 'nvme0n1',
                    'path': '/dev/nvme0n1',
                    'type': 'disk',
                    'size': 500107862016,
                    'model': 'CT500P310SSD8',
                    'serial': serial,
                    'children': list(children),
                }
            ]
        }
    )


def ours(mounts=(None,)):
    return {
        'name': 'nvme0n1p1',
        'path': '/dev/nvme0n1p1',
        'type': 'part',
        'fstype': 'ext4',
        'label': 'racecar-data',
        'uuid': UUID,
        'mountpoints': list(mounts),
    }


class FakeRun:
    """Answers the read commands; records every command run."""

    def __init__(self, lsblk_json, root_src='/dev/mmcblk0p2', active=(), wipefs=''):
        self.lsblk_json, self.root_src = lsblk_json, root_src
        self.active, self.wipefs = set(active), wipefs
        self.calls = []
        self.inputs = {}

    def __call__(self, argv, **kw):
        argv = list(argv)
        self.calls.append(argv)
        if 'input' in kw:
            self.inputs[tuple(argv)] = kw['input']
        out = ''
        if argv[0] == 'lsblk':
            out = self.lsblk_json
        elif argv[:3] == ['findmnt', '-no', 'SOURCE']:
            out = self.root_src
        elif argv[:2] == ['systemctl', 'is-active']:
            out = 'active' if argv[2] in self.active else 'inactive'
        elif argv[-2:] == ['stop', argv[-1]] and argv[0] == 'systemctl':
            self.active.discard(argv[-1])
        elif argv[:3] == ['sudo', 'wipefs', '-n']:
            out = self.wipefs
        elif argv[:2] == ['sudo', 'blkid']:
            out = 'new-uuid\n'
        elif argv[0] == 'du':
            out = '1000\t/x\n'
        return subprocess.CompletedProcess(argv, 0, out, '')

    def sudo(self):
        return [c[1:] for c in self.calls if c[0] == 'sudo']


@pytest.fixture
def env(disk, monkeypatch, tmp_path):
    fstab = tmp_path / 'fstab'
    fstab.write_text(FSTAB)
    mount = tmp_path / 'mnt_nvme'
    (mount / 'data').mkdir(parents=True)
    monkeypatch.setenv('HOME', str(tmp_path / 'home'))
    (tmp_path / 'home' / 'jupyter_ws').mkdir(parents=True)
    monkeypatch.setattr(disk, 'FSTAB', fstab)
    monkeypatch.setattr(disk, 'MOUNT', mount)
    monkeypatch.setattr(disk, 'mapping_now', lambda: False)
    monkeypatch.setattr(disk, 'recording_now', lambda: False)
    monkeypatch.setattr(disk.data_root, 'fallback', lambda: tmp_path / 'home_data')
    monkeypatch.setattr(disk.data_root, 'resolve', lambda *a: (mount / 'data', ''))
    return fstab, mount


class TestPure:
    def test_parse_lsblk(self, disk):
        drive = disk.parse_lsblk(lsblk([ours(['/data'])]))
        assert drive.serial == '25475480ECE6' and drive.size == 500107862016
        assert drive.ours().uuid == UUID and drive.mounts == ['/data']

    def test_other_filesystems_are_not_ours(self, disk):
        other = {**ours(), 'label': 'backup'}
        assert disk.parse_lsblk(lsblk([other])).ours() is None
        assert disk.parse_lsblk(lsblk([ours(), ours()])).ours() is None

    def test_rewrite_moves_the_line_and_keeps_the_rest(self, disk):
        text = disk.rewrite_fstab(FSTAB, UUID)
        lines = text.splitlines()
        assert lines[:3] == FSTAB.splitlines()[:3]
        assert lines[3] == f'UUID={UUID}  /mnt/nvme  ext4  {disk.MOUNT_OPTIONS}  0  2'
        assert len(lines) == 4 and 'device-timeout=45' in lines[3]

    def test_rewrite_is_idempotent_and_drops_old_uuids(self, disk):
        once = disk.rewrite_fstab(FSTAB, UUID)
        assert disk.rewrite_fstab(once, UUID) == once
        fresh = disk.rewrite_fstab(once, 'new', old_uuids=[UUID])
        assert UUID not in fresh and fresh.count('/mnt/nvme') == 1

    def test_serial_must_match(self, disk):
        assert disk.serial_matches(' 25475480ece6 ', '25475480ECE6')
        assert not disk.serial_matches('', '')
        assert not disk.serial_matches('25475480ECE7', '25475480ECE6')


class TestSetup:
    def test_moves_the_old_mount_without_erasing(self, disk, env):
        fstab, mount = env
        run = FakeRun(lsblk([ours(['/data'])]), active=['racecar-autonomy.service'])
        answers = iter(['y', 'n'])
        assert disk.setup(run, lambda s: None, lambda p: next(answers), NOW) == 0
        sudo = run.sudo()
        assert ['sudo', 'umount', '/data'] in run.calls
        assert not any(c[0] in ('mkfs.ext4', 'wipefs', 'sgdisk') for c in sudo)
        new = run.inputs[('sudo', 'tee', str(fstab))]
        assert '/mnt/nvme' in new and ' /data ' not in new
        assert ['systemctl', 'restart', 'racecar-data.service'] in sudo
        stops = [c for c in run.calls if c[:3] == ['systemctl', '--no-ask-password', 'stop']]
        assert stops == [['systemctl', '--no-ask-password', 'stop', 'racecar-autonomy.service']]

    def test_declined_changes_nothing(self, disk, env):
        run = FakeRun(lsblk([ours(['/data'])]))
        assert disk.setup(run, lambda s: None, lambda p: 'n', NOW) == 1
        assert run.sudo() == []

    def test_blank_drive_is_formatted(self, disk, env):
        run = FakeRun(lsblk([]))
        answers = iter(['y', 'n'])
        assert disk.setup(run, lambda s: None, lambda p: next(answers), NOW) == 0
        tools = [c[0] for c in run.sudo()]
        assert 'mkfs.ext4' in tools and 'sgdisk' in tools
        assert 'UUID=new-uuid' in run.inputs[('sudo', 'tee', str(env[0]))]

    def test_refuses_a_drive_with_other_data(self, disk, env):
        other = {**ours(), 'label': 'photos', 'fstype': 'ntfs'}
        with pytest.raises(disk.DiskError, match='racecar disk format'):
            disk.setup(FakeRun(lsblk([other])), lambda s: None, lambda p: 'y', NOW)

    def test_refuses_signatures_on_a_blank_looking_drive(self, disk, env):
        run = FakeRun(lsblk([]), wipefs='DEVICE OFFSET TYPE\nnvme0n1 0x1fe dos')
        with pytest.raises(disk.DiskError, match='signatures'):
            disk.setup(run, lambda s: None, lambda p: 'y', NOW)

    def test_refuses_the_system_drive(self, disk, env):
        run = FakeRun(lsblk([]), root_src='/dev/nvme0n1p2')
        with pytest.raises(disk.DiskError, match='holds the system'):
            disk.setup(run, lambda s: None, lambda p: 'y', NOW)

    def test_refuses_while_mapping(self, disk, env, monkeypatch):
        monkeypatch.setattr(disk, 'mapping_now', lambda: True)
        run = FakeRun(lsblk([ours(['/data'])]), active=['racecar-autonomy.service'])
        with pytest.raises(disk.DiskError, match='mapping'):
            disk.setup(run, lambda s: None, lambda p: 'y', NOW)

    def test_copies_home_data_with_renames(self, disk, env, tmp_path):
        home = tmp_path / 'home_data'
        (home / 'maps' / 'room').mkdir(parents=True)
        (home / 'camlabel').mkdir()
        run = FakeRun(lsblk([ours([str(env[1])])]))
        assert disk.setup(run, lambda s: None, lambda p: 'y', NOW) == 0
        links = [c for c in run.calls if c[:2] == ['ln', '-sfn']]
        assert [c[-1] for c in links] == [
            str(tmp_path / 'home' / 'jupyter_ws' / 'data'),
            str(tmp_path / 'home' / 'jupyter_ws' / 'maps'),
        ]
        copies = [c for c in run.calls if c[:2] == ['cp', '-a']]
        assert [c[-1] for c in copies] == [
            str(env[1] / 'data' / 'datasets' / 'camlabel'),
            str(env[1] / 'data' / 'maps'),
        ]


class TestFormat:
    def test_wrong_serial_changes_nothing(self, disk, env):
        run = FakeRun(lsblk([ours([str(env[1])])]))
        assert disk.format_drive(run, lambda s: None, lambda p: 'WRONG', NOW) == 1
        assert not any(c[0] in ('mkfs.ext4', 'umount', 'wipefs') for c in run.sudo())

    def test_right_serial_formats_and_replaces_the_line(self, disk, env):
        fstab, mount = env
        fstab.write_text(disk.rewrite_fstab(FSTAB, UUID))
        run = FakeRun(lsblk([ours([str(mount)])]), active=['racecar-mapview.service'])
        assert disk.format_drive(run, lambda s: None, lambda p: '25475480ece6', NOW) == 0
        new = run.inputs[('sudo', 'tee', str(fstab))]
        assert UUID not in new and new.count('UUID=new-uuid') == 1
        assert ['sudo', 'umount', str(mount)] in run.calls

    def test_refuses_while_recording(self, disk, env, monkeypatch):
        monkeypatch.setattr(disk, 'recording_now', lambda: True)
        with pytest.raises(disk.DiskError, match='recording'):
            disk.format_drive(FakeRun(lsblk([ours()])), lambda s: None, lambda p: '', NOW)


class TestStatusAndData:
    def test_status_names_the_old_mount(self, disk, env):
        lines = []
        assert disk.status(FakeRun(lsblk([ours()])), lines.append, env[0]) == 0
        text = '\n'.join(lines)
        assert 'CT500P310SSD8' in text and '25475480ECE6' in text
        assert 'racecar disk setup' in text

    def test_no_drive(self, disk, env, monkeypatch):
        monkeypatch.setattr(disk, 'read_drive', lambda **kw: None)
        lines = []
        assert disk.status(FakeRun(''), lines.append) == 1
        assert lines[0].startswith('No NVMe drive')

    def test_data_path(self, disk, data_root_dir):
        lines = []
        assert disk.data_command(['path'], lines.append) == 0
        assert lines == [str(data_root_dir)]


def test_tool_runs_it():
    tool = (Path(__file__).resolve().parent.parent / 'scripts' / 'racecar-tool.sh').read_text()
    assert 'scripts/racecar_disk.py}" data "$@"' in tool
    assert 'compgen -W "status setup format check"' in tool
