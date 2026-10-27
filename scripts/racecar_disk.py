#!/usr/bin/env python3
"""
`racecar disk` and `racecar data`: the NVMe data drive and the data root.

    racecar data             data root, the device it is on, free space
    racecar data path        the data root only (for scripts)
    racecar disk [status]    drive model, serial, size, mount, fstab, health
    racecar disk setup       set up a blank drive, or move one mounted at /data,
                             at /mnt/nvme; then offer to copy ~/data onto it
    racecar disk format      erase a drive that has data on it (type its serial)
    racecar disk check       unmount, e2fsck, mount again

The OS stays on the SD card. The data root rule is in
racecar_neo_ros2_driver/data_root.py. Steps that change the drive or
/etc/fstab run through sudo, which may ask for a password.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
from typing import Any

from racecar_neo_ros2_driver import data_root

DEVICE = '/dev/nvme0n1'
LABEL = 'racecar-data'
MOUNT = data_root.NVME_MOUNT
OLD_MOUNT = data_root.OLD_MOUNT
FSTAB = Path('/etc/fstab')
# The reference car's drive took 30 s to answer its first read at one boot;
# a shorter wait skipped the mount. nofail keeps a dead drive from stopping boot.
DEVICE_TIMEOUT_SEC = 45
MOUNT_OPTIONS = f'defaults,noatime,nofail,x-systemd.device-timeout={DEVICE_TIMEOUT_SEC}'
# Units that write to the data root; stopped before the drive is unmounted.
WRITERS = (
    'racecar-autonomy.service',
    'racecar-mapview.service',
    'racecar-webteleop.service',
    'racecar-linefollow.service',
    'racecar-wallfollow.service',
    'racecar-camlabel.service',
)
# ~/data folders whose place under the data root differs.
COPY_RENAMES = {'camlabel': 'datasets/camlabel'}

Run = Callable[..., subprocess.CompletedProcess]


class DiskError(Exception):
    """A refusal or a failed step; the message is for the user."""


@dataclass
class Partition:
    path: str
    fstype: str = ''
    label: str = ''
    uuid: str = ''
    mounts: list[str] = field(default_factory=list)


@dataclass
class Drive:
    path: str
    size: int
    model: str
    serial: str
    partitions: list[Partition]

    @property
    def mounts(self) -> list[str]:
        return [m for p in self.partitions for m in p.mounts]

    def ours(self) -> Partition | None:
        """Return the single ext4 partition labeled racecar-data, or None."""
        if len(self.partitions) == 1:
            part = self.partitions[0]
            if part.fstype == 'ext4' and part.label == LABEL:
                return part
        return None


def run_command(argv: Sequence[str], **kw: Any) -> subprocess.CompletedProcess:
    return subprocess.run(list(argv), capture_output=True, text=True, check=False, **kw)


def parse_lsblk(text: str) -> Drive | None:
    """Read `lsblk -J -b -o NAME,PATH,TYPE,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS,MODEL,SERIAL`."""
    devices = json.loads(text).get('blockdevices') or []
    if not devices:
        return None
    disk = devices[0]
    parts = [
        Partition(
            path=c.get('path') or '',
            fstype=c.get('fstype') or '',
            label=c.get('label') or '',
            uuid=c.get('uuid') or '',
            mounts=[m for m in (c.get('mountpoints') or []) if m],
        )
        for c in disk.get('children') or []
    ]
    return Drive(
        path=disk.get('path') or DEVICE,
        size=int(disk.get('size') or 0),
        model=(disk.get('model') or '').strip(),
        serial=(disk.get('serial') or '').strip(),
        partitions=parts,
    )


def read_drive(device: str = DEVICE, run: Run = run_command) -> Drive | None:
    if not Path(device).exists():
        return None
    r = run(
        [
            'lsblk',
            '-J',
            '-b',
            '-o',
            'NAME,PATH,TYPE,SIZE,FSTYPE,LABEL,UUID,MOUNTPOINTS,MODEL,SERIAL',
            device,
        ]
    )
    if r.returncode != 0:
        raise DiskError(f'lsblk failed: {r.stderr.strip()}')
    return parse_lsblk(r.stdout)


def fstab_line(uuid: str, mount: Path = MOUNT) -> str:
    return f'UUID={uuid}  {mount}  ext4  {MOUNT_OPTIONS}  0  2'


def rewrite_fstab(text: str, uuid: str, mount: Path = MOUNT, old_uuids: Sequence[str] = ()) -> str:
    """
    Return fstab with one line for the data drive at `mount`.

    Drops every line that mounts at `mount` or the old /data, or names `uuid`
    or one of `old_uuids`; keeps everything else as it was.
    """
    targets = {str(mount), str(OLD_MOUNT)}
    names = {f'UUID={u}' for u in (uuid, *old_uuids) if u}
    kept = []
    for line in text.splitlines():
        fields = line.split()
        if (
            fields
            and not fields[0].startswith('#')
            and (fields[0] in names or (len(fields) > 1 and fields[1] in targets))
        ):
            continue
        kept.append(line)
    while kept and not kept[-1].strip():
        kept.pop()
    return '\n'.join([*kept, fstab_line(uuid, mount)]) + '\n'


def fstab_entry(text: str, uuid: str) -> str:
    for line in text.splitlines():
        if line.split()[:1] == [f'UUID={uuid}']:
            return line.strip()
    return ''


def human(num: float) -> str:
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if num < 1000 or unit == 'TB':
            return f'{num:.0f} {unit}' if unit == 'B' else f'{num:.1f} {unit}'
        num /= 1000
    return ''


def serial_matches(typed: str, serial: str) -> bool:
    return bool(serial) and typed.strip().upper() == serial.strip().upper()


# -- what is running --


def unit_active(unit: str, run: Run = run_command) -> bool:
    return run(['systemctl', 'is-active', unit]).stdout.strip() in ('active', 'activating')


def mapping_now() -> bool:
    from map_control import read_mode_file

    return read_mode_file().get('AUTONOMY_MODE') == 'mapping'


def recording_now() -> bool:
    from racecar_log import STATE_FILE

    try:
        state = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        return False
    pid = state.get('pid')
    return bool(pid) and Path(f'/proc/{pid}').exists()


def refuse_while_busy(run: Run = run_command) -> None:
    if unit_active('racecar-autonomy.service', run) and mapping_now():
        raise DiskError('mapping is running. Save and stop it in the map viewer first.')
    if recording_now():
        raise DiskError('a recording is running. Stop it first: racecar log stop')


# -- steps --


class Steps:
    """Privileged steps, run through sudo; each prints what it does."""

    def __init__(self, run: Run = run_command, out: Callable[[str], None] = print) -> None:
        self.run = run
        self.out = out

    def sudo(self, *argv: str, input_text: str | None = None) -> str:
        self.out(f'  sudo {shlex.join(argv)}')
        r = (
            self.run(['sudo', *argv], input=input_text)
            if input_text is not None
            else self.run(['sudo', *argv])
        )
        if r.returncode != 0:
            raise DiskError(f'{argv[0]} failed: {(r.stderr or r.stdout).strip()}')
        return r.stdout

    def stop_writers(self) -> list[str]:
        stopped = []
        for unit in WRITERS:
            if unit_active(unit, self.run):
                self.run(['systemctl', '--no-ask-password', 'stop', unit])
                if unit_active(unit, self.run):
                    self.sudo('systemctl', 'stop', unit)
                stopped.append(unit)
        if stopped:
            self.out(f'Stopped {", ".join(stopped)}.')
        return stopped

    def unmount(self, mount: str) -> None:
        r = self.run(['sudo', 'umount', mount])
        if r.returncode != 0:
            users = self.run(['sudo', 'fuser', '-vm', mount])
            raise DiskError(
                f'{mount} is busy; close what is using it and retry.\n'
                f'{(users.stderr or users.stdout).strip()}'
            )
        self.out(f'  sudo umount {mount}')

    def write_fstab(self, text: str, now: datetime) -> None:
        backup = f'{FSTAB}.bak.{now:%Y%m%d_%H%M%S}'
        self.sudo('cp', str(FSTAB), backup)
        self.sudo('tee', str(FSTAB), input_text=text)
        self.sudo('systemctl', 'daemon-reload')
        self.out(f'Updated {FSTAB}; the old one is {backup}.')

    def mount_and_own(self) -> None:
        self.sudo('mkdir', '-p', str(MOUNT))
        self.sudo('mount', str(MOUNT))
        self.sudo('mkdir', '-p', str(MOUNT / 'data'))
        user = os.environ.get('USER') or 'racecar'
        self.sudo('chown', f'{user}:{user}', str(MOUNT / 'data'))

    def partition_and_format(self, drive: Drive) -> str:
        self.sudo('wipefs', '--all', drive.path)
        self.sudo('sgdisk', '--zap-all', drive.path)
        self.sudo(
            'sgdisk', '--new=1:0:0', '--typecode=1:8300', f'--change-name=1:{LABEL}', drive.path
        )
        self.sudo('partprobe', drive.path)
        self.run(['udevadm', 'settle'])
        part = f'{drive.path}p1'
        # -m 0: no root reserve; 5% of a data-only drive is wasted space.
        self.sudo('mkfs.ext4', '-F', '-L', LABEL, '-m', '0', part)
        uuid = self.sudo('blkid', '-s', 'UUID', '-o', 'value', part).strip()
        if not uuid:
            raise DiskError(f'could not read the UUID of {part}')
        return uuid

    def refresh_data_root(self) -> None:
        self.sudo('systemctl', 'restart', 'racecar-data.service')

    def link_jupyter(self, root: Path, ws: Path | None = None) -> None:
        """Point ~/jupyter_ws/data and maps at the new root; a real folder there is left alone."""
        ws = ws or Path.home() / 'jupyter_ws'
        if not ws.is_dir():
            return
        for name, target in (('data', root), ('maps', root / 'maps')):
            link = ws / name
            if link.is_symlink() or not link.exists():
                target.mkdir(parents=True, exist_ok=True)
                self.run(['ln', '-sfn', str(target), str(link)])
                self.out(f'Linked {link} -> {target}')


# -- commands --


def data_command(args: Sequence[str], out: Callable[[str], None] = print) -> int:
    root, warning = data_root.current()
    if list(args) == ['path']:
        out(str(root))
        return 0
    if args:
        raise DiskError(f'unknown: racecar data {" ".join(args)}')
    where = 'NVMe' if data_root.on_nvme(root) else 'SD card'
    out(f'Data root: {root} ({where})')
    if root.is_dir():
        usage = shutil.disk_usage(root)
        out(f'Free: {human(usage.free)} of {human(usage.total)}')
    if warning:
        out(f'Warning: {warning}')
    return 0


def health_lines(device: str, run: Run = run_command) -> list[str]:
    """Return drive health from `nvme smart-log` when it can be read without a password."""
    if shutil.which('nvme') is None:
        return ['Health: install nvme-cli to read it (sudo apt install nvme-cli).']
    controller = re.sub(r'n\d+$', '', device)
    r = run(['sudo', '-n', 'nvme', 'smart-log', '-o', 'json', controller])
    if r.returncode != 0:
        return [f'Health: sudo nvme smart-log {device}']
    try:
        log = json.loads(r.stdout)
    except ValueError:
        return ['Health: unreadable']
    kelvin = log.get('temperature') or 0
    return [
        f'Health: {kelvin - 273 if kelvin > 200 else kelvin} C, '
        f'{log.get("percent_used", "?")}% used, '
        f'{log.get("media_errors", "?")} media errors, '
        f'{log.get("unsafe_shutdowns", "?")} unsafe shutdowns',
    ]


def status(run: Run = run_command, out: Callable[[str], None] = print, fstab: Path = FSTAB) -> int:
    drive = read_drive(run=run)
    if drive is None:
        out(f'No NVMe drive at {DEVICE}.')
        data_command([], out)
        return 1
    out(
        f'Drive: {drive.model or "unknown model"}, serial {drive.serial or "unknown"}, '
        f'{human(drive.size)}'
    )
    part = drive.ours()
    for p in drive.partitions:
        mounted = ', '.join(p.mounts) or 'not mounted'
        out(f'  {p.path}: {p.fstype or "no filesystem"} {p.label} ({mounted})')
    if not drive.partitions:
        out('  No partitions: blank. Run racecar disk setup.')
    if part is not None:
        try:
            entry = fstab_entry(fstab.read_text(), part.uuid)
        except OSError:
            entry = ''
        out(f'fstab: {entry or "no line for this drive"}')
        if str(OLD_MOUNT) in part.mounts or f' {OLD_MOUNT} ' in f' {entry} ':
            out(f'Set up at {OLD_MOUNT}; run racecar disk setup to move it to {MOUNT}.')
    for line in health_lines(drive.path, run):
        out(line)
    data_command([], out)
    return 0


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except EOFError:
        return ''


def copy_home_data(
    steps: Steps, home_data: Path, root: Path, ask_fn: Callable[[str], str] = ask
) -> None:
    """Copy ~/data to the new root, then compare sizes; ~/data is left in place."""
    if not home_data.is_dir() or not any(home_data.iterdir()):
        return
    if ask_fn(f'Copy {home_data} to {root}? {home_data} is left in place. [y/N] ').lower() != 'y':
        return
    for item in sorted(home_data.iterdir()):
        dest = root / COPY_RENAMES.get(item.name, item.name)
        dest.parent.mkdir(parents=True, exist_ok=True)
        # -T copies into dest whether or not it exists; -n keeps files already there.
        r = steps.run(['cp', '-a', '-n', '-T', str(item), str(dest)])
        if r.returncode != 0:
            raise DiskError(f'copying {item} failed: {r.stderr.strip()}')
    src = steps.run(['du', '-sb', str(home_data)]).stdout.split()[:1]
    dst = steps.run(['du', '-sb', str(root)]).stdout.split()[:1]
    steps.out(
        f'Copied: {home_data} {human(float(src[0] if src else 0))}, '
        f'{root} now {human(float(dst[0] if dst else 0))}. '
        f'Clear {home_data} by hand once you have checked the copy.'
    )


def move_old_contents(steps: Steps) -> None:
    """Move what was at the top of the old /data mount (bags) into data/bags."""
    top = [p for p in MOUNT.iterdir() if p.name not in ('lost+found', 'data')]
    if not top:
        return
    bags = MOUNT / 'data' / 'bags'
    bags.mkdir(parents=True, exist_ok=True)
    for item in top:
        steps.sudo('mv', '-n', str(item), str(bags) + '/')
    user = os.environ.get('USER') or 'racecar'
    steps.sudo('chown', '-R', f'{user}:{user}', str(bags))
    steps.out(f'Moved {len(top)} item(s) from the old {OLD_MOUNT} into {bags}.')


def setup(
    run: Run = run_command,
    out: Callable[[str], None] = print,
    ask_fn: Callable[[str], str] = ask,
    now: datetime | None = None,
) -> int:
    steps = Steps(run, out)
    now = now or datetime.now()
    drive = read_drive(run=run)
    if drive is None:
        raise DiskError(f'no NVMe drive at {DEVICE}.')
    root_src = run(['findmnt', '-no', 'SOURCE', '/']).stdout.strip()
    if root_src.startswith(drive.path):
        raise DiskError(f'{drive.path} holds the system. Refusing.')
    part = drive.ours()

    if part is not None and str(MOUNT) in part.mounts:
        out(f'{drive.path} is set up at {MOUNT}.')
    elif part is not None:
        out(
            f'{drive.path} ({drive.model}, {human(drive.size)}) holds a racecar-data '
            f'filesystem, mounted at {", ".join(part.mounts) or "nothing"}.'
        )
        if ask_fn(f'Move it to {MOUNT}? Nothing is erased. [y/N] ').lower() != 'y':
            out('Nothing changed.')
            return 1
        refuse_while_busy(run)
        steps.stop_writers()
        for mount in part.mounts:
            steps.unmount(mount)
        steps.write_fstab(rewrite_fstab(FSTAB.read_text(), part.uuid), now)
        steps.mount_and_own()
        move_old_contents(steps)
    elif drive.partitions or drive.mounts:
        raise DiskError(f'{drive.path} holds other data. To erase it: racecar disk format')
    else:
        sigs = run(['sudo', 'wipefs', '-n', drive.path]).stdout.strip()
        if sigs:
            raise DiskError(
                f'{drive.path} carries signatures:\n{sigs}\n' 'To erase it: racecar disk format'
            )
        out(
            f'Blank drive: {drive.path}, {drive.model}, {human(drive.size)}, '
            f'serial {drive.serial}.'
        )
        if ask_fn(f'Set it up at {MOUNT}? [y/N] ').lower() != 'y':
            out('Nothing changed.')
            return 1
        uuid = steps.partition_and_format(drive)
        steps.write_fstab(rewrite_fstab(FSTAB.read_text(), uuid), now)
        steps.mount_and_own()

    steps.refresh_data_root()
    data_root.reset()
    root = data_root.resolve()[0]
    copy_home_data(steps, data_root.fallback(), root, ask_fn)
    steps.link_jupyter(root)
    out('Done. Reboot so every service writes to the new data root: sudo reboot')
    return 0


def describe_contents(mounts: Sequence[str], run: Run = run_command) -> list[str]:
    lines = []
    for mount in mounts:
        r = run(['du', '-sh', '--max-depth=1', mount])
        lines += [f'  {line}' for line in r.stdout.splitlines()[-8:]]
    return lines


def format_drive(
    run: Run = run_command,
    out: Callable[[str], None] = print,
    ask_fn: Callable[[str], str] = ask,
    now: datetime | None = None,
) -> int:
    steps = Steps(run, out)
    now = now or datetime.now()
    drive = read_drive(run=run)
    if drive is None:
        raise DiskError(f'no NVMe drive at {DEVICE}.')
    if run(['findmnt', '-no', 'SOURCE', '/']).stdout.strip().startswith(drive.path):
        raise DiskError(f'{drive.path} holds the system. Refusing.')
    refuse_while_busy(run)
    out(f'Drive: {drive.path}, {drive.model}, {human(drive.size)}, serial {drive.serial}')
    for line in describe_contents(drive.mounts, run) or ['  (not mounted; contents not shown)']:
        out(line)
    out('Formatting ERASES everything on this drive.')
    if not serial_matches(ask_fn('Type the serial shown above to continue: '), drive.serial):
        out('The serial does not match. Nothing changed.')
        return 1
    steps.stop_writers()
    for mount in drive.mounts:
        steps.unmount(mount)
    old = [p.uuid for p in drive.partitions if p.uuid]
    uuid = steps.partition_and_format(drive)
    steps.write_fstab(rewrite_fstab(FSTAB.read_text(), uuid, old_uuids=old), now)
    steps.mount_and_own()
    steps.refresh_data_root()
    steps.link_jupyter(MOUNT / 'data')
    out(f'Formatted and mounted at {MOUNT}. Reboot so every service writes there: sudo reboot')
    return 0


def check(run: Run = run_command, out: Callable[[str], None] = print) -> int:
    steps = Steps(run, out)
    drive = read_drive(run=run)
    part = drive.ours() if drive else None
    if part is None:
        raise DiskError('no racecar-data drive to check.')
    refuse_while_busy(run)
    stopped = steps.stop_writers()
    for mount in part.mounts:
        steps.unmount(mount)
    r = run(['sudo', 'e2fsck', '-f', '-p', part.path])
    out(r.stdout.strip() or r.stderr.strip())
    # e2fsck: 0 clean, 1 errors fixed; anything higher needs a person.
    if r.returncode > 1:
        out(f'e2fsck exit {r.returncode}: run sudo e2fsck -f {part.path} by hand.')
    steps.sudo('mount', str(MOUNT))
    for unit in stopped:
        run(['systemctl', '--no-ask-password', 'start', unit])
        if not unit_active(unit, run):
            steps.sudo('systemctl', 'start', unit)
    out('Checked and mounted again.' if r.returncode <= 1 else 'Mounted again.')
    return 0 if r.returncode <= 1 else 1


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['data']:
        try:
            return data_command(argv[1:])
        except DiskError as exc:
            print(exc, file=sys.stderr)
            return 2
    parser = argparse.ArgumentParser(prog='racecar disk', description='The NVMe data drive.')
    parser.add_argument(
        'action', nargs='?', default='status', choices=('status', 'setup', 'format', 'check')
    )
    args = parser.parse_args(argv)
    actions: dict[str, Callable[[], int]] = {
        'status': status,
        'setup': setup,
        'format': format_drive,
        'check': check,
    }
    try:
        return actions[args.action]()
    except DiskError as exc:
        print(f'racecar disk {args.action}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
