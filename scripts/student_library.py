#!/usr/bin/env python3
"""
Student library and labs on the car, behind `racecar library install` and `status`.

The library is cloned as root at the tag in config/student_library.yaml into
/opt/racecar-neo/library-<tag>/, so students cannot edit it, and its library/
folder is linked at ~/jupyter_ws/racecar-neo/library. Older tags stay beside
it. The labs repo's labs/ folder, at the tip of its branch, is unpacked into
~/jupyter_ws/racecar-neo/labs from a bare clone kept in
~/.cache/racecar-neo/labs.git. A labs folder already there, or a library that
is a real folder rather than the link, is moved to
~/backups/YYYYMMDD_HHMMSS_racecar-neo-<labs|library>-<tag> first, so a
student's work is kept.

`racecar library --select` still chooses which library Python imports; the
first install selects racecar-neo when nothing is selected.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import os
from pathlib import Path
import re
import site
import subprocess
import sys
import time
from typing import Any

import yaml

OPT_ROOT = Path('/opt/racecar-neo')
LABS_CACHE = Path('.cache') / 'racecar-neo' / 'labs.git'  # under the home folder
FOLDER = 'racecar-neo'
STAMP_FMT = '%Y%m%d_%H%M%S'
NETWORK_TIMEOUT_SEC = 20.0

Run = Callable[..., subprocess.CompletedProcess]


def pin_file() -> Path:
    return Path(__file__).resolve().parent.parent / 'config' / 'student_library.yaml'


def load_pin(path: Path | None = None) -> dict[str, Any]:
    data = yaml.safe_load((path or pin_file()).read_text())
    return {
        'library_repo': data['library']['repo'],
        'tag': data['library']['tag'],
        'labs_repo': data['labs']['repo'],
        'branch': data['labs']['branch'],
    }


def library_dir(tag: str, root: Path = OPT_ROOT) -> Path:
    return root / f'library-{tag}'


def backup_name(kind: str, tag: str, now: float) -> str:
    """Return YYYYMMDD_HHMMSS_racecar-neo-<kind>-<tag> for a moved folder."""
    safe = re.sub(r'[^A-Za-z0-9._-]+', '-', tag).strip('-') or 'untracked'
    return f'{time.strftime(STAMP_FMT, time.localtime(now))}_racecar-neo-{kind}-{safe}'


VERSION_FILE = '.racecar-labs-version'


def describe(folder: Path, run: Run = subprocess.run) -> str:
    """
    Return the folder's version, or 'untracked' for a plain copy.

    Installed labs carry it on the first line of their version file; a clone
    answers `git describe`.
    """
    stamp = folder / VERSION_FILE
    if stamp.is_file():
        first = stamp.read_text().splitlines()
        return first[0].strip() if first and first[0].strip() else 'untracked'
    if not (folder / '.git').exists():
        return 'untracked'
    r = run(
        ['git', '-C', str(folder), 'describe', '--tags', '--always'],
        capture_output=True,
        text=True,
    )
    return r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else 'untracked'


def move_to_backups(folder: Path, kind: str, backups: Path, now: float, run: Run) -> Path:
    backups.mkdir(parents=True, exist_ok=True)
    target = backups / backup_name(kind, describe(folder, run), now)
    os.rename(folder, target)
    return target


def reachable(repo: str, ref: str, run: Run = subprocess.run) -> bool:
    try:
        r = run(
            ['git', 'ls-remote', '--exit-code', repo, ref],
            capture_output=True,
            text=True,
            timeout=NETWORK_TIMEOUT_SEC,
        )
    except subprocess.TimeoutExpired:
        return False
    return r.returncode == 0


def install_library(pin: dict[str, Any], root: Path, run: Run, say: Callable[[str], None]) -> Path:
    """Clone the pinned tag into root unless it is there; return its library/ folder."""
    final = library_dir(pin['tag'], root)
    if (final / 'library' / 'racecar_core.py').is_file():
        say(f'library {pin["tag"]} already in {final}')
        return final / 'library'
    tmp = root / f'.library-{pin["tag"]}.new'
    if tmp.exists():
        raise RuntimeError(f'{tmp} is left from an earlier run; move it away and retry')
    steps = [
        ['sudo', 'mkdir', '-p', str(root)],
        ['sudo', 'git', 'clone', '--quiet', '--depth', '1', '--branch', pin['tag'],
         pin['library_repo'], str(tmp)],
        ['sudo', 'python3', '-m', 'compileall', '-q', str(tmp / 'library')],
        ['sudo', 'mv', '-T', str(tmp), str(final)],
    ]  # fmt: skip
    for argv in steps:
        if run(argv).returncode != 0:
            raise RuntimeError(f'failed: {" ".join(argv)}')
    say(f'library {pin["tag"]} installed in {final}')
    return final / 'library'


def link_library(
    target: Path, jws: Path, backups: Path, now: float, run: Run, say: Callable[[str], None]
) -> None:
    """Point jws/library at target; a real folder there is moved to backups first."""
    jws.mkdir(parents=True, exist_ok=True)
    link = jws / 'library'
    if link.exists() and not link.is_symlink():
        moved = move_to_backups(link, 'library', backups, now, run)
        say(f'moved the old library folder to {moved}')
    tmp = jws / '.library.link'
    if tmp.is_symlink() or tmp.exists():
        tmp.unlink()
    tmp.symlink_to(target)
    os.replace(tmp, link)
    say(f'{link} -> {target}')


def _git(run: Run, argv: list[str]) -> str:
    r = run(argv, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'failed: {" ".join(argv)}')
    return str(r.stdout).strip()


def install_labs(
    pin: dict[str, Any],
    jws: Path,
    backups: Path,
    now: float,
    run: Run,
    say: Callable[[str], None],
    cache: Path,
) -> None:
    """
    Put the repo's labs/ folder at jws/labs from the branch tip, one level deep.

    The repo is kept as a bare clone in `cache` (fetched on later installs);
    only its labs/ folder is unpacked, stamped with the version. An existing
    labs folder goes to backups first.
    """
    jws.mkdir(parents=True, exist_ok=True)
    labs = jws / 'labs'
    tmp = jws / '.labs.new'
    if tmp.exists():
        raise RuntimeError(f'{tmp} is left from an earlier run; move it away and retry')
    git = ['git', '--git-dir', str(cache)]
    branch = pin['branch']
    if (cache / 'HEAD').is_file():
        _git(run, [*git, 'fetch', '--quiet', '--tags', 'origin', '+refs/heads/*:refs/heads/*'])
    else:
        cache.parent.mkdir(parents=True, exist_ok=True)
        _git(run, ['git', 'clone', '--quiet', '--bare', pin['labs_repo'], str(cache)])
    version = _git(run, [*git, 'describe', '--tags', '--always', branch])
    commit = _git(run, [*git, 'rev-parse', branch])
    tmp.mkdir()
    archive = jws / '.labs.tar'
    _git(run, [*git, 'archive', '--format=tar', '-o', str(archive), branch, 'labs'])
    try:
        _git(run, ['tar', '-xf', str(archive), '-C', str(tmp)])
    finally:
        archive.unlink(missing_ok=True)
    if not (tmp / 'labs').is_dir():
        raise RuntimeError(f'no labs/ folder in {pin["labs_repo"]} at {branch}')
    (tmp / 'labs' / VERSION_FILE).write_text(f'{version}\n{commit}\n{pin["labs_repo"]} {branch}\n')
    if labs.exists() or labs.is_symlink():
        moved = move_to_backups(labs, 'labs', backups, now, run)
        say(f'moved the old labs folder to {moved}')
    os.rename(tmp / 'labs', labs)
    tmp.rmdir()
    say(f'labs {version} installed in {labs}')


def select_if_unset(lib_link: Path, pth: Path, say: Callable[[str], None]) -> None:
    """Write the .pth for racecar-neo when no library is selected yet."""
    if pth.is_file():
        return
    pth.parent.mkdir(parents=True, exist_ok=True)
    pth.write_text(f'{lib_link}\n')
    say(f'selected {lib_link} (racecar library --select changes it)')


def cmd_install(args: argparse.Namespace, run: Run = subprocess.run) -> int:
    pin = load_pin()
    if args.tag:
        pin['tag'] = args.tag
    home = Path.home()
    jws, backups = home / 'jupyter_ws' / FOLDER, home / 'backups'
    now = time.time()
    labs_wanted = not args.no_labs and not (args.labs_if_missing and (jws / 'labs').exists())
    if not reachable(pin['library_repo'], pin['tag'], run) or (
        labs_wanted and not reachable(pin['labs_repo'], pin['branch'], run)
    ):
        print(
            'Cannot reach GitHub for the student library and labs. Connect the car to the '
            'internet, then run: racecar library install',
            file=sys.stderr,
        )
        return 3
    try:
        lib = install_library(pin, OPT_ROOT, run, print)
        link_library(lib, jws, backups, now, run, print)
        if labs_wanted:
            install_labs(pin, jws, backups, now, run, print, home / LABS_CACHE)
        elif args.labs_if_missing:
            print(f'labs kept in {jws / "labs"} (racecar library install replaces them)')
    except (RuntimeError, OSError) as exc:
        print(f'racecar library install: {exc}', file=sys.stderr)
        return 1
    select_if_unset(
        jws / 'library', Path(site.getusersitepackages()) / 'racecar_student.pth', print
    )
    return 0


def status_lines(
    pin: dict[str, Any], root: Path, jws: Path, run: Run = subprocess.run
) -> list[str]:
    installed = sorted(p.name.removeprefix('library-') for p in root.glob('library-*'))
    link = jws / 'library'
    linked = os.readlink(link) if link.is_symlink() else ('folder' if link.exists() else 'none')
    labs = jws / 'labs'
    return [
        f'pinned    library {pin["tag"]}, labs {pin["branch"]}',
        f'installed {", ".join(installed) or "none"} (in {root})',
        f'library   {link} -> {linked}',
        f'labs      {describe(labs, run) if labs.exists() else "none"}',
    ]


def cmd_status(_args: argparse.Namespace) -> int:
    jws = Path.home() / 'jupyter_ws' / FOLDER
    print('\n'.join(status_lines(load_pin(), OPT_ROOT, jws)))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='racecar library', description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    inst = sub.add_parser('install', help='install the pinned library and the labs')
    inst.add_argument('--tag', help='library tag to install instead of the pinned one')
    inst.add_argument('--no-labs', action='store_true', help='leave the labs folder as is')
    inst.add_argument(
        '--labs-if-missing',
        action='store_true',
        help='clone the labs only when there is no labs folder (first setup)',
    )
    inst.set_defaults(func=cmd_install)
    stat = sub.add_parser('status', help='pinned and installed versions')
    stat.set_defaults(func=cmd_status)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    sys.exit(main())
