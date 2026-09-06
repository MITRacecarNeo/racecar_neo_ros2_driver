#!/usr/bin/env python3
"""
ROS 2 bag recording and analysis, behind `racecar log`.

One recording runs at a time. `start` spawns `ros2 bag record` in its own
session and records where it went; `status` reports on it without attaching to
it; `stop` sends SIGINT, which is what lets rosbag2 finalize the bag rather
than leaving it unindexed.

Storage resolves at runtime rather than assuming a layout: a car imaged onto
NVMe records to /data, one on an SD card falls back to ~/logs/bags. Recording
the camera topics to an SD card outruns the card, so `start` says so instead of
letting the bag quietly drop messages.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

CONFIG_DIR = Path.home() / '.config' / 'racecar'
CONFIG_FILE = CONFIG_DIR / 'log.conf'
STATE_FILE = CONFIG_DIR / 'log.state'

# Preference order for the bag root. /data is where an NVMe car mounts its
# scratch disk; the home fallback keeps an SD-card car working.
ROOT_CANDIDATES = ('/data', str(Path.home() / 'logs' / 'bags'))

TIMESTAMP_FMT = '%Y%m%d_%H%M%S'
DEFAULT_STORAGE = 'mcap'

# Image topics dominate the write rate; these are the ones worth warning about.
# Rough uncompressed rates for the shipped RealSense stream profiles, in MB/s.
HIGH_RATE_TOPICS = {
    '/camera/color': 55.0,
    '/camera/depth': 18.0,
}
# Sustained write a decent SD card manages, in MB/s. NVMe is far above this.
SD_SUSTAINED_MBPS = 30.0


def sanitize_name(name):
    """Reduce a test name to something safe for a directory name."""
    cleaned = re.sub(r'[^A-Za-z0-9._-]+', '-', (name or '').strip())
    return cleaned.strip('-.') or 'run'


def build_bag_name(test_name, now, prefix_fmt=TIMESTAMP_FMT):
    """Compose the bag directory name: timestamp first, then the test name."""
    stamp = time.strftime(prefix_fmt, time.localtime(now))
    return f'{stamp}_{sanitize_name(test_name)}'


def resolve_log_root(explicit=None, candidates=ROOT_CANDIDATES, environ=None):
    """
    Pick where bags are written, most specific source first.

    An explicit --dir wins, then RACECAR_LOG_DIR, then the first candidate that
    already exists and is writable, then the last candidate, which gets created.
    """
    if explicit:
        return Path(explicit).expanduser()
    environ = os.environ if environ is None else environ
    if environ.get('RACECAR_LOG_DIR'):
        return Path(environ['RACECAR_LOG_DIR']).expanduser()
    for cand in candidates:
        p = Path(cand)
        if p.is_dir() and os.access(p, os.W_OK):
            return p
    return Path(candidates[-1])


def build_record_argv(bag_dir, topics=None, exclude=None, storage=DEFAULT_STORAGE):
    """Assemble the `ros2 bag record` command line."""
    argv = ['ros2', 'bag', 'record', '--output', str(bag_dir), '--storage', storage]
    if topics:
        argv += list(topics)
    else:
        argv.append('--all-topics')
    for pattern in exclude or []:
        argv += ['--exclude-regex', pattern]
    return argv


def estimate_mbps(topics):
    """Estimate the write rate for a topic selection, in MB/s."""
    if not topics:
        return sum(HIGH_RATE_TOPICS.values())
    return sum(HIGH_RATE_TOPICS.get(t, 0.0) for t in topics)


def format_size(num_bytes):
    """Render a byte count in the largest unit that keeps it above 1."""
    value = float(num_bytes)
    if value < 1024.0:
        return f'{int(value)} B'
    for unit in ('KB', 'MB', 'GB', 'TB'):
        value /= 1024.0
        if value < 1024.0 or unit == 'TB':
            return f'{value:.1f} {unit}'
    return f'{value:.1f} TB'


def format_duration(seconds):
    """Render an elapsed time as h:mm:ss, dropping empty leading units."""
    seconds = int(max(0, seconds))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    return f'{hours}:{minutes:02d}:{secs:02d}' if hours else f'{minutes}:{secs:02d}'


def unmounted_nvme(lsblk_output=None):
    """
    Name an NVMe disk that carries no mounted filesystem, or None.

    The intended bag target is /data on NVMe. A car whose drive is still raw
    falls back to the SD card, which is slower and smaller, so that is worth
    saying out loud rather than leaving someone to find it from a full disk.
    """
    if lsblk_output is None:
        try:
            lsblk_output = subprocess.run(
                ['lsblk', '-rno', 'NAME,TYPE,MOUNTPOINT'],
                capture_output=True, text=True, timeout=5,
            ).stdout
        except (OSError, subprocess.SubprocessError):
            return None
    disks, mounted = [], set()
    for line in lsblk_output.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        kind = parts[1] if len(parts) > 1 else ''
        mountpoint = parts[2] if len(parts) > 2 else ''
        if kind == 'disk' and name.startswith('nvme'):
            disks.append(name)
        if mountpoint:
            mounted.add(name)
    for disk in disks:
        if not any(m.startswith(disk) for m in mounted):
            return disk
    return None


def storage_hint(root):
    """Suggest making /data when the NVMe is present but unused."""
    if str(root).startswith('/data'):
        return None
    disk = unmounted_nvme()
    if not disk:
        return None
    return (
        f'/dev/{disk} is present but not mounted, so bags are going to {root} '
        f'on the SD card. Partition and mount it at /data to use it.'
    )


def root_is_sd(root):
    """Report whether the bag root sits on an SD card rather than NVMe."""
    try:
        dev = subprocess.run(
            ['findmnt', '-no', 'SOURCE', '--target', str(root)],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return False
    return 'mmcblk' in dev


def bandwidth_warning(topics, root, sustained=SD_SUSTAINED_MBPS, free_bytes=None):
    """
    Return a warning when the selection outruns the disk, else None.

    Recording every topic pulls in the RealSense colour and depth streams,
    which together outrun an SD card by roughly two to one. rosbag2 drops
    messages rather than blocking, so the bag looks healthy until it is read.
    """
    rate = estimate_mbps(topics)
    if rate <= sustained or not root_is_sd(root):
        return None
    if free_bytes is None:
        try:
            free_bytes = shutil.disk_usage(root).free
        except OSError:
            free_bytes = 0
    fill = format_duration(free_bytes / 1e6 / rate) if rate else '?'
    return (
        f'{rate:.0f} MB/s of image topics onto an SD card that sustains about '
        f'{sustained:.0f} MB/s. Messages will be dropped, and the disk fills in '
        f'about {fill}. Pass --exclude, or name the topics you need.'
    )


def dir_size(path):
    """Total size of a directory tree, in bytes."""
    total = 0
    for dirpath, _dirnames, filenames in os.walk(path):
        for name in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, name))
            except OSError:
                pass
    return total


def load_config(path=None):
    """Read the persisted KEY=VALUE defaults."""
    path = Path(path) if path else CONFIG_FILE
    cfg = {}
    try:
        for line in Path(path).read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, value = line.partition('=')
            cfg[key.strip()] = value.strip()
    except OSError:
        pass
    return cfg


def save_config(cfg, path=None):
    """Persist the defaults, replacing the file."""
    path = Path(path) if path else CONFIG_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    body = ''.join(f'{k}={v}\n' for k, v in sorted(cfg.items()) if v != '')
    path.write_text('# racecar log defaults\n' + body)


def read_state(path=None):
    """Load the active recording's state, or None when nothing is recording."""
    path = Path(path) if path else STATE_FILE
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def write_state(state, path=None):
    """Record where the running bag went, so status and stop can find it."""
    path = Path(path) if path else STATE_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2) + '\n')


def clear_state(path=None):
    """Forget the active recording."""
    path = Path(path) if path else STATE_FILE
    try:
        path.unlink()
    except OSError:
        pass


def pid_alive(pid):
    """Report whether a PID is still running, without signalling it."""
    try:
        os.kill(int(pid), 0)
    except (OSError, TypeError, ValueError):
        return False
    return True


def summarize_topics(counts, duration_sec):
    """
    Build per-topic rows of message count and mean rate.

    Sorted by count, so the topics that dominate a bag come first; that is what
    someone reading an oversized recording is looking for.
    """
    rows = []
    for topic, count in counts.items():
        rate = (count / duration_sec) if duration_sec > 0 else 0.0
        rows.append({'topic': topic, 'count': count, 'hz': rate})
    return sorted(rows, key=lambda r: r['count'], reverse=True)


def active_state():
    """
    Return the running recording's state, clearing it when the process is gone.

    A bag whose recorder died (OOM, a reboot mid-run) leaves a state file
    behind. Treating that as "recording" would make stop and status lie, so it
    is cleared on sight and reported as finished.
    """
    state = read_state()
    if state is None:
        return None
    if not pid_alive(state.get('pid')):
        clear_state()
        return None
    return state


# ---------------------------------------------------------------- subcommands

def cmd_start(args):
    """Spawn `ros2 bag record` in its own session and remember where it went."""
    if active_state() is not None:
        state = read_state()
        print(f"already recording: {state['bag']} (pid {state['pid']})", file=sys.stderr)
        print('stop it first with: racecar log stop', file=sys.stderr)
        return 1

    cfg = load_config()
    topics = args.topics or (cfg.get('TOPICS', '').split(',') if cfg.get('TOPICS') else [])
    topics = [t for t in topics if t]
    exclude = args.exclude or []
    storage = args.storage or cfg.get('STORAGE', DEFAULT_STORAGE)
    root = resolve_log_root(args.dir or cfg.get('DIR'))

    try:
        root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f'cannot create {root}: {exc}', file=sys.stderr)
        return 1

    name = args.name or build_bag_name(args.test_name, time.time())
    bag = root / name
    if bag.exists():
        print(f'{bag} already exists', file=sys.stderr)
        return 1

    hint = storage_hint(root)
    if hint:
        print(f'note: {hint}')

    warning = bandwidth_warning(topics, root)
    if warning:
        print(f'warning: {warning}')
        if not args.force:
            print('re-run with --force to record anyway.')
            return 1

    argv = build_record_argv(bag, topics, exclude, storage)
    stdout_path = root / f'{name}.record.log'
    with open(stdout_path, 'wb') as out:
        proc = subprocess.Popen(
            argv, stdout=out, stderr=subprocess.STDOUT, start_new_session=True,
        )

    write_state({
        'pid': proc.pid,
        'bag': str(bag),
        'name': name,
        'topics': topics,
        'exclude': exclude,
        'storage': storage,
        'started': time.time(),
        'stdout': str(stdout_path),
    })
    print(f'recording {"all topics" if not topics else f"{len(topics)} topics"} '
          f'to {bag}')
    print(f'  pid {proc.pid}; stop with: racecar log stop')
    return 0


def cmd_stop(args):
    """Signal the recorder and wait for rosbag2 to finalize the bag."""
    state = active_state()
    if state is None:
        print('not recording')
        return 0

    pid = int(state['pid'])
    # SIGINT is what rosbag2 finalizes on. SIGTERM leaves the bag unindexed,
    # which ros2 bag info then refuses to read.
    try:
        os.killpg(os.getpgid(pid), signal.SIGINT)
    except OSError:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass

    deadline = time.time() + args.timeout
    while time.time() < deadline and pid_alive(pid):
        time.sleep(0.2)

    if pid_alive(pid):
        print(f'recorder {pid} did not exit within {args.timeout}s; sending SIGTERM',
              file=sys.stderr)
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except OSError:
            pass
        time.sleep(1.0)

    bag = Path(state['bag'])
    size = dir_size(bag) if bag.exists() else 0
    elapsed = time.time() - state['started']
    clear_state()
    print(f'stopped {bag.name}: {format_size(size)} over {format_duration(elapsed)}')
    print(f'  {bag}')
    return 0


def cmd_status(args):
    """Report on the running recording without attaching to it."""
    state = active_state()
    if state is None:
        if args.json:
            print(json.dumps({'recording': False}))
        else:
            print('not recording')
        return 0

    bag = Path(state['bag'])
    size = dir_size(bag) if bag.exists() else 0
    elapsed = time.time() - state['started']
    rate = size / elapsed if elapsed > 0 else 0.0
    try:
        free = shutil.disk_usage(bag.parent).free
    except OSError:
        free = 0
    remaining = (free / rate) if rate > 0 else None

    if args.json:
        print(json.dumps({
            'recording': True,
            'bag': str(bag),
            'pid': state['pid'],
            'elapsed_sec': elapsed,
            'size_bytes': size,
            'rate_bytes_per_sec': rate,
            'free_bytes': free,
            'seconds_until_full': remaining,
            'topics': state.get('topics') or 'all',
        }, indent=2))
        return 0

    print(f'recording  {bag.name}')
    print(f'  bag      {bag}')
    print(f'  pid      {state["pid"]}')
    print(f'  elapsed  {format_duration(elapsed)}')
    print(f'  size     {format_size(size)}  ({format_size(rate)}/s)')
    if size == 0 and elapsed > 2.0:
        # mcap writes in chunks, so a slow topic shows nothing on disk for the
        # first few seconds. Absence of size is not absence of recording.
        print('           (mcap flushes in chunks; a slow bag reads 0 until the'
              ' first flush)')
    print(f'  free     {format_size(free)}' +
          (f'  (full in about {format_duration(remaining)})' if remaining else ''))
    topics = state.get('topics')
    print(f'  topics   {", ".join(topics) if topics else "all"}')
    return 0


def cmd_list(args):
    """List recorded bags under the log root, newest first."""
    root = resolve_log_root(args.dir or load_config().get('DIR'))
    if not root.is_dir():
        print(f'no bags: {root} does not exist')
        return 0
    bags = [p for p in root.iterdir() if p.is_dir()]
    if not bags:
        print(f'no bags in {root}')
        return 0
    running = active_state()
    current = running['bag'] if running else None
    print(f'{root}:')
    for bag in sorted(bags, key=lambda p: p.stat().st_mtime, reverse=True):
        mark = '*' if str(bag) == current else ' '
        print(f'  {mark} {bag.name:<44} {format_size(dir_size(bag))}')
    if current:
        print('\n* currently recording')
    return 0


def _read_metadata(bag):
    """Read a bag's metadata via rosbag2_py, or None when it cannot be read."""
    try:
        import rosbag2_py
    except ImportError:
        return None
    try:
        return rosbag2_py.Info().read_metadata(str(bag), '')
    except (RuntimeError, OSError):
        return None


def cmd_analyze(args):
    """Summarize a bag: duration, size, and per-topic counts and rates."""
    if args.bag:
        bag = Path(args.bag).expanduser()
        if not bag.is_absolute() and not bag.exists():
            bag = resolve_log_root(load_config().get('DIR')) / args.bag
    else:
        root = resolve_log_root(load_config().get('DIR'))
        bags = [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
        if not bags:
            print(f'no bags in {root}', file=sys.stderr)
            return 1
        bag = max(bags, key=lambda p: p.stat().st_mtime)

    if not bag.is_dir():
        print(f'no such bag: {bag}', file=sys.stderr)
        return 1

    meta = _read_metadata(bag)
    if meta is None:
        print(f'cannot read bag metadata for {bag}', file=sys.stderr)
        print('is this a bag directory? is rosbag2_py available?', file=sys.stderr)
        return 1

    duration = meta.duration.nanoseconds / 1e9
    counts = {t.topic_metadata.name: t.message_count for t in meta.topics_with_message_count}
    rows = summarize_topics(counts, duration)
    size = dir_size(bag)

    if args.json:
        print(json.dumps({
            'bag': str(bag),
            'duration_sec': duration,
            'size_bytes': size,
            'messages': meta.message_count,
            'topics': rows,
        }, indent=2))
        return 0

    print(f'{bag.name}')
    print(f'  path      {bag}')
    print(f'  duration  {format_duration(duration)}')
    print(f'  size      {format_size(size)}')
    print(f'  messages  {meta.message_count}')
    print(f'  topics    {len(rows)}')
    if rows:
        print()
        print(f'  {"topic":<40} {"messages":>10} {"mean Hz":>9}')
        for row in rows:
            print(f'  {row["topic"]:<40} {row["count"]:>10} {row["hz"]:>9.1f}')
    empty = [r['topic'] for r in rows if r['count'] == 0]
    if empty:
        print(f'\n  {len(empty)} topic(s) recorded no messages: {", ".join(empty)}')
    return 0


def cmd_config(args):
    """Show, set, or clear the persisted defaults."""
    if args.reset:
        try:
            CONFIG_FILE.unlink()
            print(f'cleared {CONFIG_FILE}')
        except OSError:
            print('nothing to clear')
        return 0

    cfg = load_config()
    changed = False
    for key, value in (('TOPICS', args.topics_csv), ('DIR', args.dir),
                       ('STORAGE', args.storage)):
        if value is not None:
            cfg[key] = value
            changed = True
    if changed:
        save_config(cfg)

    root = resolve_log_root(cfg.get('DIR'))
    print(f'topics   {cfg.get("TOPICS") or "all"}')
    print(f'dir      {cfg.get("DIR") or f"{root}  (resolved)"}')
    print(f'storage  {cfg.get("STORAGE", DEFAULT_STORAGE)}')
    print(f'config   {CONFIG_FILE}')
    hint = storage_hint(root)
    if hint:
        print(f'\nnote: {hint}')
    return 0


def build_parser():
    """Assemble the argument parser for every `racecar log` subcommand."""
    parser = argparse.ArgumentParser(
        prog='racecar log', description='Record and analyze ROS 2 bags.',
    )
    sub = parser.add_subparsers(dest='action')

    start = sub.add_parser('start', help='start recording')
    start.add_argument('test_name', nargs='?', default='run',
                       help='name appended after the timestamp')
    start.add_argument('--topics', nargs='*', help='topics to record (default: all)')
    start.add_argument('--exclude', nargs='*', help='regex of topics to skip')
    start.add_argument('--dir', help='bag root (default: /data, else ~/logs/bags)')
    start.add_argument('--name', help='full bag name, replacing timestamp_testname')
    start.add_argument('--storage', choices=('mcap', 'sqlite3'), help='storage plugin')
    start.add_argument('--force', action='store_true',
                       help='record even when the rate outruns the disk')
    start.set_defaults(func=cmd_start)

    stop = sub.add_parser('stop', help='stop the running recording')
    stop.add_argument('--timeout', type=float, default=15.0,
                      help='seconds to wait for a clean finalize (default 15)')
    stop.set_defaults(func=cmd_stop)

    status = sub.add_parser('status', help='size and progress of the running bag')
    status.add_argument('--json', action='store_true')
    status.set_defaults(func=cmd_status)

    listing = sub.add_parser('list', help='list recorded bags')
    listing.add_argument('--dir')
    listing.set_defaults(func=cmd_list)

    analyze = sub.add_parser('analyze', help='summarize a bag (default: newest)')
    analyze.add_argument('bag', nargs='?')
    analyze.add_argument('--json', action='store_true')
    analyze.set_defaults(func=cmd_analyze)

    config = sub.add_parser('config', help='show or set persisted defaults')
    config.add_argument('--topics', dest='topics_csv',
                        help='comma-separated default topic list')
    config.add_argument('--dir', help='default bag root')
    config.add_argument('--storage', choices=('mcap', 'sqlite3'))
    config.add_argument('--reset', action='store_true')
    config.set_defaults(func=cmd_config)

    return parser


def main(argv=None):
    """Dispatch a `racecar log` subcommand; status is the default."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, 'func', None):
        return cmd_status(argparse.Namespace(json=False))
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
