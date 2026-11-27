#!/usr/bin/env python3
"""
Autosave a mapping run before the autonomy unit stops (its ExecStop).

Calls /autonomy/autosave while slam_toolbox is still alive, so a stop, a teleop
restart, or a teleop shutdown keeps the map up to that moment. In localization
the call returns "not mapping". The unit passes its main PID: while that
process runs, discovering the service is worth up to SERVICE_WAIT_SEC (a new
process on the car takes 1 to 4 s); after a crash it is gone and the wait is
short. Always exits 0 so the stop goes ahead.
"""

from __future__ import annotations

import os
import sys

SERVICE_WAIT_SEC = 10.0
GONE_WAIT_SEC = 1.0
CALL_TIMEOUT_SEC = 20.0


def running(pid_text: str) -> bool:
    """Return True when the given PID (the unit's main process) exists."""
    try:
        pid = int(pid_text)
    except ValueError:
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    wait = SERVICE_WAIT_SEC if args and running(args[0]) else GONE_WAIT_SEC
    from racecar_neo_ros2_driver.ros_cli import call_service, ros_node
    from std_srvs.srv import Trigger

    with ros_node('autonomy_stop') as node:
        status, result = call_service(
            node,
            Trigger,
            '/autonomy/autosave',
            Trigger.Request(),
            wait_sec=wait,
            timeout_sec=CALL_TIMEOUT_SEC,
        )
    if status == 'unavailable':
        print('autonomy_stop: no /autonomy/autosave; nothing saved.')
    elif status == 'timeout':
        print('autonomy_stop: autosave timed out.')
    else:
        print(f'autonomy_stop: {result.message}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
