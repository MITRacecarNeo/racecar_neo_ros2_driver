"""
Log call sites that pick their severity at run time.

rclpy fixes a call site's severity at its first call, so a site that picks
the severity with a conditional raises ValueError on the first change.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent.parent
PICKED = re.compile(r'get_logger\(\)\.(?:debug|info|warn|warning|error|fatal) if\b')


def test_no_log_call_site_picks_its_severity():
    paths = [
        *ROOT.joinpath('racecar_neo_ros2_driver').rglob('*.py'),
        *ROOT.joinpath('scripts').rglob('*.py'),
    ]
    found = [
        str(path.relative_to(ROOT))
        for path in paths
        if PICKED.search(re.sub(r'\s+', ' ', path.read_text(encoding='utf-8')))
    ]
    assert found == []
