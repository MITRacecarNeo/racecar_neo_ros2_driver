"""Where the package's config files are: the installed share, else the source tree."""

from __future__ import annotations

from pathlib import Path

PACKAGE = 'racecar_neo_ros2_driver'


def config_dir() -> Path:
    """Return the config dir the running stack reads: installed share, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory(PACKAGE)) / 'config'
    except (ImportError, LookupError):
        share = None
    if share is not None and share.is_dir():
        return share
    return Path(__file__).resolve().parent.parent / 'config'
