"""ament_pep257 docstring check."""

from pathlib import Path

from ament_pep257.main import main
import pytest

# Retired lab dashboards stay on some cars for reference; they are not shipped.
# ament_pep257 resolves excludes to absolute paths, so this works whether or
# not the directories exist.
DASHBOARDS = Path(__file__).parent.parent / 'scripts' / 'dashboards'
RETIRED = ['eps_dashboard', 'pursuit_dashboard', 'smartfollow_dashboard', '.migrated']


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--exclude', *[str(DASHBOARDS / d) for d in RETIRED]])
    assert rc == 0, 'Found code style errors / warnings'
