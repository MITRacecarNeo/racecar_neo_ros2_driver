"""ament_flake8 lint check."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # Retired lab dashboards stay on some cars for reference; they are not shipped.
    retired = 'eps_dashboard,pursuit_dashboard,smartfollow_dashboard,.migrated'
    rc, errors = main_with_errors(argv=[f'--exclude=build,install,log,{retired}'])
    assert rc == 0, '\n'.join(errors)
