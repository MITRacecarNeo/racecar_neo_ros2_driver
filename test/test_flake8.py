"""ament_flake8 lint check."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # `dashboards` holds upstream lab-dashboard checkouts (see
    # scripts/setup_dashboards.sh). They are third-party code with their own
    # style; linting them would swamp this package's own result.
    rc, errors = main_with_errors(argv=['--exclude=build,install,log,dashboards'])
    assert rc == 0, '\n'.join(errors)
