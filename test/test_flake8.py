"""ament_flake8 lint check."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=['--exclude=build,install,log'])
    assert rc == 0, '\n'.join(errors)
