"""racecar_neo_ros2_driver.units: unit names agree with setup and the app registry."""

from pathlib import Path
import re

from racecar_neo_ros2_driver import apps, units

SETUP = Path(__file__).resolve().parent.parent / 'scripts' / 'setup_services.sh'


def _bash_array(name):
    body = re.search(rf'^{name}=\((.*?)^\)', SETUP.read_text(), re.S | re.M).group(1)
    return [w.removesuffix('.service') for w in body.split()]


def test_core_units_are_the_boot_services_but_the_data_oneshot():
    assert set(_bash_array('SERVICES')) == {*units.CORE, units.DATA}


def test_on_demand_units_are_install_only():
    assert set(_bash_array('INSTALL_ONLY')) == {units.AUTONOMY, units.TERMINAL}


def test_app_units_in_core_match_the_registry():
    registry = apps.registry()
    named = {registry['home']['unit'], apps.app('jupyter')['unit'], apps.app('mapview')['unit']}
    assert named <= set(units.CORE)
    assert apps.app('terminal')['unit'] == units.TERMINAL


def test_service_suffix():
    assert units.service(units.TELEOP) == 'racecar-teleop.service'
    assert units.service('racecar-teleop.service') == 'racecar-teleop.service'


def test_systemctl_override(monkeypatch):
    monkeypatch.delenv('RACECAR_SYSTEMCTL', raising=False)
    assert units.systemctl_argv() == ['systemctl']
    monkeypatch.setenv('RACECAR_SYSTEMCTL', 'python3 fake.py')
    assert units.systemctl_argv() == ['python3', 'fake.py']
    monkeypatch.setenv('RACECAR_SYSTEMCTL', '')
    assert units.systemctl_argv() == ['systemctl']
