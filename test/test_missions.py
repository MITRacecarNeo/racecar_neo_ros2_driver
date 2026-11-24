"""racecar_neo_ros2_driver.missions: the mission format and its files beside the map."""

import pytest
from racecar_neo_ros2_driver import missions
import yaml


def raw(**over):
    point = {'x': 1, 'y': 2, 'heading_deg': -90, 'radius_m': 0.5, 'speed_mps': 0.7,
             'dwell_sec': 3}  # fmt: skip
    return {'loops': 2, 'waypoints': [{**point, **over}]}


class TestParse:
    def test_fills_and_normalizes(self):
        m = missions.parse({'waypoints': [{'x': 1.23456, 'y': 2}]})
        assert m == {'version': 1, 'loops': 1, 'waypoints': [
            {'x': 1.235, 'y': 2.0, 'heading_deg': None, 'radius_m': 0.25,
             'speed_mps': None, 'dwell_sec': 0.0}]}  # fmt: skip
        assert missions.parse(raw())['waypoints'][0]['heading_deg'] == 270.0

    @pytest.mark.parametrize(
        'bad',
        [
            raw(radius_m=0.1), raw(radius_m=3.5), raw(speed_mps=0.3), raw(speed_mps=1.2),
            raw(dwell_sec=-1), raw(dwell_sec=601), raw(x=float('nan')), raw(heading_deg='inf'),
            {'loops': 100, 'waypoints': raw()['waypoints']},
            {'loops': True, 'waypoints': raw()['waypoints']},
            {'loops': 1, 'waypoints': []},
            {'loops': 1, 'waypoints': raw()['waypoints'] * 101},
            {'loops': 1}, [],
        ],
    )  # fmt: skip
    def test_refuses_out_of_range(self, bad):
        with pytest.raises((ValueError, KeyError, TypeError)):
            missions.parse(bad)

    def test_until_stop_is_zero_loops(self):
        assert missions.parse({**raw(), 'loops': 0})['loops'] == 0


class TestFiles:
    def test_saved_beside_the_map(self, tmp_path):
        path = missions.save('hall', 'loop', raw(), tmp_path)
        assert path == tmp_path / 'hall' / 'missions' / 'loop.yaml'
        assert yaml.safe_load(path.read_text())['loops'] == 2
        assert missions.load('hall', 'loop', tmp_path) == missions.parse(raw())
        assert missions.exists('hall', 'loop', tmp_path)
        assert not missions.exists('other', 'loop', tmp_path)

    def test_saving_over_keeps_the_old_one(self, tmp_path):
        missions.save('hall', 'loop', raw(), tmp_path)
        missions.save('hall', 'loop', {**raw(), 'loops': 5}, tmp_path)
        older = list((tmp_path / 'hall' / 'missions' / 'previous').iterdir())
        assert len(older) == 1 and yaml.safe_load(older[0].read_text())['loops'] == 2
        assert missions.load('hall', 'loop', tmp_path)['loops'] == 5

    def test_listing_skips_broken_files(self, tmp_path):
        missions.save('hall', 'a', raw(), tmp_path)
        (tmp_path / 'hall' / 'missions' / 'broken.yaml').write_text('loops: [')
        found = missions.listing('hall', tmp_path)
        assert [(m['name'], m['waypoints'], m['loops']) for m in found] == [('a', 1, 2)]
        assert missions.listing('empty', tmp_path) == []

    def test_names_are_checked(self, tmp_path):
        for args in (('hall', '../x'), ('../hall', 'x'), ('hall', 'a b')):
            with pytest.raises(ValueError):
                missions.save(*args, raw(), tmp_path)
        with pytest.raises(FileNotFoundError):
            missions.load('hall', 'none', tmp_path)
