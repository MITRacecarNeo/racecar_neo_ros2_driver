"""Unit tests for racecar_neo_ros2_driver/autonomy.py (autonomy_node logic without ROS)."""

from datetime import datetime
import math

import pytest
from racecar_neo_ros2_driver import autonomy as au
import yaml

LIMITS = au.Limits()


def _cov(xy_sd: float, yaw_sd_deg: float) -> list[float]:
    c = [0.0] * 36
    c[0] = c[7] = xy_sd**2 / 2
    c[35] = math.radians(yaw_sd_deg) ** 2
    return c


class TestLocalized:
    def test_covariance_sd(self):
        xy, yaw = au.covariance_sd(_cov(0.2, 10.0))
        assert xy == pytest.approx(0.2) and yaw == pytest.approx(10.0)

    def test_mapping_needs_only_map_odom_and_scans(self):
        # slam_toolbox publishes its pose only once the car moves.
        assert au.is_localized('mapping', True, 0.1, None, 0, False, LIMITS)

    def test_tracking_amcl(self):
        assert au.is_localized('localization', True, 0.1, (0.11, 5.3), 10, False, LIMITS)

    @pytest.mark.parametrize('sd', [(2.1, 114.0), (0.3, 5.0), (0.1, 25.0)])
    def test_a_lost_amcl(self, sd):
        assert not au.is_localized('localization', True, 0.1, sd, 10, False, LIMITS)

    @pytest.mark.parametrize('scan_age', [None, 0.6])
    def test_lidar_loss(self, scan_age):
        assert not au.is_localized('localization', True, scan_age, (0.1, 5.0), 10, False, LIMITS)
        assert not au.is_localized('mapping', True, scan_age, None, 10, False, LIMITS)

    def test_no_map_odom(self):
        assert not au.is_localized('mapping', False, 0.1, None, 10, False, LIMITS)

    def test_global_search_waits(self):
        assert not au.is_localized('localization', True, 0.1, (0.1, 5.0), 10, True, LIMITS)

    def test_status_level(self):
        assert au.status_level(True, True, 1.0) == au.LEVEL_OK
        assert au.status_level(False, True, 1.0) == au.LEVEL_WARN
        assert au.status_level(False, False, 2.0) == au.LEVEL_WARN  # starting up
        assert au.status_level(False, False, 6.0) == au.LEVEL_ERROR


class TestFiles:
    def test_atomic_yaml(self, tmp_path):
        path = tmp_path / 'm' / 'last_pose.yaml'
        au.write_yaml_atomic(path, {'x': 1.0})
        assert yaml.safe_load(path.read_text()) == {'x': 1.0}
        assert not path.with_name('last_pose.yaml.tmp').exists()

    def test_replace_dir(self, tmp_path):
        final, tmp = tmp_path / 'autosave', tmp_path / 'autosave.tmp'
        final.mkdir()
        (final / 'old').write_text('1')
        tmp.mkdir()
        (tmp / 'new').write_text('2')
        au.replace_dir(tmp, final)
        assert [p.name for p in final.iterdir()] == ['new']
        assert not tmp.exists() and not (tmp_path / 'autosave.old').exists()

    def test_replace_dir_first_time(self, tmp_path):
        tmp = tmp_path / 'autosave.tmp'
        tmp.mkdir()
        au.replace_dir(tmp, tmp_path / 'autosave')
        assert (tmp_path / 'autosave').is_dir()

    def test_pose_round_trip(self, tmp_path):
        path = tmp_path / 'last_pose.yaml'
        au.write_yaml_atomic(path, au.pose_record((1.234567, -2.0, 0.5), datetime(2026, 10, 9)))
        assert au.read_pose(path) == (1.2346, -2.0, 0.5)

    @pytest.mark.parametrize('text', ['', 'x: 1\n', 'not: [yaml'])
    def test_bad_pose_file(self, tmp_path, text):
        path = tmp_path / 'last_pose.yaml'
        path.write_text(text)
        assert au.read_pose(path) is None
        assert au.read_pose(tmp_path / 'missing.yaml') is None

    def test_meta(self):
        meta = au.meta_record(
            'room', datetime(2026, 10, 9, 1, 2, 3), '0.9.0', {'a_mm': 1.0}, '', 's'
        )
        assert meta['saved'] == '2026-10-09 01:02:03' and meta['frames'] == {'a_mm': 1.0}
        assert meta['hostname']


def test_status_values():
    got = dict(
        au.status_values(
            mode='mapping', localized=True, tf_age_ms=12.6, pose_sd_m=0.1234, last_save=None
        )
    )
    assert got == {
        'mode': 'mapping',
        'localized': 'true',
        'tf_age_ms': '13',
        'pose_sd_m': '0.123',
        'last_save': '',
    }
