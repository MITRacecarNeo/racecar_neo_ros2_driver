"""Unit tests for the IMU bias calibrators and their shared local-YAML output."""

from fnmatch import fnmatch
from pathlib import Path

from conftest import load_script as _load
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def common():
    return _load('calibrate_common')


@pytest.fixture(scope='module', params=['calibrate_imu', 'calibrate_realsense_imu'])
def target(request):
    return _load(request.param).TARGET


def _positions(accel_per_position):
    return [[list(a)] * 10 for a in accel_per_position]


class TestLocalYamlOutput:
    def test_source_config_dir_is_the_repo_config(self, common):
        assert common.source_config_dir() == ROOT / 'config'

    def test_written_path_is_name_dot_local_yaml(self, common, tmp_path):
        written = common.write_local_yaml(
            'lsm9ds1_cal', 'pit_node', {'gyroscope.bias': [0.0, 0.0, 0.0]}, [tmp_path], 'test'
        )
        assert written == [tmp_path / 'lsm9ds1_cal.local.yaml']

    def test_written_file_is_gitignored(self, common, tmp_path):
        written = common.write_local_yaml('realsense_cal', 'imu_fusion_node', {}, [tmp_path], 't')
        patterns = [
            ln.strip()
            for ln in (ROOT / '.gitignore').read_text().splitlines()
            if ln.strip() and not ln.startswith('#')
        ]
        rel = f'config/{written[0].name}'
        assert any(fnmatch(rel, p) for p in patterns), rel

    def test_written_file_is_ros_parameter_yaml(self, common, tmp_path):
        params = {'accelerometer.bias': [0.1, 0.2, 0.3]}
        (path,) = common.write_local_yaml('lsm9ds1_cal', 'pit_node', params, [tmp_path], 'test')
        text = path.read_text()
        assert text.startswith('#')
        assert yaml.safe_load(text) == {'pit_node': {'ros__parameters': params}}

    def test_every_dir_gets_a_copy(self, common, tmp_path):
        dirs = [tmp_path / 'a', tmp_path / 'b']
        for d in dirs:
            d.mkdir()
        written = common.write_local_yaml('x', 'n', {}, dirs, 'test')
        assert [p.parent for p in written] == dirs


class TestTargets:
    def test_output_matches_the_tracked_default_yaml(self, target):
        tracked = yaml.safe_load((ROOT / 'config' / f'{target.cal_name}.yaml').read_text())
        assert set(tracked) == {target.node}
        assert set(tracked[target.node]['ros__parameters']) == {
            target.accel_key,
            target.gyro_key,
        }

    def test_realsense_writes_for_the_fusion_node(self):
        tgt = _load('calibrate_realsense_imu').TARGET
        assert (tgt.topic, tgt.node, tgt.cal_name) == (
            '/imu/realsense',
            'imu_fusion_node',
            'realsense_cal',
        )

    def test_lsm9ds1_reads_the_raw_topic(self):
        tgt = _load('calibrate_imu').TARGET
        assert (tgt.topic, tgt.node, tgt.cal_name) == (
            '/imu/lsm9ds1/raw',
            'pit_node',
            'lsm9ds1_cal',
        )


class TestBiasParams:
    def test_gravity_cancels_across_six_positions(self, common, target):
        g, b = 9.81, (0.1, -0.2, 0.05)
        accel = [
            (b[0] + g, b[1], b[2]),
            (b[0] - g, b[1], b[2]),
            (b[0], b[1] + g, b[2]),
            (b[0], b[1] - g, b[2]),
            (b[0], b[1], b[2] + g),
            (b[0], b[1], b[2] - g),
        ]
        gyro = [[0.01, -0.02, 0.03]] * 20
        params = common.imu_bias_params(target, gyro, _positions(accel))
        assert params[target.accel_key] == pytest.approx(list(b))
        assert params[target.gyro_key] == pytest.approx([0.01, -0.02, 0.03])

    def test_missing_position_is_no_result(self, common, target):
        positions = _positions([(0.0, 0.0, 9.81)] * 6)
        positions[3] = []
        assert common.imu_bias_params(target, [[0.0, 0.0, 0.0]], positions) is None

    def test_missing_gyro_is_no_result(self, common, target):
        positions = _positions([(0.0, 0.0, 9.81)] * 6)
        assert common.imu_bias_params(target, [], positions) is None
