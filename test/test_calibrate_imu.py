"""Unit tests for the IMU bias calibrators and their shared local-YAML output."""

from fnmatch import fnmatch
import json
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
            target.scale_key,
            target.gyro_key,
        }

    def test_nodes_declare_the_scale_key(self, target):
        node_file = {'pit_node': 'pit_node.py', 'imu_fusion_node': 'imu_fusion_node.py'}
        src = (ROOT / 'racecar_neo_ros2_driver' / node_file[target.node]).read_text()
        assert f"declare_parameter('{target.scale_key}'" in src

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
        g, b = common.GRAVITY, (0.1, -0.2, 0.05)
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
        assert params[target.scale_key] == pytest.approx([1.0, 1.0, 1.0])
        assert params[target.gyro_key] == pytest.approx([0.01, -0.02, 0.03])

    def test_low_reading_axis_gets_a_scale_that_restores_g(self, common, target):
        # The RealSense read gravity 8% low on the reference car.
        g, k, b = common.GRAVITY, (0.92, 1.0, 1.03), (0.2, 0.0, -0.1)
        accel = []
        for axis in range(3):
            for sign in (1, -1):
                v = list(b)
                v[axis] += sign * g * k[axis]
                accel.append(tuple(v))
        params = common.imu_bias_params(target, [[0.0, 0.0, 0.0]], _positions(accel))
        scale, bias = params[target.scale_key], params[target.accel_key]
        assert bias == pytest.approx(list(b))
        assert scale == pytest.approx([1 / x for x in k])
        corrected_up_x = (b[0] + g * k[0] - bias[0]) * scale[0]
        assert corrected_up_x == pytest.approx(g)

    def test_wrong_position_is_refused(self, common, target):
        g = common.GRAVITY
        # Y+ held with Z up: the Y axis never sees gravity.
        accel = [(g, 0, 0), (-g, 0, 0), (0, 0, g), (0, -g, 0), (0, 0, g), (0, 0, -g)]
        with pytest.raises(ValueError, match='Y axis'):
            common.imu_bias_params(target, [[0.0, 0.0, 0.0]], _positions(accel))

    def test_missing_position_is_no_result(self, common, target):
        positions = _positions([(0.0, 0.0, 9.81)] * 6)
        positions[3] = []
        assert common.imu_bias_params(target, [[0.0, 0.0, 0.0]], positions) is None

    def test_missing_gyro_is_no_result(self, common, target):
        positions = _positions([(0.0, 0.0, 9.81)] * 6)
        assert common.imu_bias_params(target, [], positions) is None


class TestRestCheck:
    def test_level_and_still_passes(self, common):
        g = common.GRAVITY
        result = common.rest_check([[0.0, -g, 0.05, 0.001, 0.0, 0.0]] * 10)
        assert result.ok

    def test_gravity_off_by_two_percent_fails(self, common):
        result = common.rest_check([[0.0, -common.GRAVITY * 1.02, 0.0, 0.0, 0.0, 0.0]])
        assert not result.ok

    def test_gyro_drift_fails(self, common):
        result = common.rest_check([[0.0, -common.GRAVITY, 0.0, 0.0, 0.02, 0.0]])
        assert not result.ok

    def test_no_samples(self, common):
        assert common.rest_check([]) is None

    def test_apply_cal_matches_the_node_formula(self, common, target):
        params = {
            target.accel_key: [0.2, 0.6, 0.1],
            target.scale_key: [1.0, 1.02, 1.0],
            target.gyro_key: [0.001, 0.002, 0.003],
        }
        (row,) = common.apply_imu_cal([[0.2, -9.0, 0.1, 0.001, 0.002, 0.003]], target, params)
        assert row == pytest.approx([0.0, (-9.0 - 0.6) * 1.02, 0.0, 0.0, 0.0, 0.0])

    def test_realsense_check_applies_the_stored_cal(self):
        tgt = _load('calibrate_realsense_imu').TARGET
        assert tgt.check_applies_cal is True

    def test_lsm9ds1_check_reads_the_calibrated_topic(self):
        tgt = _load('calibrate_imu').TARGET
        assert (tgt.check_topic, tgt.check_applies_cal) == ('/imu/lsm9ds1', False)


class TestNodeParams:
    def test_local_overrides_base(self, common, tmp_path):
        (tmp_path / 'x.yaml').write_text(
            yaml.safe_dump({'n': {'ros__parameters': {'a': [0, 0, 0], 'b': [1, 1, 1]}}})
        )
        (tmp_path / 'x.local.yaml').write_text(
            yaml.safe_dump({'n': {'ros__parameters': {'a': [1, 2, 3]}}})
        )
        assert common.node_params('x', 'n', tmp_path) == {'a': [1, 2, 3], 'b': [1, 1, 1]}

    def test_missing_files(self, common, tmp_path):
        assert common.node_params('absent', 'n', tmp_path) == {}


class TestCheckReport:
    def test_overall_ignores_info_and_takes_the_worst(self, common):
        C = common.Criterion
        assert common.overall([C('a', '', '', 'PASS'), C('b', '', '', 'INFO')]) == 'PASS'
        assert common.overall([C('a', '', '', 'WARN'), C('b', '', '', 'PASS')]) == 'WARN'
        assert common.overall([C('a', '', '', 'WARN'), C('b', '', '', 'FAIL')]) == 'FAIL'

    def test_advice_only_under_warn_and_fail(self, common):
        C = common.Criterion
        lines = common.criteria_lines(
            [C('ok', '1', '2', 'PASS', 'hidden'), C('bad', '3', '4', 'FAIL', 'do this')]
        )
        assert len(lines) == 3 and lines[-1].strip() == '-> do this'

    def test_finish_check_writes_the_report_and_exit_code(self, common, tmp_path):
        class _Log:
            def info(self, _msg):
                pass

        report = tmp_path / 'r.json'
        crit = [common.Criterion('g', '9.8', '1%', 'WARN', 'x')]
        assert common.finish_check(_Log(), crit, str(report)) == 0
        data = json.loads(report.read_text())
        assert data['overall'] == 'WARN' and data['criteria'][0]['name'] == 'g'

    def test_rest_criteria_names(self, common):
        result = common.rest_check([[0.0, -common.GRAVITY, 0.0, 0.0, 0.0, 0.0]])
        assert [c.name for c in result.criteria()] == ['gravity', 'gyro at rest']
