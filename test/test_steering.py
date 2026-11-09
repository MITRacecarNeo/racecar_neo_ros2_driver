"""racecar_neo_ros2_driver.steering: the command-to-curvature table."""

import pytest
from racecar_neo_ros2_driver import car_profile, steering
import yaml

# Bill, 2026-10-05: right side first.
COMMAND = [-1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0]
CURVATURE = [-2.15, -2.05, -1.02, 0.0, 0.90, 1.85, 2.39]


def table(**kw):
    return steering.SteeringTable(tuple(COMMAND), tuple(CURVATURE), **kw)


class TestLookup:
    def test_both_directions_and_clamping(self):
        t = table()
        assert t.curvature_for(0.25) == pytest.approx(0.90)
        assert t.curvature_for(0.375) == pytest.approx((0.90 + 1.85) / 2)
        assert t.command_for(1.85) == pytest.approx(0.5)
        assert t.command_for(-1.53) == pytest.approx(-0.375, abs=0.01)
        assert t.command_for(9.0) == 1.0 and t.command_for(-9.0) == -1.0
        assert t.max_curvature() == (-2.15, 2.39)

    def test_reverse_table_used_only_when_present(self):
        rev = tuple(k * 1.2 for k in CURVATURE)
        assert table().curvature_for(0.5, reverse=True) == pytest.approx(1.85)
        assert table(curvature_reverse=rev).curvature_for(0.5, reverse=True) == pytest.approx(
            1.85 * 1.2
        )


class TestProblems:
    def test_good_table(self):
        assert steering.table_problem(COMMAND, CURVATURE) == ''

    def test_flat_or_falling_curvature_is_named(self):
        bad = list(CURVATURE)
        bad[5] = 0.80
        assert 'falls from command +0.25' in steering.table_problem(COMMAND, bad)

    def test_unsorted_or_short(self):
        assert 'increasing order' in steering.table_problem([0, -1, 1], [0, -1, 1])
        assert 'at least three' in steering.table_problem([0, 1], [0, 1])


class TestStorage:
    def test_round_trip_through_the_profile(self, car_profile_dir):
        t = table(curvature_reverse=tuple(CURVATURE), speed=0.25, trim=0.01, measured='today')
        car_profile.write_params(steering.NAME, '/**', steering.params_of(t), '# test\n')
        assert steering.load() == t
        raw = yaml.safe_load(car_profile.param_path(steering.NAME).read_text())
        assert raw['/**']['ros__parameters']['steering_table.command'] == COMMAND

    def test_nested_keys_also_read(self, tmp_path):
        path = tmp_path / 't.yaml'
        path.write_text(
            yaml.safe_dump(
                {
                    '/**': {
                        'ros__parameters': {
                            'steering_table': {'command': COMMAND, 'curvature_per_m': CURVATURE}
                        }
                    }
                }
            )
        )
        assert steering.load(path).command == tuple(COMMAND)

    def test_missing_or_broken_is_none(self, tmp_path):
        assert steering.load(tmp_path / 'none.yaml') is None
        bad = tmp_path / 'bad.yaml'
        bad.write_text(
            yaml.safe_dump(
                {'/**': {'ros__parameters': {'steering_table.command': [0, 1], 'x': 1}}}
            )
        )
        assert steering.load(bad) is None


class TestFullLockFlat:
    def test_flat_end_allowed_and_inverse_uses_the_smaller_command(self):
        k = (-2.27, -2.27, -2.19, 0.0, 1.86, 2.48, 2.55)
        c = (-1.0, -0.75, -0.5, 0.0, 0.5, 0.75, 1.0)
        assert steering.table_problem(list(c), list(k)) == ''
        t = steering.SteeringTable(c, k)
        assert t.command_for(-2.27) == -0.75 and t.command_for(-9.0) == -0.75
        assert t.curvature_for(-1.0) == -2.27
