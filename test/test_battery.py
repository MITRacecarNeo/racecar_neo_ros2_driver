"""Unit tests for battery: charge curve, cell count, config, estimate, capacity learning."""

from itertools import pairwise
from pathlib import Path

import pytest
from racecar_neo_ros2_driver.battery import (
    battery_soc,
    BatteryConfig,
    CapacityLearner,
    CURVE_MV,
    CURVE_PCT,
    detect_cells,
    Estimate,
    estimate,
    soc_percent,
)
import yaml

CONFIG = Path(__file__).parent.parent / 'config' / 'battery.yaml'
NOW = 1000.0
# Charge left at the 7.1 V floor on the default curve: 2% at 7195 mV, 0% at 7000.
FLOOR_SOC = 2.0 * 100 / 195


def _steady(volts, amps, n=300, now=NOW):
    """Return n samples at 1 Hz ending at `now`."""
    return [(now - n + 1 + k, volts, amps) for k in range(n)]


class TestSocPercent:
    @pytest.mark.parametrize('mv,pct', zip(CURVE_MV[1:], CURVE_PCT[1:], strict=True))
    def test_curve_points(self, mv, pct):
        assert soc_percent(mv / 1000, CURVE_MV, CURVE_PCT) == pytest.approx(pct)

    def test_interpolates_between_points(self):
        # 7545 mV is 50%, 7620 mV is 60%.
        assert soc_percent(7.5825, CURVE_MV, CURVE_PCT) == pytest.approx(55.0)

    def test_clamps_above_full_and_below_empty(self):
        assert soc_percent(8.6, CURVE_MV, CURVE_PCT) == 100.0
        assert soc_percent(8.3, CURVE_MV, CURVE_PCT) == 100.0
        assert soc_percent(6.5, CURVE_MV, CURVE_PCT) == 0.0

    def test_falls_with_voltage(self):
        socs = [soc_percent(mv / 1000, CURVE_MV, CURVE_PCT) for mv in range(8400, 6999, -5)]
        assert all(a >= b for a, b in pairwise(socs))

    def test_scales_per_cell(self):
        cfg = BatteryConfig()
        assert battery_soc(7.545, 2, cfg) == pytest.approx(50.0)
        assert battery_soc(7.545 * 1.5, 3, cfg) == pytest.approx(50.0)


class TestDetectCells:
    @pytest.mark.parametrize(
        'volts,cells',
        [(8.4, 2), (7.0, 2), (6.6, 2), (12.6, 3), (10.0, 3), (4.0, 1), (16.0, 4), (25.0, 6)],
    )
    def test_counts(self, volts, cells):
        assert detect_cells(volts) == cells

    @pytest.mark.parametrize('volts', [0.0, 3.0, 5.0, 9.0, 30.0])
    def test_unknown(self, volts):
        assert detect_cells(volts) is None


class TestConfig:
    def _yaml_params(self):
        return yaml.safe_load(CONFIG.read_text())['battery_guard']['ros__parameters']

    def test_yaml_matches_defaults(self):
        assert BatteryConfig.from_params(self._yaml_params()) == BatteryConfig()

    def test_yaml_sets_every_field(self):
        assert set(self._yaml_params()) == set(BatteryConfig.__dataclass_fields__)

    def test_from_params_ignores_unknown_and_keeps_defaults(self):
        cfg = BatteryConfig.from_params({'capacity_mah': 5000.0, 'use_sim_time': False})
        assert cfg.capacity_mah == 5000.0
        assert cfg.window_sec == 300.0

    def test_from_params_takes_lists(self):
        cfg = BatteryConfig.from_params({'curve_mv': [8400, 7000], 'curve_pct': [100, 0]})
        assert cfg.curve_mv == (8400, 7000)

    def test_floor_and_plausible_scale_with_cells(self):
        cfg = BatteryConfig()
        assert cfg.floor_volts(2) == pytest.approx(7.1)
        assert cfg.floor_volts(3) == pytest.approx(10.65)
        assert cfg.plausible(7.5, 2) and not cfg.plausible(11.1, 2)
        assert cfg.plausible(11.1, 3) and not cfg.plausible(7.5, 3)

    @pytest.mark.parametrize(
        'mv,pct',
        [
            ((8400, 7000), (100, 50, 0)),
            ((8400,), (100,)),
            ((8400, 8400, 7000), (100, 90, 0)),
            ((7000, 8400), (0, 100)),
            ((8400, 7000), (0, 100)),
            ((8400, 7000), (120, 0)),
        ],
    )
    def test_rejects_bad_curves(self, mv, pct):
        with pytest.raises(ValueError):
            BatteryConfig(curve_mv=mv, curve_pct=pct)

    @pytest.mark.parametrize(
        'kwargs',
        [
            {'cells': 7},
            {'cells': -1},
            {'curve_cells': 0},
            {'learn_min_drop_pct': 40.0, 'learn_full_drop_pct': 40.0},
        ],
    )
    def test_rejects_bad_settings(self, kwargs):
        with pytest.raises(ValueError):
            BatteryConfig(**kwargs)


class TestEstimate:
    cfg = BatteryConfig()

    def test_steady_discharge(self):
        est = estimate(_steady(7.545, 2.0), NOW, self.cfg, cells=2)
        assert est.valid
        assert est.soc == pytest.approx(50.0)
        assert est.power_w == pytest.approx(15.09)
        # The charge above the floor, of 4.7 Ah, at 2 A.
        assert est.minutes_left == pytest.approx((50.0 - FLOOR_SOC) / 100 * 4.7 / 2.0 * 60)

    def test_capacity_override(self):
        est = estimate(_steady(7.545, 2.0), NOW, self.cfg, cells=2, capacity_mah=4000.0)
        assert est.minutes_left == pytest.approx((50.0 - FLOOR_SOC) / 100 * 4.0 / 2.0 * 60)

    def test_three_cells(self):
        est = estimate(_steady(7.545 * 1.5, 2.0), NOW, self.cfg, cells=3)
        assert est.soc == pytest.approx(50.0)

    def test_empty_is_invalid(self):
        assert estimate([], NOW, self.cfg, cells=2) == Estimate(valid=False)

    def test_stale_is_invalid(self):
        est = estimate(_steady(7.545, 2.0, now=NOW - 6), NOW, self.cfg, cells=2)
        assert not est.valid
        assert est.minutes_left is None

    @pytest.mark.parametrize('volts', [0.0, 5.9, 8.7, 11.1])
    def test_implausible_is_invalid(self, volts):
        samples = _steady(7.545, 2.0)[:-1] + [(NOW, volts, 2.0)]
        assert not estimate(samples, NOW, self.cfg, cells=2).valid

    def test_too_few_samples_has_no_values(self):
        est = estimate(_steady(7.545, 2.0, n=29), NOW, self.cfg, cells=2)
        assert est.valid
        assert est.volts == 7.545
        assert est.soc is None and est.power_w is None and est.minutes_left is None

    def test_low_current_has_power_but_no_minutes(self):
        est = estimate(_steady(7.545, 0.05), NOW, self.cfg, cells=2)
        assert est.power_w == pytest.approx(7.545 * 0.05)
        assert est.minutes_left is None

    def test_old_samples_leave_the_window(self):
        old = [(t - 600, v, 10.0) for t, v, _ in _steady(7.545, 0.0)]
        est = estimate(old + _steady(7.545, 2.0), NOW, self.cfg, cells=2)
        assert est.power_w == pytest.approx(15.09)

    def test_median_ignores_a_load_spike(self):
        samples = _steady(7.545, 2.0)
        samples[-5:] = [(t, 7.2, 8.0) for t, _, _ in samples[-5:]]
        assert estimate(samples, NOW, self.cfg, cells=2).soc == pytest.approx(50.0)

    def test_r_internal_raises_the_voltage(self):
        cfg = BatteryConfig(r_internal_ohm=0.0375)
        # 7.47 V + 2 A * 0.0375 = 7.545 V, 50%.
        assert estimate(_steady(7.47, 2.0), NOW, cfg, cells=2).soc == pytest.approx(50.0)

    @pytest.mark.parametrize('volts', [7.1, 7.05, 7.0])
    def test_no_time_left_at_or_below_the_floor(self, volts):
        assert estimate(_steady(volts, 2.5), NOW, self.cfg, cells=2).minutes_left == 0.0


def _discharge(learner, true_mah, amps=2.0, to_drop=50.0, start_soc=95.0, skip=()):
    """Feed a discharge at a true capacity; return the learner after `to_drop` points."""
    t = 0.0
    used = 0.0
    while start_soc - used / true_mah * 100 > start_soc - to_drop:
        if not any(a <= t < b for a, b in skip):
            learner.update(t, amps, start_soc - used / true_mah * 100)
        t += 1.0
        used += amps / 3.6
    return learner


class TestCapacityLearner:
    cfg = BatteryConfig()

    def test_config_value_before_the_minimum_drop(self):
        learner = _discharge(CapacityLearner(self.cfg), 4000.0, to_drop=15.0)
        assert learner.measured_mah is None
        assert learner.capacity_mah == 4700.0

    def test_measures_a_smaller_battery(self):
        learner = _discharge(CapacityLearner(self.cfg), 4000.0, to_drop=45.0)
        assert learner.measured_mah == pytest.approx(4000.0, rel=0.01)
        assert learner.capacity_mah == pytest.approx(4000.0, rel=0.01)

    def test_blends_between_the_drops(self):
        learner = _discharge(CapacityLearner(self.cfg), 4000.0, to_drop=30.0)
        assert learner.capacity_mah == pytest.approx(4350.0, rel=0.01)

    def test_clamps_to_the_ratio(self):
        learner = _discharge(CapacityLearner(self.cfg), 10000.0, to_drop=45.0)
        assert learner.capacity_mah == pytest.approx(1.2 * 4700.0)

    def test_fills_a_gap_at_the_last_current(self):
        learner = _discharge(CapacityLearner(self.cfg), 4000.0, to_drop=45.0, skip=[(600, 1200)])
        assert learner.measured_mah == pytest.approx(4000.0, rel=0.01)

    def test_counts_from_the_first_charge_reading(self):
        learner = CapacityLearner(self.cfg)
        for t in range(100):
            learner.update(float(t), 5.0, None)
        assert learner.drop_pct == 0.0
        _discharge(learner, 4000.0, to_drop=45.0)
        assert learner.measured_mah == pytest.approx(4000.0, rel=0.01)

    def test_starts_over_when_charge_rises(self):
        learner = _discharge(CapacityLearner(self.cfg), 4000.0, to_drop=45.0)
        learner.update(1e6, 2.0, 95.0)
        assert learner.drop_pct == 0.0
        assert learner.capacity_mah == 4700.0
