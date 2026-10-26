"""Unit tests for battery: charge curve, config, and the time-left estimate."""

from itertools import pairwise
from pathlib import Path

import pytest
from racecar_neo_ros2_driver.battery import (
    BatteryConfig,
    CURVE_MV,
    CURVE_PCT,
    Estimate,
    estimate,
    soc_percent,
)
import yaml

CONFIG = Path(__file__).parent.parent / 'config' / 'battery.yaml'
NOW = 1000.0


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


class TestConfig:
    def test_yaml_matches_defaults(self):
        params = yaml.safe_load(CONFIG.read_text())['battery_guard']['ros__parameters']
        assert BatteryConfig.from_params(params) == BatteryConfig()

    def test_yaml_sets_every_field(self):
        params = yaml.safe_load(CONFIG.read_text())['battery_guard']['ros__parameters']
        assert set(params) == set(BatteryConfig.__dataclass_fields__)

    def test_from_params_ignores_unknown_and_keeps_defaults(self):
        cfg = BatteryConfig.from_params({'capacity_mah': 5000.0, 'use_sim_time': False})
        assert cfg.capacity_mah == 5000.0
        assert cfg.window_sec == 300.0

    def test_from_params_takes_lists(self):
        cfg = BatteryConfig.from_params({'curve_mv': [8400, 7000], 'curve_pct': [100, 0]})
        assert cfg.curve_mv == (8400, 7000)

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


class TestEstimate:
    cfg = BatteryConfig()

    def test_steady_discharge(self):
        est = estimate(_steady(7.545, 2.0), NOW, self.cfg)
        assert est.valid
        assert est.soc == pytest.approx(50.0)
        assert est.power_w == pytest.approx(15.09)
        # 50% of 4.7 Ah at 2 A.
        assert est.minutes_left == pytest.approx(70.5)

    def test_capacity_scales_minutes(self):
        cfg = BatteryConfig(capacity_mah=5000.0)
        assert estimate(_steady(7.545, 2.0), NOW, cfg).minutes_left == pytest.approx(75.0)

    def test_empty_is_invalid(self):
        assert estimate([], NOW, self.cfg) == Estimate(valid=False)

    def test_stale_is_invalid(self):
        est = estimate(_steady(7.545, 2.0, now=NOW - 6), NOW, self.cfg)
        assert not est.valid
        assert est.minutes_left is None

    @pytest.mark.parametrize('volts', [0.0, 5.9, 8.7])
    def test_implausible_is_invalid(self, volts):
        samples = _steady(7.545, 2.0)[:-1] + [(NOW, volts, 2.0)]
        assert not estimate(samples, NOW, self.cfg).valid

    def test_too_few_samples_has_no_values(self):
        est = estimate(_steady(7.545, 2.0, n=29), NOW, self.cfg)
        assert est.valid
        assert est.volts == 7.545
        assert est.soc is None and est.power_w is None and est.minutes_left is None

    def test_low_current_has_power_but_no_minutes(self):
        est = estimate(_steady(7.545, 0.05), NOW, self.cfg)
        assert est.power_w == pytest.approx(7.545 * 0.05)
        assert est.minutes_left is None

    def test_old_samples_leave_the_window(self):
        old = [(t - 600, v, 10.0) for t, v, _ in _steady(7.545, 0.0)]
        est = estimate(old + _steady(7.545, 2.0), NOW, self.cfg)
        assert est.power_w == pytest.approx(15.09)

    def test_median_ignores_a_load_spike(self):
        samples = _steady(7.545, 2.0)
        samples[-5:] = [(t, 7.2, 8.0) for t, _, _ in samples[-5:]]
        assert estimate(samples, NOW, self.cfg).soc == pytest.approx(50.0)

    def test_r_internal_raises_the_voltage(self):
        cfg = BatteryConfig(r_internal_ohm=0.0375)
        # 7.47 V + 2 A * 0.0375 = 7.545 V, 50%.
        assert estimate(_steady(7.47, 2.0), NOW, cfg).soc == pytest.approx(50.0)

    def test_floor_reads_empty(self):
        est = estimate(_steady(7.0, 2.5), NOW, self.cfg)
        assert est.soc == 0.0
        assert est.minutes_left == 0.0
