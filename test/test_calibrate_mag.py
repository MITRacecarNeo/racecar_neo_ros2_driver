"""Unit tests for scripts/calibrate_mag.py (ellipsoid fit and local YAML output)."""

from pathlib import Path

from conftest import load_script
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def mag():
    return load_script('calibrate_mag')


def _ellipsoid(n, offset, axes, radius=50e-6, seed=1):
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return (v * radius * np.asarray(axes) + np.asarray(offset)).tolist()


class TestFailurePath:
    def test_too_few_samples_is_no_fit(self, mag):
        samples = _ellipsoid(mag.MIN_SAMPLES - 1, (0, 0, 0), (1, 1, 1))
        assert mag.fit_ellipsoid(samples) is None

    def test_too_few_samples_writes_nothing(self, mag, tmp_path):
        samples = _ellipsoid(mag.MIN_SAMPLES - 1, (1e-5, 0, 0), (1, 1, 1))
        assert mag.fit_and_write(samples, [tmp_path]) is None
        assert list(tmp_path.iterdir()) == []

    def test_degenerate_data_writes_nothing(self, mag, tmp_path):
        samples = [[1e-5, 2e-5, 3e-5]] * (mag.MIN_SAMPLES * 2)
        assert mag.fit_and_write(samples, [tmp_path]) is None
        assert list(tmp_path.iterdir()) == []

    def test_planar_data_writes_nothing(self, mag, tmp_path):
        # Yaw-only rotation leaves z unconstrained; that is not an ellipsoid.
        samples = [[5e-5 * np.cos(t), 5e-5 * np.sin(t), 0.0] for t in np.linspace(0, 6, 300)]
        assert mag.fit_and_write(samples, [tmp_path]) is None
        assert list(tmp_path.iterdir()) == []


class TestFit:
    OFFSET = (1.0e-5, -5.0e-6, 3.0e-6)
    AXES = (1.2, 1.0, 0.8)

    def test_recovers_hard_iron_offset(self, mag):
        fit = mag.fit_ellipsoid(_ellipsoid(500, self.OFFSET, self.AXES))
        assert fit is not None
        assert fit.hard_iron == pytest.approx(self.OFFSET, abs=1e-9)

    def test_correction_maps_ellipsoid_to_sphere(self, mag):
        samples = _ellipsoid(500, self.OFFSET, self.AXES)
        fit = mag.fit_ellipsoid(samples)
        corrected = (fit.soft_iron @ (np.asarray(samples) - fit.hard_iron).T).T
        norms = np.linalg.norm(corrected, axis=1)
        assert norms.std() / norms.mean() < 1e-6


class TestLocalYaml:
    def test_writes_local_yaml_in_every_dir(self, mag, tmp_path):
        src, share = tmp_path / 'src', tmp_path / 'share'
        src.mkdir()
        share.mkdir()
        result = mag.fit_and_write(_ellipsoid(500, (1e-5, 0, 0), (1, 1, 1)), [src, share])
        assert result is not None
        _, written = result
        assert written == [
            src / 'lsm9ds1_mag_cal.local.yaml',
            share / 'lsm9ds1_mag_cal.local.yaml',
        ]
        assert all(p.is_file() for p in written)

    def test_writes_only_keys_pit_node_declares(self, mag, tmp_path):
        mag.fit_and_write(_ellipsoid(500, (1e-5, 0, 0), (1, 1, 1)), [tmp_path])
        doc = yaml.safe_load((tmp_path / 'lsm9ds1_mag_cal.local.yaml').read_text())
        tracked = yaml.safe_load((ROOT / 'config' / 'lsm9ds1_mag_cal.yaml').read_text())
        assert set(doc) == {'pit_node'}
        assert set(doc['pit_node']['ros__parameters']) == set(
            tracked['pit_node']['ros__parameters']
        )
        soft = doc['pit_node']['ros__parameters']['magnetometer.soft_iron_matrix.data']
        assert len(soft) == 9
