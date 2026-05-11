"""Unit tests for imu_node math helpers."""

import numpy as np
import pytest

from racecar_neo_ros2_driver.imu_node import apply_mag_calibration, twos_complement


class TestTwosComplement:
    def test_zero(self):
        assert twos_complement(0, 16) == 0

    def test_max_positive_16bit(self):
        assert twos_complement(0x7FFF, 16) == 32767

    def test_min_negative_16bit(self):
        assert twos_complement(0x8000, 16) == -32768

    def test_negative_one_16bit(self):
        assert twos_complement(0xFFFF, 16) == -1

    def test_8bit(self):
        assert twos_complement(0xFF, 8) == -1
        assert twos_complement(0x80, 8) == -128
        assert twos_complement(0x7F, 8) == 127


class TestMagCalibration:
    def test_identity_no_op(self):
        raw = np.array([1e-5, 2e-5, 3e-5])
        result = apply_mag_calibration(raw, [0, 0, 0], np.identity(3).flatten())
        assert np.allclose(result, raw)

    def test_hard_iron_subtracts(self):
        raw = np.array([5.0, 5.0, 5.0])
        result = apply_mag_calibration(raw, [1.0, 2.0, 3.0], np.identity(3).flatten())
        assert np.allclose(result, [4.0, 3.0, 2.0])

    def test_soft_iron_scales(self):
        raw = np.array([1.0, 1.0, 1.0])
        soft = [2.0, 0.0, 0.0,
                0.0, 3.0, 0.0,
                0.0, 0.0, 4.0]
        result = apply_mag_calibration(raw, [0, 0, 0], soft)
        assert np.allclose(result, [2.0, 3.0, 4.0])

    def test_soft_iron_after_hard_iron(self):
        # Order matters: hard-iron offset applied first, then soft-iron matrix.
        raw = np.array([3.0, 5.0, 7.0])
        hard = [1.0, 1.0, 1.0]
        soft = [2.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 2.0]
        # (raw - hard) = [2, 4, 6]; soft @ ... = [4, 8, 12]
        result = apply_mag_calibration(raw, hard, soft)
        assert np.allclose(result, [4.0, 8.0, 12.0])

    def test_accepts_list_inputs(self):
        result = apply_mag_calibration(
            [1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        assert np.allclose(result, [1.0, 2.0, 3.0])


@pytest.mark.parametrize('bits', [4, 8, 12, 16, 24, 32])
def test_twos_complement_round_trip(bits):
    for value in [0, 1, (1 << (bits - 1)) - 1, 1 << (bits - 1), (1 << bits) - 1]:
        signed = twos_complement(value, bits)
        # Convert back to unsigned and verify it round-trips.
        if signed < 0:
            assert (signed + (1 << bits)) == value
        else:
            assert signed == value
