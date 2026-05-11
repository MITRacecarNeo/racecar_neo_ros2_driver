"""Unit tests for pwm_node.command_to_pwm helper."""

from racecar_neo_ros2_driver.pwm_node import command_to_pwm


class TestMotor:
    """sign=+1 (motor): cmd=+1 → center + magnitude."""

    def test_neutral(self):
        assert command_to_pwm(0.0, center=6000, magnitude=3000, sign=+1) == 6000

    def test_full_forward(self):
        assert command_to_pwm(1.0, center=6000, magnitude=3000, sign=+1) == 9000

    def test_full_reverse(self):
        assert command_to_pwm(-1.0, center=6000, magnitude=3000, sign=+1) == 3000

    def test_clamp_above_one(self):
        assert command_to_pwm(2.0, center=6000, magnitude=3000, sign=+1) == 9000

    def test_clamp_below_neg_one(self):
        assert command_to_pwm(-2.0, center=6000, magnitude=3000, sign=+1) == 3000

    def test_asymmetric_center(self):
        # ESCs with off-center trim still map linearly.
        assert command_to_pwm(1.0, center=5800, magnitude=3000, sign=+1) == 8800
        assert command_to_pwm(-1.0, center=5800, magnitude=3000, sign=+1) == 2800


class TestSteering:
    """sign=-1 (steering): cmd=+1 (left) → center - magnitude."""

    def test_neutral(self):
        assert command_to_pwm(0.0, center=6000, magnitude=2000, sign=-1) == 6000

    def test_full_left(self):
        assert command_to_pwm(1.0, center=6000, magnitude=2000, sign=-1) == 4000

    def test_full_right(self):
        assert command_to_pwm(-1.0, center=6000, magnitude=2000, sign=-1) == 8000


class TestV1Equivalence:
    """End-to-end: with throttle defaults + pwm defaults, recover v1's PWM targets."""

    def test_full_forward_matches_v1_target(self):
        # stick=+1 → throttle (max_fwd=0.17) → 0.17 → pwm → 6000 + 0.17*3000 = 6510
        cmd = 0.17  # output of scale_speed(1.0, 0.17, 0.24)
        assert command_to_pwm(cmd, 6000, 3000, sign=+1) == 6510

    def test_full_reverse_matches_v1_target(self):
        # stick=-1 → throttle (max_back=0.24) → -0.24 → pwm → 6000 - 0.24*3000 = 5280
        cmd = -0.24
        assert command_to_pwm(cmd, 6000, 3000, sign=+1) == 5280

    def test_full_steering_matches_v1_target(self):
        # stick=+1 → throttle (max_steer=0.625) → 0.625 → pwm → 6000 - 0.625*2000 = 4750
        cmd = 0.625
        assert command_to_pwm(cmd, 6000, 2000, sign=-1) == 4750
