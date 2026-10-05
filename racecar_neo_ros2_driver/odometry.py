"""
Odometry: integrate encoder speed and gyro yaw rate into a planar pose.

pit_node feeds every decoded telemetry frame (encoder speed and gyro from the
same frame, stamped with its Teensy sample time) to OdomIntegrator and
publishes the result as TF odom -> base_link and /odom/integrated. Running it
inside pit_node costs about 3% of a core; a separate node paid about 8% per
subscribed topic to deserialize the same data (autonomy plan, "Odometry").

The model is a unicycle at base_link, the rear axle midpoint, where a
front-steered car has no sideslip:

    theta += omega dt;  x += v cos(theta_mid) dt;  y += v sin(theta_mid) dt

The gyro measures yaw directly, so no wheelbase or steering constant is
needed.

Gyro bias: with the wheels stopped (speed exactly 0 for zero_hold_sec) and
the yaw rate under zero_max_rate, the true yaw rate is 0 (a front-steered car
cannot turn in place), so the reading feeds a low-pass bias estimate
(bias_tau_sec) and heading is held. The rate limit skips a car being carried.

No ROS dependency, so it is unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class OdomState:
    """Pose in odom, latest velocities, and the sample time they belong to (s)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    t: float | None = None


class OdomIntegrator:
    """
    Unicycle integration with a stopped-car gyro bias estimate.

    `step(t, v, omega_raw)` takes a sample time (s), speed (m/s, scaled) and
    yaw rate (rad/s, scaled, bias not yet removed). Returns False when the
    sample was not integrated: the first sample, a time that did not move
    forward, or a gap over `stale_sec` (the pose is held across it).
    """

    def __init__(
        self,
        zero_hold_sec: float = 1.0,
        zero_max_rate: float = math.radians(2.0),
        bias_tau_sec: float = 30.0,
        stale_sec: float = 0.25,
    ) -> None:
        self.zero_hold_sec = zero_hold_sec
        self.zero_max_rate = zero_max_rate
        self.bias_tau_sec = bias_tau_sec
        self.stale_sec = stale_sec
        self.state = OdomState()
        self.bias = 0.0
        self._stopped_since: float | None = None
        self._held = False
        self.gaps = 0

    @property
    def holding(self) -> bool:
        """Return whether the stopped-car condition holds and heading is frozen."""
        return self._held

    def step(self, t: float, v: float, omega_raw: float) -> bool:
        s = self.state
        last, s.t = s.t, t
        if last is None or t <= last:
            return False
        dt = t - last
        if dt > self.stale_sec:
            self.gaps += 1
            self._stopped_since = None
            self._held = False
            return False

        omega = omega_raw - self.bias
        self._held = False
        if v == 0.0 and abs(omega) < self.zero_max_rate:
            if self._stopped_since is None:
                self._stopped_since = t
            if t - self._stopped_since >= self.zero_hold_sec:
                self._held = True
                self.bias += (omega_raw - self.bias) * min(1.0, dt / self.bias_tau_sec)
                omega = 0.0
        else:
            self._stopped_since = None

        mid = s.theta + 0.5 * omega * dt
        s.x += v * math.cos(mid) * dt
        s.y += v * math.sin(mid) * dt
        s.theta = math.atan2(math.sin(s.theta + omega * dt), math.cos(s.theta + omega * dt))
        s.v, s.omega = v, omega
        return True


def yaw_quaternion(theta: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation of theta about z."""
    return 0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)
