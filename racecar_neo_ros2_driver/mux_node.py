"""
Command mux: gates /gamepad_drive (LB) or /drive (RB) onto /mux_out.

Timer-driven so the Maestro stays fed and the watchdog sees a steady publish
rate. Zeroes on joy disconnect or stale upstream commands (>0.5s).

A FlySky transmitter can take the gate instead of the bumpers. That is off by
default (rc_authority_enable): not every car has a transmitter, and the link
predicate is the least-verified part of the path. Where it is enabled and a
live transmitter is present, the mode channel selects idle, manual or
autonomous and the bumpers are ignored; everywhere else the bumpers govern
exactly as before.
"""

from enum import auto, Enum
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray


class MuxMode(Enum):
    IDLE = auto()
    GAMEPAD = auto()
    AUTONOMY = auto()


def select_mode(buttons, gamepad_btn: int, auto_btn: int) -> MuxMode:
    """Pick the mux mode from the latest /joy button state."""
    gp = len(buttons) > gamepad_btn and bool(buttons[gamepad_btn])
    ao = len(buttons) > auto_btn and bool(buttons[auto_btn])
    if gp and not ao:
        return MuxMode.GAMEPAD
    if ao and not gp:
        return MuxMode.AUTONOMY
    return MuxMode.IDLE


def select_rc_mode(channels, channel: int, deadband: float) -> MuxMode:
    """
    Pick the mux mode from the RC mode channel.

    Up (positive) is manual, down (negative) is autonomous, the middle band is
    idle. A channel index past the end of the frame reads as idle rather than
    raising, so a short frame cannot arm anything.
    """
    if channel < 0 or channel >= len(channels):
        return MuxMode.IDLE
    value = float(channels[channel])
    if value > deadband:
        return MuxMode.GAMEPAD
    if value < -deadband:
        return MuxMode.AUTONOMY
    return MuxMode.IDLE


class RcAuthority:
    """
    Decide whether a live FlySky transmitter may hold the drive gate.

    Kept out of the node so the rules can be exercised without ROS. Granting is
    deliberately slow and revoking immediate: the link must look good
    continuously for hold_sec before it takes the gate, and a single bad frame
    hands the gate straight back to the bumpers. The two errors do not cost the
    same, so they are not treated the same.
    """

    def __init__(self, timeout_sec: float = 0.5, hold_sec: float = 1.0):
        self._timeout = timeout_sec
        self._hold = hold_sec
        self._valid_since = None
        self._seen_change = False
        self._last_channels = None
        self.armed = False

    def reset(self):
        """Drop any progress toward authority, including the arming state."""
        self._valid_since = None
        self._seen_change = False
        self.armed = False

    def observe_channels(self, channels):
        """
        Record a channel frame, tracking whether the data ever moves.

        A receiver jitters; an uninitialized or frozen buffer holding a
        plausible mid-band constant does not. Authority is withheld until at
        least one frame differs from the one before it, which the band check
        alone cannot catch. This gates the initial grant only: a transmitter
        held perfectly still after that keeps the gate.
        """
        current = tuple(float(c) for c in channels)
        if self._last_channels is not None and current != self._last_channels:
            self._seen_change = True
        self._last_channels = current

    def update(self, now, link_up, link_stamp, channels, channels_stamp) -> bool:
        """Return whether the transmitter holds the gate at time `now`."""
        fresh = (
            link_stamp is not None
            and channels_stamp is not None
            and (now - link_stamp) <= self._timeout
            and (now - channels_stamp) <= self._timeout
        )
        if not (fresh and link_up and channels):
            self.reset()
            return False
        if self._valid_since is None:
            self._valid_since = now
        if not self._seen_change:
            return False
        return (now - self._valid_since) >= self._hold

    def gate(self, mode: MuxMode) -> MuxMode:
        """
        Hold the mode at idle until the switch has been seen at middle.

        The analogue of the joystick arming check: a car powered on with the
        switch already down must not go autonomous the moment the link comes up.
        """
        if mode == MuxMode.IDLE:
            self.armed = True
        return mode if self.armed else MuxMode.IDLE


def joy_is_centered(axes, threshold: float = 0.2, ignore_axes=()) -> bool:
    """
    Return True when every non-ignored axis magnitude is below threshold.

    Xbox-mode triggers (axes 2 and 5 on the EasySMX) rest at +1.0, not 0.0,
    so they must be excluded from the arming check or the mux never arms.
    """
    ignore = set(ignore_axes)
    return all(
        abs(float(a)) < threshold
        for i, a in enumerate(axes)
        if i not in ignore
    )


class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)
        self.declare_parameter('joystick_timeout_sec', 0.5)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('startup_grace_sec', 1.0)
        self.declare_parameter('arm_axis_threshold', 0.2)
        self.declare_parameter('arm_ignore_axes', [2, 5])

        # FlySky authority. Off by default: not every car has a transmitter,
        # and rc_link_up is the least-verified part of this path. Enable per
        # car once the mode channel has been confirmed on the bench.
        self.declare_parameter('rc_authority_enable', False)
        self.declare_parameter('rc_mode_channel', 5)     # 0-indexed; FlySky CH6
        self.declare_parameter('rc_mode_deadband', 0.35)
        self.declare_parameter('rc_timeout_sec', 0.5)
        self.declare_parameter('rc_link_hold_sec', 1.0)
        self.declare_parameter('rc_link_topic', '/rc/link')
        self.declare_parameter('rc_channels_topic', '/rc/channels')

        self._gamepad_btn = self.get_parameter('gamepad_enable_button').value
        self._auto_btn = self.get_parameter('autonomy_enable_button').value
        self._joy_timeout = self.get_parameter('joystick_timeout_sec').value
        self._cmd_timeout = self.get_parameter('command_timeout_sec').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self._startup_grace = self.get_parameter('startup_grace_sec').value
        self._arm_threshold = self.get_parameter('arm_axis_threshold').value
        self._arm_ignore_axes = tuple(self.get_parameter('arm_ignore_axes').value)

        self._rc_enable = bool(self.get_parameter('rc_authority_enable').value)
        self._rc_channel = int(self.get_parameter('rc_mode_channel').value)
        self._rc_deadband = float(self.get_parameter('rc_mode_deadband').value)
        self._rc = RcAuthority(
            timeout_sec=float(self.get_parameter('rc_timeout_sec').value),
            hold_sec=float(self.get_parameter('rc_link_hold_sec').value),
        )
        self._rc_link = False
        self._rc_link_stamp = None
        self._rc_channels = []
        self._rc_channels_stamp = None
        self._rc_held = False

        self._latest_joy: Joy = None
        self._joy_stamp = 0.0
        self._joy_connected = False

        self._latest_gamepad: AckermannDriveStamped = None
        self._gamepad_stamp = 0.0

        self._latest_auto: AckermannDriveStamped = None
        self._auto_stamp = 0.0

        self._last_mode = MuxMode.IDLE
        self._armed = False
        self._boot_time = time.monotonic()

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/mux_out', qos)

        self.create_subscription(Joy, '/joy', self._joy_cb, qos)
        self.create_subscription(
            AckermannDriveStamped, '/gamepad_drive', self._gamepad_cb, qos
        )
        self.create_subscription(
            AckermannDriveStamped, '/drive', self._auto_cb, qos
        )

        # Subscribed only when enabled, so a car with the feature off carries
        # no extra subscriptions and behaves exactly as it did before.
        if self._rc_enable:
            self.create_subscription(
                Bool, self.get_parameter('rc_link_topic').value, self._rc_link_cb, qos
            )
            self.create_subscription(
                Float32MultiArray,
                self.get_parameter('rc_channels_topic').value,
                self._rc_channels_cb,
                qos,
            )

        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'Mux ready: gamepad btn={self._gamepad_btn}, autonomy btn={self._auto_btn}, '
            f'joy timeout={self._joy_timeout}s, cmd timeout={self._cmd_timeout}s, '
            f'rate={publish_rate}Hz, '
            f'rc_authority={"on" if self._rc_enable else "off"}'
        )

    def _joy_cb(self, msg: Joy):
        self._latest_joy = msg
        self._joy_stamp = time.monotonic()
        if not self._joy_connected:
            self._joy_connected = True
            self.get_logger().info('Controller connected')

    def _rc_link_cb(self, msg: Bool):
        self._rc_link = bool(msg.data)
        self._rc_link_stamp = time.monotonic()

    def _rc_channels_cb(self, msg: Float32MultiArray):
        self._rc_channels = list(msg.data)
        self._rc_channels_stamp = time.monotonic()
        self._rc.observe_channels(self._rc_channels)

    def _gamepad_cb(self, msg: AckermannDriveStamped):
        self._latest_gamepad = msg
        self._gamepad_stamp = time.monotonic()

    def _auto_cb(self, msg: AckermannDriveStamped):
        self._latest_auto = msg
        self._auto_stamp = time.monotonic()

    def _rc_has_authority(self, now) -> bool:
        """Check whether a live transmitter holds the gate, and log transitions."""
        if not self._rc_enable:
            return False
        held = self._rc.update(
            now,
            self._rc_link,
            self._rc_link_stamp,
            self._rc_channels,
            self._rc_channels_stamp,
        )
        if held != self._rc_held:
            self._rc_held = held
            self.get_logger().info(
                'RC transmitter holds the drive gate' if held
                else 'RC transmitter released the drive gate'
            )
        return held

    def _joy_mode(self, now):
        """
        Pick the mode from the gamepad, or None when it cannot be trusted.

        None means publish zero and log no mode change: the controller is
        missing, has gone stale, or the boot-time arming check has not passed.
        """
        joy = self._latest_joy
        if joy is None or (now - self._joy_stamp) > self._joy_timeout:
            if self._joy_connected and joy is not None:
                self._joy_connected = False
                self.get_logger().warn('Controller disconnected — publishing zero')
            return None

        # Boot-time arming: require an idle period plus a centered Joy frame
        # before honoring bumper presses, so a stuck stick at power-on can't move the robot.
        if not self._armed:
            grace_elapsed = (now - self._boot_time) >= self._startup_grace
            if grace_elapsed and joy_is_centered(
                joy.axes, self._arm_threshold, self._arm_ignore_axes,
            ):
                self._armed = True
                self.get_logger().info('Mux armed')
            else:
                return None

        return select_mode(joy.buttons, self._gamepad_btn, self._auto_btn)

    def _publish(self):
        now = time.monotonic()
        out = AckermannDriveStamped()

        if self._rc_has_authority(now):
            mode = self._rc.gate(
                select_rc_mode(self._rc_channels, self._rc_channel, self._rc_deadband)
            )
        else:
            mode = self._joy_mode(now)
            if mode is None:
                self._pub.publish(out)
                self._last_mode = MuxMode.IDLE
                return

        if mode == MuxMode.GAMEPAD:
            if (
                self._latest_gamepad is not None
                and (now - self._gamepad_stamp) <= self._cmd_timeout
            ):
                out = self._latest_gamepad
        elif mode == MuxMode.AUTONOMY:
            if (
                self._latest_auto is not None
                and (now - self._auto_stamp) <= self._cmd_timeout
            ):
                out = self._latest_auto

        if mode != self._last_mode:
            self.get_logger().info(f'Mode → {mode.name}')
            self._last_mode = mode

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
