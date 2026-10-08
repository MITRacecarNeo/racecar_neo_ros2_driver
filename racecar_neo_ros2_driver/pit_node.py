"""
NEO-PIT board driver: the Pi's single owner of the Teensy UART link.

One node owns the serial port because only one process can:

  - subscribes /motor (AckermannDriveStamped, normalized [-1, 1]) and streams
    command frames to the Teensy at command_rate_hz; the Teensy maps the values
    to servo/ESC PWM;
  - reads telemetry frames and republishes the LSM9DS1 as /imu/lsm9ds1 + /mag
    (imu_fusion_node blends /imu/lsm9ds1 with /imu/realsense into /imu/fused).

Also republishes encoder speed, /odom, battery power and the RC channels, and
forwards the drive mode and display content in each command frame. A stale or
missing /motor command sends neutral; a missing serial device is retried.

The gyro and mag unit scales are unverified; see config/pit.yaml. The
physical-axis convention students rely on is documented in racecar-neo-library
physics.py.
"""

from collections.abc import Sequence
import math
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, Joy, MagneticField
import serial
from std_msgs.msg import Bool, Float32, Float32MultiArray, UInt8MultiArray
from tf2_ros import TransformBroadcaster

from . import pit_protocol as pit
from .limits import clamp
from .mux_node import MuxMode, select_mode
from .odometry import OdomIntegrator, OdomState, yaw_quaternion
from .teensy_clock import TeensyClock

# RxPacket.SystemState bit layout; mirror of the firmware cfg::SYS_STATE.
SYS_MODE_MASK = 0x03
SYS_MODE_IDLE = 0
SYS_MODE_MANUAL = 1
SYS_MODE_AUTO = 2
SYS_DOTMATRIX_ACTIVE = 0x04
SYS_LED_ACTIVE = 0x08
SYS_DRIVER_STARTING = 0x10

# Odometry covariance sentinels. ROS convention puts -1 in the first element of
# a block that was not estimated at all; a large finite variance marks a single
# unestimated component inside a block that is otherwise real.
POSE_UNESTIMATED = -1.0
TWIST_COMPONENT_UNKNOWN = 1e6
SPEED_VARIANCE = 1e-2


def build_odom(speed_mps: float, frame_id: str, child_frame_id: str) -> Odometry:
    """
    Wrap encoder speed as Odometry with only the forward twist filled in.

    The car reports speed, not pose. There is no wheelbase or steering-angle
    calibration on this platform and /motor carries steering normalized to
    [-1, 1] rather than radians, so heading is not integrated and angular.z is
    not derived from the command. Pose is marked unestimated rather than
    published as zeros, which would read as a car sitting at the origin.
    """
    odom = Odometry()
    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id
    odom.twist.twist.linear.x = float(speed_mps)
    odom.pose.covariance[0] = POSE_UNESTIMATED
    odom.twist.covariance[0] = SPEED_VARIANCE
    for i in (7, 14, 21, 28, 35):
        odom.twist.covariance[i] = TWIST_COMPONENT_UNKNOWN
    return odom


# Display payload sizes: 8x24 monochrome bitmap, 84 RGB LED triplets. The
# firmware's led[255] field has 3 spare bytes; encode_command zero-pads them.
DOT_FRAME_LEN = pit.DOT_MATRIX_LEN
LED_FRAME_LEN = 84 * 3

_MODE_BITS = {
    MuxMode.IDLE: SYS_MODE_IDLE,
    MuxMode.GAMEPAD: SYS_MODE_MANUAL,
    MuxMode.AUTONOMY: SYS_MODE_AUTO,
}


def remap_axes(
    vec: Sequence[float], order: Sequence[int], sign: Sequence[float], scale: float = 1.0
) -> np.ndarray:
    """Reorder, sign-flip and scale a 3-vector: out[i] = scale * sign[i] * vec[order[i]]."""
    v = np.asarray(vec, dtype=float)
    return np.array([sign[i] * v[order[i]] for i in range(3)]) * scale


# Odometry: logged when the gyro bias estimate moves by more than this.
ODOM_BIAS_LOG_STEP = math.radians(0.05)


def build_integrated_odom(
    state: OdomState, odom_frame: str, base_frame: str
) -> tuple[TransformStamped, Odometry]:
    """Return the odom -> base_link transform and its Odometry message for a pose."""
    assert state.t is not None
    stamp = Time(nanoseconds=round(state.t * 1e9)).to_msg()
    qx, qy, qz, qw = yaw_quaternion(state.theta)

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = odom_frame
    tf.child_frame_id = base_frame
    tf.transform.translation.x = state.x
    tf.transform.translation.y = state.y
    tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
    tf.transform.rotation.z, tf.transform.rotation.w = qz, qw

    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = odom_frame
    odom.child_frame_id = base_frame
    odom.pose.pose.position.x = state.x
    odom.pose.pose.position.y = state.y
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
    odom.twist.twist.linear.x = state.v
    odom.twist.twist.angular.z = state.omega
    return tf, odom


class PitNode(Node):
    def __init__(self) -> None:
        super().__init__('pit_node')

        self.declare_parameter('serial_port', '/dev/neo-pit-pcb')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('command_rate_hz', 60.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('reconnect_period_sec', 2.0)
        self.declare_parameter('steering_sign', 1)
        self.declare_parameter('speed_sign', 1)
        self.declare_parameter('require_crc', False)

        # Power telemetry republish (INA226 bus voltage V, battery current A) and
        # the eight FlySky RC channels normalized to [-1, 1].
        self.declare_parameter('voltage_topic', '/battery/voltage')
        self.declare_parameter('current_topic', '/battery/current')
        self.declare_parameter('rc_topic', '/rc/channels')
        # Link state from raw pulse widths; see Telemetry.rc_link_up.
        self.declare_parameter('rc_link_topic', '/rc/link')

        # Encoder speed republish + display/LED command forwarding to the Teensy.
        self.declare_parameter('encoder_topic', '/encoder/speed')
        # Odometry wrapper over the same encoder reading. Forward twist only;
        # see build_odom for why pose is left unestimated.
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_child_frame_id', 'base_link')
        self.declare_parameter('dotmatrix_topic', '/dotmatrix/frame')
        self.declare_parameter('led_topic', '/led/pixels')
        # A display command counts as "active" (sets the SystemState flag the
        # Teensy watches) only while received within this window.
        self.declare_parameter('display_timeout_sec', 0.5)
        # Hold the driver_starting flag this long after launch so the Teensy
        # plays its LED loading sweep on a genuine driver start.
        self.declare_parameter('led_startup_sec', 10.0)
        # Mirror the mux buttons so the Teensy dot-matrix glyph tracks the mode.
        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)

        # Integrated odometry (odometry.py): TF odom -> base_link and
        # odometry.topic, from each frame's encoder speed and gyro. Values in
        # config/odom.yaml; per-car scales in odom.local.yaml.
        self.declare_parameter('odometry.enable', True)
        self.declare_parameter('odometry.topic', '/odom/integrated')
        self.declare_parameter('odometry.yaw_rate_axis', 2)
        self.declare_parameter('odometry.yaw_rate_sign', 1.0)
        self.declare_parameter('odometry.speed_scale', 1.0)
        self.declare_parameter('odometry.yaw_rate_scale', 1.0)
        self.declare_parameter('odometry.publish_rate_hz', 50.0)
        self.declare_parameter('odometry.stale_input_sec', 0.25)
        self.declare_parameter('odometry.zero_hold_sec', 1.0)
        self.declare_parameter('odometry.zero_max_rate_dps', 2.0)
        self.declare_parameter('odometry.bias_tau_sec', 30.0)
        self.declare_parameter('odometry.turn_gain', 0.0)
        self.declare_parameter('odometry.wheelbase_m', 0.230)
        self.declare_parameter('odometry.max_steer_deg', 30.0)
        # Stamp telemetry with the Teensy's sample time (teensy_clock.py) in
        # place of the Pi's bursty receive time; false falls back to receive time.
        self.declare_parameter('use_teensy_stamps', True)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('imu_topic', '/imu/lsm9ds1')
        self.declare_parameter('mag_topic', '/mag')
        # Axis remap + unit scales; the verified values live in config/pit.yaml.
        self.declare_parameter('imu.accel_gyro_axis_order', [0, 1, 2])
        self.declare_parameter('imu.accel_gyro_axis_sign', [1.0, 1.0, 1.0])
        self.declare_parameter('imu.mag_axis_order', [0, 1, 2])
        self.declare_parameter('imu.mag_axis_sign', [1.0, 1.0, 1.0])
        self.declare_parameter('imu.accel_scale', 1.0)
        self.declare_parameter('imu.gyro_scale', 1.0)  # deg/s -> rad/s = 0.01745329
        self.declare_parameter('imu.mag_scale', 1.0e-6)  # firmware uT -> Tesla
        # Keys match config/lsm9ds1_cal.yaml and config/lsm9ds1_mag_cal.yaml.
        self.declare_parameter('accelerometer.bias', [0.0, 0.0, 0.0])
        self.declare_parameter('accelerometer.scale', [1.0, 1.0, 1.0])
        self.declare_parameter('gyroscope.bias', [0.0, 0.0, 0.0])
        self.declare_parameter('magnetometer.hard_iron_bias', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'magnetometer.soft_iron_matrix.data', np.identity(3).flatten().tolist()
        )

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('baud').value)
        self._cmd_timeout = float(self.get_parameter('command_timeout_sec').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_sec').value)
        self._steer_sign = int(self.get_parameter('steering_sign').value)
        self._speed_sign = int(self.get_parameter('speed_sign').value)
        self._require_crc = bool(self.get_parameter('require_crc').value)
        self._frame = self.get_parameter('frame_id').value
        self._teensy_clock = (
            TeensyClock() if self.get_parameter('use_teensy_stamps').value else None
        )
        self._publish_mag = bool(self.get_parameter('publish_mag').value)
        self._imu_topic = self.get_parameter('imu_topic').value
        self._mag_topic = self.get_parameter('mag_topic').value
        self._encoder_topic = self.get_parameter('encoder_topic').value
        self._publish_odom = bool(self.get_parameter('publish_odom').value)
        self._odom_topic = self.get_parameter('odom_topic').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._odom_child_frame = self.get_parameter('odom_child_frame_id').value
        self._voltage_topic = self.get_parameter('voltage_topic').value
        self._current_topic = self.get_parameter('current_topic').value
        self._rc_topic = self.get_parameter('rc_topic').value
        self._rc_link_topic = self.get_parameter('rc_link_topic').value
        self._dotmatrix_topic = self.get_parameter('dotmatrix_topic').value
        self._led_topic = self.get_parameter('led_topic').value
        self._display_timeout = float(self.get_parameter('display_timeout_sec').value)
        self._led_startup = float(self.get_parameter('led_startup_sec').value)
        self._gamepad_btn = int(self.get_parameter('gamepad_enable_button').value)
        self._auto_btn = int(self.get_parameter('autonomy_enable_button').value)

        self._ag_order = list(self.get_parameter('imu.accel_gyro_axis_order').value)
        self._ag_sign = list(self.get_parameter('imu.accel_gyro_axis_sign').value)
        self._mag_order = list(self.get_parameter('imu.mag_axis_order').value)
        self._mag_sign = list(self.get_parameter('imu.mag_axis_sign').value)
        self._accel_scale = float(self.get_parameter('imu.accel_scale').value)
        self._gyro_scale = float(self.get_parameter('imu.gyro_scale').value)
        self._mag_scale = float(self.get_parameter('imu.mag_scale').value)
        self._accel_bias = np.array(self.get_parameter('accelerometer.bias').value, float)
        self._accel_cal_scale = np.array(self.get_parameter('accelerometer.scale').value, float)
        self._gyro_bias = np.array(self.get_parameter('gyroscope.bias').value, float)
        self._mag_hard = np.array(self.get_parameter('magnetometer.hard_iron_bias').value, float)
        self._mag_soft = np.array(
            self.get_parameter('magnetometer.soft_iron_matrix.data').value, float
        ).reshape(3, 3)

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub_imu = self.create_publisher(Imu, self._imu_topic, qos)
        self._pub_mag = self.create_publisher(MagneticField, self._mag_topic, qos)
        self._pub_imu_raw = self.create_publisher(Imu, f'{self._imu_topic}/raw', qos)
        self._pub_mag_raw = self.create_publisher(MagneticField, f'{self._mag_topic}/raw', qos)
        self._pub_encoder = self.create_publisher(Float32, self._encoder_topic, qos)
        self._pub_odom = (
            self.create_publisher(Odometry, self._odom_topic, qos) if self._publish_odom else None
        )
        self._pub_voltage = self.create_publisher(Float32, self._voltage_topic, qos)
        self._pub_current = self.create_publisher(Float32, self._current_topic, qos)
        self._pub_rc = self.create_publisher(Float32MultiArray, self._rc_topic, qos)
        self._pub_rc_link = self.create_publisher(Bool, self._rc_link_topic, qos)
        self.create_subscription(AckermannDriveStamped, '/motor', self._motor_cb, qos)
        self.create_subscription(UInt8MultiArray, self._dotmatrix_topic, self._dot_cb, qos)
        self.create_subscription(UInt8MultiArray, self._led_topic, self._led_cb, qos)
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)
        self._setup_odometry(qos)

        self._latest_speed = 0.0
        self._latest_steer = 0.0
        self._cmd_stamp = 0.0
        self._dot_frame: bytes | None = None
        self._dot_stamp = 0.0
        self._led_frame: bytes | None = None
        self._led_stamp = 0.0
        self._joy_buttons: list[int] = []
        self._start_time = time.monotonic()
        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()
        self._crc_fail_count = 0
        self._running = True

        self._open_serial()

        rate = float(self.get_parameter('command_rate_hz').value)
        self.create_timer(1.0 / rate, self._send_command)

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f'PIT ready: port={self._port}@{self._baud}, cmd_rate={rate}Hz, '
            f'require_crc={self._require_crc}'
        )
        self.get_logger().warn(
            'IMU gyro/mag unit scales are unverified; check imu.gyro_scale and '
            f'imu.mag_scale before trusting {self._imu_topic} and {self._mag_topic}.'
        )

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
            self.get_logger().info(f'Opened {self._port}')
            return True
        except (serial.SerialException, OSError) as exc:
            self._ser = None
            self.get_logger().warn(f'Cannot open {self._port}: {exc}; retrying')
            return False

    def _motor_cb(self, msg: AckermannDriveStamped) -> None:
        self._latest_speed = clamp(msg.drive.speed)
        self._latest_steer = clamp(msg.drive.steering_angle)
        self._cmd_stamp = time.monotonic()

    def _dot_cb(self, msg: UInt8MultiArray) -> None:
        data = bytes(bytearray(msg.data))[:DOT_FRAME_LEN]
        self._dot_frame = data.ljust(DOT_FRAME_LEN, b'\x00')
        self._dot_stamp = time.monotonic()

    def _led_cb(self, msg: UInt8MultiArray) -> None:
        data = bytes(bytearray(msg.data))[:LED_FRAME_LEN]
        self._led_frame = data.ljust(LED_FRAME_LEN, b'\x00')
        self._led_stamp = time.monotonic()

    def _joy_cb(self, msg: Joy) -> None:
        self._joy_buttons = list(msg.buttons)

    def _build_display(self, now: float) -> tuple[int, bytes | None, bytes | None]:
        """Build the SystemState byte + dot-matrix/LED payloads for this frame."""
        mode = select_mode(self._joy_buttons, self._gamepad_btn, self._auto_btn)
        state = _MODE_BITS.get(mode, SYS_MODE_IDLE)

        dot = None
        if self._dot_frame is not None and (now - self._dot_stamp) <= self._display_timeout:
            dot = self._dot_frame
            state |= SYS_DOTMATRIX_ACTIVE

        led = None
        if self._led_frame is not None and (now - self._led_stamp) <= self._display_timeout:
            led = self._led_frame
            state |= SYS_LED_ACTIVE

        if (now - self._start_time) < self._led_startup:
            state |= SYS_DRIVER_STARTING

        return state, dot, led

    def _send_command(self) -> None:
        if self._ser is None or not self._ser.is_open:
            return
        now = time.monotonic()
        fresh = (now - self._cmd_stamp) <= self._cmd_timeout
        speed = self._speed_sign * self._latest_speed if fresh else 0.0
        steer = self._steer_sign * self._latest_steer if fresh else 0.0
        state, dot, led = self._build_display(now)
        wire = pit.encode_command(
            clamp(steer), clamp(speed), system_state=state, dot_matrix=dot, led=led
        )
        try:
            with self._write_lock:
                self._ser.write(wire)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'Serial write failed: {exc}; reconnecting')
            self._close_serial()

    def _read_loop(self) -> None:
        buffer = bytearray()
        while self._running:
            if self._ser is None or not self._ser.is_open:
                time.sleep(self._reconnect_period)
                self._open_serial()
                continue
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f'Serial read failed: {exc}; reconnecting')
                self._close_serial()
                continue
            if not chunk:
                continue
            buffer.extend(chunk)
            while True:
                telem, consumed = pit.scan_for_packet(buffer)
                if telem is not None:
                    self._publish_telemetry(telem)
                elif consumed == 0:
                    break

    def _setup_odometry(self, qos: QoSProfile) -> None:
        p = self.get_parameter
        self._odometry: OdomIntegrator | None = None
        if not p('odometry.enable').value:
            return
        axis = int(p('odometry.yaw_rate_axis').value)
        if axis not in (0, 1, 2):
            raise ValueError(f'odometry.yaw_rate_axis must be 0, 1 or 2, got {axis}')
        self._yaw_axis = axis
        self._yaw_gain = float(p('odometry.yaw_rate_sign').value) * float(
            p('odometry.yaw_rate_scale').value
        )
        self._speed_scale = float(p('odometry.speed_scale').value)
        self._odometry = OdomIntegrator(
            zero_hold_sec=float(p('odometry.zero_hold_sec').value),
            zero_max_rate=math.radians(float(p('odometry.zero_max_rate_dps').value)),
            bias_tau_sec=float(p('odometry.bias_tau_sec').value),
            stale_sec=float(p('odometry.stale_input_sec').value),
            turn_gain=float(p('odometry.turn_gain').value),
            wheelbase=float(p('odometry.wheelbase_m').value),
            max_steer=math.radians(float(p('odometry.max_steer_deg').value)),
        )
        # Published from the serial reader thread, not a timer: each timer wake-up
        # rebuilds the executor's wait set over every entity of this node, which
        # costs several times the integration itself.
        self._odom_period = 1.0 / float(p('odometry.publish_rate_hz').value)
        self._odom_next_pub: float | None = None
        self._odom_gaps_logged = 0
        self._odom_bias_logged = 0.0
        self._pub_odom_integrated = self.create_publisher(
            Odometry, str(p('odometry.topic').value), qos
        )
        self._tf = TransformBroadcaster(self)

    def _integrate_odometry(self, stamp_ns: int, speed: float, gyro: Sequence[float]) -> None:
        assert self._odometry is not None
        omega = float(gyro[self._yaw_axis]) * self._yaw_gain
        t = stamp_ns * 1e-9
        self._odometry.step(t, speed * self._speed_scale, omega)
        # A running deadline holds the average at the configured rate; after a
        # gap it restarts from now rather than publishing a burst.
        due = self._odom_next_pub
        if due is None or t >= due:
            self._publish_odometry()
            self._odom_next_pub = (
                t + self._odom_period
                if due is None or t - due > self._odom_period
                else due + self._odom_period
            )
        gaps, bias = self._odometry.gaps, self._odometry.bias
        if gaps != self._odom_gaps_logged:
            self._odom_gaps_logged = gaps
            self.get_logger().warn(
                f'Odometry input gap over {self._odometry.stale_sec:g} s; pose held '
                f'({gaps} so far)'
            )
        if abs(bias - self._odom_bias_logged) > ODOM_BIAS_LOG_STEP:
            self._odom_bias_logged = bias
            self.get_logger().info(f'Gyro bias estimate {math.degrees(bias):+.3f} deg/s')

    def _publish_odometry(self) -> None:
        assert self._odometry is not None
        state = self._odometry.state
        if state.t is None:
            return
        tf, odom = build_integrated_odom(state, self._odom_frame, self._odom_child_frame)
        self._tf.sendTransform(tf)
        self._pub_odom_integrated.publish(odom)

    def _telemetry_stamp(self, timestamp_us: int) -> TimeMsg:
        now = self.get_clock().now()
        if self._teensy_clock is None:
            return now.to_msg()
        restarts = self._teensy_clock.restarts
        stamp_ns = self._teensy_clock.stamp(timestamp_us, now.nanoseconds)
        if self._teensy_clock.restarts != restarts:
            self.get_logger().info(
                'Teensy clock mapping restarted (Teensy reset, reconnect, or Pi clock step)'
            )
        return Time(nanoseconds=stamp_ns).to_msg()

    def _publish_telemetry(self, telem: pit.Telemetry) -> None:
        if self._require_crc and not telem.crc_ok:
            self._crc_fail_count += 1
            if self._crc_fail_count % 100 == 1:
                self.get_logger().warn(
                    f'Dropping telemetry on CRC mismatch (count={self._crc_fail_count})'
                )
            return

        stamp = self._telemetry_stamp(telem.timestamp_us)
        raw_accel = remap_axes(telem.accel, self._ag_order, self._ag_sign, self._accel_scale)
        raw_gyro = remap_axes(telem.gyro, self._ag_order, self._ag_sign, self._gyro_scale)
        accel = (raw_accel - self._accel_bias) * self._accel_cal_scale
        gyro = raw_gyro - self._gyro_bias

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame
        imu.linear_acceleration.x = float(accel[0])
        imu.linear_acceleration.y = float(accel[1])
        imu.linear_acceleration.z = float(accel[2])
        imu.angular_velocity.x = float(gyro[0])
        imu.angular_velocity.y = float(gyro[1])
        imu.angular_velocity.z = float(gyro[2])
        self._pub_imu.publish(imu)

        imu_raw = Imu()
        imu_raw.header.stamp = stamp
        imu_raw.header.frame_id = self._frame
        imu_raw.linear_acceleration.x = float(raw_accel[0])
        imu_raw.linear_acceleration.y = float(raw_accel[1])
        imu_raw.linear_acceleration.z = float(raw_accel[2])
        imu_raw.angular_velocity.x = float(raw_gyro[0])
        imu_raw.angular_velocity.y = float(raw_gyro[1])
        imu_raw.angular_velocity.z = float(raw_gyro[2])
        self._pub_imu_raw.publish(imu_raw)

        if self._odometry is not None:
            stamp_ns = stamp.sec * 10**9 + stamp.nanosec
            self._integrate_odometry(stamp_ns, float(telem.encoder), gyro)

        # Encoder telemetry is vehicle speed in m/s (firmware getCurrentSpeed()).
        enc = Float32()
        enc.data = float(telem.encoder)
        self._pub_encoder.publish(enc)

        if self._pub_odom is not None:
            odom = build_odom(telem.encoder, self._odom_frame, self._odom_child_frame)
            odom.header.stamp = stamp
            self._pub_odom.publish(odom)

        volt = Float32()
        volt.data = float(telem.voltage_v)
        self._pub_voltage.publish(volt)
        curr = Float32()
        curr.data = float(telem.current_a)
        self._pub_current.publish(curr)
        rc = Float32MultiArray()
        rc.data = [float(c) for c in telem.rc_normalized]
        self._pub_rc.publish(rc)
        rc_link = Bool()
        rc_link.data = bool(telem.rc_link_up)
        self._pub_rc_link.publish(rc_link)

        if self._publish_mag:
            raw_mag = remap_axes(telem.mag, self._mag_order, self._mag_sign, self._mag_scale)
            mag_vec = self._mag_soft @ (raw_mag - self._mag_hard)

            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self._frame
            mag.magnetic_field.x = float(mag_vec[0])
            mag.magnetic_field.y = float(mag_vec[1])
            mag.magnetic_field.z = float(mag_vec[2])
            self._pub_mag.publish(mag)

            mag_raw = MagneticField()
            mag_raw.header.stamp = stamp
            mag_raw.header.frame_id = self._frame
            mag_raw.magnetic_field.x = float(raw_mag[0])
            mag_raw.magnetic_field.y = float(raw_mag[1])
            mag_raw.magnetic_field.z = float(raw_mag[2])
            self._pub_mag_raw.publish(mag_raw)

    def _close_serial(self) -> None:
        try:
            if self._ser is not None:
                self._ser.close()
        except (serial.SerialException, OSError):
            pass
        self._ser = None

    def shutdown(self) -> None:
        self._running = False
        if self._ser is not None and self._ser.is_open:
            try:
                with self._write_lock:
                    self._ser.write(pit.encode_command(0.0, 0.0))
            except (serial.SerialException, OSError):
                pass
        self._close_serial()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PitNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
