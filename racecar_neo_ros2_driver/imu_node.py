"""
LSM9DS1 9-DoF IMU driver — reads accel/gyro/mag over I²C, publishes /imu + /mag.

Accel/gyro share I²C address 0x6B; magnetometer is at 0x1E. Both on bus 1.
Calibration biases load from config/lsm9ds1_cal.yaml and
config/lsm9ds1_mag_cal.yaml at startup.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import smbus


# I²C addresses
ACCEL_GYRO_ADDR = 0x6B
MAG_ADDR = 0x1E

# Sensitivity constants (LSM9DS1 datasheet)
SENSITIVITY_ACCEL_2G = -0.000061    # g/LSB; sign flipped for our mounting
SENSITIVITY_GYRO_245DPS = 0.00875   # dps/LSB
SENSITIVITY_MAG_4G = 0.00014        # gauss/LSB

# Accel/gyro registers
CTRL_REG1_G = 0x10
CTRL_REG6_XL = 0x20
OUT_X_L_G = 0x18
OUT_X_L_XL = 0x28

# Magnetometer registers
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23
OUT_X_L_M = 0x28


def twos_complement(value, bits):
    """Convert an unsigned `bits`-wide integer to signed two's complement."""
    sign_bit = 1 << (bits - 1)
    if value & sign_bit:
        return value - (1 << bits)
    return value


def apply_mag_calibration(raw_tesla, hard_iron, soft_iron):
    """Apply hard-iron offset and soft-iron matrix to a 3-vector reading."""
    raw = np.asarray(raw_tesla, dtype=float)
    hard = np.asarray(hard_iron, dtype=float)
    soft = np.asarray(soft_iron, dtype=float).reshape(3, 3)
    return soft @ (raw - hard)


def _read_i16(bus, addr, low_reg):
    """Read a signed 16-bit little-endian value starting at `low_reg`."""
    lo = bus.read_byte_data(addr, low_reg)
    hi = bus.read_byte_data(addr, low_reg + 1)
    return twos_complement((hi << 8) | lo, 16)


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('accelerometer.bias', [0.0, 0.0, 0.0])
        self.declare_parameter('gyroscope.bias', [0.0, 0.0, 0.0])
        self.declare_parameter('magnetometer.hard_iron_bias', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'magnetometer.soft_iron_matrix.data',
            np.identity(3).flatten().tolist(),
        )

        bus_id = self.get_parameter('i2c_bus').value
        rate = self.get_parameter('update_rate').value
        self._frame = self.get_parameter('frame_id').value
        self._accel_bias = np.array(
            self.get_parameter('accelerometer.bias').value, dtype=float
        )
        self._gyro_bias = np.array(
            self.get_parameter('gyroscope.bias').value, dtype=float
        )
        self._mag_hard_iron = np.array(
            self.get_parameter('magnetometer.hard_iron_bias').value, dtype=float
        )
        self._mag_soft_iron = np.array(
            self.get_parameter('magnetometer.soft_iron_matrix.data').value,
            dtype=float,
        ).reshape(3, 3)

        self._bus = smbus.SMBus(bus_id)
        self._enable_sensors()

        self._pub_imu = self.create_publisher(Imu, '/imu', 10)
        self._pub_mag = self.create_publisher(MagneticField, '/mag', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'IMU ready: bus={bus_id}, rate={rate}Hz, frame={self._frame}'
        )
        self.get_logger().info(f'  accel bias = {self._accel_bias.tolist()}')
        self.get_logger().info(f'  gyro bias  = {self._gyro_bias.tolist()}')
        self.get_logger().info(f'  mag hard   = {self._mag_hard_iron.tolist()}')

    def _enable_sensors(self):
        # Gyro: 952 Hz ODR, ±245 dps
        self._bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG1_G, 0b11000011)
        # Accel: 952 Hz ODR, ±2g
        self._bus.write_byte_data(ACCEL_GYRO_ADDR, CTRL_REG6_XL, 0b11000110)
        # Mag: ultra-high performance, 80 Hz ODR
        self._bus.write_byte_data(MAG_ADDR, CTRL_REG1_M, 0b11111100)
        self._bus.write_byte_data(MAG_ADDR, CTRL_REG2_M, 0b00000000)
        self._bus.write_byte_data(MAG_ADDR, CTRL_REG3_M, 0b00000000)
        self._bus.write_byte_data(MAG_ADDR, CTRL_REG4_M, 0b00001100)

    def _read_gyro_rads(self):
        raw = [
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_G),
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_G + 2),
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_G + 4),
        ]
        dps = np.array(raw, dtype=float) * SENSITIVITY_GYRO_245DPS
        return dps * (math.pi / 180.0) - self._gyro_bias

    def _read_accel_mps2(self):
        raw = [
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_XL),
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_XL + 2),
            _read_i16(self._bus, ACCEL_GYRO_ADDR, OUT_X_L_XL + 4),
        ]
        g = np.array(raw, dtype=float) * SENSITIVITY_ACCEL_2G
        return g * 9.80665 - self._accel_bias

    def _read_mag_tesla(self):
        raw = [
            _read_i16(self._bus, MAG_ADDR, OUT_X_L_M),
            _read_i16(self._bus, MAG_ADDR, OUT_X_L_M + 2),
            _read_i16(self._bus, MAG_ADDR, OUT_X_L_M + 4),
        ]
        gauss = np.array(raw, dtype=float) * SENSITIVITY_MAG_4G
        return apply_mag_calibration(
            gauss * 1e-4, self._mag_hard_iron, self._mag_soft_iron
        )

    def _tick(self):
        stamp = self.get_clock().now().to_msg()

        gyro = self._read_gyro_rads()
        accel = self._read_accel_mps2()
        mag = self._read_mag_tesla()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
            float(gyro[0]), float(gyro[1]), float(gyro[2])
        )
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            float(accel[0]), float(accel[1]), float(accel[2])
        )
        self._pub_imu.publish(imu)

        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame
        msg.magnetic_field.x = float(mag[0])
        msg.magnetic_field.y = float(mag[1])
        msg.magnetic_field.z = float(mag[2])
        self._pub_mag.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
