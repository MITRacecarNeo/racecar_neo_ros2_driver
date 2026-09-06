"""
Wire protocol for the NEO-PIT (Peripheral Interface for Teensy) drive PCB.

Pure encode/decode helpers for the binary UART link between the Pi and the
Teensy 4.1 on the PIT board. No ROS or serial dependencies, so the byte layout
is unit-testable off-hardware.

The layout mirrors the firmware structs in racecar-pit-firmware
(racecar-pit-teensy/include/packets.h). Both directions are little-endian and
byte-packed (structs are __attribute__((packed))).

Telemetry (Teensy -> Pi), 94 bytes, framed by magic 0xDEADBEEF:
    u32 magic | u8 version | u8 mcu_id | u16 packet_bytes | u32 frame_bytes
    | u16 packet_index | u16 packets_per_frame | i32 timestamp_us
    | u16 volt_curr[2] | u16 rc[8] | f32 imu[9] | f32 encoder | f32 ekf[3]
    | u16 checksum

Command (Pi -> Teensy), 4-byte magic 0xBEEFDEAD prefix + 470-byte body:
    u8 version | u8 mcu_id | u16 packet_bytes | u32 frame_bytes
    | u16 packet_index | u16 packets_per_frame | f32 servo | f32 motor
    | u8 dot_matrix[192] | u8 led[255] | u8 system_state | u16 checksum

The firmware frames the command by scanning for the 4 magic bytes, then reading
sizeof(RxPacket) further bytes; the magic is not part of the RxPacket struct.

checksum is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over every byte that
precedes the 2-byte checksum field, magic included. The current firmware does
not yet compute or verify it (see decode_telemetry crc_ok and the migration
notes); callers decide whether to enforce.
"""

from dataclasses import dataclass
import struct

PROTO_VERSION = 1

TX_MAGIC = 0xDEADBEEF  # Teensy -> Pi
RX_MAGIC = 0xBEEFDEAD  # Pi -> Teensy

# Plausible FlySky pulse-width band, in microseconds. Channels sit near
# 1000-2000 us; a receiver with no transmitter signal reports near 0.
RC_PULSE_MIN_US = 900
RC_PULSE_MAX_US = 2100

# Telemetry: header(16) + timestamp(4) + volt_curr(4) + rc(16) + imu(36)
# + encoder(4) + ekf(12) + checksum(2)
_TX_FMT = '<IBBHIHHi2H8H9ff3fH'
TX_PACKET_SIZE = struct.calcsize(_TX_FMT)

# Command body (no magic): header(12) + servo(4) + motor(4) + dot(192)
# + led(255) + state(1) + checksum(2)
_RX_BODY_FMT = '<BBHIHHff192B255BBH'
RX_BODY_SIZE = struct.calcsize(_RX_BODY_FMT)
RX_WIRE_SIZE = 4 + RX_BODY_SIZE  # magic prefix + body

DOT_MATRIX_LEN = 192
LED_LEN = 255

assert TX_PACKET_SIZE == 94, TX_PACKET_SIZE
assert RX_BODY_SIZE == 470, RX_BODY_SIZE

_TX_MAGIC_LE = struct.pack('<I', TX_MAGIC)
_RX_MAGIC_LE = struct.pack('<I', RX_MAGIC)


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Telemetry:
    """
    One decoded Teensy -> Pi telemetry frame.

    imu holds the nine LSM9DS1 channels in firmware order
    [ax, ay, az, gx, gy, gz, mx, my, mz], in whatever units the firmware's
    Adafruit_LSM9DS1 getEvent() emits (accel m/s^2; gyro and mag units are
    unverified, see pit_node scale params). volt_curr is [bus voltage mV,
    current mA]; rc is the eight FlySky channels as raw pulse widths in us
    (~1000-2000). crc_ok reflects the CRC-16 check; it is False against current
    firmware, which leaves the checksum field uninitialized.
    """

    version: int
    mcu_id: int
    timestamp_us: int
    volt_curr: tuple
    rc: tuple
    imu: tuple
    encoder: float
    ekf: tuple
    checksum: int
    crc_ok: bool

    @property
    def accel(self) -> tuple:
        return self.imu[0:3]

    @property
    def gyro(self) -> tuple:
        return self.imu[3:6]

    @property
    def mag(self) -> tuple:
        return self.imu[6:9]

    @property
    def voltage_v(self) -> float:
        """Bus voltage in volts (firmware packs mV)."""
        return self.volt_curr[0] / 1000.0

    @property
    def current_a(self) -> float:
        """Battery current in amps (firmware packs mA)."""
        return self.volt_curr[1] / 1000.0

    @property
    def rc_link_up(self) -> bool:
        """
        Report whether every RC channel carries a plausible pulse width.

        Read this, not rc_normalized, to decide whether a transmitter is
        present. The normalization clamps into [-1, 1], which puts a dead
        channel (near 0 us) on exactly -1.0 -- the same value a switch held at
        its low end produces. The two are indistinguishable after the clamp, so
        link state has to come from the raw widths.

        This is a single frame's verdict. Deciding that a link is live enough
        to hand it authority also needs freshness and a sustained hold, which
        are stateful and belong to the consumer.
        """
        return all(RC_PULSE_MIN_US <= c <= RC_PULSE_MAX_US for c in self.rc)

    @property
    def rc_normalized(self) -> tuple:
        """
        Map the eight FlySky channels to [-1, 1].

        Pulse widths (~1000-2000 us, 1500 us center) are clamped to [-1, 1];
        channels with no transmitter signal read near 0.
        """
        return tuple(max(-1.0, min(1.0, (c - 1500.0) / 500.0)) for c in self.rc)


def decode_telemetry(packet: bytes) -> Telemetry:
    """
    Decode a full 94-byte telemetry packet (magic included).

    Raises ValueError on wrong length or a bad magic number. A CRC mismatch
    does not raise; it is reported via Telemetry.crc_ok.
    """
    if len(packet) != TX_PACKET_SIZE:
        raise ValueError(f'telemetry packet is {len(packet)} bytes, want {TX_PACKET_SIZE}')

    fields = struct.unpack(_TX_FMT, packet)
    magic = fields[0]
    if magic != TX_MAGIC:
        raise ValueError(f'telemetry magic 0x{magic:08X} != 0x{TX_MAGIC:08X}')

    checksum = fields[-1]
    crc_ok = crc16_ccitt(packet[:-2]) == checksum

    return Telemetry(
        version=fields[1],
        mcu_id=fields[2],
        timestamp_us=fields[7],
        volt_curr=fields[8:10],
        rc=fields[10:18],
        imu=fields[18:27],
        encoder=fields[27],
        ekf=fields[28:31],
        checksum=checksum,
        crc_ok=crc_ok,
    )


def encode_command(
    servo: float,
    motor: float,
    system_state: int = 0,
    dot_matrix: bytes = None,
    led: bytes = None,
) -> bytes:
    """
    Build a full command frame (magic prefix + body + CRC), RX_WIRE_SIZE bytes.

    servo and motor are the normalized [-1, 1] steering and speed commands
    (normalized-passthrough: the Teensy maps them to servo/ESC PWM). dot_matrix
    (<=192 bytes) and led (<=255 bytes) are zero-padded display payloads; the
    current firmware ignores them.
    """
    dot = bytes(dot_matrix or b'')[:DOT_MATRIX_LEN].ljust(DOT_MATRIX_LEN, b'\x00')
    led_bytes = bytes(led or b'')[:LED_LEN].ljust(LED_LEN, b'\x00')

    body_wo_crc = (
        struct.pack('<BBHIHHff', PROTO_VERSION, 0, RX_BODY_SIZE, RX_BODY_SIZE, 0, 1,
                    float(servo), float(motor))
        + dot + led_bytes + struct.pack('<B', system_state & 0xFF)
    )
    wire_wo_crc = _RX_MAGIC_LE + body_wo_crc
    crc = crc16_ccitt(wire_wo_crc)
    return wire_wo_crc + struct.pack('<H', crc)


def scan_for_packet(buffer: bytearray) -> tuple:
    """
    Extract the first complete telemetry frame from a rolling byte buffer.

    Returns (Telemetry, consumed) once a full framed packet is found, dropping
    any leading garbage; returns (None, drop) when no complete packet is
    present yet, where drop bytes have already been discarded from the front of
    buffer (kept small so a truncated magic straddling reads is not lost).
    Mutates buffer in place: consumed/dropped bytes are removed.
    """
    idx = buffer.find(_TX_MAGIC_LE)
    if idx == -1:
        # No magic yet; keep the last 3 bytes in case a magic straddles reads.
        drop = max(0, len(buffer) - 3)
        del buffer[:drop]
        return None, drop

    if idx > 0:
        del buffer[:idx]

    if len(buffer) < TX_PACKET_SIZE:
        return None, 0

    packet = bytes(buffer[:TX_PACKET_SIZE])
    try:
        telem = decode_telemetry(packet)
    except ValueError:
        # Magic matched mid-stream but the frame is invalid; step past it.
        del buffer[:4]
        return None, 4
    del buffer[:TX_PACKET_SIZE]
    return telem, TX_PACKET_SIZE
