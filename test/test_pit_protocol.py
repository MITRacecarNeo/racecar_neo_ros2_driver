"""Unit tests for the NEO-PIT wire protocol (pit_protocol)."""

import struct

import pytest

from racecar_neo_ros2_driver import pit_protocol as pit


def _build_telemetry(
    timestamp=0,
    volt_curr=(0, 0),
    rc=(0,) * 8,
    imu=(0.0,) * 9,
    encoder=0.0,
    ekf=(0.0,) * 3,
    good_crc=True,
):
    header = struct.pack(
        '<IBBHIHH',
        pit.TX_MAGIC,
        pit.PROTO_VERSION,
        0,
        pit.TX_PACKET_SIZE,
        pit.TX_PACKET_SIZE,
        0,
        1,
    )
    body = struct.pack('<i2H8H9ff3f', timestamp, *volt_curr, *rc, *imu, encoder, *ekf)
    pre = header + body
    crc = pit.crc16_ccitt(pre) if good_crc else (pit.crc16_ccitt(pre) ^ 0x1)
    return pre + struct.pack('<H', crc)


class TestSizes:
    def test_packet_sizes(self):
        assert pit.TX_PACKET_SIZE == 94
        assert pit.RX_BODY_SIZE == 470
        assert pit.RX_WIRE_SIZE == 474


class TestCrc:
    def test_known_answer(self):
        # CRC-16/CCITT-FALSE check value for the ASCII string "123456789".
        assert pit.crc16_ccitt(b'123456789') == 0x29B1

    def test_empty(self):
        assert pit.crc16_ccitt(b'') == 0xFFFF


class TestEncodeCommand:
    def test_wire_length_and_magic(self):
        wire = pit.encode_command(0.0, 0.0)
        assert len(wire) == pit.RX_WIRE_SIZE
        assert struct.unpack('<I', wire[:4])[0] == pit.RX_MAGIC

    def test_servo_motor_roundtrip(self):
        wire = pit.encode_command(0.5, -0.25, system_state=3)
        _, _, _, _, _, _, servo, motor = struct.unpack('<BBHIHHff', wire[4:24])
        assert servo == pytest.approx(0.5)
        assert motor == pytest.approx(-0.25)
        # system_state sits just before the 2-byte checksum.
        assert wire[-3] == 3

    def test_crc_covers_everything_but_the_checksum(self):
        wire = pit.encode_command(0.1, 0.2)
        assert pit.crc16_ccitt(wire[:-2]) == struct.unpack('<H', wire[-2:])[0]

    def test_display_payloads_are_padded(self):
        wire = pit.encode_command(0.0, 0.0, dot_matrix=b'\x01\x02', led=b'\xff')
        # dot_matrix starts after magic(4)+header(12)+servo(4)+motor(4) = 24.
        assert wire[24] == 1 and wire[25] == 2 and wire[26] == 0
        assert wire[24 + pit.DOT_MATRIX_LEN] == 0xFF

    def test_oversized_payloads_are_truncated(self):
        wire = pit.encode_command(0.0, 0.0, dot_matrix=b'\x01' * 500)
        assert len(wire) == pit.RX_WIRE_SIZE


class TestDecodeTelemetry:
    def test_roundtrip_fields(self):
        pkt = _build_telemetry(
            timestamp=123456,
            volt_curr=(111, 222),
            rc=tuple(range(1, 9)),
            imu=tuple(float(i) for i in range(9)),
            encoder=4900.0,
            ekf=(0.1, 0.2, 0.3),
        )
        t = pit.decode_telemetry(pkt)
        assert t.timestamp_us == 123456
        assert t.volt_curr == (111, 222)
        assert t.rc == tuple(range(1, 9))
        assert t.encoder == pytest.approx(4900.0)
        assert t.crc_ok is True

    def test_accessor_slices(self):
        imu = tuple(float(i) for i in range(9))
        t = pit.decode_telemetry(_build_telemetry(imu=imu))
        assert t.accel == (0.0, 1.0, 2.0)
        assert t.gyro == (3.0, 4.0, 5.0)
        assert t.mag == (6.0, 7.0, 8.0)

    def test_power_unit_conversions(self):
        # volt_curr is [bus mV, current mA]; properties report V and A.
        t = pit.decode_telemetry(_build_telemetry(volt_curr=(7912, 2500)))
        assert t.voltage_v == pytest.approx(7.912)
        assert t.current_a == pytest.approx(2.5)

    def test_rc_normalized(self):
        # 1500 us -> 0, 2000 -> +1, 1000 -> -1; out-of-range clamps to [-1, 1].
        t = pit.decode_telemetry(
            _build_telemetry(rc=(1500, 2000, 1000, 1750, 1250, 900, 2100, 1500))
        )
        got = t.rc_normalized
        assert got[0] == pytest.approx(0.0)
        assert got[1] == pytest.approx(1.0)
        assert got[2] == pytest.approx(-1.0)
        assert got[3] == pytest.approx(0.5)
        assert got[4] == pytest.approx(-0.5)
        assert got[5] == pytest.approx(-1.0)  # 900 us clamps
        assert got[6] == pytest.approx(1.0)  # 2100 us clamps
        assert len(got) == 8

    def test_rc_link_up_with_a_transmitter(self):
        t = pit.decode_telemetry(
            _build_telemetry(rc=(1500, 2000, 1000, 1750, 1250, 1000, 2000, 1500))
        )
        assert t.rc_link_up is True

    def test_rc_link_down_with_no_transmitter(self):
        # A receiver with no signal reports near 0 us on every channel.
        t = pit.decode_telemetry(_build_telemetry(rc=(0,) * 8))
        assert t.rc_link_up is False

    def test_rc_link_down_when_any_channel_is_dead(self):
        rc = [1500] * 8
        rc[3] = 0
        t = pit.decode_telemetry(_build_telemetry(rc=tuple(rc)))
        assert t.rc_link_up is False

    def test_rc_link_band_edges(self):
        lo = pit.decode_telemetry(_build_telemetry(rc=(pit.RC_PULSE_MIN_US,) * 8))
        hi = pit.decode_telemetry(_build_telemetry(rc=(pit.RC_PULSE_MAX_US,) * 8))
        assert lo.rc_link_up is True
        assert hi.rc_link_up is True
        below = pit.decode_telemetry(_build_telemetry(rc=(pit.RC_PULSE_MIN_US - 1,) * 8))
        above = pit.decode_telemetry(_build_telemetry(rc=(pit.RC_PULSE_MAX_US + 1,) * 8))
        assert below.rc_link_up is False
        assert above.rc_link_up is False

    def test_no_signal_is_distinguishable_from_switch_low(self):
        # rc_normalized clamps both to -1.0; rc_link_up must not.
        dead = pit.decode_telemetry(_build_telemetry(rc=(0,) * 8))
        low = pit.decode_telemetry(_build_telemetry(rc=(1000,) * 8))
        assert dead.rc_normalized[5] == low.rc_normalized[5] == pytest.approx(-1.0)
        assert dead.rc_link_up is not low.rc_link_up

    def test_bad_crc_flag(self):
        t = pit.decode_telemetry(_build_telemetry(good_crc=False))
        assert t.crc_ok is False

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError):
            pit.decode_telemetry(b'\x00' * 10)

    def test_bad_magic_raises(self):
        pkt = bytearray(_build_telemetry())
        pkt[0] ^= 0xFF
        with pytest.raises(ValueError):
            pit.decode_telemetry(bytes(pkt))


class TestFraming:
    def test_strips_leading_garbage(self):
        pkt = _build_telemetry(timestamp=42)
        buf = bytearray(b'\xaa\xbb\xcc' + pkt)
        assert pit.scan_for_packet(buf) == (None, 3)
        telem, consumed = pit.scan_for_packet(buf)
        assert telem.timestamp_us == 42
        assert consumed == pit.TX_PACKET_SIZE
        assert len(buf) == 0

    def test_waits_for_full_packet(self):
        pkt = _build_telemetry()
        buf = bytearray(pkt[:20])
        telem, consumed = pit.scan_for_packet(buf)
        assert telem is None and consumed == 0
        assert len(buf) == 20

    def test_two_packets_back_to_back(self):
        buf = bytearray(_build_telemetry(timestamp=1) + _build_telemetry(timestamp=2))
        first, _ = pit.scan_for_packet(buf)
        second, _ = pit.scan_for_packet(buf)
        assert first.timestamp_us == 1
        assert second.timestamp_us == 2

    def test_no_magic_keeps_tail(self):
        buf = bytearray(b'\x00' * 50)
        telem, dropped = pit.scan_for_packet(buf)
        assert telem is None
        assert dropped == 47 and len(buf) == 3


# Reference packet shared with the firmware's tests (racecar-pit-firmware
# docs/protocol.md): payload "info=1\n", revision 3, chunk 0 of 1.
INFO_REFERENCE = bytes.fromhex(
    'de c0 ed fe 01 03 19 00 07 00 00 00 00 00 01 00 69 6e 66 6f 3d 31 0a d4 60'
)


def _build_info(data, revision=1, index=0, count=1, payload_bytes=None, good_crc=True):
    size = pit.INFO_HEADER_SIZE + len(data) + 2
    total = len(data) if payload_bytes is None else payload_bytes
    body = struct.pack('<IBBHIHH', pit.INFO_MAGIC, 1, revision, size, total, index, count) + data
    crc = pit.crc16_ccitt(body) ^ (0 if good_crc else 0xFFFF)
    return body + struct.pack('<H', crc)


class TestInfoChunk:
    def test_reference_packet(self):
        chunk = pit.decode_info_chunk(INFO_REFERENCE)
        assert chunk == pit.InfoChunk(
            revision=3, payload_bytes=7, index=0, count=1, data=b'info=1\n'
        )

    def test_builder_matches_reference(self):
        assert _build_info(b'info=1\n', revision=3) == INFO_REFERENCE

    def test_bad_crc_is_none(self):
        assert pit.decode_info_chunk(_build_info(b'a=1\n', good_crc=False)) is None

    @pytest.mark.parametrize('index, count', [(1, 1), (0, 0)])
    def test_bad_index_is_none(self, index, count):
        assert pit.decode_info_chunk(_build_info(b'a=1\n', index=index, count=count)) is None

    def test_length_field_must_match(self):
        pkt = bytearray(_build_info(b'a=1\n'))
        pkt[6] += 1
        assert pit.decode_info_chunk(bytes(pkt)) is None


class TestInfoFraming:
    def test_info_between_telemetry(self):
        buf = bytearray(
            _build_telemetry(timestamp=1) + INFO_REFERENCE + _build_telemetry(timestamp=2)
        )
        first, _ = pit.scan_for_packet(buf)
        info, consumed = pit.scan_for_packet(buf)
        second, _ = pit.scan_for_packet(buf)
        assert first.timestamp_us == 1 and second.timestamp_us == 2
        assert isinstance(info, pit.InfoChunk) and consumed == len(INFO_REFERENCE)
        assert len(buf) == 0

    def test_waits_for_whole_chunk(self):
        buf = bytearray(INFO_REFERENCE[:10])
        assert pit.scan_for_packet(buf) == (None, 0)
        buf.extend(INFO_REFERENCE[10:20])
        assert pit.scan_for_packet(buf) == (None, 0)
        buf.extend(INFO_REFERENCE[20:])
        chunk, _ = pit.scan_for_packet(buf)
        assert chunk.data == b'info=1\n'

    def test_bad_chunk_resyncs_on_the_next_telemetry(self):
        buf = bytearray(_build_info(b'a=1\n', good_crc=False) + _build_telemetry(timestamp=7))
        packets = []
        dropped = 0
        while True:
            packet, consumed = pit.scan_for_packet(buf)
            if packet is not None:
                packets.append(packet)
            elif consumed == 0:
                break
            else:
                dropped += consumed
        assert [p.timestamp_us for p in packets] == [7]
        assert dropped == pit.INFO_HEADER_SIZE + 4 + 2

    def test_implausible_length_steps_past_the_magic(self):
        pkt = bytearray(INFO_REFERENCE)
        pkt[6:8] = struct.pack('<H', 4000)
        buf = bytearray(pkt)
        assert pit.scan_for_packet(buf) == (None, 4)
