from dm6220_gimbal.protocol import (
    MitLimits,
    decode_feedback,
    encode_mit,
    encode_pos_vel,
    encode_velocity,
    pack_usb2can_frame,
    parse_usb2can_frames,
)

LIMITS = MitLimits(12.5, 45.0, 10.0)


def test_mit_zero_frame_matches_dmtool_observation():
    payload = encode_mit(0.0, 0.0, 0.0, 0.0, 0.0, LIMITS)
    assert payload.hex(" ").upper() == "7F FF 7F F0 00 00 07 FF"


def test_feedback_decode_known_frame():
    data = bytes.fromhex("01 63 52 7F C7 FF 1F 1B")
    fb = decode_feedback(data, LIMITS)
    assert fb.status == 0
    assert fb.motor_id == 1
    assert abs(fb.pos - (-2.8006)) < 0.001
    assert abs(fb.vel - (-0.0769)) < 0.001
    assert abs(fb.tau - (-0.00244)) < 0.0001
    assert fb.t_mos == 31
    assert fb.t_rotor == 27


def test_velocity_payload():
    assert encode_velocity(0.5).hex(" ").upper() == "00 00 00 3F 00 00 00 00"


def test_pos_vel_payload():
    import struct

    payload = encode_pos_vel(-1.5, 0.8)
    assert len(payload) == 8
    assert abs(struct.unpack("<f", payload[:4])[0] + 1.5) < 1e-6
    assert abs(struct.unpack("<f", payload[4:8])[0] - 0.8) < 1e-6


def test_usb2can_frame_layout():
    frame = pack_usb2can_frame(0x201, bytes.fromhex("00 00 00 00 00 00 00 00"))
    assert len(frame) == 30
    assert frame[0:2] == b"\x55\xAA"
    assert frame[2] == 0x1E
    assert frame[3] == 0x01
    assert frame[11:13] == b"\x00\x00"
    assert frame[13:15] == b"\x01\x02"
    assert frame[18] == 8
    assert frame[29] == 0


def test_parse_rx_frame():
    raw = bytes.fromhex("AA 11 08 11 00 00 00 01 63 52 7F C7 FF 1F 1B 55")
    buffer = bytearray(raw)
    frames, remaining = parse_usb2can_frames(buffer)
    assert not remaining
    assert len(frames) == 1
    frame = frames[0]
    assert frame.cmd == 0x11
    assert frame.can_id == 0x11
    assert frame.data == bytes.fromhex("01 63 52 7F C7 FF 1F 1B")
