import struct

import pytest

from rm2027_cboard_comm.protocol import (
    build_target_frame,
    crc16_ccitt_false,
    parse_feedback_frame,
)


def make_feedback_frame(rx_count=0x1234, flags=0x01):
    """构造模拟C板反馈帧。"""

    payload = bytearray([flags])

    for i in range(1, 8):
        payload.extend(
            struct.pack("<ff", float(i), float(i) / 10)
        )

    header = struct.pack(
        "<BBH",
        0x81,
        57,
        rx_count
    )

    body = header + payload

    crc = crc16_ccitt_false(body)

    return (
        bytes([0xA5])
        + body
        + struct.pack("<H", crc)
    )


def test_crc16_standard():
    """验证CRC16标准测试向量。"""

    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_build_target_frame():
    """验证上位机目标帧。"""

    frame = build_target_frame(
        joints_rad=[0.0] * 6,
        seq=1,
        gripper_field=0.0
    )

    assert len(frame) == 35
    assert frame[0] == 0xA5
    assert frame[1] == 0x01
    assert frame[2] == 28

    assert frame == (
        bytes([0xA5, 0x01, 0x1C, 0x01, 0x00])
        + bytes(28)
        + bytes.fromhex("a7 eb")
    )


def test_build_target_frame_invalid_joint_count():
    """关节数量错误时应拒绝生成数据帧。"""

    with pytest.raises(ValueError):
        build_target_frame(
            joints_rad=[0.0] * 5,
            seq=1,
            gripper_field=0.0
        )


@pytest.mark.parametrize("seq", [-1, 65536, True, 1.5])
def test_invalid_sequence(seq):
    """序号超出范围或类型错误时应拒绝。"""

    with pytest.raises(ValueError):
        build_target_frame(
            joints_rad=[0.0] * 6,
            seq=seq,
            gripper_field=0.0
        )


def test_parse_feedback_frame():
    """验证反馈帧解析。"""

    frame = make_feedback_frame()

    result = parse_feedback_frame(frame)

    assert len(frame) == 64
    assert result["rx_count_low16"] == 0x1234
    assert result["link_alive"] is True
    assert len(result["motors"]) == 7

    assert result["motors"][0]["motor_id"] == 1
    assert result["motors"][0]["turn"] == pytest.approx(1.0)
    assert result["motors"][0]["rad"] == pytest.approx(0.1)


def test_parse_feedback_crc_error():
    """CRC错误时应拒绝反馈帧。"""

    frame = bytearray(make_feedback_frame())

    frame[10] ^= 0x01

    with pytest.raises(ValueError, match="CRC错误"):
        parse_feedback_frame(frame)


def test_parse_feedback_length_error():
    """长度错误时应拒绝反馈帧。"""

    with pytest.raises(ValueError, match="长度错误"):
        parse_feedback_frame(bytes(63))


def test_parse_feedback_header_error():
    """帧头错误时应拒绝反馈帧。"""

    frame = bytearray(make_feedback_frame())
    frame[0] = 0x00

    with pytest.raises(ValueError, match="帧头错误"):
        parse_feedback_frame(frame)


def test_parse_feedback_type_error():
    """消息类型错误时应拒绝反馈帧。"""

    frame = bytearray(make_feedback_frame())
    frame[1] = 0x01

    with pytest.raises(ValueError, match="类型错误"):
        parse_feedback_frame(frame)


def test_parse_feedback_payload_length_error():
    """Payload长度错误时应拒绝反馈帧。"""

    frame = bytearray(make_feedback_frame())
    frame[2] = 56

    with pytest.raises(ValueError, match="payload长度错误"):
        parse_feedback_frame(frame)
