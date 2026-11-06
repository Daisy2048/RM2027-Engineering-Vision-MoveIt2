import struct

from rm2027_cboard_comm.protocol import crc16_ccitt_false
from rm2027_cboard_comm.stream import FeedbackStreamDecoder


def make_feedback_frame():

    payload = bytearray([0x01])

    for i in range(1, 8):
        payload.extend(
            struct.pack("<ff", float(i), float(i) / 10)
        )

    header = struct.pack("<BBH", 0x81, 57, 0x1234)

    body = header + payload

    crc = crc16_ccitt_false(body)

    return (
        bytes([0xA5])
        + body
        + struct.pack("<H", crc)
    )


def test_complete_frame():

    decoder = FeedbackStreamDecoder()

    result = decoder.feed(make_feedback_frame())

    assert len(result) == 1
    assert result[0]["rx_count_low16"] == 0x1234


def test_split_frame():

    decoder = FeedbackStreamDecoder()

    frame = make_feedback_frame()

    assert decoder.feed(frame[:10]) == []
    assert decoder.feed(frame[10:30]) == []

    result = decoder.feed(frame[30:])

    assert len(result) == 1


def test_multiple_frames():

    decoder = FeedbackStreamDecoder()

    frame = make_feedback_frame()

    result = decoder.feed(frame + frame)

    assert len(result) == 2


def test_noise_recovery():

    decoder = FeedbackStreamDecoder()

    frame = make_feedback_frame()

    result = decoder.feed(
        bytes.fromhex("00 11 22 33") + frame
    )

    assert len(result) == 1


def test_crc_error_recovery():

    decoder = FeedbackStreamDecoder()

    frame = bytearray(make_feedback_frame())

    frame[15] ^= 0xFF

    valid_frame = make_feedback_frame()

    result = decoder.feed(bytes(frame) + valid_frame)

    assert len(result) == 1
    assert result[0]["rx_count_low16"] == 0x1234
