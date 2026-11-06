import pytest

from rm2027_cboard_comm.transport import CBoardTxTransport
from rm2027_cboard_comm.protocol import build_target_frame


class FakeSerial:

    def __init__(self, is_open=True, write_result=None):
        self.is_open = is_open
        self.write_result = write_result
        self.written = bytearray()
        self.flush_count = 0

    def write(self, data):
        if self.write_result is None:
            count = len(data)
        else:
            count = self.write_result

        self.written.extend(data[:count])
        return count

    def flush(self):
        self.flush_count += 1


def test_tx_disabled_by_default():

    fake = FakeSerial()
    tx = CBoardTxTransport(fake)

    with pytest.raises(PermissionError):
        tx.send_target(
            [0.0] * 6,
            seq=1,
            gripper_field=0.0
        )

    assert len(fake.written) == 0


def test_send_target_frame():

    fake = FakeSerial()

    tx = CBoardTxTransport(
        fake,
        enable_tx=True
    )

    joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

    frame = tx.send_target(
        joints,
        seq=0x1234,
        gripper_field=0.0
    )

    expected = build_target_frame(
        joints,
        seq=0x1234,
        gripper_field=0.0
    )

    assert frame == expected
    assert bytes(fake.written) == expected
    assert len(frame) == 35
    assert tx.frames_sent == 1
    assert tx.bytes_sent == 35
    assert fake.flush_count == 1


def test_closed_serial_rejected():

    fake = FakeSerial(is_open=False)

    tx = CBoardTxTransport(
        fake,
        enable_tx=True
    )

    with pytest.raises(RuntimeError):
        tx.send_target(
            [0.0] * 6,
            seq=1,
            gripper_field=0.0
        )


def test_partial_write_rejected():

    fake = FakeSerial(write_result=34)

    tx = CBoardTxTransport(
        fake,
        enable_tx=True
    )

    with pytest.raises(IOError):
        tx.send_target(
            [0.0] * 6,
            seq=1,
            gripper_field=0.0
        )

    assert tx.frames_sent == 0

