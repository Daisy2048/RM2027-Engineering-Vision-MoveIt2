import struct
import unittest

from comm_pkg.protocol import (
    CAMERA_IMU_PAYLOAD,
    CMD_CAMERA_IMU,
    CMD_ISGET,
    CMD_SEND_POSE,
    CMD_START,
    CMD_STOP,
    FRAME_HEADER,
    FRAME_TAIL,
    HEAD,
    ISGET_PAYLOAD,
    POSE_PAYLOAD,
    TAIL,
    pack_dual_pose,
    pack_point_mask,
    pack_point_masks,
    pack_pose,
    unpack_control_frame,
    unpack_camera_imu_payload,
    unpack_frame,
)


def control_frame(command):
    return HEAD.pack(FRAME_HEADER, HEAD.size + TAIL.size, command) + TAIL.pack(0, FRAME_TAIL)


class ProtocolTest(unittest.TestCase):
    def test_pack_dual_pose_right_then_left(self):
        right = (1.0, 2.0, 3.0, 40.0, 50.0, 60.0)
        left = (-1.0, -2.0, -3.0, -40.0, -50.0, -60.0)
        frame = pack_dual_pose(7, right, left)
        header, length, command = HEAD.unpack_from(frame)
        values = POSE_PAYLOAD.unpack_from(frame, HEAD.size)
        checksum, tail = TAIL.unpack_from(frame, length - TAIL.size)

        self.assertEqual((header, length, command), (FRAME_HEADER, len(frame), CMD_SEND_POSE))
        self.assertEqual(values, (7, *right, *left))
        self.assertEqual(length, 57)
        self.assertEqual(checksum, sum(frame[HEAD.size:-TAIL.size]) & 0xFF)
        self.assertEqual(tail, FRAME_TAIL)

    def test_legacy_pack_pose_uses_right_and_zeros_left(self):
        frame = pack_pose(7, 1.0, 2.0, 3.0, 40.0, 50.0, 60.0)
        values = POSE_PAYLOAD.unpack_from(frame, HEAD.size)
        self.assertEqual(
            values,
            (7, 1.0, 2.0, 3.0, 40.0, 50.0, 60.0, *(0.0,) * 6),
        )

    def test_unpack_start_and_stop(self):
        self.assertEqual(unpack_control_frame(control_frame(CMD_START)), (CMD_START, None))
        self.assertEqual(unpack_control_frame(control_frame(CMD_STOP)), (CMD_STOP, None))

    def test_unpack_camera_imu_frame(self):
        payload = CAMERA_IMU_PAYLOAD.pack(9, 10.5, -20.25, 1.5)
        frame = (
            HEAD.pack(
                FRAME_HEADER,
                HEAD.size + len(payload) + TAIL.size,
                CMD_CAMERA_IMU,
            )
            + payload
            + TAIL.pack(sum(payload) & 0xFF, FRAME_TAIL)
        )
        command, received_payload, error = unpack_frame(frame)
        self.assertIsNone(error)
        self.assertEqual(command, CMD_CAMERA_IMU)
        values, error = unpack_camera_imu_payload(received_payload)
        self.assertIsNone(error)
        self.assertEqual(values, (9, 10.5, -20.25, 1.5))

    def test_reject_wrong_camera_imu_payload_size(self):
        values, error = unpack_camera_imu_payload(b"\x00" * 15)
        self.assertIsNone(values)
        self.assertIn("payload 长度错误", error)

    def test_reject_bad_checksum(self):
        frame = bytearray(control_frame(CMD_START))
        frame[-2] = 1
        command, error = unpack_control_frame(bytes(frame))
        self.assertIsNone(command)
        self.assertIn("校验错误", error)

    def test_pack_point_masks_right_then_left(self):
        right_mask = (1 << 0) | (1 << 10)
        left_mask = (1 << 1) | (1 << 9)
        frame = pack_point_masks(right_mask, left_mask)
        header, length, command = HEAD.unpack_from(frame)
        values = ISGET_PAYLOAD.unpack_from(frame, HEAD.size)
        checksum, tail = TAIL.unpack_from(frame, length - TAIL.size)
        payload = frame[HEAD.size:length - TAIL.size]

        self.assertEqual((header, length, command), (FRAME_HEADER, 9, CMD_ISGET))
        self.assertEqual(values, (0x0401, 0x0202))
        self.assertEqual(checksum, sum(payload) & 0xFF)
        self.assertEqual(tail, FRAME_TAIL)
        self.assertEqual(struct.unpack_from("<H", frame, HEAD.size)[0], 0x0401)
        self.assertEqual(struct.unpack_from("<H", frame, HEAD.size + 2)[0], 0x0202)

        frame0 = pack_point_mask(0)
        self.assertEqual(frame0[2], CMD_ISGET)
        self.assertEqual(ISGET_PAYLOAD.unpack_from(frame0, HEAD.size), (0, 0))

        # bit11 及以上被截断
        frame_hi = pack_point_masks(0xFFFF, 0xFFFF)
        self.assertEqual(
            ISGET_PAYLOAD.unpack_from(frame_hi, HEAD.size),
            (0x07FF, 0x07FF),
        )


if __name__ == "__main__":
    unittest.main()
