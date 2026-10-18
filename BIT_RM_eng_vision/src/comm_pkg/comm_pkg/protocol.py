"""电控串口协议的打包与解包。"""

import struct


FRAME_HEADER = 0xA5
FRAME_TAIL = 0x5A

CMD_SEND_POSE = 0x01
CMD_START = 0x02
CMD_STOP = 0x03
CMD_ISGET = 0x04
CMD_CAMERA_IMU = 0x05

HEAD = struct.Struct("<BBB")  # 帧头、帧长、命令字
TAIL = struct.Struct("<BB")   # payload 校验和、帧尾
POSE_PAYLOAD = struct.Struct("<i12f")  # count、right 6 floats、left 6 floats
ISGET_PAYLOAD = struct.Struct("<HH")  # right/left 各 11 点可见性位图
CAMERA_IMU_PAYLOAD = struct.Struct("<ifff")  # 计数、Yaw/Pitch/Roll
POINT_MASK_BITS = 11
POINT_MASK_MAX = (1 << POINT_MASK_BITS) - 1  # 0x07FF
MIN_FRAME_LENGTH = HEAD.size + TAIL.size


def pack_point_masks(right_mask: int, left_mask: int) -> bytes:
    """打包 0x04：right mask 在前，left mask 在后，均为 uint16 LE。"""
    payload = ISGET_PAYLOAD.pack(
        int(right_mask) & POINT_MASK_MAX,
        int(left_mask) & POINT_MASK_MAX,
    )
    length = HEAD.size + len(payload) + TAIL.size
    return (
        HEAD.pack(FRAME_HEADER, length, CMD_ISGET)
        + payload
        + TAIL.pack(sum(payload) & 0xFF, FRAME_TAIL)
    )


def pack_point_mask(mask: int) -> bytes:
    """兼容旧调用：单 mask 作为 right，left 置零。"""
    return pack_point_masks(mask, 0)


def pack_point_count(count: int) -> bytes:
    """兼容旧名：将 count 当作 mask 传入（新协议请用 pack_point_mask）。"""
    return pack_point_mask(count)


def _pose_values(pose):
    values = tuple(float(value) for value in pose)
    if len(values) != 6:
        raise ValueError("位姿必须包含 XYZ/Yaw/Pitch/Roll 共 6 个值")
    return values


def pack_dual_pose(count, right_pose, left_pose):
    """打包 0x01：count + right pose + left pose。"""
    payload = POSE_PAYLOAD.pack(
        int(count),
        *_pose_values(right_pose),
        *_pose_values(left_pose),
    )
    length = HEAD.size + len(payload) + TAIL.size
    return (
        HEAD.pack(FRAME_HEADER, length, CMD_SEND_POSE)
        + payload
        + TAIL.pack(sum(payload) & 0xFF, FRAME_TAIL)
    )


def pack_pose(count, x_mm, y_mm, z_mm, yaw_deg, pitch_deg, roll_deg):
    """兼容旧调用：旧单位姿放入 right，left 置零。"""
    return pack_dual_pose(
        count,
        (x_mm, y_mm, z_mm, yaw_deg, pitch_deg, roll_deg),
        (0.0,) * 6,
    )


def unpack_frame(data):
    """校验并拆分通用串口帧，返回 ``(命令字, payload, 错误)``。"""
    if len(data) < MIN_FRAME_LENGTH:
        return None, None, f"帧过短: {len(data)}"

    header, length, cmd_id = HEAD.unpack_from(data)
    if header != FRAME_HEADER:
        return None, None, f"帧头错误: 0x{header:02X}"
    if length != len(data):
        return None, None, f"帧长错误: field={length}, actual={len(data)}"

    checksum, tail = TAIL.unpack_from(data, length - TAIL.size)
    if tail != FRAME_TAIL:
        return None, None, f"帧尾错误: 0x{tail:02X}"

    payload = data[HEAD.size:length - TAIL.size]
    expected = sum(payload) & 0xFF
    if checksum != expected:
        return None, None, (
            f"校验错误: got=0x{checksum:02X}, expected=0x{expected:02X}"
        )
    return cmd_id, payload, None


def unpack_control_frame(data):
    """解析无 payload 的开始/停止控制帧，失败时返回错误原因。"""
    cmd_id, payload, error = unpack_frame(data)
    if error is not None:
        return None, error
    if cmd_id not in (CMD_START, CMD_STOP):
        return None, f"未知命令: 0x{cmd_id:02X}"
    if payload:
        return None, f"控制帧 payload 应为空: {len(payload)} bytes"

    return cmd_id, None


def unpack_camera_imu_payload(payload):
    """解析 0x05 payload，返回 ``(count, yaw, pitch, roll)``（角度单位：度）。"""
    if len(payload) != CAMERA_IMU_PAYLOAD.size:
        return None, (
            f"0x05 payload 长度错误: {len(payload)} != {CAMERA_IMU_PAYLOAD.size}"
        )
    return CAMERA_IMU_PAYLOAD.unpack(payload), None
