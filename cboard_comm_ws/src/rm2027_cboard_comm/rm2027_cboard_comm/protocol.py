"""
RM2027 C板通信协议模块。

当前协议：
    上位机 -> C板：35字节
    C板 -> 上位机：64字节

注意：
    本模块只负责离线协议编解码。
    不负责串口通信，也不发送实际电机指令。
"""

import math
import struct
from typing import Sequence


# =========================================================
# 1. 协议基本参数
# =========================================================

SOF = 0xA5

TYPE_TARGETS = 0x01
TYPE_FEEDBACK = 0x81

JOINT_NUM = 6
COMMAND_FLOAT_NUM = 7

COMMAND_PAYLOAD_LEN = 28
FEEDBACK_PAYLOAD_LEN = 57

COMMAND_FRAME_LEN = 35
FEEDBACK_FRAME_LEN = 64


# =========================================================
# 2. CRC16/CCITT-FALSE
# =========================================================

def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC16/CCITT-FALSE

    Polynomial: 0x1021
    Initial:    0xFFFF
    RefIn:      False
    RefOut:     False
    XorOut:     0x0000
    """

    crc = 0xFFFF

    for byte in data:

        crc ^= byte << 8

        for _ in range(8):

            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


# =========================================================
# 3. 输入数据检查
# =========================================================

def _check_float32(value, name: str) -> float:
    """检查数据是否可以安全表示为有限的 float32。"""

    try:
        value = float(value)

        if not math.isfinite(value):
            raise ValueError(f"{name} 不能是 NaN 或 Inf")

        struct.pack("<f", value)

    except (TypeError, OverflowError, struct.error) as exc:
        raise ValueError(f"{name} 不是有效的 float32") from exc

    return value


# =========================================================
# 4. 上位机 -> C板：生成35字节目标帧
# =========================================================

def build_target_frame(
    joints_rad: Sequence[float],
    *,
    seq: int,
    gripper_field: float,
) -> bytes:
    """
    生成上位机目标帧。

    joints_rad:
        六轴目标角度，单位 rad。

    seq:
        uint16 序号。

    gripper_field:
        原协议第七个 float，即夹爪字段。

        当前固件快照中该字段会进入夹爪控制路径。
        此参数必须显式提供，不能默认填0。

        目前仅用于离线协议测试。
        未经电控确认，不得发送到真实C板。

    返回：
        35字节 bytes。
    """

    if len(joints_rad) != JOINT_NUM:
        raise ValueError("必须提供正好6个关节角度")

    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ValueError("seq 必须是整数")

    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq 必须在 0~65535 范围内")

    joints = [
        _check_float32(value, f"joint{i + 1}")
        for i, value in enumerate(joints_rad)
    ]

    gripper = _check_float32(
        gripper_field,
        "gripper_field"
    )

    # 7个float32：6轴 + 原协议夹爪字段
    payload = struct.pack(
        "<7f",
        *joints,
        gripper
    )

    # type + payload_length + sequence
    header = struct.pack(
        "<BBH",
        TYPE_TARGETS,
        COMMAND_PAYLOAD_LEN,
        seq
    )

    # CRC覆盖范围：type开始至payload结束
    body = header + payload

    crc = crc16_ccitt_false(body)

    frame = (
        bytes([SOF])
        + body
        + struct.pack("<H", crc)
    )

    if len(frame) != COMMAND_FRAME_LEN:
        raise RuntimeError("生成的目标帧长度异常")

    return frame


# =========================================================
# 5. C板 -> 上位机：解析64字节反馈帧
# =========================================================

def parse_feedback_frame(frame: bytes) -> dict:
    """
    解析C板返回的反馈帧。

    返回：

    {
        "rx_count_low16": 1,
        "flags": 1,
        "link_alive": True,
        "motors": [
            {
                "motor_id": 1,
                "turn": 0.1,
                "rad": 0.2
            }
        ]
    }
    """

    frame = bytes(frame)

    # 检查长度
    if len(frame) != FEEDBACK_FRAME_LEN:
        raise ValueError(
            f"反馈帧长度错误：{len(frame)}，"
            f"期望：{FEEDBACK_FRAME_LEN}"
        )

    # 检查帧头
    if frame[0] != SOF:
        raise ValueError("反馈帧帧头错误")

    # 检查消息类型
    if frame[1] != TYPE_FEEDBACK:
        raise ValueError("反馈帧类型错误")

    # 检查payload长度
    if frame[2] != FEEDBACK_PAYLOAD_LEN:
        raise ValueError("反馈帧payload长度错误")

    # 检查CRC
    crc_received = struct.unpack(
        "<H",
        frame[-2:]
    )[0]

    crc_calculated = crc16_ccitt_false(
        frame[1:-2]
    )

    if crc_received != crc_calculated:
        raise ValueError(
            f"CRC错误：收到0x{crc_received:04X}，"
            f"计算得到0x{crc_calculated:04X}"
        )

    # 读取反馈帧序号字段
    # 当前C板实际发送的是成功接收目标帧的计数低16位
    rx_count = struct.unpack(
        "<H",
        frame[3:5]
    )[0]

    # 链路状态
    flags = frame[5]

    link_alive = bool(flags & 0x01)

    # 解析7路电机反馈
    motors = []

    offset = 6

    for motor_id in range(1, 8):

        turn, rad = struct.unpack(
            "<ff",
            frame[offset:offset + 8]
        )

        motors.append({
            "motor_id": motor_id,
            "turn": turn,
            "rad": rad
        })

        offset += 8

    return {
        "rx_count_low16": rx_count,
        "flags": flags,
        "link_alive": link_alive,
        "motors": motors
    }


