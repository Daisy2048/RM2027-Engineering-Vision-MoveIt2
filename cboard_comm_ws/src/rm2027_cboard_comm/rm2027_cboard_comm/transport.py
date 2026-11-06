"""
RM2027 C板通信发送模块。

职责：
1. 将六轴目标位置编码为C板协议帧。
2. 通过已有串口对象发送数据。
3. 检查发送长度。
4. 默认禁止真实硬件发送。

注意：
本模块不负责打开串口。
串口由统一通信节点管理，避免多个节点竞争同一个USB CDC。
"""

from typing import Sequence

from .protocol import (
    build_target_frame,
    COMMAND_FRAME_LEN,
)


class CBoardTxTransport:

    def __init__(
        self,
        serial_port,
        *,
        enable_tx: bool = False
    ):
        """
        serial_port:
            已经打开的pyserial串口对象。

        enable_tx:
            False：禁止发送，默认状态。
            True：允许发送，必须在完成安全确认后启用。
        """

        self.serial = serial_port
        self.enable_tx = enable_tx

        self.frames_sent = 0
        self.bytes_sent = 0

    def send_target(
        self,
        joints_rad: Sequence[float],
        *,
        seq: int,
        gripper_field: float
    ) -> bytes:
        """
        发送一帧六轴目标数据。

        注意：
        当前电控固件实际存在第七个float进入夹爪控制路径的代码。
        因此gripper_field必须显式提供，不能擅自默认。
        """

        # 1. 发送安全开关
        if not self.enable_tx:
            raise PermissionError(
                "真实C板发送功能尚未启用"
            )

        # 2. 检查串口状态
        if not self.serial.is_open:
            raise RuntimeError(
                "C板串口尚未打开"
            )

        # 3. 生成协议帧
        frame = build_target_frame(
            joints_rad=joints_rad,
            seq=seq,
            gripper_field=gripper_field
        )

        # 4. 检查数据帧长度
        if len(frame) != COMMAND_FRAME_LEN:
            raise RuntimeError(
                "C板目标数据帧长度错误"
            )

        # 5. 发送数据
        written = self.serial.write(frame)

        if written != len(frame):
            raise IOError(
                f"数据发送不完整："
                f"{written}/{len(frame)}"
            )

        self.serial.flush()

        # 6. 更新统计
        self.frames_sent += 1
        self.bytes_sent += written

        return frame

