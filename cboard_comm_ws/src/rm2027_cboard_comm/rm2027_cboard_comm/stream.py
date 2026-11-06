"""
RM2027 C板串口数据流拆包模块。

功能：
1. 接收不定长度的串口字节流
2. 自动寻找反馈帧头
3. 根据协议长度提取完整数据帧
4. 调用 protocol.py 进行CRC校验和解析
5. 支持连续数据包和异常数据恢复

注意：
本模块不直接操作串口，也不发送电机指令。
"""

from typing import List, Dict

from .protocol import (
    SOF,
    TYPE_FEEDBACK,
    FEEDBACK_PAYLOAD_LEN,
    FEEDBACK_FRAME_LEN,
    parse_feedback_frame,
)


class FeedbackStreamDecoder:

    def __init__(self):
        self._buffer = bytearray()

    def reset(self):
        """清空接收缓存。"""
        self._buffer.clear()

    @property
    def buffered_bytes(self):
        """当前缓存中的字节数量。"""
        return len(self._buffer)

    def feed(self, data: bytes) -> List[Dict]:
        """
        输入任意长度的串口数据。

        返回：
            已经通过CRC校验的完整反馈帧列表。
        """

        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("输入必须是bytes类型")

        self._buffer.extend(data)

        packets = []

        while True:

            # 1. 寻找帧头
            start = self._buffer.find(bytes([SOF]))

            if start < 0:
                self._buffer.clear()
                break

            # 2. 删除帧头之前的无效数据
            if start > 0:
                del self._buffer[:start]

            # 3. 至少需要3字节才能检查消息类型和长度
            if len(self._buffer) < 3:
                break

            # 4. 检查消息类型
            if self._buffer[1] != TYPE_FEEDBACK:
                del self._buffer[0]
                continue

            # 5. 检查Payload长度
            if self._buffer[2] != FEEDBACK_PAYLOAD_LEN:
                del self._buffer[0]
                continue

            # 6. 等待完整数据帧
            if len(self._buffer) < FEEDBACK_FRAME_LEN:
                break

            candidate = bytes(
                self._buffer[:FEEDBACK_FRAME_LEN]
            )

            # 7. 调用协议模块进行CRC校验
            try:

                result = parse_feedback_frame(candidate)

            except ValueError:

                # 当前帧错误，丢弃一个字节后重新同步
                del self._buffer[0]
                continue

            # 8. 成功解析
            packets.append(result)

            # 9. 删除已经处理的数据
            del self._buffer[:FEEDBACK_FRAME_LEN]

        return packets
