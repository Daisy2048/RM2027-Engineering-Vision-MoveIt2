"""基于 multiprocessing.shared_memory 的最新帧缓冲区。"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from multiprocessing import resource_tracker, shared_memory

import numpy as np


DEFAULT_SHM_NAME = "roi_camera_frame"
FRAME_TOPIC = "/camera/frame_ready"

MAGIC = b"ROIFRAME"
VERSION = 1
# magic, sequence, version, timestamp_ns, width, height, channels, step,
# encoding, frame_id, data_size, padding。sequence 位于 8 字节对齐地址。
HEADER = struct.Struct("<8sQIQIIII8s32sQ4x")
SEQUENCE = struct.Struct("<Q")
SEQUENCE_OFFSET = 8
HEADER_SIZE = HEADER.size


@dataclass(frozen=True)
class SharedFrame:
    image: np.ndarray
    sequence: int
    timestamp_ns: int
    frame_id: str
    encoding: str


def _encode_fixed(value, size):
    encoded = value.encode("ascii")
    if len(encoded) >= size:
        raise ValueError(f"字符串过长: {value!r}, max={size - 1}")
    return encoded.ljust(size, b"\0")


def _decode_fixed(value):
    return value.split(b"\0", 1)[0].decode("ascii")


class SharedFrameWriter:
    """共享内存唯一所有者；关闭时负责 unlink。"""

    def __init__(self, capacity_bytes, name=DEFAULT_SHM_NAME):
        self.name = str(name)
        self.capacity_bytes = int(capacity_bytes)
        if self.capacity_bytes <= 0:
            raise ValueError("capacity_bytes 必须大于 0")
        self._sequence = 0
        self._shm = self._create_segment()

    def _create_segment(self):
        try:
            return shared_memory.SharedMemory(
                name=self.name,
                create=True,
                size=HEADER_SIZE + self.capacity_bytes,
            )
        except FileExistsError:
            # 启动脚本已保证只有一个相机；这里清理上次异常退出的残留段。
            stale = shared_memory.SharedMemory(name=self.name, create=False)
            stale.close()
            stale.unlink()
            return shared_memory.SharedMemory(
                name=self.name,
                create=True,
                size=HEADER_SIZE + self.capacity_bytes,
            )

    def write(self, image, timestamp_ns, frame_id="camera", encoding="bgr8"):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise TypeError("image 必须是 uint8 numpy.ndarray")
        if image.ndim == 2:
            height, width = image.shape
            channels = 1
        elif image.ndim == 3:
            height, width, channels = image.shape
        else:
            raise ValueError(f"不支持的图像维度: {image.shape}")
        if channels not in (1, 3):
            raise ValueError(f"不支持的通道数: {channels}")

        contiguous = np.ascontiguousarray(image)
        data_size = int(contiguous.nbytes)
        if data_size > self.capacity_bytes:
            raise ValueError(
                f"图像超过共享内存容量: {data_size} > {self.capacity_bytes}"
            )
        step = int(width * channels)
        completed_sequence = self._sequence + 2
        writing_sequence = completed_sequence - 1

        HEADER.pack_into(
            self._shm.buf,
            0,
            MAGIC,
            writing_sequence,
            VERSION,
            int(timestamp_ns),
            int(width),
            int(height),
            int(channels),
            step,
            _encode_fixed(encoding, 8),
            _encode_fixed(frame_id, 32),
            data_size,
        )
        self._shm.buf[HEADER_SIZE:HEADER_SIZE + data_size] = memoryview(contiguous).cast("B")
        SEQUENCE.pack_into(self._shm.buf, SEQUENCE_OFFSET, completed_sequence)
        self._sequence = completed_sequence
        return completed_sequence

    def close(self):
        if self._shm is None:
            return
        shm = self._shm
        self._shm = None
        shm.close()
        try:
            shm.unlink()
        except FileNotFoundError:
            pass


class SharedFrameReader:
    """按需连接共享段，每次返回一份不会被生产者覆盖的稳定快照。"""

    def __init__(self, name=DEFAULT_SHM_NAME, unregister_tracker=True):
        self.name = str(name)
        self._unregister_tracker = bool(unregister_tracker)
        self._shm = None

    def _connect(self):
        if self._shm is not None:
            return True
        try:
            shm = shared_memory.SharedMemory(name=self.name, create=False)
        except FileNotFoundError:
            return False
        if self._unregister_tracker:
            # Python 3.10 的独立进程会错误地追踪并 unlink 非自身创建的共享段。
            resource_tracker.unregister(shm._name, "shared_memory")
        self._shm = shm
        return True

    def _disconnect(self):
        if self._shm is not None:
            self._shm.close()
            self._shm = None

    def read_latest(self, max_retries=8):
        if not self._connect():
            return None

        for _ in range(max_retries):
            try:
                values = HEADER.unpack_from(self._shm.buf, 0)
            except (TypeError, ValueError):
                self._disconnect()
                return None
            (
                magic,
                sequence_before,
                version,
                timestamp_ns,
                width,
                height,
                channels,
                step,
                encoding_raw,
                frame_id_raw,
                data_size,
            ) = values

            if sequence_before & 1:
                time.sleep(0)
                continue
            if magic != MAGIC or version != VERSION or sequence_before == 0:
                time.sleep(0)
                continue
            expected_size = int(height * step)
            if (
                width <= 0
                or height <= 0
                or channels not in (1, 3)
                or step != width * channels
                or data_size != expected_size
                or HEADER_SIZE + data_size > self._shm.size
            ):
                return None

            flat = np.empty(data_size, dtype=np.uint8)
            flat[:] = np.frombuffer(
                self._shm.buf,
                dtype=np.uint8,
                count=data_size,
                offset=HEADER_SIZE,
            )
            sequence_after = SEQUENCE.unpack_from(
                self._shm.buf, SEQUENCE_OFFSET
            )[0]
            if sequence_before != sequence_after or sequence_after & 1:
                time.sleep(0)
                continue

            if channels == 1:
                image = flat.reshape(height, width)
            else:
                image = flat.reshape(height, width, channels)
            return SharedFrame(
                image=image,
                sequence=int(sequence_after),
                timestamp_ns=int(timestamp_ns),
                frame_id=_decode_fixed(frame_id_raw),
                encoding=_decode_fixed(encoding_raw),
            )
        return None

    def reconnect(self):
        self._disconnect()
        return self._connect()

    def close(self):
        self._disconnect()
