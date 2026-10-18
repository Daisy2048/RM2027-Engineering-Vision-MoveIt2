import os
import subprocess
from datetime import datetime
from typing import Optional

import numpy as np


class VideoWriter:
    """
    MKV 分段录制器
    """

    def __init__(
        self,
        output_dir: str,
        output_fps: float,       # 播放帧率
        write_every_n: int,      # 每 N 帧输入写 1 帧到文件
        segment_frames: int,     # 每段多少帧，如 20fps*60s=1200
        crf: int,                # 质量/体积平衡（18~28，越大越小）
        codec: str,              # 编码器
        preset: str,             # 编码速度优先
        pix_fmt: str,            # 输出像素格式，保证兼容性
    ):
        self._output_dir = output_dir
        self._output_fps = output_fps
        self._write_every_n = write_every_n
        self._segment_frames = segment_frames
        self._crf = crf
        self._codec = codec
        self._preset = preset
        self._pix_fmt = pix_fmt

        self._input_counter = 0         # 收到的原始帧计数,用于抽帧
        self._segment_frame_count = 0   # 当前段已写入帧数
        self._pipe: Optional[subprocess.Popen] = None
        self._current_path: Optional[str] = None
        self._shape: Optional[tuple] = None  # (h, w)

        os.makedirs(self._output_dir, exist_ok=True)

    def _make_path(self) -> str:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._output_dir, f"record_{now}.mkv")

    def _build_cmd(self, w: int, h: int, path: str) -> list:
        """构造 ffmpeg 命令。如需适配硬件编码，可在此修改。"""
        # 关键帧间隔：每 2 秒一个关键帧，方便拖动进度条
        gop = int(self._output_fps * 2)

        cmd = [
            "ffmpeg",
            "-y",                       # 覆盖已存在文件
            "-hide_banner",
            "-loglevel", "error",       # 只报错，减少刷屏
            "-f", "rawvideo",
            "-vcodec", "rawvideo",
            "-s", f"{w}x{h}",
            "-pix_fmt", "bgr24",        # OpenCV 默认 BGR
            "-r", str(self._output_fps),
            "-i", "-",                  # 从 stdin 读取
            "-c:v", self._codec,
            "-preset", self._preset,
            "-crf", str(self._crf),
            "-pix_fmt", self._pix_fmt,
            "-g", str(gop),
            "-f", "matroska",           # ← MKV 容器，流式，断电安全
            path,
        ]
        return cmd

    def _open_writer(self, frame: np.ndarray) -> None:
        """根据当前帧创建新的视频文件和 ffmpeg 进程。"""
        h, w = frame.shape[:2]
        self._shape = (h, w)
        self._current_path = self._make_path()

        cmd = self._build_cmd(w, h, self._current_path)

        self._pipe = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._segment_frame_count = 0
        print(f"[VideoWriter] 开始录制: {self._current_path} "
              f"({w}x{h} @ {self._output_fps}fps, codec={self._codec})")

    def write(self, frame: np.ndarray) -> None:
        """
        写入图像
        """
        self._input_counter += 1

        # 1. 抽帧
        if self._input_counter % self._write_every_n != 0:
            return

        # 2. 检查是否需要开新文件
        need_new = (
            self._pipe is None
            or self._pipe.poll() is not None          # ffmpeg 挂了
            or self._segment_frame_count >= self._segment_frames
        )

        if need_new:
            self.release()
            self._open_writer(frame)

        # 3. 写入
        if self._pipe is not None and self._pipe.poll() is None:
            try:
                self._pipe.stdin.write(frame.tobytes())
                self._segment_frame_count += 1
            except (BrokenPipeError, OSError):
                # ffmpeg 异常退出，不抛异常，下一段会自动重启
                pass

    def release(self) -> None:
        """安全关闭当前 ffmpeg 进程。"""
        if self._pipe is None:
            return

        try:
            self._pipe.stdin.close()
            self._pipe.wait(timeout=2.0)
            print(f"[VideoWriter] 结束录制: {self._current_path}")
        except subprocess.TimeoutExpired:
            print(f"[VideoWriter] 关闭超时，强制杀死 ffmpeg")
            self._pipe.kill()
            self._pipe.wait()
        except Exception as exc:
            print(f"[VideoWriter] 关闭异常: {exc}")
            try:
                self._pipe.kill()
            except Exception:
                pass
        finally:
            self._pipe = None
            self._segment_frame_count = 0
