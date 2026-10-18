"""Minimal Daheng (Galaxy / gxipy) camera driver."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from gxipy import DeviceManager, GxAccessMode
from gxipy.gxidef import (
    GxAutoEntry,
    GxBalanceRatioSelectorEntry,
    GxSwitchEntry,
    GxTriggerSourceEntry,
)
from gxipy.gxiapi import InvalidAccess

logger = logging.getLogger("DaHengCamera")

BAYER_MODES = (
    # OpenCV 两字母 Bayer 常量按模式的第二行/第二列命名，
    # 与相机 PixelColorFilter 的左上角命名正好互为相反排列。
    ("Bayer RG", cv2.COLOR_BayerBG2BGR),
    ("Bayer BG", cv2.COLOR_BayerRG2BGR),
    ("Bayer GR", cv2.COLOR_BayerGB2BGR),
    ("Bayer GB", cv2.COLOR_BayerGR2BGR),
)

TRIGGER_SOURCE_MAP = {
    "software": GxTriggerSourceEntry.SOFTWARE,
    "line0": GxTriggerSourceEntry.LINE0,
    "line1": GxTriggerSourceEntry.LINE1,
    "line2": GxTriggerSourceEntry.LINE2,
    "line3": GxTriggerSourceEntry.LINE3,
}

GAIN_MIN_DB = 0.0
GAIN_MAX_DB = 24.0


class DahengCamera:
    def __init__(
        self,
        sn: str,
        width: int = 960,
        height: int = 720,
        binning: int = 1,
        exposure_us: float = 10000.0,
        gain: float = 19.0,
        bayer_mode: int = 0,
        frame_rate: float = 60.0,
        trigger_enable: bool = False,
        trigger_source: str = "software",
        wb_mode: str = "once",
        wb_r: float = 1.0,
        wb_g: float = 1.0,
        wb_b: float = 1.0,
    ):
        self.device_sn = str(sn).strip()
        self.width = int(width)
        self.height = int(height)
        self.binning = max(1, int(binning))
        self.exposure_us = float(exposure_us)
        self.gain = float(gain)
        self.bayer_mode = int(bayer_mode) % len(BAYER_MODES)
        self.frame_rate = float(frame_rate)
        self.trigger_enable = bool(trigger_enable)
        self.trigger_source = str(trigger_source).strip().lower()
        self.wb_mode = str(wb_mode).strip().lower()
        self.wb_r = float(wb_r)
        self.wb_g = float(wb_g)
        self.wb_b = float(wb_b)

        self.camera = None
        self.camera_manager = None
        self._stream_thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[np.ndarray, float]] = None
        self._connected = False

    def open(self) -> bool:
        try:
            self.camera_manager = DeviceManager()
            dev_num, dev_info_list = self.camera_manager.update_device_list()
            if dev_num == 0:
                logger.error("No Daheng camera found")
                return False

            available = [str(info.get("sn", "")).strip() for info in dev_info_list]
            for i, info in enumerate(dev_info_list):
                logger.info("Device %d: %s (%s)", i + 1, info.get("model_name"), info.get("sn"))

            if not self.device_sn:
                # 未配置 SN：自动使用第一台识别到的相机
                self.device_sn = available[0]
                logger.info("No SN configured, auto-selected first device: %s", self.device_sn)
            elif self.device_sn not in available:
                raise RuntimeError(
                    f"Configured SN not found: {self.device_sn}; visible: {available}"
                )

            try:
                self.camera = self.camera_manager.open_device_by_sn(
                    self.device_sn, GxAccessMode.CONTROL
                )
            except InvalidAccess as exc:
                raise RuntimeError(f"Open failed SN={self.device_sn}: {exc}") from exc

            self._apply_parameters()
            try:
                if hasattr(self.camera.data_stream[0], "set_acquisition_buffer_number"):
                    self.camera.data_stream[0].set_acquisition_buffer_number(3)
            except Exception:
                pass

            self.camera.stream_on()
            try:
                self.camera.data_stream[0].flush_queue()
            except Exception:
                pass

            self._connected = True
            return True
        except Exception:
            logger.exception("Daheng camera open failed")
            self._connected = False
            return False

    def _apply_parameters(self) -> None:
        cam = self.camera
        try:
            try:
                cam.stream_off()
            except Exception:
                pass

            for attr in ("BinningVertical", "BinningHorizontal"):
                if hasattr(cam, attr):
                    getattr(cam, attr).set(self.binning)

            for off_attr in ("OffsetX", "OffsetY"):
                if hasattr(cam, off_attr):
                    try:
                        getattr(cam, off_attr).set(0)
                    except Exception:
                        pass

            if hasattr(cam, "Width"):
                cam.Width.set(int(self.width))
            if hasattr(cam, "Height"):
                cam.Height.set(int(self.height))
            try:
                self.width = int(cam.Width.get())
                self.height = int(cam.Height.get())
            except Exception:
                pass

            # 触发
            if hasattr(cam, "TriggerMode"):
                cam.TriggerMode.set(
                    GxSwitchEntry.ON if self.trigger_enable else GxSwitchEntry.OFF
                )
            if self.trigger_enable and hasattr(cam, "TriggerSource"):
                src = TRIGGER_SOURCE_MAP.get(
                    self.trigger_source, GxTriggerSourceEntry.SOFTWARE
                )
                cam.TriggerSource.set(src)

            # 连续模式下开帧率控制
            if (not self.trigger_enable) and hasattr(cam, "AcquisitionFrameRateMode"):
                try:
                    cam.AcquisitionFrameRateMode.set(GxSwitchEntry.ON)
                except Exception:
                    pass
            if (not self.trigger_enable) and hasattr(cam, "AcquisitionFrameRate"):
                try:
                    cam.AcquisitionFrameRate.set(float(self.frame_rate))
                except Exception:
                    pass

            if hasattr(cam, "ExposureAuto"):
                try:
                    cam.ExposureAuto.set(GxAutoEntry.OFF)
                except Exception:
                    pass
            if hasattr(cam, "GainAuto"):
                try:
                    cam.GainAuto.set(GxAutoEntry.OFF)
                except Exception:
                    pass

            if hasattr(cam, "ExposureTime"):
                cam.ExposureTime.set(self.exposure_us)
            if hasattr(cam, "Gain"):
                gain = max(GAIN_MIN_DB, min(GAIN_MAX_DB, self.gain))
                cam.Gain.set(gain)
                self.gain = gain

            # 白平衡
            if hasattr(cam, "BalanceWhiteAuto"):
                mode_map = {
                    "off": GxAutoEntry.OFF,
                    "once": GxAutoEntry.ONCE,
                    "continuous": GxAutoEntry.CONTINUOUS,
                }
                cam.BalanceWhiteAuto.set(mode_map.get(self.wb_mode, GxAutoEntry.ONCE))
            if self.wb_mode == "off" and hasattr(cam, "BalanceRatioSelector") and hasattr(cam, "BalanceRatio"):
                for selector, value in (
                    (GxBalanceRatioSelectorEntry.RED, self.wb_r),
                    (GxBalanceRatioSelectorEntry.GREEN, self.wb_g),
                    (GxBalanceRatioSelectorEntry.BLUE, self.wb_b),
                ):
                    cam.BalanceRatioSelector.set(selector)
                    cam.BalanceRatio.set(float(value))
        except Exception:
            logger.exception("Failed to apply camera parameters")

    def _grab_bgr(self) -> Optional[np.ndarray]:
        if self.camera is None:
            return None
        img_raw = self.camera.data_stream[0].get_image(timeout=1000)
        if img_raw is None:
            return None
        width = img_raw.get_width()
        height = img_raw.get_height()
        raw_data = img_raw.get_data()
        if raw_data is None or len(raw_data) != width * height:
            return None
        mono = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width))
        _, convert = BAYER_MODES[self.bayer_mode]
        return cv2.cvtColor(mono, convert)

    def start_streaming(self) -> bool:
        if self._stream_thread is not None and self._stream_thread.is_alive():
            return True
        if not self._connected and not self.open():
            return False
        self._stop_event = threading.Event()
        self._stream_thread = threading.Thread(
            target=self._stream_loop, name="daheng-stream", daemon=True
        )
        self._stream_thread.start()
        return True

    def _stream_loop(self) -> None:
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            try:
                # 软触发：每帧发一次
                if (
                    self.trigger_enable
                    and self.trigger_source == "software"
                    and self.camera is not None
                    and hasattr(self.camera, "TriggerSoftware")
                ):
                    try:
                        self.camera.TriggerSoftware.send_command()
                    except Exception:
                        pass

                frame = self._grab_bgr()
                if frame is None:
                    time.sleep(0.001)
                    continue
                with self._lock:
                    self._latest = (frame, time.time())
            except Exception as exc:
                logger.warning("Stream grab error: %s", exc)
                time.sleep(0.01)

    def get_latest(self) -> Tuple[Optional[np.ndarray], Optional[float]]:
        with self._lock:
            if self._latest is None:
                return None, None
            frame, ts = self._latest
            return frame, ts

    def is_connected(self) -> bool:
        return self._connected

    def _feature_range(self, feature_name: str):
        if self.camera is None or not hasattr(self.camera, feature_name):
            return None, None, None
        feature = getattr(self.camera, feature_name)
        try:
            current = feature.get()
        except Exception:
            current = None
        min_v = max_v = None
        if hasattr(feature, "get_range"):
            try:
                rng = feature.get_range()
                if isinstance(rng, dict):
                    min_v = rng.get("min")
                    max_v = rng.get("max")
            except Exception:
                pass
        return min_v, max_v, current

    def get_capabilities(self) -> dict:
        exp_min, exp_max, exp_cur = self._feature_range("ExposureTime")
        gain_min, gain_max, gain_cur = self._feature_range("Gain")
        fps_min, fps_max, fps_cur = self._feature_range("AcquisitionFrameRate")
        return {
            "exposure_min_us": exp_min,
            "exposure_max_us": exp_max,
            "exposure_us": exp_cur if exp_cur is not None else self.exposure_us,
            "gain_min_db": gain_min if gain_min is not None else GAIN_MIN_DB,
            "gain_max_db": gain_max if gain_max is not None else GAIN_MAX_DB,
            "gain_db": gain_cur if gain_cur is not None else self.gain,
            "frame_rate_min": fps_min,
            "frame_rate_max": fps_max,
            "frame_rate": fps_cur if fps_cur is not None else self.frame_rate,
        }

    def close(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
        if self.camera is not None:
            try:
                self.camera.stream_off()
            except Exception:
                pass
            try:
                self.camera.close_device()
            except Exception:
                pass
            self.camera = None
        self.camera_manager = None
        self._connected = False
        with self._lock:
            self._latest = None
