#!/usr/bin/env python3
import os
from datetime import datetime

import cv2
import numpy as np

CAMERA = "/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0"
TARGET_ID = 0
TAG_SIZE_M = 0.100  # AprilTag黑色外框实测边长；当前按100 mm设置

# 相机内参：对应1280x720分辨率
K = np.array([
    [484.41033408, 0.0, 572.73666357],
    [0.0, 484.10315031, 330.27487861],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([
    0.12854910, -0.08752819, -0.01061238, -0.00696327, 0.01335067
], dtype=np.float64)

ROOT = os.path.expanduser("~/rm2027")
IMAGE_PATH = os.path.join(ROOT, "vision_test", "apriltag_pnp_result.jpg")
REPORT_PATH = os.path.join(
    ROOT, "docs", "devlog", "2026-11-03-apriltag-pnp-validation.md"
)


def main():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("当前OpenCV没有aruco模块。")

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    parameters = cv2.aruco.DetectorParameters_create()

    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise RuntimeError("摄像头打开失败：" + CAMERA)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera opened: {width}x{height}")

    if (width, height) != (1280, 720):
        cap.release()
        raise RuntimeError(
            f"当前摄像头输出为{width}x{height}，但内参对应1280x720；"
            "请先统一分辨率，不能直接用这组K。"
        )

    print("[PASS] USB摄像头启动成功")
    print("[INFO] 请将完整AprilTag放在镜头前，保持清晰、正面可见。")
    print("[INFO] 正在收集目标ID 0的10帧位姿；按Q可退出。")

    translations = []
    rotations = []
    success = False

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] 读取图像失败，重试中...")
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters
            )

            display = frame.copy()

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                for index, marker_id in enumerate(ids.flatten()):
                    if int(marker_id) != TARGET_ID:
                        continue

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[index]], TAG_SIZE_M, K, D
                    )
                    rvec = rvecs[0].reshape(3)
                    tvec = tvecs[0].reshape(3)

                    rotations.append(rvec)
                    translations.append(tvec)

                    cv2.drawFrameAxes(
                        display, K, D, rvec, tvec, TAG_SIZE_M * 0.5
                    )
                    cv2.putText(
                        display,
                        f"AprilTag ID 0: {len(translations)}/10",
                        (25, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2
                    )

                    if len(translations) >= 10:
                        t = np.median(np.asarray(translations[-10:]), axis=0)
                        r = np.median(np.asarray(rotations[-10:]), axis=0)

                        os.makedirs(os.path.dirname(IMAGE_PATH), exist_ok=True)
                        os.makedirs(os.path.dirname(REPORT_PATH), exist_ok=True)

                        cv2.putText(
                            display,
                            f"X={t[0]*1000:.1f} Y={t[1]*1000:.1f} "
                            f"Z={t[2]*1000:.1f} mm",
                            (25, 80), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 255, 0), 2
                        )
                        cv2.imwrite(IMAGE_PATH, display)

                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        report = f"""# RM2027 AprilTag PnP 三维定位验证

## 实验状态

- USB 摄像头：PASS
- AprilTag 36h11 / ID 0：PASS
- 连续有效位姿采样：10 帧
- PnP 解算：PASS
- 实验时间：{timestamp}

## 实验参数

- 图像分辨率：{width} × {height}
- AprilTag 黑色外框边长：{TAG_SIZE_M * 1000:.1f} mm
- 相机内参：`camera_calibration/camera_params.yaml` 中本轮结果
- 标定 RMS：1.952713 px（偏高，结果仅用于链路验证）

## PnP 结果（10帧中位数）

相机坐标系下的平移：

- X：{t[0] * 1000:.2f} mm（图像向右为正）
- Y：{t[1] * 1000:.2f} mm（图像向下为正）
- Z：{t[2] * 1000:.2f} mm（相机前方为正）

旋转向量（Rodrigues）：

```text
{np.array2string(r, precision=6)}
```

平移向量（米）：

```text
{np.array2string(t, precision=6)}
```

## 结果边界与风险

本实验只验证相机对 AprilTag 的检测和 PnP 位姿解算。
当前结果不是龙门架坐标，也不是磁铁中心坐标，不能直接作为电机运动指令。
本轮相机标定 RMS 为 1.952713 px，且部分标定图像有模糊，
因此定位数值是算法链路验证结果，不代表已达到机械对接精度。

## 下一步

1. 重新采集清晰、平整、覆盖画面各区域的标定图像并优化内参。
2. 测量相机相对于龙门架基准坐标系的外参。
3. 测量 AprilTag 相对于磁铁中心的固定安装偏移。
4. 在低速、限位和急停保护下验证龙门架视觉对接。
"""
                        with open(REPORT_PATH, "w", encoding="utf-8") as f:
                            f.write(report)

                        print("\n========== PnP定位结果 ==========")
                        print(f"AprilTag ID: {TARGET_ID}")
                        print(f"X: {t[0] * 1000:.2f} mm")
                        print(f"Y: {t[1] * 1000:.2f} mm")
                        print(f"Z: {t[2] * 1000:.2f} mm")
                        print("Rvec:", r)
                        print("=================================")
                        print("[PASS] PnP三维定位成功")
                        print("[PASS] 实验报告已生成:", REPORT_PATH)
                        print("[PASS] 定位截图已保存:", IMAGE_PATH)
                        success = True
                        break

            cv2.imshow("RM2027 AprilTag PnP", display)
            if success or (cv2.waitKey(1) & 0xFF == ord("q")):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    if not success:
        print("[INFO] 未完成10帧有效定位；本次未生成PASS报告。")


if __name__ == "__main__":
    main()
