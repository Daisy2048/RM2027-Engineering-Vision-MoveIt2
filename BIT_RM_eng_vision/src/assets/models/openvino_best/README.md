# 检测模型放置说明

本目录**必须**放入香港科技大学（HKUST Enterprize）开源的 OpenVINO IR 权重。
权重不随本仓库发布。

## 放置位置（文件名不要改）

```text
src/assets/models/openvino_best/best.xml   # 模型结构
src/assets/models/openvino_best/best.bin   # 模型权重
```

放置后执行 `colcon build`（或 `./start.sh`）。加载路径见
`src/pose_detect_pkg/config/detect.yaml` 的 `model_path`
（相对 `assets` 包目录，默认 `models/openvino_best/best.xml`）。

## 来源（必须致谢）

- **开源权重**：https://huggingface.co/hkustenterprize/RM26_engineer_exchange_model  
  本链路使用 **yolopose/v11.02**（YOLO11s-Pose，OpenVINO FP16：`best.xml` + `best.bin`）。
- **训练数据集**：https://huggingface.co/datasets/hkustenterprize/RM26_engineer_exchange  

权重与数据均由**香港科技大学（HKUST）开源**，在此致谢。

## 模型格式约定

- **输入**：`1x3x640x640`，BGR 图像等比缩放 + 灰边 padding 到 640x640，转 RGB 后归一化到 [0,1]（letterbox 预处理见 `pose_detect_pkg/detect_roi.py`）。
- **输出**：`[1, N, 6 + 12*3]`，每行一个候选检测：

  | 字段 | 含义 |
  | --- | --- |
  | 0..3 | bbox `[x1, y1, x2, y2]`（640 输入尺度） |
  | 4 | 置信度 score |
  | 5 | 类别 id：`0`=pillar，`1`=exchange |
  | 6..41 | 12 个关键点 `(x, y, score)` |

- **关键点顺序**（与 `detect_roi.py` 的 `KEYPOINT_NAMES` 一致）：

  | 索引 | 名称 | 所属类别 |
  | --- | --- | --- |
  | 0 | TL | pillar |
  | 1 | TR | pillar |
  | 2 | BL | pillar |
  | 3 | BR | pillar |
  | 4 | ring | pillar |
  | 5 | light_BR | exchange |
  | 6 | light_TR | exchange |
  | 7 | shell_R | exchange |
  | 8 | shell_M | exchange |
  | 9 | shell_L | exchange |
  | 10 | light_TL | exchange |
  | 11 | light_BL | exchange |
