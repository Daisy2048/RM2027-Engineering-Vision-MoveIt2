# MoveIt Calibration — Panthera-HT 手眼标定

基于 MoveIt Calibration 插件，为 Panthera-HT 六轴机械臂提供 RViz 手眼标定工具。

由 Intel Yu Yan 博士开发，上游仓库：https://github.com/ros-planning/moveit_calibration。

---

## 包结构

```
moveit2_calibration/
├── moveit_calibration_gui/          # RViz 手眼标定面板插件
│   └── handeye_calibration_rviz_plugin/
│       ├── handeye_calibration_display   # RViz Display 入口
│       ├── handeye_calibration_frame     # 主窗口框架（3个Tab）
│       ├── handeye_target_widget         # "Target" Tab — 标定板检测
│       ├── handeye_context_widget        # "Context" Tab — 坐标系/初始猜测
│       └── handeye_control_widget        # "Calibrate" Tab — 采集/求解/自动标定
├── moveit_calibration_plugins/      # 核心算法插件
│   ├── handeye_calibration_target/       # 标定板检测（ArUco / ChArUco）
│   └── handeye_calibration_solver/       # AX=XB 求解器（OpenCV 实现）
└── moveit_calibration_demos/        # Gazebo 仿真演示（COLCON_IGNORE，不参与编译）
```

---

## RViz 面板说明

启动标定后，RViz 中会出现 **HandEye Calibration** 面板，包含三个标签页：

### 1. Target（标定板设置）

- **Target Type**：选择标定板类型
  - `HandEyeTarget/Charuco`（推荐，精度更高）
  - `HandEyeTarget/Aruco`
- **Camera Image Topic**：选择相机图像话题，面板会自动检测并可视化标定板检测结果
- **标定板参数**：根据打印的标定板设置（方格数、标记尺寸、字典类型、实际物理尺寸等）
- **Create Target**：生成标定板图片，可保存为 PNG 打印使用

### 2. Context（场景设置）

- **Sensor configuration**：
  - `Eye-to-hand`：相机固定，机械臂末端夹持标定板
  - `Eye-in-hand`：相机装在机械臂末端，标定板固定
- **Frames Selection**：四个关键坐标系
  - Sensor frame：相机坐标系
  - Object frame：标定板坐标系
  - End-effector frame：机械臂末端
  - Robot base frame：机械臂基座
- **Camera Pose Initial Guess**：相机位姿的初始猜测（6 自由度滑块），提供好的初值可加速求解收敛

### 3. Calibrate（标定执行）

- **AX=XB Solver**：求解器算法选择（OpenCV 实现，包含 Tsai1989 / Park1994 / Horaud1995 / Andreff1999 / Daniilidis1998）
- **Manual Calibration**：
  - `Take Sample`：手动采集单帧（机械臂末端位姿 + 标定板检测结果）
  - `Clear Samples`：清除已采集样本
  - `Solve`：求解手眼变换矩阵，并在 RViz 中发布 TF 预览
  - 至少需要 5 组样本才能求解
- **Auto Calibration（需预录关节状态）**：
  - `Load joint states`：加载预录的关节状态 YAML 文件
  - `Plan` → `Execute` → 自动采集 → 循环，直至所有状态执行完毕
  - `Skip`：跳过当前状态
- **Save camera pose**：将标定结果保存为 launch 文件（.py / .xml / .yaml）

---

## Panthera-HT 标定流程

### 前置条件

- 机械臂已连接并上电
- RealSense D435/D435i 相机已连接
- 已打印 ChArUco 标定板（可在 RViz Target Tab 中点击 Create Target → Save Target 生成 PNG）

### 步骤

**1. 启动标定环境**

```bash
source install/setup.bash
ros2 launch panthera_ht_config calibration.launch.py
```

此 launch 文件自动启动：

| 组件 | 说明 |
|------|------|
| `hardware_moveit_rviz.launch.py` | 机械臂硬件驱动 + MoveIt（无 RViz） |
| `realsense2_camera_node` | RealSense RGB 相机（640x480@30Hz，仅彩色） |
| `static_transform_publisher` | `camera_mount_link` → `camera_link` 静态 TF |
| `rviz2` | 加载 `calibration.rviz` 配置（含 HandEyeCalibration 面板） |

**2. 放置标定板**

将 ChArUco 板放置于相机视野内的固定位置（Eye-to-hand 模式）。

**3. 配置 RViz 面板**

- **Target Tab**：选择图像话题（默认 `/camera/realsense2_camera/color/image_raw`），确认标定板参数与打印的一致
- **Context Tab**：
  - 选择 `Eye-to-hand`
  - Sensor frame: `camera_link`
  - Object frame: `handeye_target`（自动发布）
  - End-effector frame: `tool0`
  - Robot base frame: `base_link`

**4. 采集样本（手动模式）**

移动机械臂使标定板在相机视野中呈现不同位姿，每次点击 `Take Sample`。建议：
- 至少 5 组样本（越多越好，建议 10-15 组）
- 覆盖不同角度和距离
- 观察 Reprojecion Error，越小越好

**5. 求解并保存**

点击 `Solve` 查看标定结果和重投影误差。满意后点击 `Save camera pose` 导出 launch 文件。

---

## 支持的求解算法

| 算法 | 来源 | 特点 |
|------|------|------|
| Tsai1989 | Tsai & Lenz, 1989 | 经典方法，分离旋转和平移 |
| Park1994 | Park & Martin, 1994 | 基于李群理论 |
| Horaud1995 | Horaud & Dornaika, 1995 | 同时优化旋转和平移 |
| Andreff1999 | Andreff et al., 1999 | 基于外极几何 |
| Daniilidis1998 | Daniilidis, 1998 | 对偶四元数方法 |

所有算法由 OpenCV 的 `cv::calibrateHandEye()` 提供。

---

## 标定结果保存格式

保存时支持三种格式：

- `.launch.py` — Python launch 文件（推荐），生成 `static_transform_publisher` 节点
- `.xml` — XML launch 文件
- `.yaml` / `.yml` — YAML 格式

生成的 launch 文件可直接集成到项目的 launch 流程中，加载标定出的相机-机械臂外参。

---

## 依赖

- ROS2 Humble（含 MoveIt、rviz2、tf2_ros）
- OpenCV（含 contrib 模块，用于 ArUco/ChArUco）
- Qt5（RViz 面板）
- RealSense ROS2 驱动（`realsense2_camera`）

---

## 参考资料

- [上游仓库](https://github.com/ros-planning/moveit_calibration)
- [Hand-Eye Calibration 官方教程](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.rst)
- [moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials)
