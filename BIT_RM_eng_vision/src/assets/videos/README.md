# 回放视频放置说明

将自备的演示视频放入本目录（默认文件名 `3.mp4`），`colcon build` 后
`virtual_camera_node` 即可循环回放（路径配置见
`src/virtual_camera_pkg/config/virtual_camera.yaml` 的 `video_path`）。

视频内容应包含待检测目标（pillar / exchange），用于无相机时的离线调试。
