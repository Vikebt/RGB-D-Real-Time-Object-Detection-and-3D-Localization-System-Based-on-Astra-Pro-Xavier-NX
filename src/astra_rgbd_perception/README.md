# astra_rgbd_perception

Astra Pro RGB-D + YOLOv5n TensorRT FP16 + 3D localization 的 ROS Melodic 包。

## 节点

- `rgbd_sync_node`：RGB-D ApproximateTime 同步、XYZ/XYZRGB 点云、统计。
- `detector_node`：letterbox + TensorRT YOLOv5n + NMS，发布 2D 检测与标注图。
- `depth_localizer_node`：检测框 ROI 深度中位数 + 内参反投影，发布 Object3D 与
  RViz markers。

## 启动

```bash
source /opt/ros/melodic/setup.bash
source ~/astra_rgbd_ws/devel/setup.bash

# 完整系统
roslaunch astra_rgbd_perception system.launch use_rviz:=false

# 分模块
roslaunch astra_rgbd_perception rgbd.launch start_camera:=false
roslaunch astra_rgbd_perception detector.launch
roslaunch astra_rgbd_perception localizer.launch
```

## 依赖的外部包（同一工作空间）

- `ros_astra_camera`（Orbbec 官方，已打补丁）
- `vision_opencv`（melodic 分支，OpenCV 4 补丁）

## 配置

- `config/rgbd_sync.yaml`
- `config/detector.yaml`
- `config/localizer.yaml`
- `config/coco.names`
- `config/rgbd_view.rviz`

## 脚本

- `scripts/record_rgbd.sh [seconds]`
- `scripts/play_rgbd.sh <bag> [options]`
- `scripts/benchmark.sh [seconds]`
- `scripts/stability_test.sh [duration] [interval]`
- `scripts/measure_distance.py --class bottle --duration 5`
- `scripts/calibrate_rgb.py`
- `scripts/setup_board.sh`

## 当前限制

彩色相机未标定前，三维 X/Y 使用临时内参；Z 来自真实深度。详细状态与安装步骤见
仓库根目录 `docs/user_manual.md`。