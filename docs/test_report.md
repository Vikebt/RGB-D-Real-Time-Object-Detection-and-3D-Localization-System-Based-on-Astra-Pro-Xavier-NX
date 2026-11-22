# 测试报告

- 测试日期：2026-11-21
- 平台：Jetson Xavier NX，JetPack 4.6.1 / L4T R32.7.1，ROS Melodic
- 相机：Astra Pro，640×480@30，`depth_align:=true`
- 模型：YOLOv5n TensorRT FP16
- 原则：只记录实际执行的测试；未执行项明确标注，不编造数据。

---

## 1. 测试结果总表

| # | 测试 | 状态 | 关键结果 |
|---|---|---|---|
| 1 | 系统环境审计 | ✅ 通过 | L4T R32.7.1、CUDA 10.2、TensorRT 8.2.1、ROS Melodic |
| 2 | Astra Pro 相机 | ✅ 通过 | RGB `rgb8` 30.1Hz；Depth `16UC1` 29.8Hz |
| 3 | CameraInfo / TF | ✅ 通过 | 彩色/深度 K 均为标定值；depth frame_id 正确 |
| 4 | RGB-D 同步 | ✅ 通过 | 时间差约 ±17ms；gap_count=0 |
| 5 | XYZ/XYZRGB 点云 | ✅ 通过 | 4.4–6.1 万点，10Hz，frame 正确 |
| 6 | YOLOv5n 检测 | ✅ 通过 | 28–30 FPS；person/laptop/chair/tv 检出 |
| 7 | 三维定位 | ✅ 通过（精度实测待做） | 27.2 FPS；3D 成功率 82.6%；使用标定内参 |
| 8 | `system.launch` 集成 | ✅ 通过 | 5 个节点、全部关键 topic 正常 |
| 9 | rosbag 录制/离线回放 | ✅ 通过 | 9.8s bag；无相机回放可检测/定位 |
| 10 | 10 分钟稳定性 | ✅ 通过 | 10 分 28 秒无崩溃/断连；RSS 稳定；gaps=0 |
| 11 | 0.8–2.0m 精度 | ❌ 未执行 | 需要用户、卷尺、实物；标定后执行 |
| 12 | RViz 截图验收 | ⚠️ 未执行 | 配置/topic 已验证；需显示器现场确认 |

---

## 2. 相机测试

命令：

```bash
roslaunch astra_camera astra_pro.launch \
  depth_align:=true enable_point_cloud:=false enable_ir:=false
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
```

结果：

- Color：30.1 Hz，`rgb8`，640×480，step=1920。
- Depth：29.7–29.8 Hz，`16UC1`，640×480，step=1280。
- 深度原始值单位毫米；`depth_scale_to_m=0.001`。
- 深度对准后 `frame_id=camera_color_optical_frame`。
- 冷启动前几帧全 0，连续订阅约 1 秒后稳定。
- 测试期间无 USB 断连/内核错误。

证据：`docs/camera_configuration.md`、`docs/images/rgbd_color.png`、
`docs/images/rgbd_depth.png`。

### 2.1 CameraInfo 与标定

2026-11-22 完成彩色相机标定：

- 文件：`~/.ros/camera_info/rgb_camera.yaml`
  （归档 `~/astra_rgbd_ws/calibration/rgb_camera_20261122_134029.yaml`）
- Color：`frame_id=camera_color_optical_frame`
  - `K=[568.6674, 0, 316.2128, 0, 568.3721, 261.5965, 0, 0, 1]`
  - `D=[0.049442, -0.007669, 0.000592, 0.002328, -0.147309]`（plumb_bob）
- Depth（对齐）：`frame_id=camera_color_optical_frame`，K/P 与彩色一致，无 NaN。
- `get_camera_params` 仍返回 NaN；该固件不提供工厂参数，因此标定文件是真实内参来源。
- 修复了 UVC 模式下 `OBCameraNode` 未创建彩色 `CameraInfoManager` 的问题，
  否则 depth CameraInfo 会一直是 0。

标定归档视图保存的是绘制角点后的图像，无法用于自动复算 RMS；RMS 以标定时终端
输出为准（本次未记录）。端到端精度将在 0.8–2.0m 实测中验证。

---

## 3. 点云测试

```bash
rostopic hz /perception/points_xyzrgb
rostopic echo -n1 --noarr /perception/points_xyzrgb
```

结果：

- `frame_id=camera_color_optical_frame`。
- fields：`x,y,z,rgb`；point_step=32。
- 点数：约 44,798（早期）/61,269（`-O3` 后、不同场景）。
- 发布频率约 10Hz。
- 构建耗时 2.4ms（优化后）。
- `points_xyz`：fields `x,y,z`，point_step=16。
- 深度有效率约 58%–80%。

RViz 配置 `config/rgbd_view.rviz` 已通过 YAML 解析；Fixed Frame 为
`camera_color_optical_frame`。**未在有显示器的会话中做截图验收**。

---

## 4. YOLOv5n 检测测试

```bash
roslaunch astra_rgbd_perception detector.launch
rostopic hz /perception/detections_2d
```

结果：

- 输出 28–30.1 FPS。
- 每帧约 7.5 个检测（阈值 0.25，办公室场景）。
- 实测类别：person、chair、tv、laptop，置信度 0.41–0.82。
- 标注图：`docs/images/detection_annotated.png`。
- 预处理 1.8ms；推理（含主机转换/拷贝）28.4ms；后处理 0.5ms。
- Engine-only（trtexec）：7.36ms，130 QPS。
- 端到端检测延迟：mean 35.6ms，p95 38.1ms。

检测框与图像内容一致（见标注图）；未发现系统性偏移。

---

## 5. 三维定位测试

```bash
roslaunch astra_rgbd_perception localizer.launch
rostopic echo -n1 /perception/objects_3d
```

8 秒采样结果：

标定后 8 秒采样结果：

| 指标 | 值 |
|---|---|
| 输出频率 | 27.2 FPS |
| objects/frame | 5.0 |
| 有效深度目标/全部目标 | 1084/1313 = **82.6%** |
| 三维延迟 | mean 53.9ms，p95 76.8ms |
| 有效深度中位数 | 2.64m（p5 0.59m，p95 4.46m） |

示例（标定内参，同一场景）：

| 类别 | depth_m | X | Y | Z | depth_valid |
|---|---|---|---|---|---|
| chair | 4.459 | 0.836 | 0.607 | 4.459 | true |
| oven | 3.397 | -1.454 | 0.619 | 3.397 | true |
| cell phone | 2.502 | 0.778 | 0.792 | 2.502 | true |
| tv | 0.0 | 0 | 0 | 0 | false |

说明：

- TV 屏幕/深色区域无深度返回，`depth_valid=false`，未输出虚假坐标。
- X/Y/Z 均使用标定内参与真实深度；`using_fallback_intrinsics=False`。
- **没有做 0.8/1.0/1.5/2.0m 卷尺对比，因此不能声称精度已验收。**

---

## 6. 系统集成测试

```bash
roslaunch astra_rgbd_perception system.launch use_rviz:=false
```

结果：

- 节点：`/camera/camera`、`/rgbd_sync`、`/detector`、`/depth_localizer`、`/rosout`。
- Topic：相机、检测、3D、点云、markers 全部有数据。
- 频率：color 30Hz、depth 30Hz、detections 30Hz、objects 30Hz、points 10Hz。
- 日志仅有预期的 IR 未启用警告。
- markers 每个有效目标 2 个（球 + 文本）。

---

## 7. 离线回放测试

录制：

```bash
bash scripts/record_rgbd.sh 10
# 9.8s，236MB（LZ4），295 帧 RGB/292 帧 Depth
```

回放：

```bash
roslaunch astra_rgbd_perception system.launch start_camera:=false use_rviz:=false
bash scripts/play_rgbd.sh ~/astra_rgbd_ws/bags/rgbd_20261121_195513.bag -r 1
```

结果：

- 无相机连接时，检测与三维定位正常输出。
- 检测：person 0.808、tv 0.641、chair 0.521 等。
- 3D：person 2.197m、chair 2.255m、person 3.118m；tv 无效。
- 说明数据链路可重复、可用于离线对比。

---

## 8. 稳定性测试

```bash
bash scripts/stability_test.sh 600 30
```

测试窗口：19:58:42 – 20:09:10，时长 10 分 28 秒。

- detector RSS：1,416,008 → 1,419,176 KB（+3.1MB，20:03:55 后稳定）。
- camera RSS：78,688 → 80,172 KB（+1.5MB 后稳定）。
- sync/localizer RSS：28,940 / 28,060 KB，保持不变。
- color/depth：大部分样本 29.3–30.3 / 29.7–30.0 Hz；20:08:00 一次短时
  27.7 / 28.5Hz，随后恢复。
- 深度有效率：78.3%–78.9%。
- gap_count=0（全部样本）。
- 五个节点始终在线；无崩溃、无 USB disconnect/reset。
- 启动阶段 dmesg 有 USB2 `buffer overrun` 警告，测试期间无功能中断。
- 日志：`~/astra_rgbd_ws/logs/stability_20261121_195841.log`。

---

## 9. 未执行/待用户配合

| 测试 | 原因 | 需要什么 |
|---|---|---|
| 0.8/1.0/1.5/2.0m 精度 | 当前环境无法远程移动实物/卷尺 | 用 `measure_distance.py` 记录系统 Z；用户配合 |
| RViz 截图 | 需要显示器现场 | `DISPLAY=:0` 或用户查看 |
| 30 分钟及以上稳定性 | 未执行 | 10 分钟已通过；可用 `stability_test.sh 1800 30` 复跑 |
| 夜间/不同光照 | 未覆盖 | 多场景复测 |

**这些项目未完成前，不把项目标记为“最终验收通过”。**

---

## 10. 结论

- MVP 功能链路已跑通：相机 → 同步/点云 → YOLO → 三维定位 → RViz markers，
  并支持录制/离线回放。
- 性能超过任务书初始目标（30Hz 采集、28–30 FPS 检测、27 FPS 3D）。
- 10 分钟稳定性通过：无崩溃、无断连、gaps=0、RSS 稳定。
- 彩色相机标定已完成，彩色/深度 CameraInfo 均使用真实内参。
- 当前主要缺口是 **0.8–2.0m 物理精度验收** 和 **RViz 现场截图**；精度测试需要
  用户现场配合。完成之前，不把项目标记为最终验收通过。