# 性能报告

- 测试日期：2026-11-21
- 设备：Jetson Xavier NX Developer Kit，16GB 模组
- 模式：`MODE_20W_6CORE`（20W，6 核）
- 软件：L4T R32.7.1 / JetPack 4.6.1 / ROS Melodic / CUDA 10.2 / TensorRT 8.2.1
- 相机：Astra Pro，640×480@30，深度对齐到彩色
- 模型：YOLOv5n，TensorRT FP16，640×640，batch=1
- 编译优化：`-O3`（`astra_rgbd_perception/CMakeLists.txt`）
- 场景：办公室，桌面/显示器/人员，距离约 2–8m

> 所有数据来自实测 `/perception/sync_stats`、`/perception/detections_2d`、
> `/perception/objects_3d`、`rostopic hz`、`tegrastats`、`ps`。没有编造。

---

## 1. 验收目标与实际结果

| 任务书目标 | 实测 | 结论 |
|---|---|---|
| RGB-D 稳定采集 ≥ 15 FPS | color 30.1–30.3Hz，depth 29.7–30.0Hz | ✅ 约 2 倍 |
| YOLO FP16 ≥ 8 FPS | 检测输出 28–30 FPS | ✅ 约 3.5 倍 |
| 二维框与三维坐标正确关联 | `Object3DArray` 由检测框 + 对齐深度生成 | ✅ 流程与标定内参正确；精度实测待做 |
| 连续运行 10 分钟不崩溃 | 10 分 28 秒，5 节点在线，无崩溃/断连 | ✅ |

---

## 2. 相机与点云

| 指标 | 实测 |
|---|---|
| Color 频率 | 30.1–30.2 Hz |
| Depth 频率 | 29.7–30.0 Hz |
| 分辨率 | 640×480 |
| 编码 | RGB `rgb8`；Depth `16UC1`（mm） |
| 同步时间差 `time_diff_ms` | 约 -17ms 到 +6ms（单次均值 -8.7ms） |
| 同步丢帧 `gap_count` | 0（采样窗口） |
| 深度有效率 | 58%–80%（场景相关；稳定性窗口约 78%） |
| 点云数量 | 约 4.4 万–6.1 万点（step=2） |
| 点云构建耗时 | 2.4ms（`-O3` 后；优化前约 6.8ms） |
| 点云发布频率 | 10Hz（限速） |

---

## 3. 检测（YOLOv5n TensorRT FP16）

### 3.1 Engine-only（trtexec）

| 指标 | 实测 |
|---|---|
| GPU latency | mean 7.36ms，median 7.35ms，p99 7.59ms |
| Host latency | mean 7.65ms |
| 吞吐 | 130 QPS（单 stream） |
| Engine 大小 | 7.0MB |
| 输入/输出 | `images 1x3x640x640` → `output0 1x25200x85`，FP16 I/O |

### 3.2 检测节点（含预处理、H2D/D2H、FP16 转换、后处理）

| 指标 | 实测 |
|---|---|
| 输出频率 | 28.0–30.1 FPS |
| 每帧检测数均值 | 7.5（办公室场景，置信度 0.25） |
| 预处理 | 1.8ms |
| 推理（含主机转换/拷贝） | 28.4ms |
| 后处理 | 0.5ms |
| 检测端到端延迟 | mean 35.6ms，p95 38.1ms，max 41.5ms |

说明：

- Engine-only 为 7.36ms；节点内 28.4ms 的主要额外开销是主机端
  `float ↔ __half` 转换和 pageable 内存的 H2D/D2H。
- 早期未开 `-O3` 时推理路径约 110ms；加入 `-O3` 后恢复到 28ms。
- 早期把 FP16 binding 当 FP32 处理时输出全 0 或异常值；当前 `TrtEngine`
  已按 binding dtype 处理。

---

## 4. 三维定位

标定后 8 秒采样结果：

| 指标 | 实测 |
|---|---|
| 输出频率 | 27.2 FPS |
| 每帧 3D 目标数 | 5.0 |
| 三维成功率（有效深度目标/全部目标） | **82.6%**（1084/1313） |
| 三维端到端延迟 | mean 53.9ms，p95 76.8ms |
| 有效深度中位数 | 2.64m（p5 0.59m，p95 4.46m） |
| 典型结果 | chair Z=4.459m；oven Z=3.397m；cell phone Z=2.502m；tv 无深度 |
| ROI | 检测框中心 50% |
| 深度聚合 | 中位数 + 有效像素平均 (u,v) |
| 最小有效像素/比例 | 10 / 5% |

> 已使用标定内参（fx≈568.67、fy≈568.37，`using_fallback_intrinsics=False`）；
> Z 来自真实深度。0.8–2.0m 卷尺精度实测仍待完成。

---

## 5. 资源占用

采样时系统正在运行相机、同步、检测、定位四个节点。

| 进程 | CPU | RSS | 说明 |
|---|---|---|---|
| `detector_node` | 88–92% | 1.41GB | YOLO + TensorRT + 图像处理 |
| `astra_camera_node` | 44% | 78–80MB | UVC MJPEG + OpenNI 深度 |
| `rgbd_sync_node` | 8.7% | 28MB | 同步 + 点云 |
| `depth_localizer_node` | 5.5% | 28MB | 3D 定位 |
| `rosmaster` | 4.9% | 61MB | |

| 系统指标 | 实测 |
|---|---|
| 内存 | 5.1–5.4GB / 15.8GB 已用；可用约 11.1GB |
| GPU 使用率 | 38–45%（采样时；推理为突发） |
| GPU 频率 | 1.109GHz |
| CPU 频率 | 1.42GHz |
| CPU 温度 | 36.5°C |
| Video RAM | 1.5GB（lfb 1541×4MB，见 tegrastats） |
| 交换 | 0 使用 |

> `detector_node` 的 1.41GB RSS 主要来自 TensorRT/CUDA 在 Jetson 统一内存上的
> 分配。当前未见持续增长；10 分钟稳定性采样见第 7 节。

---

## 6. 端到端时延

```
RGB 采集 → 检测输出：mean 35.6ms, p95 38.1ms
RGB 采集 → 3D 输出： mean 55.8ms, p95 78.0ms
```

三维比检测多约 20ms，包含 ApproximateTime 配对、深度 ROI 扫描、中位数计算和 Marker 构建。

---

## 7. 10 分钟稳定性

测试脚本：

```bash
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/stability_test.sh 600 30
```

测试窗口：2026-11-21 19:58:42 – 20:09:10（**10 分 28 秒**）。

| 指标 | 开始 | 结束 | 结论 |
|---|---|---|---|
| detector RSS | 1,416,008 KB | 1,419,176 KB | +3.1MB，20:03:55 后稳定，无持续增长 |
| camera RSS | 78,688 KB | 80,172 KB | +1.5MB 后稳定 |
| sync / localizer RSS | 28,940 / 28,060 KB | 不变 | 稳定 |
| color / depth | 30.1 / 29.8 Hz | 30.3 / 30.0 Hz | 保持约 30Hz |
| 深度有效率 | 78.3% | 78.4% | 稳定 |
| gap_count | 0 | 0 | 无同步丢帧 |
| 节点数 | 5 | 5 | 无崩溃/退出 |

观察：

- 10 分钟内 5 个节点始终在线：`/camera/camera`、`/rgbd_sync`、`/detector`、
  `/depth_localizer`、`/rosout`。
- 20:08:00 出现一次约 27.7 / 28.5Hz 的短时下降，随后恢复约 30Hz；未导致
  丢帧（`gap_count=0`）或节点退出。
- detector RSS 1.42GB 主要来自 TensorRT/CUDA 在 Jetson 统一内存上的分配；
  10 分钟窗口内仅增加约 0.2%，无明显泄漏趋势。
- 内核日志没有 disconnect/reset；启动阶段有 USB2 `buffer overrun` 警告
  （见 `dmesg`），测试期间未出现功能中断。长时间演示建议使用带供电 USB hub。
- 日志：`~/astra_rgbd_ws/logs/stability_20261121_195841.log`。

---

## 8. 分析与优化建议

当前已超过 MVP 性能目标。进一步优化方向：

1. **减少主机端 FP16 转换**：
   - 用 `trtexec --inputIOFormats=fp32:chw --outputIOFormats=fp32:chw` 重建
     engine；或
   - 使用 CUDA kernel 在 GPU 上做预处理/后处理，配合 pinned memory。
   - 目标：检测节点推理路径从 28ms 接近 engine-only 的 7–10ms。
2. **降低 CPU 占用**：
   - 当前 detector 单核 88%；CPU 转换是主因。
   - 相机 44% 主要来自 MJPEG 解码 + OpenNI；可尝试降低 RGB 帧率或使用
     `/dev/video0` 原生格式（YVYU）对比。
3. **点云**：
   - 当前 step=2、10Hz；如只需 RViz 观察，可 step=3/4 降低带宽。
4. **INT8**：
   - 本项目未启用 INT8；需要校准集和精度评估，不建议在 MVP 阶段引入。
5. **多 stream**：
   - 当前单 stream；相机 30Hz、检测 28–30Hz，已基本匹配，无需多 stream。

---

## 9. 复现命令

```bash
# 启动完整系统
roslaunch astra_rgbd_perception system.launch use_rviz:=false

# Topic 频率/同步/检测耗时/资源
bash ~/astra_rgbd_ws/src/astra_rgbd_perception/scripts/benchmark.sh 10

# 端到端延迟/3D 成功率
python2 - <<'PY'
import rospy, numpy as np
from astra_rgbd_perception.msg import Detection2DArray, Object3DArray
rospy.init_node('latency_probe')
d=[]; o=[]; valid=total=0
def dcb(m): d.append((rospy.Time.now()-m.header.stamp).to_sec()*1000)
def ocb(m):
    global valid,total
    o.append((rospy.Time.now()-m.header.stamp).to_sec()*1000)
    for x in m.objects:
        valid += int(x.depth_valid); total += 1
rospy.Subscriber('/perception/detections_2d', Detection2DArray, dcb)
rospy.Subscriber('/perception/objects_3d', Object3DArray, ocb)
rospy.Timer(rospy.Duration(8), lambda e: rospy.signal_shutdown('done')); rospy.spin()
print('det mean/p95 ms', np.mean(d), np.percentile(d,95))
print('obj mean/p95 ms', np.mean(o), np.percentile(o,95))
print('3D success', valid, '/', total)
PY

# Engine-only
/usr/src/tensorrt/bin/trtexec --loadEngine=~/astra_rgbd_ws/models/yolov5n_fp16.engine \
  --iterations=100 --avgRuns=10
```