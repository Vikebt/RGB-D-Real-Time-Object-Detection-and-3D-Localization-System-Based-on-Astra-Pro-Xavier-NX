# YOLOv5n 模型部署（ONNX → TensorRT FP16）

## 1. 模型来源

使用 Ultralytics YOLOv5 v7.0 官方 release 的预导出 ONNX：

```
https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
```

实测文件：

| 项目 | 值 |
|---|---|
| 大小 | 3,981,910 bytes（约 3.8MB） |
| SHA256 | `04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262` |
| 输入 | `images`，`1x3x640x640`，NCHW |
| 输出 | `output0`，`1x25200x85` |
| 类别 | COCO 80 类（`config/coco.names`） |

> 没有在 Xavier 上安装 PyTorch。任务要求在 PC 端导出 ONNX；本项目的流程是
> 使用官方预导出 ONNX，在 Xavier 上用 `trtexec` 构建本机 engine。

下载：

```bash
cd ~/astra_rgbd_ws
mkdir -p models
curl -L --retry 3 -o models/yolov5n.onnx \
  https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx
sha256sum models/yolov5n.onnx
```

---

## 2. 构建 TensorRT engine

环境：TensorRT 8.2.1、CUDA 10.2、Xavier NX 20W 6 核。

```bash
export PATH=/usr/local/cuda/bin:$PATH
cd ~/astra_rgbd_ws
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov5n.onnx \
  --saveEngine=models/yolov5n_fp16.engine \
  --fp16
```

构建日志中的关键行：

```
[TRT] Created input binding for images with dimensions 1x3x640x640
[TRT] Created output binding for output0 with dimensions 1x25200x85
```

构建结果：

| 项目 | 实测 |
|---|---|
| 构建时间 | 约 15–25 分钟（Xavier NX） |
| engine 大小 | 7.0 MB |
| Workspace | 16 MiB（trtexec 默认） |
| 输入/输出 dtype | **FP16**（`dtype=1`） |
| Engine-only GPU latency | 均值 **7.36 ms**（trtexec） |
| Engine-only Host latency | 均值 7.65 ms |
| 吞吐 | **130 QPS**（trtexec 单 stream） |

验证：

```bash
/usr/src/tensorrt/bin/trtexec \
  --loadEngine=models/yolov5n_fp16.engine \
  --iterations=100 --avgRuns=10
```

> engine 与设备、TensorRT 版本绑定；换板/升级 TensorRT 后必须重新构建。
> 不要把 engine 提交到 Git。

### 2.1 可选：FP32 I/O

`--fp16` 默认会得到 FP16 binding。当前 C++ 封装已经处理 FP16/FP32 两种 binding。
如果希望减少主机端 `float ↔ __half` 转换，可重建为 FP32 I/O（计算仍为 FP16）：

```bash
trtexec --onnx=models/yolov5n.onnx \
  --saveEngine=models/yolov5n_fp16io32.engine \
  --fp16 --inputIOFormats=fp32:chw --outputIOFormats=fp32:chw
```

本项目实测的 28.4ms 检测耗时包含 H2D/D2H 和主机端 FP16 转换；engine-only 为
7.36ms。若后续要做极致优化，可改用 FP32 I/O、pinned memory 或 GPU 端转换。

---

## 3. C++ 推理实现

### 3.1 引擎封装

`include/astra_rgbd_perception/trt_engine.hpp` / `src/trt_engine.cpp`：

- 反序列化 engine，查找唯一输入/输出 binding。
- 读取 binding dtype 和 dims。
- 为 FP16/FP32 分配对应字节数的 device buffer。
- `infer()`：
  - 输入是 `std::vector<float>`；
  - 若 input binding 为 FP16，先转换到 `__half`；
  - `cudaMemcpyAsync` H2D；
  - `enqueueV2`；
  - D2H；
  - 若 output binding 为 FP16，转换为 `std::vector<float>`。

> 早期版本把 FP16 binding 当 FP32 读取，导致输出全是 0 或 1e20 级异常值。
> 启动日志中的 `binding ... dtype=` 可用于确认。当前代码已修复。

### 3.2 预处理

`src/detector_node.cpp`：

1. 输入 `sensor_msgs/Image`，要求 `rgb8` 或 `bgr8`。
2. 按比例缩放到 640×640，保持宽高比，空白用 114 填充（letterbox）。
3. 转 NCHW、`/255.0`。
4. 记录 `preprocess_ms`。

### 3.3 后处理

YOLOv5 v7.0 ONNX 输出为解码后的 `[1, 25200, 85]`：

```
cx, cy, w, h, objectness, class_score[80]
```

- 置信度 = `objectness * max(class_score)`。
- 反 letterbox：

```
x = (cx - w/2 - pad_x) / scale
y = (cy - h/2 - pad_y) / scale
```

- 按类别做 IoU NMS（`iou_threshold`）。
- 最多保留 `max_detections` 个。
- 发布 `Detection2DArray` 和标注图。

实测 COCO 类别名见 `config/coco.names`。

---

## 4. 性能与调优

| 指标 | 实测 |
|---|---|
| 检测节点输出 | 28–30 FPS |
| 预处理 | 1.8 ms |
| 推理（含主机转换/拷贝） | 28.4 ms |
| 后处理 | 0.5 ms |
| 检测端到端延迟 | 均值 35.6 ms，p95 38.1 ms |
| CPU（detector） | ~88%（单核） |
| GPU（采样） | 38–45% |

关键点：

- **必须 `-O3`**：`src/astra_rgbd_perception/CMakeLists.txt` 中有
  `add_compile_options(-O3)`。去掉后推理路径会退化到约 110ms。
- `debug_output: false`，避免每 2 秒扫描 2.1M 个输出值。
- 订阅队列为 1，避免积压旧帧。
- 点云按 10Hz 限速，不和检测抢带宽。

---

## 5. 常见错误

| 现象 | 原因 | 处理 |
|---|---|---|
| engine 加载失败 | engine 与设备/TensorRT 不匹配 | 在本机重新 trtexec |
| 输出全 0 / 1e20 | FP16 binding 被当 FP32 | 使用当前 `TrtEngine`，或重建 engine 为 FP32 I/O |
| 检测框位置偏移 | 预处理 RGB/BGR 或 letterbox 参数错误 | 相机是 `rgb8`，不要按 BGR 处理；检查 pad/scale |
| 类别名不对 | `coco.names` 缺失/顺序错误 | 使用 `config/coco.names` 80 类 |
| 推理很慢 | 未开 `-O3` / 调试输出 / CPU 争抢 | 检查编译参数、关闭 debug、用 tegrastats 观察 |
| 输出形状不是 25200×85 | 使用了不同导出配置（动态尺寸/其他版本） | 重新按 v7.0 640×640 静态导出 |

---

## 6. 重新训练/替换模型

如果需要换模型：

1. 在 PC 上导出静态 640×640、opset 12、batch=1 的 ONNX。
2. 确认输出布局为 `1xNx85` 或 `1x85xN`（当前后处理支持两种）。
3. 在 Xavier 上重新 `trtexec` 构建 engine。
4. 更新 `config/coco.names`（类别顺序必须与训练一致）。
5. 如输入尺寸不是 640，修改 `config/detector.yaml` 的 `input_size`，并确认
   `detector_node` 的输入形状检查通过。

不要直接在 Xavier 上安装最新 PyTorch/Ultralytics；JetPack 4.6 + CUDA 10.2 的兼容性
风险很高。