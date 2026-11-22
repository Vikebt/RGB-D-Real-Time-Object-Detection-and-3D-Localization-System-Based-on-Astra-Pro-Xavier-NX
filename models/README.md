# models/

本目录存放 YOLOv5n 的 ONNX 与 TensorRT engine。**这两个文件不提交到 Git**
（见仓库根目录 `.gitignore`）：ONNX 可通过官方 release 下载，engine 与设备和
TensorRT 版本绑定。

## 下载 ONNX

```bash
cd ~/astra_rgbd_ws
mkdir -p models
curl -L --retry 3 \
  -o models/yolov5n.onnx \
  https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx

sha256sum models/yolov5n.onnx
# 04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262
```

## 构建 engine（在 Xavier NX 上）

```bash
export PATH=/usr/local/cuda/bin:$PATH
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/yolov5n.onnx \
  --saveEngine=models/yolov5n_fp16.engine \
  --fp16
```

预期：

- 输入 `images 1x3x640x640`
- 输出 `output0 1x25200x85`
- engine 约 7.0MB
- Engine-only GPU 约 7.36ms/帧，约 130 QPS (trtexec)

详见 `docs/model_deployment.md`。

## 注意

- engine 只能在本机（同 TensorRT 版本/同 GPU 架构）使用；换板后重新构建。
- 当前 engine 的 binding 为 FP16；`TrtEngine` 会自动处理 FP16/FP32。
- 如果重新导出模型，类别顺序必须与 `src/astra_rgbd_perception/config/coco.names`
  一致。