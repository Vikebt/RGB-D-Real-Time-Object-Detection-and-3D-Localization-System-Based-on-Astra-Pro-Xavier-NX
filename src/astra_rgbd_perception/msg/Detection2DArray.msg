# 中文：一帧 2D 检测结果，含图像尺寸与预处理/推理/后处理耗时。
Header header

uint32 image_width
uint32 image_height

float32 preprocess_ms
float32 inference_ms
float32 postprocess_ms

Detection2D[] detections