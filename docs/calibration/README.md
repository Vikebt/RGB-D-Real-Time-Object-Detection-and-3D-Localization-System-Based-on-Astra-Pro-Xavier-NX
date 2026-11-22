# 标定归档

本目录保存 **本机 Astra Pro（序列号 17121920428）** 的彩色相机标定结果，
用于文档追溯。

- 文件：`rgb_camera_20261122_134029.yaml`
- 标定日期：2026-11-22
- 分辨率：640×480
- 内角点：9×6，方格 25mm（`tools/chessboard_9x6_25mm.pdf`）
- K：`fx=568.6674, fy=568.3721, cx=316.2128, cy=261.5965`
- D：`plumb_bob [0.049442, -0.007669, 0.000592, 0.002328, -0.147309]`

驱动实际加载路径：

```
~/.ros/camera_info/rgb_camera.yaml
```

## 注意

1. 标定与设备、分辨率绑定，**不要**直接把本文件复制给另一台相机使用。
2. 换相机、改分辨率或重新打印棋盘格后，用
   `src/astra_rgbd_perception/scripts/calibrate_rgb.py` 重新标定。
3. 标定脚本归档的 `calibration/rgb/view_*.png` 是绘制角点后的图像，不能用于
   自动重算 RMS；本次 RMS 以标定终端输出为准（未归档）。
4. 安装后应确认：

```bash
rostopic echo -n1 /camera/color/camera_info | head -20
rostopic echo -n1 /camera/depth/camera_info | head -20
```

彩色和深度的 K 都应为非 0，且 depth 对齐模式下的 K 与彩色一致。