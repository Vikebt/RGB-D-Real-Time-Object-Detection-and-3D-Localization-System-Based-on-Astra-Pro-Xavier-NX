/**
 * @file detector_node.cpp
 * @brief YOLOv5n TensorRT detector for Astra Pro color images.
 * @details
 * Pipeline:
 * `sensor_msgs/Image (rgb8/bgr8)` -> letterbox 640x640 -> TensorRT FP16
 * engine -> YOLO decode + class-wise NMS -> `Detection2DArray` and an
 * annotated BGR image.
 *
 * The node reads raw image buffers into `cv::Mat` views; it does not depend on
 * cv_bridge, avoiding the OpenCV 3.2/4.1 ABI issue on JetPack 4.6.
 *
 * @copyright MIT License, 2026.
 */

// 中文说明：YOLOv5n TensorRT 检测节点。
// 流程：rgb8/bgr8 -> letterbox 640x640 -> FP16 engine -> 解码 + 按类 NMS
//       -> Detection2DArray + 标注图。
// 直接读原始图像缓冲，不依赖 cv_bridge，避免 OpenCV ABI 冲突。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "astra_rgbd_perception/Detection2DArray.h"
#include "astra_rgbd_perception/trt_engine.hpp"

namespace {

/**
 * @struct Detection
 * @brief Internal 2D detection in original color-image coordinates.
 */
// 中文：内部 2D 检测结果，坐标已映射回原始彩色图像。
struct Detection {
  int class_id = -1;  ///< COCO class index.
  float score = 0.0f; ///< objectness * class score.
  float x1 = 0.0f;    ///< Left pixel coordinate.
  float y1 = 0.0f;    ///< Top pixel coordinate.
  float x2 = 0.0f;    ///< Right pixel coordinate.
  float y2 = 0.0f;    ///< Bottom pixel coordinate.
};

/**
 * @class DetectorNode
 * @brief ROS node for YOLOv5n TensorRT inference and 2D detection output.
 *
 * The class owns the TensorRT engine, class-name table, subscribers and
 * publishers. It performs preprocessing, inference, postprocessing and
 * optional annotated-image publication in one callback to keep the pipeline
 * simple and avoid frame backlog.
 */
// 中文：检测节点。持有 engine、类别表、订阅/发布；一次回调完成整条检测链路。
class DetectorNode {
 public:
  /**
   * @brief Load parameters, TensorRT engine and class names.
   *
   * Fails fast if the engine path is empty or the engine cannot be loaded.
   */
  // 中文：加载参数、engine、类别名；engine 失败则快速退出。
  DetectorNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("engine_path", engine_path_, "");
    pnh_.param<std::string>("color_topic", color_topic_, "/camera/color/image_raw");
    pnh_.param<std::string>("detections_topic", detections_topic_,
                            "/perception/detections_2d");
    pnh_.param<std::string>("annotated_topic", annotated_topic_,
                            "/perception/image_annotated");
    pnh_.param<std::string>("class_names_file", class_names_file_, "");
    pnh_.param<int>("input_size", input_size_, 640);
    pnh_.param<double>("conf_threshold", conf_threshold_, 0.25);
    pnh_.param<double>("iou_threshold", iou_threshold_, 0.45);
    pnh_.param<int>("max_detections", max_detections_, 100);
    pnh_.param<bool>("publish_annotated", publish_annotated_, true);
    pnh_.param<bool>("debug_output", debug_output_, false);

    if (input_size_ != 640) {
      ROS_WARN("input_size=%d but the bundled engine expects 640; the engine "
               "will report a mismatch if it is not rebuilt",
               input_size_);
    }

    if (class_names_file_.empty()) {
      class_names_file_ =
          ros::package::getPath("astra_rgbd_perception") + "/config/coco.names";
    }
    loadClassNames();

    if (engine_path_.empty()) {
      ROS_FATAL("engine_path is empty; pass the TensorRT engine path");
      return;
    }
    if (!engine_.load(engine_path_)) {
      ROS_FATAL("Failed to load TensorRT engine: %s", engine_path_.c_str());
      return;
    }
    if (engine_.input_dims().size() != 4 ||
        engine_.input_dims()[1] != 3 ||
        engine_.input_dims()[2] != input_size_ ||
        engine_.input_dims()[3] != input_size_) {
      ROS_FATAL("Engine input shape is not [1,3,%d,%d]", input_size_,
                input_size_);
      return;
    }

    if (debug_output_) {
      size_t self_elements = 1;
      for (int dim : engine_.input_dims()) {
        self_elements *= static_cast<size_t>(dim);
      }
      std::vector<float> self_input(self_elements, 0.5f);
      std::vector<float> self_output;
      if (engine_.infer(self_input.data(), self_input.size(), &self_output)) {
        float min_value = 1e30f;
        float max_value = -1e30f;
        size_t nonzero = 0;
        for (float value : self_output) {
          min_value = std::min(min_value, value);
          max_value = std::max(max_value, value);
          if (value != 0.0f) {
            ++nonzero;
          }
        }
        ROS_WARN("engine self-test: min=%.4f max=%.4f nonzero=%zu/%zu",
                 min_value, max_value, nonzero, self_output.size());
      } else {
        ROS_ERROR("engine self-test inference failed");
      }
    }

    detections_pub_ =
        nh_.advertise<astra_rgbd_perception::Detection2DArray>(detections_topic_,
                                                               5);
    if (publish_annotated_) {
      annotated_pub_ = nh_.advertise<sensor_msgs::Image>(annotated_topic_, 2);
    }
    color_sub_ = nh_.subscribe(color_topic_, 1, &DetectorNode::imageCallback,
                               this);

    ROS_INFO_STREAM("detector_node started");
    ROS_INFO_STREAM("  engine: " << engine_path_);
    ROS_INFO_STREAM("  color: " << color_topic_
                                << "  detections: " << detections_topic_);
    ROS_INFO_STREAM("  conf=" << conf_threshold_ << " iou=" << iou_threshold_
                              << " max_det=" << max_detections_);
  }

  /// @return true when the TensorRT engine is initialized.
  bool ready() const { return engine_.ok(); }

 private:
  /**
   * @brief Load COCO class names from `class_names_file_`.
   *
   * Falls back to `class_<id>` names if the file cannot be read.
   */
  // 中文：读取 COCO 类别名；读不到则用 class_<id> 占位。
  void loadClassNames() {
    std::ifstream file(class_names_file_);
    if (!file) {
      ROS_WARN("Cannot open class names file: %s", class_names_file_.c_str());
    } else {
      std::string line;
      while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        if (!line.empty()) {
          class_names_.push_back(line);
        }
      }
    }
    if (class_names_.empty()) {
      ROS_WARN("Using class_<id> placeholders; expected 80 COCO names");
    }
  }

  /**
   * @brief Resolve a class index to a human-readable name.
   * @param id COCO class index.
   * @return Class name, or `class_<id>` when unknown.
   */
  // 中文：类别索引 -> 名称。
  std::string className(int id) const {
    if (id >= 0 && id < static_cast<int>(class_names_.size())) {
      return class_names_[id];
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "class_%d", id);
    return std::string(buffer);
  }

  /**
   * @brief Process one color image through the full detection pipeline.
   * @param msg Incoming color image (`rgb8` or `bgr8`).
   */
  // 中文：一帧图像的完整检测流程。
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!engine_.ok()) {
      return;
    }
    if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
      ROS_WARN_THROTTLE(5.0, "Unsupported image encoding: %s",
                        msg->encoding.c_str());
      return;
    }
    if (msg->step < msg->width * 3 ||
        msg->data.size() < msg->step * msg->height) {
      ROS_WARN_THROTTLE(5.0, "Malformed image buffer");
      return;
    }

    cv::Mat wrapped(static_cast<int>(msg->height), static_cast<int>(msg->width),
                    CV_8UC3, const_cast<uint8_t*>(msg->data.data()),
                    static_cast<size_t>(msg->step));
    cv::Mat rgb;
    if (msg->encoding == "rgb8") {
      rgb = wrapped;
    } else {
      cv::cvtColor(wrapped, rgb, cv::COLOR_BGR2RGB);
    }

    std::vector<float> input;
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    double preprocess_ms = 0.0;
    if (!preprocess(rgb, &input, &scale, &pad_x, &pad_y, &preprocess_ms)) {
      return;
    }

    if (debug_output_) {
      static ros::WallTime last_input_debug;
      const ros::WallTime now = ros::WallTime::now();
      if ((now - last_input_debug).toSec() >= 2.0) {
        float min_value = 1e30f;
        float max_value = -1e30f;
        size_t nonzero = 0;
        for (float value : input) {
          min_value = std::min(min_value, value);
          max_value = std::max(max_value, value);
          if (value > 0.0f) {
            ++nonzero;
          }
        }
        ROS_WARN("input stats: min=%.4f max=%.4f nonzero=%zu/%zu",
                 min_value, max_value, nonzero, input.size());
        last_input_debug = now;
      }
    }

    std::vector<float> output;
    const ros::WallTime infer_start = ros::WallTime::now();
    const bool ok = engine_.infer(input.data(), input.size(), &output);
    const double inference_ms =
        (ros::WallTime::now() - infer_start).toSec() * 1000.0;
    if (!ok) {
      return;
    }

    const ros::WallTime post_start = ros::WallTime::now();
    std::vector<Detection> detections;
    postprocess(output, static_cast<int>(msg->width),
                static_cast<int>(msg->height), scale, pad_x, pad_y,
                &detections);
    const double postprocess_ms =
        (ros::WallTime::now() - post_start).toSec() * 1000.0;

    publishDetections(msg->header, msg->width, msg->height, preprocess_ms,
                      inference_ms, postprocess_ms, detections);
    if (publish_annotated_) {
      publishAnnotated(rgb, msg->header, detections);
    }

    ++frames_since_log_;
    const ros::WallTime now = ros::WallTime::now();
    const double elapsed = (now - last_log_time_).toSec();
    if (elapsed >= 1.0) {
      ROS_INFO("detector: %.1f FPS pre=%.1fms infer=%.1fms post=%.1fms det=%zu",
               frames_since_log_ / elapsed, preprocess_ms, inference_ms,
               postprocess_ms, detections.size());
      frames_since_log_ = 0;
      last_log_time_ = now;
    }
  }

  /**
   * @brief Letterbox a color image and produce normalized NCHW FP32 input.
   * @param rgb Input RGB image.
   * @param input Output vector, resized to 3*640*640.
   * @param scale Scale factor used for the letterbox transform.
   * @param pad_x Horizontal padding in the 640x640 canvas.
   * @param pad_y Vertical padding in the 640x640 canvas.
   * @param elapsed_ms Preprocessing time in milliseconds.
   * @return true on success.
   */
  // 中文：letterbox 预处理，输出归一化 NCHW FP32。
  bool preprocess(const cv::Mat& rgb, std::vector<float>* input, float* scale,
                  int* pad_x, int* pad_y, double* elapsed_ms) {
    const ros::WallTime start = ros::WallTime::now();
    const int width = rgb.cols;
    const int height = rgb.rows;
    // 中文：保持宽高比缩放到 640，空白用 114 填充（letterbox）。
    *scale = std::min(static_cast<float>(input_size_) / width,
                      static_cast<float>(input_size_) / height);
    const int resized_w = std::max(1, static_cast<int>(std::lround(width * *scale)));
    const int resized_h = std::max(1, static_cast<int>(std::lround(height * *scale)));
    *pad_x = (input_size_ - resized_w) / 2;
    *pad_y = (input_size_ - resized_h) / 2;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(resized_w, resized_h), 0.0, 0.0,
               cv::INTER_LINEAR);
    cv::Mat canvas(input_size_, input_size_, CV_8UC3,
                   cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));

    const int plane = input_size_ * input_size_;
    input->resize(static_cast<size_t>(3 * plane));
    float* dst = input->data();
    // 中文：转 NCHW 并归一化到 [0,1]；输入保持 RGB 顺序。
    constexpr float kNorm = 1.0f / 255.0f;
    for (int y = 0; y < input_size_; ++y) {
      const cv::Vec3b* row = canvas.ptr<cv::Vec3b>(y);
      const int row_offset = y * input_size_;
      for (int x = 0; x < input_size_; ++x) {
        const cv::Vec3b& pixel = row[x];
        const int idx = row_offset + x;
        dst[idx] = pixel[0] * kNorm;
        dst[plane + idx] = pixel[1] * kNorm;
        dst[2 * plane + idx] = pixel[2] * kNorm;
      }
    }
    *elapsed_ms = (ros::WallTime::now() - start).toSec() * 1000.0;
    return true;
  }

  /**
   * @brief Compute intersection-over-union of two 2D boxes.
   * @param a First detection.
   * @param b Second detection.
   * @return IoU in [0, 1].
   */
  // 中文：计算两个框的 IoU。
  static float iou(const Detection& a, const Detection& b) {
    const float inter_w =
        std::max(0.0f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    const float inter_h =
        std::max(0.0f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    const float inter = inter_w * inter_h;
    const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    const float united = area_a + area_b - inter;
    return united > 0.0f ? inter / united : 0.0f;
  }

  /**
   * @brief Decode YOLOv5 output and apply class-wise NMS.
   * @param output Raw TensorRT output (`1x25200x85` or transposed).
   * @param image_width Original color image width.
   * @param image_height Original color image height.
   * @param scale Letterbox scale factor.
   * @param pad_x Letterbox horizontal padding.
   * @param pad_y Letterbox vertical padding.
   * @param detections Output detections in original image coordinates.
   */
  // 中文：解码 YOLO 输出并按类别做 NMS。
  void postprocess(const std::vector<float>& output, int image_width,
                   int image_height, float scale, int pad_x, int pad_y,
                   std::vector<Detection>* detections) {
    detections->clear();
    const std::vector<int>& dims = engine_.output_dims();
    int num = 0;
    int stride = 0;
    bool transposed = false;
    if (dims.size() == 3) {
      if (dims[2] == 85) {
        num = dims[1];
        stride = dims[2];
      } else if (dims[1] == 85) {
        num = dims[2];
        stride = dims[1];
        transposed = true;
      }
    }
    if (num <= 0 || stride <= 5) {
      ROS_WARN_THROTTLE(5.0, "Unexpected YOLO output shape");
      return;
    }
    const int num_classes = stride - 5;
    auto at = [&](int index, int channel) -> float {
      return transposed ? output[static_cast<size_t>(channel) * num + index]
                        : output[static_cast<size_t>(index) * stride + channel];
    };

    if (debug_output_) {
      static ros::WallTime last_debug;
      const ros::WallTime now = ros::WallTime::now();
      if ((now - last_debug).toSec() >= 2.0) {
        float max_obj = -1e30f;
        float max_all = -1e30f;
        float min_all = 1e30f;
        int over_conf = 0;
        int best_row = -1;
        for (int i = 0; i < num; ++i) {
          const float obj = at(i, 4);
          if (obj > max_obj) {
            max_obj = obj;
            best_row = i;
          }
          if (obj >= conf_threshold_) {
            ++over_conf;
          }
          for (int c = 0; c < stride; ++c) {
            const float v = at(i, c);
            max_all = std::max(max_all, v);
            min_all = std::min(min_all, v);
          }
        }
        if (best_row >= 0) {
          int best_class = 0;
          float best_score = at(best_row, 5);
          for (int c = 1; c < num_classes; ++c) {
            if (at(best_row, 5 + c) > best_score) {
              best_score = at(best_row, 5 + c);
              best_class = c;
            }
          }
          ROS_WARN("yolo debug: num=%d stride=%d min=%.3f max=%.3f max_obj=%.3f "
                   "over_conf=%d | best row cx=%.2f cy=%.2f w=%.2f h=%.2f "
                   "obj=%.3f cls=%d score=%.3f",
                   num, stride, min_all, max_all, max_obj, over_conf,
                   at(best_row, 0), at(best_row, 1), at(best_row, 2),
                   at(best_row, 3), max_obj, best_class, best_score);
        }
        last_debug = now;
      }
    }

    std::vector<Detection> candidates;
    candidates.reserve(256);
    for (int i = 0; i < num; ++i) {
      const float objectness = at(i, 4);
      if (!std::isfinite(objectness) || objectness < conf_threshold_) {
        continue;
      }
      int best_class = 0;
      float best_score = at(i, 5);
      for (int c = 1; c < num_classes; ++c) {
        const float score = at(i, 5 + c);
        if (score > best_score) {
          best_score = score;
          best_class = c;
        }
      }
      // 中文：YOLOv5 置信度 = objectness * 最大类别分数。
      const float score = objectness * best_score;
      if (score < conf_threshold_) {
        continue;
      }
      const float cx = at(i, 0);
      const float cy = at(i, 1);
      const float w = at(i, 2);
      const float h = at(i, 3);
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) ||
          !std::isfinite(h) || w <= 1.0f || h <= 1.0f) {
        continue;
      }
      Detection det;
      det.class_id = best_class;
      det.score = score;
      // 中文：把 letterbox 坐标反变换回原始图像坐标。
      det.x1 = (cx - 0.5f * w - pad_x) / scale;
      det.y1 = (cy - 0.5f * h - pad_y) / scale;
      det.x2 = (cx + 0.5f * w - pad_x) / scale;
      det.y2 = (cy + 0.5f * h - pad_y) / scale;
      det.x1 = std::max(0.0f, std::min(det.x1, image_width - 1.0f));
      det.y1 = std::max(0.0f, std::min(det.y1, image_height - 1.0f));
      det.x2 = std::max(0.0f, std::min(det.x2, image_width - 1.0f));
      det.y2 = std::max(0.0f, std::min(det.y2, image_height - 1.0f));
      if (det.x2 <= det.x1 || det.y2 <= det.y1) {
        continue;
      }
      candidates.push_back(det);
    }

    // 中文：按置信度排序后做按类别 NMS，抑制同类高 IoU 框。
    std::sort(candidates.begin(), candidates.end(),
              [](const Detection& a, const Detection& b) {
                return a.score > b.score;
              });
    std::vector<bool> removed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (removed[i]) {
        continue;
      }
      detections->push_back(candidates[i]);
      if (static_cast<int>(detections->size()) >= max_detections_) {
        break;
      }
      for (size_t j = i + 1; j < candidates.size(); ++j) {
        if (removed[j] || candidates[j].class_id != candidates[i].class_id) {
          continue;
        }
        if (iou(candidates[i], candidates[j]) > iou_threshold_) {
          removed[j] = true;
        }
      }
    }
  }

  /**
   * @brief Publish a `Detection2DArray` message.
   * @param header Image header copied to the message.
   * @param image_width Source image width.
   * @param image_height Source image height.
   * @param preprocess_ms Preprocessing time.
   * @param inference_ms TensorRT + copy time.
   * @param postprocess_ms Decode/NMS time.
   * @param detections Detections to publish.
   */
  // 中文：发布 Detection2DArray。
  void publishDetections(const std_msgs::Header& header, uint32_t image_width,
                         uint32_t image_height, double preprocess_ms,
                         double inference_ms, double postprocess_ms,
                         const std::vector<Detection>& detections) {
    astra_rgbd_perception::Detection2DArray array;
    array.header = header;
    array.image_width = image_width;
    array.image_height = image_height;
    array.preprocess_ms = static_cast<float>(preprocess_ms);
    array.inference_ms = static_cast<float>(inference_ms);
    array.postprocess_ms = static_cast<float>(postprocess_ms);
    array.detections.reserve(detections.size());
    for (const Detection& det : detections) {
      astra_rgbd_perception::Detection2D out;
      out.class_id = static_cast<uint32_t>(det.class_id);
      out.class_name = className(det.class_id);
      out.confidence = det.score;
      out.xmin = static_cast<uint32_t>(det.x1);
      out.ymin = static_cast<uint32_t>(det.y1);
      out.xmax = static_cast<uint32_t>(det.x2);
      out.ymax = static_cast<uint32_t>(det.y2);
      array.detections.push_back(out);
    }
    detections_pub_.publish(array);
  }

  /**
   * @brief Draw boxes/labels and publish a BGR annotated image.
   * @param rgb Source RGB image.
   * @param header Image header copied to the output message.
   * @param detections Detections to draw.
   */
  // 中文：画框/标签并发布 BGR 标注图。
  void publishAnnotated(const cv::Mat& rgb, const std_msgs::Header& header,
                        const std::vector<Detection>& detections) {
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    for (const Detection& det : detections) {
      const cv::Scalar color(80 + (det.class_id * 37) % 175,
                             80 + (det.class_id * 91) % 175,
                             80 + (det.class_id * 53) % 175);
      cv::rectangle(bgr,
                    cv::Rect(static_cast<int>(det.x1), static_cast<int>(det.y1),
                             static_cast<int>(det.x2 - det.x1),
                             static_cast<int>(det.y2 - det.y1)),
                    color, 2);
      char label[128];
      std::snprintf(label, sizeof(label), "%s %.2f",
                    className(det.class_id).c_str(), det.score);
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      cv::rectangle(bgr,
                    cv::Rect(static_cast<int>(det.x1),
                             std::max(0, static_cast<int>(det.y1) - text_size.height - 4),
                             text_size.width + 4, text_size.height + 4),
                    color, cv::FILLED);
      cv::putText(bgr, label,
                  cv::Point(static_cast<int>(det.x1) + 2,
                            std::max(text_size.height,
                                     static_cast<int>(det.y1) - 2)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    sensor_msgs::Image output;
    output.header = header;
    output.height = static_cast<uint32_t>(bgr.rows);
    output.width = static_cast<uint32_t>(bgr.cols);
    output.encoding = "bgr8";
    output.is_bigendian = false;
    output.step = static_cast<uint32_t>(bgr.step);
    output.data.assign(bgr.data, bgr.data + bgr.step * bgr.rows);
    annotated_pub_.publish(output);
  }

  /// @name ROS interfaces
  /// @{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber color_sub_;
  ros::Publisher detections_pub_;
  ros::Publisher annotated_pub_;

  /// @}

  /// @name Model and class names
  /// @{
  astra_rgbd_perception::TrtEngine engine_;
  std::vector<std::string> class_names_;

  /// @}

  /// @name Topic parameters
  /// @{
  std::string engine_path_;
  std::string color_topic_;
  std::string detections_topic_;
  std::string annotated_topic_;
  std::string class_names_file_;
  /// @}

  /// @name Detection parameters
  /// @{
  int input_size_ = 640;
  double conf_threshold_ = 0.25;
  double iou_threshold_ = 0.45;
  int max_detections_ = 100;
  bool publish_annotated_ = true;
  bool debug_output_ = false;

  /// @}

  /// @name Runtime state
  /// @{
  ros::WallTime last_log_time_ = ros::WallTime::now();
  int frames_since_log_ = 0;
  /// @}
};

}  // namespace

/**
 * @brief Entry point for `detector_node`.
 * @param argc ROS argument count.
 * @param argv ROS argument vector.
 * @return 0 on normal shutdown, 1 if the engine cannot be initialized.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "detector_node");
  DetectorNode node;
  if (!node.ready()) {
    ROS_FATAL("Detector node is not ready");
    return 1;
  }
  ros::spin();
  return 0;
}