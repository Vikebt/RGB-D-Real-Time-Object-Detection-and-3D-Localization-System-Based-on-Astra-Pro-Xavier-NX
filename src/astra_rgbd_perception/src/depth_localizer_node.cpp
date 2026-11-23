/**
 * @file depth_localizer_node.cpp
 * @brief 3D localization of YOLO detections using the aligned depth image.
 * @details
 * For each 2D detection box, the central `roi_ratio` of the box is sampled.
 * Zero, non-finite and out-of-range depths are rejected. The median of the
 * remaining depths is used as Z; the mean pixel position of valid samples is
 * back-projected with CameraInfo intrinsics:
 *
 * @f[
 *   X = (u - c_x) \cdot Z / f_x,\quad
 *   Y = (v - c_y) \cdot Z / f_y
 * @f]
 *
 * Results are published as `Object3DArray` and RViz `MarkerArray`.
 * If the valid depth count/ratio is too low, `depth_valid` is false and no
 * position is emitted.
 *
 * @copyright MIT License, 2026.
 */

// 中文说明：三维定位节点。
// 对每个检测框取中心 ROI，过滤无效深度，取中位数作为 Z，
// 用有效像素平均 (u,v) 和 CameraInfo 内参反投影得到 X/Y。
// 有效深度不足时 depth_valid=false，不输出虚假坐标。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

#include "astra_rgbd_perception/Detection2DArray.h"
#include "astra_rgbd_perception/Object3DArray.h"

namespace {

typedef message_filters::sync_policies::ApproximateTime<
    astra_rgbd_perception::Detection2DArray, sensor_msgs::Image>
    SyncPolicy;

/**
 * @class DepthLocalizerNode
 * @brief Converts 2D detections into 3D positions using aligned depth.
 *
 * The node synchronizes `Detection2DArray` with the depth image, caches
 * CameraInfo, extracts the central ROI of each box, computes a median depth
 * and publishes `Object3DArray` plus `MarkerArray` for RViz.
 */
// 中文：把 2D 检测转成 3D 位置，并发布 Object3DArray 和 RViz Marker。
class DepthLocalizerNode {
 public:
  /**
   * @brief Read parameters and create ROS interfaces.
   */
  // 中文：读取参数并创建 ROS 接口。
  DepthLocalizerNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("detections_topic", detections_topic_,
                            "/perception/detections_2d");
    pnh_.param<std::string>("depth_topic", depth_topic_,
                            "/camera/depth/image_raw");
    pnh_.param<std::string>("depth_info_topic", depth_info_topic_,
                            "/camera/depth/camera_info");
    pnh_.param<std::string>("color_info_topic", color_info_topic_,
                            "/camera/color/camera_info");
    pnh_.param<std::string>("objects_topic", objects_topic_,
                            "/perception/objects_3d");
    pnh_.param<std::string>("markers_topic", markers_topic_,
                            "/perception/markers");
    pnh_.param<std::string>("frame_id_override", frame_id_override_, "");

    pnh_.param<double>("depth_scale_to_m", depth_scale_to_m_, 0.001);
    pnh_.param<double>("min_range_m", min_range_m_, 0.3);
    pnh_.param<double>("max_range_m", max_range_m_, 8.0);
    pnh_.param<double>("roi_ratio", roi_ratio_, 0.5);
    pnh_.param<int>("min_valid_pixels", min_valid_pixels_, 10);
    pnh_.param<double>("min_valid_ratio", min_valid_ratio_, 0.05);
    pnh_.param<bool>("require_aligned", require_aligned_, true);
    pnh_.param<double>("marker_lifetime_s", marker_lifetime_s_, 0.5);
    pnh_.param<double>("approx_slop_s", approx_slop_s_, 0.05);

    pnh_.param<bool>("fallback_intrinsics", fallback_intrinsics_, false);
    pnh_.param<double>("fallback_fx", fallback_fx_, 570.3422);
    pnh_.param<double>("fallback_fy", fallback_fy_, 570.3422);
    pnh_.param<double>("fallback_cx", fallback_cx_, 319.5);
    pnh_.param<double>("fallback_cy", fallback_cy_, 239.5);

    if (roi_ratio_ <= 0.0 || roi_ratio_ > 1.0) {
      roi_ratio_ = 0.5;
    }
    if (min_valid_pixels_ < 1) {
      min_valid_pixels_ = 1;
    }

    SyncPolicy policy(10);
    policy.setMaxIntervalDuration(ros::Duration(approx_slop_s_));
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(policy));
    detections_sub_.subscribe(nh_, detections_topic_, 5);
    depth_sub_.subscribe(nh_, depth_topic_, 5);
    sync_->connectInput(detections_sub_, depth_sub_);
    sync_->registerCallback(
        boost::bind(&DepthLocalizerNode::syncCallback, this, _1, _2));

    depth_info_sub_ = nh_.subscribe(depth_info_topic_, 2,
                                    &DepthLocalizerNode::depthInfoCallback, this);
    color_info_sub_ = nh_.subscribe(color_info_topic_, 2,
                                    &DepthLocalizerNode::colorInfoCallback, this);

    objects_pub_ =
        nh_.advertise<astra_rgbd_perception::Object3DArray>(objects_topic_, 5);
    markers_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(markers_topic_, 2);

    last_log_time_ = ros::WallTime::now();
    ROS_INFO_STREAM("depth_localizer_node started");
    ROS_INFO_STREAM("  detections: " << detections_topic_
                                     << "  depth: " << depth_topic_);
    ROS_INFO_STREAM("  roi_ratio=" << roi_ratio_ << " range=[" << min_range_m_
                                   << ", " << max_range_m_ << "] m");
    if (fallback_intrinsics_) {
      ROS_WARN("fallback_intrinsics is enabled: X/Y use provisional intrinsics "
               "until the color camera is calibrated");
    }
  }

 private:
  /**
   * @brief Cache the latest depth CameraInfo.
   * @param msg New CameraInfo message.
   */
  // 中文：缓存深度 CameraInfo。
  void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    depth_info_ = msg;
  }

  /**
   * @brief Cache the latest color CameraInfo.
   * @param msg New CameraInfo message.
   */
  // 中文：缓存彩色 CameraInfo。
  void colorInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    color_info_ = msg;
  }

  /**
   * @brief Resolve intrinsics from depth/color CameraInfo or fallback values.
   * @param fx Output horizontal focal length.
   * @param fy Output vertical focal length.
   * @param cx Output principal point x.
   * @param cy Output principal point y.
   * @param using_fallback Set to true when fallback values are used.
   * @return true if valid intrinsics are available.
   */
  // 中文：解析内参；优先深度/彩色 CameraInfo，无效且允许时用 fallback。
  bool lookupIntrinsics(double* fx, double* fy, double* cx, double* cy,
                        bool* using_fallback) {
    if (depth_info_ && depth_info_->K.size() >= 9 && depth_info_->K[0] > 1.0 &&
        depth_info_->K[4] > 1.0) {
      *fx = depth_info_->K[0];
      *fy = depth_info_->K[4];
      *cx = depth_info_->K[2];
      *cy = depth_info_->K[5];
      *using_fallback = false;
      return true;
    }
    if (color_info_ && color_info_->K.size() >= 9 && color_info_->K[0] > 1.0 &&
        color_info_->K[4] > 1.0) {
      *fx = color_info_->K[0];
      *fy = color_info_->K[4];
      *cx = color_info_->K[2];
      *cy = color_info_->K[5];
      *using_fallback = false;
      return true;
    }
    if (fallback_intrinsics_) {
      *fx = fallback_fx_;
      *fy = fallback_fy_;
      *cx = fallback_cx_;
      *cy = fallback_cy_;
      *using_fallback = true;
      if (!warned_fallback_) {
        ROS_WARN("CameraInfo fx/fy is invalid; using provisional intrinsics "
                 "fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                 *fx, *fy, *cx, *cy);
        warned_fallback_ = true;
      }
      return true;
    }
    ROS_WARN_THROTTLE(5.0,
                      "CameraInfo fx/fy is invalid and fallback_intrinsics is "
                      "false; 3D positions will not be published");
    return false;
  }

  /**
   * @brief Process one synchronized detection/depth pair.
   * @param detections 2D detections from `detector_node`.
   * @param depth Aligned depth image (`16UC1` millimetres or `32FC1` metres).
   *
   * For each detection, samples the central ROI, computes the median valid
   * depth, back-projects the mean valid pixel and publishes `Object3D` and
   * marker messages. Invalid-depth objects are still reported with
   * `depth_valid=false`.
   */
  // 中文：处理一对同步的检测/深度，计算 3D 并发布。
  void syncCallback(
      const astra_rgbd_perception::Detection2DArray::ConstPtr& detections,
      const sensor_msgs::Image::ConstPtr& depth) {
    if (depth->encoding != "16UC1" && depth->encoding != "32FC1") {
      ROS_WARN_THROTTLE(5.0, "Unsupported depth encoding: %s",
                        depth->encoding.c_str());
      return;
    }
    if (require_aligned_ && detections->header.frame_id != depth->header.frame_id) {
      ROS_WARN_THROTTLE(
          5.0,
          "Detection frame '%s' differs from depth frame '%s'; enable "
          "depth_align:=true or set require_aligned:=false.",
          detections->header.frame_id.c_str(), depth->header.frame_id.c_str());
      return;
    }

    double fx, fy, cx, cy;
    bool using_fallback = false;
    if (!lookupIntrinsics(&fx, &fy, &cx, &cy, &using_fallback)) {
      return;
    }

    const ros::WallTime start = ros::WallTime::now();
    const int width = static_cast<int>(depth->width);
    const int height = static_cast<int>(depth->height);
    const std::string frame_id =
        frame_id_override_.empty() ? depth->header.frame_id : frame_id_override_;
    const bool depth_u16 = depth->encoding == "16UC1";
    const double depth_scale = depth_u16 ? depth_scale_to_m_ : 1.0;

    astra_rgbd_perception::Object3DArray objects;
    objects.header = detections->header;
    objects.header.frame_id = frame_id;
    objects.objects.reserve(detections->detections.size());

    visualization_msgs::MarkerArray markers;
    markers.markers.reserve(detections->detections.size() * 2);

    size_t valid_objects = 0;
    for (size_t i = 0; i < detections->detections.size(); ++i) {
      const auto& det = detections->detections[i];
      astra_rgbd_perception::Object3D object;
      object.header = detections->header;
      object.header.frame_id = frame_id;
      object.class_id = det.class_id;
      object.class_name = det.class_name;
      object.confidence = det.confidence;
      object.xmin = det.xmin;
      object.ymin = det.ymin;
      object.xmax = det.xmax;
      object.ymax = det.ymax;
      object.depth_valid = false;
      object.depth_m = 0.0f;
      object.valid_depth_pixels = 0;
      object.valid_depth_ratio = 0.0f;

      const int x0 = std::max(0, std::min(static_cast<int>(det.xmin), width - 1));
      const int x1 = std::max(0, std::min(static_cast<int>(det.xmax), width - 1));
      const int y0 = std::max(0, std::min(static_cast<int>(det.ymin), height - 1));
      const int y1 = std::max(0, std::min(static_cast<int>(det.ymax), height - 1));
      if (x1 <= x0 || y1 <= y0) {
        objects.objects.push_back(object);
        continue;
      }

      const int box_w = x1 - x0 + 1;
      const int box_h = y1 - y0 + 1;
      // 中文：取检测框中心 roi_ratio 比例的 ROI，避免边缘/背景深度干扰。
      const int roi_w = std::max(1, static_cast<int>(box_w * roi_ratio_));
      const int roi_h = std::max(1, static_cast<int>(box_h * roi_ratio_));
      const int center_x = (x0 + x1) / 2;
      const int center_y = (y0 + y1) / 2;
      const int roi_x0 = std::max(x0, center_x - roi_w / 2);
      const int roi_x1 = std::min(x1, roi_x0 + roi_w - 1);
      const int roi_y0 = std::max(y0, center_y - roi_h / 2);
      const int roi_y1 = std::min(y1, roi_y0 + roi_h - 1);
      const int roi_area = std::max(1, (roi_x1 - roi_x0 + 1) *
                                           (roi_y1 - roi_y0 + 1));

      std::vector<float> depths_m;
      depths_m.reserve(roi_area);
      double sum_u = 0.0;
      double sum_v = 0.0;
      for (int v = roi_y0; v <= roi_y1; ++v) {
        const uint8_t* row = depth->data.data() + v * depth->step;
        for (int u = roi_x0; u <= roi_x1; ++u) {
          double z = 0.0;
          if (depth_u16) {
            uint16_t raw = 0;
            std::memcpy(&raw, row + u * sizeof(uint16_t), sizeof(raw));
            if (raw == 0) {
              continue;
            }
            z = static_cast<double>(raw) * depth_scale;
          } else {
            float raw = 0.0f;
            std::memcpy(&raw, row + u * sizeof(float), sizeof(raw));
            if (!std::isfinite(raw) || raw <= 0.0f) {
              continue;
            }
            z = static_cast<double>(raw);
          }
          if (z < min_range_m_ || z > max_range_m_) {
            continue;
          }
          depths_m.push_back(static_cast<float>(z));
          sum_u += u;
          sum_v += v;
        }
      }

      const size_t valid_pixels = depths_m.size();
      object.valid_depth_pixels = static_cast<uint32_t>(valid_pixels);
      object.valid_depth_ratio =
          static_cast<float>(valid_pixels) / static_cast<float>(roi_area);
      if (valid_pixels < static_cast<size_t>(min_valid_pixels_) ||
          object.valid_depth_ratio < min_valid_ratio_) {
        objects.objects.push_back(object);
        continue;
      }

      // 中文：用中位数而不是单像素/均值，抗空洞和飞点。
      auto median_it = depths_m.begin() + valid_pixels / 2;
      std::nth_element(depths_m.begin(), median_it, depths_m.end());
      const double z = *median_it;
      const double u = sum_u / valid_pixels;
      const double v = sum_v / valid_pixels;

      object.depth_m = static_cast<float>(z);
      // 中文：针孔反投影，得到相机光学坐标系下的 X/Y/Z（米）。
      object.position.x = (u - cx) * z / fx;
      object.position.y = (v - cy) * z / fy;
      object.position.z = z;
      object.depth_valid = true;
      ++valid_objects;
      objects.objects.push_back(object);

      const float r = static_cast<float>((det.class_id * 53) % 200 + 55) / 255.0f;
      const float g = static_cast<float>((det.class_id * 97) % 200 + 55) / 255.0f;
      const float b =
          static_cast<float>((det.class_id * 131) % 200 + 55) / 255.0f;

      // 中文：RViz 标记：球体表示位置，文本显示类别和距离。
      visualization_msgs::Marker sphere;
      sphere.header = object.header;
      sphere.ns = "object_positions";
      sphere.id = static_cast<int>(i * 2);
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position = object.position;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.05;
      sphere.scale.y = 0.05;
      sphere.scale.z = 0.05;
      sphere.color.r = r;
      sphere.color.g = g;
      sphere.color.b = b;
      sphere.color.a = 0.85;
      sphere.lifetime = ros::Duration(marker_lifetime_s_);
      markers.markers.push_back(sphere);

      visualization_msgs::Marker text;
      text.header = object.header;
      text.ns = "object_labels";
      text.id = static_cast<int>(i * 2 + 1);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position = object.position;
      text.pose.position.z += 0.08;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.08;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      char label[160];
      std::snprintf(label, sizeof(label), "%s %.2fm (z=%.2f)",
                    object.class_name.c_str(), object.depth_m, object.position.z);
      text.text = label;
      text.lifetime = ros::Duration(marker_lifetime_s_);
      markers.markers.push_back(text);
    }

    objects_pub_.publish(objects);
    markers_pub_.publish(markers);

    const double elapsed_ms =
        (ros::WallTime::now() - start).toSec() * 1000.0;
    ++frames_since_log_;
    const ros::WallTime now = ros::WallTime::now();
    const double elapsed = (now - last_log_time_).toSec();
    if (elapsed >= 1.0) {
      ROS_INFO("localizer: %.1f FPS objects=%zu valid=%zu %.2fms%s",
               frames_since_log_ / elapsed, objects.objects.size(), valid_objects,
               elapsed_ms, using_fallback ? " (fallback intrinsics)" : "");
      frames_since_log_ = 0;
      last_log_time_ = now;
    }
  }

  /// @name ROS interfaces
  /// @{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  message_filters::Subscriber<astra_rgbd_perception::Detection2DArray>
      detections_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber depth_info_sub_;
  ros::Subscriber color_info_sub_;
  ros::Publisher objects_pub_;
  ros::Publisher markers_pub_;

  /// @}

  /// @name Latest CameraInfo
  /// @{
  sensor_msgs::CameraInfo::ConstPtr depth_info_;
  sensor_msgs::CameraInfo::ConstPtr color_info_;

  /// @}

  /// @name Topic and frame parameters
  /// @{
  std::string detections_topic_;
  std::string depth_topic_;
  std::string depth_info_topic_;
  std::string color_info_topic_;
  std::string objects_topic_;
  std::string markers_topic_;
  std::string frame_id_override_;

  /// @}

  /// @name Localization parameters
  /// @{
  double depth_scale_to_m_ = 0.001;
  double min_range_m_ = 0.3;
  double max_range_m_ = 8.0;
  double roi_ratio_ = 0.5;
  int min_valid_pixels_ = 10;
  double min_valid_ratio_ = 0.05;
  bool require_aligned_ = true;
  double marker_lifetime_s_ = 0.5;
  double approx_slop_s_ = 0.05;

  bool fallback_intrinsics_ = false;
  double fallback_fx_ = 570.3422;
  double fallback_fy_ = 570.3422;
  double fallback_cx_ = 319.5;
  double fallback_cy_ = 239.5;
  bool warned_fallback_ = false;

  /// @}

  /// @name Runtime state
  /// @{
  ros::WallTime last_log_time_;
  int frames_since_log_ = 0;
  /// @}
};

}  // namespace

/**
 * @brief Entry point for `depth_localizer_node`.
 * @param argc ROS argument count.
 * @param argv ROS argument vector.
 * @return Process exit code.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_localizer_node");
  DepthLocalizerNode node;
  ros::spin();
  return 0;
}