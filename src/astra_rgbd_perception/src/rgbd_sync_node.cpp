/**
 * @file rgbd_sync_node.cpp
 * @brief RGB-D synchronization, point cloud generation and sync statistics.
 * @details
 * The depth stream is expected to be registered to the color camera
 * (`astra_camera` with `depth_align:=true`), so depth and color pixels share
 * the same optical frame and intrinsics. The node pairs color/depth images
 * with `message_filters::ApproximateTime`, builds XYZ and XYZRGB PCL clouds
 * from CameraInfo intrinsics, and publishes synchronization statistics.
 *
 * The node does not use cv_bridge; it reads the raw image buffers directly to
 * avoid OpenCV 3.2/4.1 ABI issues on JetPack 4.6.
 *
 * @copyright MIT License, 2026.
 */

// 中文说明：RGB-D 同步与点云节点。
// 深度已对齐到彩色，因此同一像素坐标可直接取深度；用 CameraInfo 内参反投影。
// 不使用 cv_bridge，直接读原始图像缓冲，避免 JetPack 4.6 的 OpenCV ABI 问题。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "astra_rgbd_perception/RgbdSyncStats.h"

namespace {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    SyncPolicy;

/**
 * @class RgbdSyncNode
 * @brief Synchronizes color/depth images and publishes PCL point clouds.
 *
 * The node:
 * - caches the latest color/depth CameraInfo;
 * - pairs images with ApproximateTime;
 * - converts depth (16UC1 millimetres or 32FC1 metres) into metric XYZ;
 * - samples every `step`-th pixel and limits cloud publishing with
 *   `cloud_rate_hz`;
 * - publishes `RgbdSyncStats` once per second.
 *
 * If `require_aligned` is true, color and depth frame IDs must match.
 */
// 中文：RGB-D 同步节点。缓存 CameraInfo，ApproximateTime 配对，生成点云并统计。
class RgbdSyncNode {
 public:
  /**
   * @brief Read parameters, create ROS interfaces and log configuration.
   */
  // 中文：读取参数、创建订阅/发布、打印配置。
  RgbdSyncNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("color_topic", color_topic_, "/camera/color/image_raw");
    pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_raw");
    pnh_.param<std::string>("color_info_topic", color_info_topic_,
                            "/camera/color/camera_info");
    pnh_.param<std::string>("depth_info_topic", depth_info_topic_,
                            "/camera/depth/camera_info");
    pnh_.param<std::string>("points_xyz_topic", points_xyz_topic_,
                            "/perception/points_xyz");
    pnh_.param<std::string>("points_xyzrgb_topic", points_xyzrgb_topic_,
                            "/perception/points_xyzrgb");
    pnh_.param<std::string>("stats_topic", stats_topic_, "/perception/sync_stats");
    pnh_.param<std::string>("frame_id_override", frame_id_override_, "");

    pnh_.param<int>("step", step_, 2);
    pnh_.param<int>("queue_size", queue_size_, 5);
    pnh_.param<double>("approx_slop_s", approx_slop_s_, 0.05);
    pnh_.param<double>("min_range_m", min_range_m_, 0.3);
    pnh_.param<double>("max_range_m", max_range_m_, 8.0);
    pnh_.param<double>("depth_scale_to_m", depth_scale_to_m_, 0.001);
    pnh_.param<double>("cloud_rate_hz", cloud_rate_hz_, 10.0);
    pnh_.param<double>("gap_threshold_ms", gap_threshold_ms_, 75.0);
    pnh_.param<bool>("publish_xyz", publish_xyz_, true);
    pnh_.param<bool>("publish_xyzrgb", publish_xyzrgb_, true);
    pnh_.param<bool>("require_aligned", require_aligned_, true);

    pnh_.param<bool>("fallback_intrinsics", fallback_intrinsics_, false);
    pnh_.param<double>("fallback_fx", fallback_fx_, 570.3422);
    pnh_.param<double>("fallback_fy", fallback_fy_, 570.3422);
    pnh_.param<double>("fallback_cx", fallback_cx_, 319.5);
    pnh_.param<double>("fallback_cy", fallback_cy_, 239.5);

    if (step_ < 1) step_ = 1;
    if (cloud_rate_hz_ <= 0.0) cloud_rate_hz_ = 10.0;

    SyncPolicy policy(queue_size_);
    policy.setMaxIntervalDuration(ros::Duration(approx_slop_s_));
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(policy));
    color_sub_.subscribe(nh_, color_topic_, queue_size_);
    depth_sub_.subscribe(nh_, depth_topic_, queue_size_);
    sync_->connectInput(color_sub_, depth_sub_);
    sync_->registerCallback(boost::bind(&RgbdSyncNode::syncCallback, this, _1, _2));

    color_info_sub_ = nh_.subscribe(color_info_topic_, 2,
                                    &RgbdSyncNode::colorInfoCallback, this);
    depth_info_sub_ = nh_.subscribe(depth_info_topic_, 2,
                                    &RgbdSyncNode::depthInfoCallback, this);

    points_xyz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_xyz_topic_, 2);
    points_xyzrgb_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(points_xyzrgb_topic_, 2);
    stats_pub_ = nh_.advertise<astra_rgbd_perception::RgbdSyncStats>(stats_topic_, 5);

    last_cloud_time_ = ros::WallTime::now();
    last_stats_time_ = ros::WallTime::now();

    ROS_INFO_STREAM("rgbd_sync_node started");
    ROS_INFO_STREAM("  color: " << color_topic_ << "  depth: " << depth_topic_);
    ROS_INFO_STREAM("  xyz: " << points_xyz_topic_
                              << "  xyzrgb: " << points_xyzrgb_topic_);
    ROS_INFO_STREAM("  step=" << step_ << " range=[" << min_range_m_ << ", "
                              << max_range_m_ << "] m  cloud_rate="
                              << cloud_rate_hz_ << " Hz");
    if (fallback_intrinsics_) {
      ROS_WARN("fallback_intrinsics is enabled: point clouds use provisional "
               "intrinsics until the color camera is calibrated");
    }
  }

 private:
  /**
   * @brief Cache the latest color CameraInfo.
   * @param msg New CameraInfo message.
   */
  // 中文：缓存最新彩色 CameraInfo。
  void colorInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    color_info_ = msg;
  }

  /**
   * @brief Cache the latest depth CameraInfo.
   * @param msg New CameraInfo message.
   */
  // 中文：缓存最新深度 CameraInfo（对齐模式下等于彩色 K）。
  void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    depth_info_ = msg;
  }

  /**
   * @brief Update an exponential moving average of a stream rate.
   * @param stamp Current message timestamp.
   * @param last_stamp Previous timestamp state.
   * @param have_last Whether @p last_stamp contains a valid timestamp.
   * @param rate Smoothed rate in Hz.
   */
  // 中文：用指数滑动平均估计某路图像帧率。
  static void updateRate(const ros::Time& stamp, ros::Time* last_stamp,
                         bool* have_last, double* rate) {
    if (*have_last) {
      const double dt = (stamp - *last_stamp).toSec();
      if (dt > 1e-5 && dt < 1.0) {
        const double instant = 1.0 / dt;
        *rate = (*rate <= 0.0) ? instant : 0.9 * (*rate) + 0.1 * instant;
      }
    }
    *last_stamp = stamp;
    *have_last = true;
  }

  /**
   * @brief Resolve camera intrinsics from depth/color CameraInfo or fallback.
   * @param fx Output horizontal focal length.
   * @param fy Output vertical focal length.
   * @param cx Output principal point x.
   * @param cy Output principal point y.
   * @param using_fallback Set to true when fallback values are used.
   * @param source Human-readable source description for logging.
   * @return `true` if valid intrinsics are available.
   */
  // 中文：优先用深度/彩色 CameraInfo 的内参；无效且允许时才用 fallback。
  bool lookupIntrinsics(double* fx, double* fy, double* cx, double* cy,
                        bool* using_fallback, std::string* source) {
    if (depth_info_ && depth_info_->K.size() >= 9 && depth_info_->K[0] > 1.0 &&
        depth_info_->K[4] > 1.0) {
      *fx = depth_info_->K[0];
      *fy = depth_info_->K[4];
      *cx = depth_info_->K[2];
      *cy = depth_info_->K[5];
      *using_fallback = false;
      *source = "depth CameraInfo";
      return true;
    }
    if (color_info_ && color_info_->K.size() >= 9 && color_info_->K[0] > 1.0 &&
        color_info_->K[4] > 1.0) {
      *fx = color_info_->K[0];
      *fy = color_info_->K[4];
      *cx = color_info_->K[2];
      *cy = color_info_->K[5];
      *using_fallback = false;
      *source = "color CameraInfo";
      return true;
    }
    if (fallback_intrinsics_) {
      *fx = fallback_fx_;
      *fy = fallback_fy_;
      *cx = fallback_cx_;
      *cy = fallback_cy_;
      *using_fallback = true;
      *source = "fallback";
      if (!warned_fallback_) {
        ROS_WARN("CameraInfo fx/fy is invalid (uncalibrated?); using provisional "
                 "intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                 *fx, *fy, *cx, *cy);
        warned_fallback_ = true;
      }
      return true;
    }
    ROS_WARN_THROTTLE(5.0,
                      "CameraInfo fx/fy is invalid and fallback_intrinsics is "
                      "false; no point cloud will be published. Calibrate the "
                      "color camera or enable fallback_intrinsics.");
    return false;
  }

  /**
   * @brief Process one synchronized color/depth pair.
   * @param color Color image (`rgb8` or `bgr8`).
   * @param depth Depth image (`16UC1` in millimetres or `32FC1` in metres).
   *
   * Validates encodings and alignment, resolves intrinsics, builds clouds at
   * the configured rate, publishes PointCloud2 messages and emits statistics.
   */
  // 中文：处理一对同步的彩色/深度图像，生成点云并发布统计。
  void syncCallback(const sensor_msgs::Image::ConstPtr& color,
                    const sensor_msgs::Image::ConstPtr& depth) {
    updateRate(color->header.stamp, &last_color_stamp_, &have_last_color_,
               &color_rate_);
    updateRate(depth->header.stamp, &last_depth_stamp_, &have_last_depth_,
               &depth_rate_);

    if (have_last_pair_) {
      const double pair_dt = (color->header.stamp - last_pair_stamp_).toSec();
      if (pair_dt > gap_threshold_ms_ / 1000.0) {
        ++gap_count_;
      }
    }
    last_pair_stamp_ = color->header.stamp;
    have_last_pair_ = true;

    const bool color_is_rgb = color->encoding == "rgb8";
    if (!color_is_rgb && color->encoding != "bgr8") {
      ROS_WARN_THROTTLE(5.0, "Unsupported color encoding '%s'",
                        color->encoding.c_str());
      return;
    }
    const bool depth_is_u16 = depth->encoding == "16UC1";
    if (!depth_is_u16 && depth->encoding != "32FC1") {
      ROS_WARN_THROTTLE(5.0, "Unsupported depth encoding '%s'",
                        depth->encoding.c_str());
      return;
    }
    if (color->width != depth->width || color->height != depth->height) {
      ROS_WARN_THROTTLE(5.0,
                        "Color/depth size mismatch: %ux%u vs %ux%u",
                        color->width, color->height, depth->width, depth->height);
      return;
    }
    if (require_aligned_ && color->header.frame_id != depth->header.frame_id) {
      ROS_WARN_THROTTLE(
          5.0,
          "Depth frame '%s' differs from color frame '%s'; enable "
          "depth_align:=true or set require_aligned:=false.",
          depth->header.frame_id.c_str(), color->header.frame_id.c_str());
      return;
    }

    double fx, fy, cx, cy;
    bool using_fallback = false;
    std::string intrinsics_source;
    if (!lookupIntrinsics(&fx, &fy, &cx, &cy, &using_fallback,
                          &intrinsics_source)) {
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    const bool make_cloud =
        (now - last_cloud_time_).toSec() >= 1.0 / cloud_rate_hz_;
    if (!make_cloud) {
      return;
    }

    const int width = static_cast<int>(color->width);
    const int height = static_cast<int>(color->height);
    // 中文：16UC1 原始值是毫米，乘 0.001 转米；32FC1 已是米。
    const double depth_scale =
        depth_is_u16 ? depth_scale_to_m_ : 1.0;  // 32FC1 is already in meters
    const std::string cloud_frame = frame_id_override_.empty()
                                        ? depth->header.frame_id
                                        : frame_id_override_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>());
    cloud_rgb->header.frame_id = cloud_frame;
    cloud_xyz->header.frame_id = cloud_frame;
    const size_t reserve =
        static_cast<size_t>((width / step_) * (height / step_));
    cloud_rgb->points.reserve(reserve);
    if (publish_xyz_) {
      cloud_xyz->points.reserve(reserve);
    }

    size_t sampled = 0;
    size_t valid = 0;
    const ros::WallTime process_start = ros::WallTime::now();

    // 中文：按 step 采样像素，过滤无效/超量程深度，反投影成 XYZ/XYZRGB。
    for (int v = 0; v < height; v += step_) {
      const uint8_t* depth_row = depth->data.data() + v * depth->step;
      const uint8_t* color_row = color->data.data() + v * color->step;
      for (int u = 0; u < width; u += step_) {
        double z = 0.0;
        if (depth_is_u16) {
          uint16_t raw = 0;
          std::memcpy(&raw, depth_row + u * sizeof(uint16_t), sizeof(raw));
          if (raw == 0) {
            ++sampled;
            continue;
          }
          z = static_cast<double>(raw) * depth_scale;
        } else {
          float raw = 0.0f;
          std::memcpy(&raw, depth_row + u * sizeof(float), sizeof(raw));
          if (!std::isfinite(raw) || raw <= 0.0f) {
            ++sampled;
            continue;
          }
          z = static_cast<double>(raw);
        }
        ++sampled;
        if (z < min_range_m_ || z > max_range_m_) {
          continue;
        }
        ++valid;

        // 中文：针孔模型反投影 X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy。
        const double x = (u - cx) * z / fx;
        const double y = (v - cy) * z / fy;
        const uint8_t* px = color_row + u * 3;
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        if (color_is_rgb) {
          point.r = px[0];
          point.g = px[1];
          point.b = px[2];
        } else {
          point.b = px[0];
          point.g = px[1];
          point.r = px[2];
        }
        cloud_rgb->points.push_back(point);

        if (publish_xyz_) {
          pcl::PointXYZ point_xyz;
          point_xyz.x = point.x;
          point_xyz.y = point.y;
          point_xyz.z = point.z;
          cloud_xyz->points.push_back(point_xyz);
        }
      }
    }

    const double process_ms = (ros::WallTime::now() - process_start).toSec() * 1000.0;
    cloud_rgb->width = static_cast<uint32_t>(cloud_rgb->points.size());
    cloud_rgb->height = 1;
    cloud_rgb->is_dense = false;
    cloud_xyz->width = static_cast<uint32_t>(cloud_xyz->points.size());
    cloud_xyz->height = 1;
    cloud_xyz->is_dense = false;

    if (publish_xyzrgb_ && !cloud_rgb->points.empty()) {
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cloud_rgb, output);
      output.header.stamp = color->header.stamp;
      output.header.frame_id = cloud_frame;
      points_xyzrgb_pub_.publish(output);
    }
    if (publish_xyz_ && !cloud_xyz->points.empty()) {
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cloud_xyz, output);
      output.header.stamp = color->header.stamp;
      output.header.frame_id = cloud_frame;
      points_xyz_pub_.publish(output);
    }

    last_valid_ratio_ =
        sampled > 0 ? static_cast<double>(valid) / static_cast<double>(sampled)
                    : 0.0;
    last_point_count_ = static_cast<uint32_t>(cloud_rgb->points.size());
    last_process_ms_ = process_ms;
    last_cloud_time_ = now;

    // 中文：每秒发布一次同步/点云统计。
    if ((now - last_stats_time_).toSec() >= 1.0) {
      astra_rgbd_perception::RgbdSyncStats stats;
      stats.header.stamp = color->header.stamp;
      stats.header.frame_id = cloud_frame;
      stats.time_diff_ms =
          (color->header.stamp - depth->header.stamp).toSec() * 1000.0;
      stats.color_rate_hz = color_rate_;
      stats.depth_rate_hz = depth_rate_;
      stats.valid_depth_ratio = last_valid_ratio_;
      stats.point_count = last_point_count_;
      stats.process_ms = last_process_ms_;
      stats.gap_count = gap_count_;
      stats.using_fallback_intrinsics = using_fallback;
      stats_pub_.publish(stats);
      ROS_INFO(
          "sync dt=%.1fms color=%.1fHz depth=%.1fHz valid=%.1f%% points=%u "
          "cloud=%.1fms gaps=%lu intrinsics=%s",
          stats.time_diff_ms, stats.color_rate_hz, stats.depth_rate_hz,
          stats.valid_depth_ratio * 100.0, stats.point_count, stats.process_ms,
          static_cast<unsigned long>(stats.gap_count),
          intrinsics_source.c_str());
      last_stats_time_ = now;
    }
  }

  /// @name ROS interfaces
  /// @{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber color_info_sub_;
  ros::Subscriber depth_info_sub_;

  ros::Publisher points_xyz_pub_;
  ros::Publisher points_xyzrgb_pub_;
  ros::Publisher stats_pub_;

  /// @}

  /// @name Latest CameraInfo
  /// @{
  sensor_msgs::CameraInfo::ConstPtr color_info_;
  sensor_msgs::CameraInfo::ConstPtr depth_info_;

  /// @}

  /// @name Topic and frame parameters
  /// @{
  std::string color_topic_;
  std::string depth_topic_;
  std::string color_info_topic_;
  std::string depth_info_topic_;
  std::string points_xyz_topic_;
  std::string points_xyzrgb_topic_;
  std::string stats_topic_;
  std::string frame_id_override_;

  /// @}

  /// @name Algorithm and QoS parameters
  /// @{
  int step_ = 2;
  int queue_size_ = 5;
  double approx_slop_s_ = 0.05;
  double min_range_m_ = 0.3;
  double max_range_m_ = 8.0;
  double depth_scale_to_m_ = 0.001;
  double cloud_rate_hz_ = 10.0;
  double gap_threshold_ms_ = 75.0;
  bool publish_xyz_ = true;
  bool publish_xyzrgb_ = true;
  bool require_aligned_ = true;

  bool fallback_intrinsics_ = false;
  double fallback_fx_ = 570.3422;
  double fallback_fy_ = 570.3422;
  double fallback_cx_ = 319.5;
  double fallback_cy_ = 239.5;
  bool warned_fallback_ = false;

  /// @}

  /// @name Runtime state
  /// @{
  ros::Time last_color_stamp_;
  ros::Time last_depth_stamp_;
  ros::Time last_pair_stamp_;
  bool have_last_color_ = false;
  bool have_last_depth_ = false;
  bool have_last_pair_ = false;
  double color_rate_ = 0.0;
  double depth_rate_ = 0.0;
  uint64_t gap_count_ = 0;

  ros::WallTime last_cloud_time_;
  ros::WallTime last_stats_time_;
  double last_valid_ratio_ = 0.0;
  uint32_t last_point_count_ = 0;
  double last_process_ms_ = 0.0;
  /// @}
};

}  // namespace

/**
 * @brief Entry point for `rgbd_sync_node`.
 * @param argc ROS argument count.
 * @param argv ROS argument vector.
 * @return Process exit code (0 after normal shutdown).
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "rgbd_sync_node");
  RgbdSyncNode node;
  ros::spin();
  return 0;
}