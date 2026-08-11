// rslidar_points_2_processor_node.cpp
// lidar2 逐点运动补偿: 把原始 /rslidar_points_2 (rslidar_2 系, 帧内 ~100ms
// 存在运动畸变) 按每个点的绝对时间戳插值 odom->rslidar_2 位姿, 变换到
// odom 系输出 /rslidar_points_2_processed, 与 lidar1 的 FAST-LIO 输出
// (/cloud_registered, odom 系) 对齐, 供双雷达融合使用。
//
// 说明: 帧内运动由 LIO 的 odom->base_link TF (200Hz, 含平移+旋转) 描述,
// tf2 在两次 TF 之间线性插值, 因此逐点补偿同时消除旋转与平移畸变;
// 为控制 CPU, 每帧按 time_bins 个时间片预取 TF, 每片内点共用该片位姿。

#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

struct Pose {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

inline float load_f(const uint8_t * p) {
  float v;
  std::memcpy(&v, p, 4);
  return v;
}

inline void store_f(float v, uint8_t * p) {
  std::memcpy(p, &v, 4);
}

inline double load_d(const uint8_t * p) {
  double v;
  std::memcpy(&v, p, 8);
  return v;
}

}  // namespace

class RslidarPoints2Processor : public rclcpp::Node
{
public:
  RslidarPoints2Processor()
  : Node("rslidar_points_2_processor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_in_     = declare_parameter<std::string>("cloud_in", "/rslidar_points_2");
    cloud_out_    = declare_parameter<std::string>("cloud_out", "/rslidar_points_2_processed");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    source_frame_ = declare_parameter<std::string>("source_frame", "rslidar_2");
    time_bins_    = declare_parameter<int>("time_bins", 32);
    time_bins_    = std::max(1, std::min(time_bins_, 512));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_, qos,
      std::bind(&RslidarPoints2Processor::cloud_cb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_, qos);

    RCLCPP_INFO(get_logger(),
      "lidar2 processor: %s -> %s, %s -> %s, time_bins=%d",
      cloud_in_.c_str(), cloud_out_.c_str(),
      source_frame_.c_str(), target_frame_.c_str(), time_bins_);
  }

private:
  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty() || msg->point_step == 0) return;

    int xo = -1, yo = -1, zo = -1, to = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") xo = static_cast<int>(f.offset);
      else if (f.name == "y") yo = static_cast<int>(f.offset);
      else if (f.name == "z") zo = static_cast<int>(f.offset);
      else if (f.name == "timestamp") to = static_cast<int>(f.offset);
    }
    if (xo < 0 || yo < 0 || zo < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "input cloud missing x/y/z, skip");
      return;
    }

    const size_t ps = msg->point_step;
    const size_t n  = msg->width * msg->height;
    const uint8_t * src = msg->data.data();

    // 帧内首/末点绝对时间 (主机秒), 用于逐点时间片分配
    double t_first = 0.0, t_last = 0.0;
    bool have_ts = to >= 0;
    if (have_ts) {
      bool first = true;
      for (size_t i = 0; i < n; ++i) {
        const double ts = load_d(src + i * ps + to);
        if (!std::isfinite(ts)) continue;
        if (first) { t_first = ts; t_last = ts; first = false; }
        else {
          if (ts < t_first) t_first = ts;
          if (ts > t_last)  t_last = ts;
        }
      }
    }
    double dur = t_last - t_first;
    if (!have_ts || !(dur > 0.0) || dur > 1.0) {
      have_ts = false;
      dur = 0.1;
      t_first = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).seconds();
      t_last  = t_first + dur;
    }

    // 最新 TF 兜底 (帧整体可用)
    Pose latest_pose;
    bool latest_ok = false;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        target_frame_, source_frame_, rclcpp::Time(), tf2::durationFromSec(0.05));
      const auto & q = tf.transform.rotation;
      Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
      latest_pose.R = quat.toRotationMatrix();
      latest_pose.t << tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z;
      latest_ok = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "no TF %s -> %s: %s", target_frame_.c_str(), source_frame_.c_str(), e.what());
      return;
    }

    // 按时间片预取 TF (每片中心时刻, 相对帧起点计算, 避免大数误差)
    std::vector<Pose> tfs(time_bins_);
    const rclcpp::Time t0(msg->header.stamp, RCL_ROS_TIME);
    for (int k = 0; k < time_bins_; ++k) {
      const double sec = t_first + dur * (k + 0.5) / time_bins_;
      const rclcpp::Time stamp =
        t0 + rclcpp::Duration::from_seconds(sec - t_first);
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame_, source_frame_, stamp, tf2::durationFromSec(0.02));
        const auto & q = tf.transform.rotation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        tfs[k].R = quat.toRotationMatrix();
        tfs[k].t << tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z;
      } catch (const tf2::TransformException & e) {
        // 时间片略新于最新 TF 等场景: 用最新 TF 兜底
        tfs[k] = latest_pose;
      }
    }

    auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    out->header.frame_id = target_frame_;
    uint8_t * dst = out->data.data();

    for (size_t i = 0; i < n; ++i) {
      uint8_t * p = dst + i * ps;
      const float x = load_f(src + i * ps + xo);
      const float y = load_f(src + i * ps + yo);
      const float z = load_f(src + i * ps + zo);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      int bin = 0;
      if (have_ts) {
        const double ts = load_d(src + i * ps + to);
        double rel = (ts - t_first) / dur;
        if (rel < 0.0) rel = 0.0;
        if (rel > 1.0) rel = 1.0;
        bin = static_cast<int>(rel * time_bins_);
        if (bin >= time_bins_) bin = time_bins_ - 1;
      }
      const Eigen::Vector3d v = tfs[bin].R * Eigen::Vector3d(x, y, z) + tfs[bin].t;
      store_f(static_cast<float>(v.x()), p + xo);
      store_f(static_cast<float>(v.y()), p + yo);
      store_f(static_cast<float>(v.z()), p + zo);
    }

    pub_->publish(*out);
  }

  std::string cloud_in_, cloud_out_, target_frame_, source_frame_;
  int time_bins_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RslidarPoints2Processor>());
  rclcpp::shutdown();
  return 0;
}
