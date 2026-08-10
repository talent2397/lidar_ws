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

class RslidarPoints2Map : public rclcpp::Node
{
public:
  RslidarPoints2Map()
  : Node("rslidar_points_2_map"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_in_    = declare_parameter<std::string>("cloud_in", "/rslidar_points_2");
    cloud_out_   = declare_parameter<std::string>("cloud_out", "/rslidar_points_2_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    source_frame_ = declare_parameter<std::string>("source_frame", "rslidar_2");
    time_bins_   = declare_parameter<int>("time_bins", 1);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_, qos,
      std::bind(&RslidarPoints2Map::cloud_cb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_, qos);
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

    // 帧内首/末点时间 (绝对主机秒), 用于逐点插值
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
    }

    // 按时间片预取 TF (odom -> rslidar_2)
    std::vector<Pose> tfs(time_bins_);
    // 最新 TF 兜底 (整个帧共用一个)
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
    for (int k = 0; k < time_bins_; ++k) {
      const double sec = have_ts ? (t_first + dur * (k + 0.5) / time_bins_) : 0.0;
      rclcpp::Time stamp = have_ts
        ? rclcpp::Time(msg->header.stamp) + rclcpp::Duration::from_seconds(
            sec - (t_first + dur * 0.5))  // 相对帧中心, 减小大数误差
        : rclcpp::Time(msg->header.stamp);
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame_, source_frame_, stamp, tf2::durationFromSec(0.01));
        const auto & q = tf.transform.rotation;
        const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        tfs[k].R = quat.toRotationMatrix();
        tfs[k].t << tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z;
      } catch (const tf2::TransformException & e) {
        // 该时间片暂时查不到(例如请求时刻略新于最新 TF), 用最新 TF 兜底
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
  rclcpp::spin(std::make_shared<RslidarPoints2Map>());
  rclcpp::shutdown();
  return 0;
}
