// dual_lidar_fusion_node.cpp
// 双雷达融合: 把 lidar1 的 LIO 稠密去畸变点云 (/cloud_registered_base,
// base 系, dense_publish_en=false 时 /cloud_registered 只是稀疏特征点)
// 用 odom->base_link TF 转到 odom 系, 再与 lidar2 的逐点补偿点云
// (/rslidar_points_2_processed, odom 系) 合并为单一点云:
//   /merged_points       [odom]  完整融合点云 (XYZI)
//   /merged_points_bev   [odom]  z 压平为 0, 即 BEV 鸟瞰视角点云
//
// 融合策略 (v3, 2026-08-11):
// - 任一侧来新帧, 就用"最新 lidar1 + 最新 lidar2"合成一帧发布;
//   两侧都在时, 每帧必然同时包含两个雷达的数据 (旧侧最多差一个扫描
//   周期 ~0.1s, 两路点云都是 odom 系, 直接叠加即可, 不会出现
//   "只有 lidar2 / 只有 lidar1" 的帧, 也不会重复发布同一帧);
// - 某一侧还没出过数据时, 才降级输出另一侧单帧。

#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

struct PointXYZI {
  float x = 0.f, y = 0.f, z = 0.f, intensity = 0.f;
};

inline float load_f(const uint8_t * p) {
  float v;
  std::memcpy(&v, p, 4);
  return v;
}

inline void store_f(float v, uint8_t * p) {
  std::memcpy(p, &v, 4);
}

inline Eigen::Matrix4d tf_to_matrix(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & q = tf.transform.rotation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.block<3, 3>(0, 0) = quat.toRotationMatrix();
  M.block<3, 1>(0, 3) << tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z;
  return M;
}

}  // namespace

class DualLidarFusion : public rclcpp::Node
{
public:
  DualLidarFusion()
  : Node("dual_lidar_fusion"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud1_in_  = declare_parameter<std::string>("cloud1_in", "/cloud_registered_base");
    cloud2_in_  = declare_parameter<std::string>("cloud2_in", "/rslidar_points_2_processed");
    merged_out_ = declare_parameter<std::string>("merged_out", "/merged_points");
    bev_out_    = declare_parameter<std::string>("bev_out", "/merged_points_bev");
    frame_id_   = declare_parameter<std::string>("frame_id", "odom");

    // 大点云 + 回调里做 TF/变换, 深度太小容易在负载高时丢消息
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    sub1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud1_in_, qos,
      std::bind(&DualLidarFusion::cloud1_cb, this, std::placeholders::_1));
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud2_in_, qos,
      std::bind(&DualLidarFusion::cloud2_cb, this, std::placeholders::_1));
    pub_     = create_publisher<sensor_msgs::msg::PointCloud2>(merged_out_, qos);
    pub_bev_ = create_publisher<sensor_msgs::msg::PointCloud2>(bev_out_, qos);

    RCLCPP_INFO(get_logger(),
      "fusion v3: %s(base->odom via TF) + %s -> %s [%s], BEV: %s (z=0)",
      cloud1_in_.c_str(), cloud2_in_.c_str(), merged_out_.c_str(),
      frame_id_.c_str(), bev_out_.c_str());
  }

private:
  void cloud1_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    c1_ = msg;
    t1_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    // 乱序/旧帧到达时不重复发布 (内容会被下一次更新带出), 保持时间戳递增
    if (!c2_ || t1_ > t2_) publish_latest();
  }

  void cloud2_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    c2_ = msg;
    t2_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    if (!c1_ || t2_ > t1_) publish_latest();
  }

  void publish_latest()
  {
    std::vector<PointXYZI> pts;
    rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    bool have1 = false, have2 = false;

    if (c1_) {
      if (transform_base_to_odom(*c1_, t1_, pts)) {
        have1 = true;
        stamp = t1_;
      }
    }
    if (c2_) {
      std::vector<PointXYZI> p2 = extract(*c2_);
      if (!p2.empty()) {
        pts.insert(pts.end(), p2.begin(), p2.end());
        have2 = true;
        if (!have1 || t2_ > stamp) stamp = t2_;
      }
    }
    if (pts.empty()) return;
    publish(stamp, pts);
  }

  static std::vector<PointXYZI> extract(const sensor_msgs::msg::PointCloud2 & msg)
  {
    std::vector<PointXYZI> out;
    if (msg.data.empty() || msg.point_step == 0) return out;

    int xo = -1, yo = -1, zo = -1, io = -1;
    for (const auto & f : msg.fields) {
      if (f.name == "x") xo = static_cast<int>(f.offset);
      else if (f.name == "y") yo = static_cast<int>(f.offset);
      else if (f.name == "z") zo = static_cast<int>(f.offset);
      else if (f.name == "intensity") io = static_cast<int>(f.offset);
    }
    if (xo < 0 || yo < 0 || zo < 0) return out;

    const size_t ps = msg.point_step;
    const size_t n  = msg.width * msg.height;
    const uint8_t * src = msg.data.data();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = src + i * ps;
      const float x = load_f(p + xo);
      const float y = load_f(p + yo);
      const float z = load_f(p + zo);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      PointXYZI pt;
      pt.x = x; pt.y = y; pt.z = z;
      if (io >= 0) pt.intensity = load_f(p + io);
      out.push_back(pt);
    }
    return out;
  }

  static sensor_msgs::msg::PointCloud2::SharedPtr make_cloud(
    const rclcpp::Time & stamp, const std::string & frame,
    const std::vector<PointXYZI> & pts, bool bev)
  {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame;
    msg->height = 1;
    msg->width = pts.size();
    msg->is_bigendian = false;
    msg->point_step = 16;
    msg->row_step = 16 * pts.size();
    msg->is_dense = true;

    sensor_msgs::msg::PointField f;
    f.name = "x";         f.offset = 0;  f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg->fields.push_back(f);
    f.name = "y";         f.offset = 4;  msg->fields.push_back(f);
    f.name = "z";         f.offset = 8;  msg->fields.push_back(f);
    f.name = "intensity"; f.offset = 12; msg->fields.push_back(f);

    msg->data.resize(pts.size() * 16);
    uint8_t * dst = msg->data.data();
    for (size_t i = 0; i < pts.size(); ++i) {
      uint8_t * p = dst + i * 16;
      store_f(pts[i].x, p + 0);
      store_f(pts[i].y, p + 4);
      store_f(bev ? 0.f : pts[i].z, p + 8);
      store_f(pts[i].intensity, p + 12);
    }
    return msg;
  }

  // 把 base 系点云按扫描时刻的 odom->base_link TF 变换到 odom 系
  bool transform_base_to_odom(
    const sensor_msgs::msg::PointCloud2 & msg, const rclcpp::Time & stamp,
    std::vector<PointXYZI> & out)
  {
    out = extract(msg);
    if (out.empty()) return true;
    geometry_msgs::msg::TransformStamped tf;
    // 非阻塞: 用 canTransform 预检, 避免阻塞 executor 造成丢帧
    if (tf_buffer_.canTransform(frame_id_, "base_link", stamp, tf2::durationFromSec(0.0))) {
      tf = tf_buffer_.lookupTransform(frame_id_, "base_link", stamp);
    } else if (tf_buffer_.canTransform(
                 frame_id_, "base_link", rclcpp::Time(), tf2::durationFromSec(0.0))) {
      tf = tf_buffer_.lookupTransform(frame_id_, "base_link", rclcpp::Time());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "no TF %s -> base_link at %.3f", frame_id_.c_str(), stamp.seconds());
      return false;
    }
    const Eigen::Matrix4d M = tf_to_matrix(tf);
    for (auto & p : out) {
      const Eigen::Vector4d v = M * Eigen::Vector4d(p.x, p.y, p.z, 1.0);
      p.x = static_cast<float>(v.x());
      p.y = static_cast<float>(v.y());
      p.z = static_cast<float>(v.z());
    }
    return true;
  }

  void publish(const rclcpp::Time & stamp, const std::vector<PointXYZI> & pts)
  {
    if (pts.empty()) return;
    pub_->publish(*make_cloud(stamp, frame_id_, pts, false));
    pub_bev_->publish(*make_cloud(stamp, frame_id_, pts, true));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "publish merged: %zu pts, stamp %.3f", pts.size(), stamp.seconds());
  }

  std::string cloud1_in_, cloud2_in_, merged_out_, bev_out_, frame_id_;
  sensor_msgs::msg::PointCloud2::SharedPtr c1_, c2_;
  rclcpp::Time t1_, t2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_, sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_, pub_bev_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualLidarFusion>());
  rclcpp::shutdown();
  return 0;
}
