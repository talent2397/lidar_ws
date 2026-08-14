#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

struct FieldOffsets {
  int x = -1, y = -1, z = -1, intensity = -1, ring = -1, timestamp = -1;
  bool valid() const {
    return x >= 0 && y >= 0 && z >= 0 && intensity >= 0 && ring >= 0 && timestamp >= 0;
  }
};

FieldOffsets find_offsets(const sensor_msgs::msg::PointCloud2 & msg) {
  FieldOffsets off;
  for (const auto & f : msg.fields) {
    if (f.name == "x") off.x = static_cast<int>(f.offset);
    else if (f.name == "y") off.y = static_cast<int>(f.offset);
    else if (f.name == "z") off.z = static_cast<int>(f.offset);
    else if (f.name == "intensity") off.intensity = static_cast<int>(f.offset);
    else if (f.name == "ring") off.ring = static_cast<int>(f.offset);
    else if (f.name == "timestamp") off.timestamp = static_cast<int>(f.offset);
  }
  return off;
}

template <typename T>
inline T load_at(const uint8_t * base, int offset) {
  T v;
  std::memcpy(&v, base + offset, sizeof(T));
  return v;
}

inline void store(float v, uint8_t * base, int offset) {
  std::memcpy(base + offset, &v, 4);
}

inline void store16(uint16_t v, uint8_t * base, int offset) {
  std::memcpy(base + offset, &v, 2);
}

}  // namespace

class RslidarLioAdapter : public rclcpp::Node
{
public:
  RslidarLioAdapter()
  : Node("rslidar_lio_adapter")
  {
    cloud_in_   = declare_parameter<std::string>("cloud_in", "/rslidar_points_1");
    imu_in_     = declare_parameter<std::string>("imu_in", "/rslidar_imu_data_1");
    cloud_out_  = declare_parameter<std::string>("cloud_out", "/fastlio/lidar_points");
    imu_out_    = declare_parameter<std::string>("imu_out", "/fastlio/imu");
    keep_frame_ = declare_parameter<std::string>("frame_id", "");

    // IMU 必须用 reliable 订阅: 录制端是 reliable 发布, best_effort 在负载下
    // 会乱序, 导致 spark-fast-lio "IMU loopback" 清缓冲 (2026-08-14 定位)
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto imu_qos   = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_, cloud_qos,
      std::bind(&RslidarLioAdapter::cloud_cb, this, std::placeholders::_1));
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in_, imu_qos,
      std::bind(&RslidarLioAdapter::imu_cb, this, std::placeholders::_1));

    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_, cloud_qos);
    pub_imu_   = create_publisher<sensor_msgs::msg::Imu>(imu_out_, imu_qos);
  }

private:
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out = std::make_shared<sensor_msgs::msg::Imu>(*msg);
    // 防御: 保证输出 IMU 时间戳严格递增 (驱动已改单调时钟, 这里双保险)
    rclcpp::Time stamp(out->header.stamp, RCL_ROS_TIME);
    if (stamp <= last_imu_stamp_) {
      // +1µs: 驱动时间戳为微秒分辨率, +1ns 在 double 秒精度下不可见
      stamp = last_imu_stamp_ + rclcpp::Duration::from_nanoseconds(1000);
      const int64_t ns = stamp.nanoseconds();
      out->header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      out->header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    }
    last_imu_stamp_ = stamp;
    if (!keep_frame_.empty()) out->header.frame_id = keep_frame_;
    pub_imu_->publish(*out);
  }

  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty() || msg->point_step == 0) return;

    const auto off = find_offsets(*msg);
    if (!off.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Input cloud is not XYZIRT (need x/y/z/intensity/ring/timestamp), skip");
      return;
    }

    const size_t ps = msg->point_step;
    const size_t n  = msg->data.size() / ps;
    const uint8_t * src = msg->data.data();

    // 1) first valid timestamp (host-clock seconds, absolute)
    double t_first = 0.0;
    bool have_first = false;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = src + i * ps;
      const float x = load_at<float>(p, off.x);
      const float y = load_at<float>(p, off.y);
      const float z = load_at<float>(p, off.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      t_first = load_at<double>(p, off.timestamp);
      have_first = true;
      break;
    }
    if (!have_first) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No valid points in cloud, skip");
      return;
    }

    // 2) build FAST-LIO velodyne-compatible cloud: x/y/z/intensity/time(float32, rel sec)/ring
    auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out->header       = msg->header;
    out->height       = 1;
    out->is_dense     = true;
    out->point_step   = 22;
    out->fields.clear();

    sensor_msgs::msg::PointField f;
    f.name = "x";         f.offset = 0;  f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    out->fields.push_back(f);
    f.name = "y";         f.offset = 4;  out->fields.push_back(f);
    f.name = "z";         f.offset = 8;  out->fields.push_back(f);
    f.name = "intensity"; f.offset = 12; out->fields.push_back(f);
    f.name = "time";      f.offset = 16; out->fields.push_back(f);
    f.name = "ring";      f.offset = 20; f.datatype = sensor_msgs::msg::PointField::UINT16;
    out->fields.push_back(f);

    out->data.reserve(n * out->point_step);
    size_t valid = 0;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = src + i * ps;
      const float x = load_at<float>(p, off.x);
      const float y = load_at<float>(p, off.y);
      const float z = load_at<float>(p, off.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      const float intensity = load_at<float>(p, off.intensity);
      const uint16_t ring  = load_at<uint16_t>(p, off.ring);
      const double ts      = load_at<double>(p, off.timestamp);
      float t_rel = static_cast<float>(ts - t_first);
      if (!std::isfinite(t_rel) || t_rel < 0.0f) t_rel = 0.0f;
      if (t_rel > 1.0f) t_rel = 1.0f;  // guard against outlier timestamps

      const size_t q = valid * out->point_step;
      out->data.resize(q + out->point_step);
      uint8_t * o = out->data.data() + q;
      store(x, o, 0);
      store(y, o, 4);
      store(z, o, 8);
      store(intensity, o, 12);
      store(t_rel, o, 16);
      store16(ring, o, 20);
      ++valid;
    }

    out->width    = static_cast<uint32_t>(valid);
    out->row_step = out->width * out->point_step;

    static size_t cb_count = 0;
    if (++cb_count % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "converted %zu -> %zu pts, t_rel last=%.4fs (frame %s)",
        n, valid,
        out->data.size() >= out->point_step ? load_at<float>(out->data.data() + (valid - 1) * 22, 16) : 0.0f,
        out->header.frame_id.c_str());
    }

    pub_cloud_->publish(*out);
  }

  std::string cloud_in_, imu_in_, cloud_out_, imu_out_, keep_frame_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RslidarLioAdapter>());
  rclcpp::shutdown();
  return 0;
}
