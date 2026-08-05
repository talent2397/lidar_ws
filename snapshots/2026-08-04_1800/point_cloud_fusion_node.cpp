#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <string>

using namespace std::chrono_literals;

/**
 * 双雷达时间同步融合 (C++ 版, 替代 Python 版)
 * ==========================================
 * Python rclpy 给 PointCloud2.data 赋值约 163ns/字节, 160k 点需 ~400ms,
 * 无法实时。C++ 版单帧处理 <5ms。
 *
 * - 同步检查用点云到达时刻 (同一系统时钟), 不用雷达 header 时钟;
 * - 直接订阅 /tf 维护 2s 环形缓冲, 按“到达时刻 - 50ms”取最近动态变换;
 * - 动态 TF 是 base_link→rslidar, 组合静态 world→base_link(z=0.395) 后变换;
 * - 发布成功后清空缓存帧, 避免重复。
 */
class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : Node("point_cloud_fusion")
  {
    declare_parameter("tf_lookup_offset", 0.05);
    declare_parameter("sync_window", 0.08);
    tf_offset_ = get_parameter("tf_lookup_offset").as_double();
    sync_window_ = get_parameter("sync_window").as_double();

    sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(200),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tf_cb(msg); });
    sub1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points_1", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cb1(msg); });
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points_2", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cb2(msg); });
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged_points", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(20ms, [this]() { fuse(); });
    // 必须与 this->now() 使用同一时间源 (RCL_SYSTEM_TIME),
    // 默认构造的 rclcpp::Time 是 RCL_ROS_TIME, 相减会抛异常
    last_fused1_ = this->now();
    last_fused2_ = this->now();

    RCLCPP_INFO(get_logger(),
                "Fusion C++ (arrival-time sync + TF ring buffer) -> /merged_points [world]");
  }

private:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using TFStamped = geometry_msgs::msg::TransformStamped;

  struct TFEntry
  {
    rclcpp::Time t;
    TFStamped tf;
  };

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<Cloud>::SharedPtr sub1_, sub2_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<TFEntry> tf_hist_[2];
  Cloud::SharedPtr c1_, c2_;
  rclcpp::Time a1_, a2_;
  bool have1_ = false, have2_ = false;
  rclcpp::Time last_fused1_, last_fused2_;
  double tf_offset_ = 0.05;
  double sync_window_ = 0.08;

  static int lid_index(const std::string & child)
  {
    return child == "rslidar_1" ? 0 : 1;
  }

  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    try {
      const rclcpp::Time now = this->now();
      for (const auto & tf : msg->transforms) {
        if (tf.child_frame_id == "rslidar_1" || tf.child_frame_id == "rslidar_2") {
          tf_hist_[lid_index(tf.child_frame_id)].push_back({now, tf});
        }
      }
      const rclcpp::Time cutoff = now - rclcpp::Duration::from_seconds(2.0);
      for (auto & h : tf_hist_) {
        while (!h.empty() && h.front().t < cutoff) {
          h.pop_front();
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "tf_cb exception: %s", e.what());
    }
  }

  const TFStamped * lookup(int i, const rclcpp::Time & t)
  {
    auto & h = tf_hist_[i];
    if (h.empty()) {
      return nullptr;
    }
    auto best = h.begin();
    double best_d = std::abs((best->t - t).seconds());
    for (auto it = std::next(h.begin()); it != h.end(); ++it) {
      const double d = std::abs((it->t - t).seconds());
      if (d < best_d) {
        best = it;
        best_d = d;
      }
    }
    if (best_d > 0.5) {
      return nullptr;
    }
    return &best->tf;
  }

  void transform_cloud(const Cloud & in, const TFStamped & tfs, Cloud & out)
  {
    out = in;
    out.header.frame_id = "world";

    int xoff = -1, yoff = -1, zoff = -1;
    for (const auto & f : in.fields) {
      if (f.name == "x") { xoff = static_cast<int>(f.offset); }
      else if (f.name == "y") { yoff = static_cast<int>(f.offset); }
      else if (f.name == "z") { zoff = static_cast<int>(f.offset); }
    }
    if (xoff < 0 || yoff < 0 || zoff < 0 || in.point_step == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "point cloud missing x/y/z fields, skip");
      out.data.clear();
      return;
    }

    const tf2::Quaternion q(
      tfs.transform.rotation.x, tfs.transform.rotation.y,
      tfs.transform.rotation.z, tfs.transform.rotation.w);
    const tf2::Matrix3x3 rot(q);
    const double tx = tfs.transform.translation.x;
    const double ty = tfs.transform.translation.y;
    const double tz = tfs.transform.translation.z + 0.395;  // world->base_link

    const size_t ps = in.point_step;
    const size_t n = std::min(
      static_cast<size_t>(in.height) * in.width,
      in.data.size() / ps);
    out.data.resize(in.data.size());
    if (out.data.size() >= in.data.size()) {
      std::memcpy(out.data.data(), in.data.data(), in.data.size());
    }

    for (size_t i = 0; i < n; ++i) {
      float ix, iy, iz;
      std::memcpy(&ix, in.data.data() + i * ps + xoff, 4);
      std::memcpy(&iy, in.data.data() + i * ps + yoff, 4);
      std::memcpy(&iz, in.data.data() + i * ps + zoff, 4);
      const tf2::Vector3 v = rot * tf2::Vector3(ix, iy, iz) +
                             tf2::Vector3(tx, ty, tz);
      float ox = static_cast<float>(v.x());
      float oy = static_cast<float>(v.y());
      float oz = static_cast<float>(v.z());
      std::memcpy(out.data.data() + i * ps + xoff, &ox, 4);
      std::memcpy(out.data.data() + i * ps + yoff, &oy, 4);
      std::memcpy(out.data.data() + i * ps + zoff, &oz, 4);
    }
  }

  void cb1(const Cloud::SharedPtr msg)
  {
    try {
      c1_ = msg;
      a1_ = this->now();
      have1_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "cb1 exception: %s", e.what());
    }
  }

  void cb2(const Cloud::SharedPtr msg)
  {
    try {
      c2_ = msg;
      a2_ = this->now();
      have2_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "cb2 exception: %s", e.what());
    }
  }

public:
  void selftest()
  {
    auto make_cloud = [](const std::string & lid, float z0) {
      auto c = std::make_shared<Cloud>();
      c->header.frame_id = lid;
      c->height = 1;
      c->width = 2;
      c->point_step = 16;
      c->row_step = 32;
      c->is_bigendian = false;
      c->is_dense = true;
      sensor_msgs::msg::PointField f;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      f.name = "x"; f.offset = 0; c->fields.push_back(f);
      f.name = "y"; f.offset = 4; c->fields.push_back(f);
      f.name = "z"; f.offset = 8; c->fields.push_back(f);
      f.name = "intensity"; f.offset = 12; c->fields.push_back(f);
      const float pts[8] = {0.f, 0.f, z0, 1.f, 1.f, 1.f, z0 + 0.1f, 1.f};
      const auto * b = reinterpret_cast<const uint8_t *>(pts);
      c->data.assign(b, b + sizeof(pts));
      return c;
    };

    cb1(make_cloud("rslidar_1", 0.1f));
    cb2(make_cloud("rslidar_2", 0.2f));

    auto tfm = std::make_shared<tf2_msgs::msg::TFMessage>();
    for (const auto & lid : {"rslidar_1", "rslidar_2"}) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "base_link";
      t.child_frame_id = lid;
      t.transform.rotation.w = 1.0;
      t.transform.translation.z = 0.02;
      tfm->transforms.push_back(t);
    }
    tf_cb(tfm);
    fuse();
    RCLCPP_INFO(get_logger(), "selftest finished");
  }

private:
  void fuse()
  {
    try {
      if (!have1_ || !have2_) {
        return;
      }
      const rclcpp::Time now = this->now();
      const rclcpp::Time newest = a1_ > a2_ ? a1_ : a2_;
      if ((now - newest).seconds() > 0.5) {
        have1_ = have2_ = false;
        return;
      }
      // 没有新点云则不重复发布 (允许一侧新、一侧用缓存)
      if ((a1_ - last_fused1_).seconds() <= 0.0 &&
          (a2_ - last_fused2_).seconds() <= 0.0)
      {
        return;
      }
      if (std::abs((a1_ - a2_).seconds()) > sync_window_) {
        return;
      }

      const rclcpp::Duration off = rclcpp::Duration::from_seconds(tf_offset_);
      const TFStamped * tf1 = lookup(0, a1_ - off);
      const TFStamped * tf2 = lookup(1, a2_ - off);
      if (tf1 == nullptr || tf2 == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF history empty, skip");
        return;
      }

      Cloud t1, t2, merged;
      transform_cloud(*c1_, *tf1, t1);
      transform_cloud(*c2_, *tf2, t2);
      if (t1.data.empty() && t2.data.empty()) {
        return;
      }

      merged = t1;
      merged.height = 1;
      merged.width = t1.height * t1.width + t2.height * t2.width;
      merged.row_step = merged.point_step * merged.width;
      merged.data.resize(t1.data.size() + t2.data.size());
      std::memcpy(merged.data.data(), t1.data.data(), t1.data.size());
      std::memcpy(
        merged.data.data() + t1.data.size(), t2.data.data(), t2.data.size());
      pub_->publish(merged);

      last_fused1_ = a1_;
      last_fused2_ = a2_;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "fuse exception: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    if (getenv("FUSION_SELFTEST") != nullptr) {
      node->selftest();
    } else {
      rclcpp::spin(node);
    }
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    fprintf(stderr, "point_cloud_fusion fatal: %s\n", e.what());
    return 1;
  }
  return 0;
}
