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
#include <vector>

using namespace std::chrono_literals;

/**
 * 双雷达时间同步融合 (C++ 版, 替代 Python 版)
 * ==========================================
 * Python rclpy 给 PointCloud2.data 赋值约 163ns/字节, 160k 点需 ~400ms,
 * 无法实时。C++ 版单帧处理 <5ms。
 *
 * - 同步检查用点云到达时刻 (同一系统时钟), 不用雷达 header 时钟;
 * - 直接订阅 /tf 维护 2s 环形缓冲, 按“到达时刻 - 50ms”取最近动态变换;
 * - 动态 TF 是 base_link→rslidar, 组合静态 world→base_link(z=0.345) 后变换;
 * - 发布成功后清空缓存帧, 避免重复。
 */
class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : Node("point_cloud_fusion")
  {
    declare_parameter("tf_lookup_offset", 0.05);
    declare_parameter("sync_window", 0.08);
    declare_parameter("deskew", true);          // 逐点去畸变 (方位角 -> TF 插值)
    declare_parameter("plane_align", true);     // 融合前双雷达地面共面校正
    declare_parameter("scan_duration", 0.10);   // 一帧扫描时长 (s)
    declare_parameter("plane_align_gain", 0.30); // 共面校正 EMA 增益
    declare_parameter("plane_align_min_pts", 300);
    declare_parameter("outlier_filter", true);    // 深点畸变保护
    declare_parameter("outlier_below", 0.25);     // 低于地面平面该深度则剔除 (m)
    tf_offset_ = get_parameter("tf_lookup_offset").as_double();
    sync_window_ = get_parameter("sync_window").as_double();
    deskew_ = get_parameter("deskew").as_bool();
    plane_align_ = get_parameter("plane_align").as_bool();
    scan_duration_ = get_parameter("scan_duration").as_double();
    align_gain_ = get_parameter("plane_align_gain").as_double();
    align_min_pts_ = get_parameter("plane_align_min_pts").as_int();
    outlier_filter_ = get_parameter("outlier_filter").as_bool();
    outlier_below_ = get_parameter("outlier_below").as_double();

    sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(200),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { tf_cb(msg); });
    sub1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points_1", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cb1(msg); });
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points_2", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cb2(msg); });
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged_points", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(20ms, [this]() { fuse(); });
    status_timer_ = create_wall_timer(5s, [this]() { status(); });
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
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::deque<TFEntry> tf_hist_[2];
  Cloud::SharedPtr c1_, c2_;
  rclcpp::Time a1_, a2_;
  bool have1_ = false, have2_ = false;
  size_t n_cloud1_ = 0, n_cloud2_ = 0, n_tf_ = 0, n_pub_ = 0;
  rclcpp::Time last_fused1_, last_fused2_;
  double tf_offset_ = 0.05;
  double sync_window_ = 0.08;
  bool deskew_ = true;
  bool plane_align_ = true;
  bool outlier_filter_ = true;
  double outlier_below_ = 0.25;
  double scan_duration_ = 0.10;
  double align_gain_ = 0.30;
  int align_min_pts_ = 300;
  // 共面校正平滑状态 (小角度旋转向量 rx/ry + z 平移)
  double align_rx_ = 0.0, align_ry_ = 0.0, align_z_ = 0.0;
  bool align_ready_ = false;
  size_t n_align_ = 0, n_align_skip_ = 0;

  static int lid_index(const std::string & child)
  {
    return child == "rslidar_1" ? 0 : 1;
  }

  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    try {
      const rclcpp::Time now = this->now();
      ++n_tf_;
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

  static tf2::Quaternion qslerp(
    const tf2::Quaternion & a, const tf2::Quaternion & b, double w)
  {
    tf2::Quaternion bb = b;
    double dot = a.x()*b.x() + a.y()*b.y() + a.z()*b.z() + a.w()*b.w();
    if (dot < 0.0) {
      bb = tf2::Quaternion(-b.x(), -b.y(), -b.z(), -b.w());
      dot = -dot;
    }
    double scale0, scale1;
    if (dot > 0.9995) {
      scale0 = 1.0 - w;
      scale1 = w;
    } else {
      const double th = std::acos(std::min(1.0, dot));
      const double st = std::sin(th);
      scale0 = std::sin((1.0 - w) * th) / st;
      scale1 = std::sin(w * th) / st;
    }
    tf2::Quaternion q(
      scale0*a.x() + scale1*bb.x(),
      scale0*a.y() + scale1*bb.y(),
      scale0*a.z() + scale1*bb.z(),
      scale0*a.w() + scale1*bb.w());
    q.normalize();
    return q;
  }

  bool interp_tf(int i, const rclcpp::Time & t, tf2::Quaternion & q,
                 tf2::Vector3 & v)
  {
    auto & h = tf_hist_[i];
    if (h.empty()) {
      return false;
    }
    if (h.size() == 1) {
      const auto & tf = h.front().tf;
      q = tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      v = tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z);
      return true;
    }
    auto it = std::lower_bound(
      h.begin(), h.end(), t,
      [](const TFEntry & e, const rclcpp::Time & tt) { return e.t < tt; });
    if (it == h.begin()) {
      const auto & tf = it->tf;
      q = tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      v = tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z);
      return true;
    }
    if (it == h.end()) {
      --it;
      const auto & tf = it->tf;
      q = tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      v = tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z);
      return true;
    }
    const auto & b = *it;
    const auto & a = *std::prev(it);
    const double dt = (b.t - a.t).seconds();
    if (dt <= 0.0) {
      const auto & tf = b.tf;
      q = tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      v = tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z);
      return true;
    }
    double w = (t - a.t).seconds() / dt;
    w = std::max(0.0, std::min(1.0, w));
    const auto & ta = a.tf;
    const auto & tb = b.tf;
    const tf2::Quaternion qa(
      ta.transform.rotation.x, ta.transform.rotation.y,
      ta.transform.rotation.z, ta.transform.rotation.w);
    const tf2::Quaternion qb(
      tb.transform.rotation.x, tb.transform.rotation.y,
      tb.transform.rotation.z, tb.transform.rotation.w);
    q = qslerp(qa, qb, w);
    v = tf2::Vector3(
      ta.transform.translation.x + w *
        (tb.transform.translation.x - ta.transform.translation.x),
      ta.transform.translation.y + w *
        (tb.transform.translation.y - ta.transform.translation.y),
      ta.transform.translation.z + w *
        (tb.transform.translation.z - ta.transform.translation.z));
    return true;
  }

  struct BinTF
  {
    tf2::Matrix3x3 R;
    double tx = 0.0, ty = 0.0, tz = 0.0;
  };

  std::vector<BinTF> build_bins(int i, const rclcpp::Time & mid)
  {
    const int K = 21;
    std::vector<BinTF> bins;
    bins.reserve(K);
    for (int k = 0; k < K; ++k) {
      const double frac = static_cast<double>(k) / (K - 1);
      const rclcpp::Time t = mid + rclcpp::Duration::from_seconds(
        (frac - 0.5) * scan_duration_);
      tf2::Quaternion q;
      tf2::Vector3 v;
      if (!interp_tf(i, t, q, v)) {
        bins.clear();
        return bins;
      }
      BinTF b;
      b.R = tf2::Matrix3x3(q);
      b.tx = v.x();
      b.ty = v.y();
      b.tz = v.z() + 0.345;   // world->base_link (球半径)
      bins.push_back(b);
    }
    return bins;
  }

  void transform_cloud_deskew(
    const Cloud & in, const std::vector<BinTF> & bins, Cloud & out)
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

    const size_t ps = in.point_step;
    const size_t n = std::min(
      static_cast<size_t>(in.height) * in.width,
      in.data.size() / ps);
    out.data.resize(in.data.size());
    if (out.data.size() >= in.data.size()) {
      std::memcpy(out.data.data(), in.data.data(), in.data.size());
    }

    const size_t K = bins.size();
    const size_t height = in.height ? in.height : 1;
    const size_t width = in.width ? in.width : 1;
    for (size_t p = 0; p < n; ++p) {
      float ix, iy, iz;
      std::memcpy(&ix, in.data.data() + p * ps + xoff, 4);
      std::memcpy(&iy, in.data.data() + p * ps + yoff, 4);
      std::memcpy(&iz, in.data.data() + p * ps + zoff, 4);
      // Airy 输出 height=900(方位角/时间) x width=96(激光通道):
      // 时间沿行推进, 同一行 96 通道共享方位角 -> 用行号估计扫描时刻
      size_t k = 0;
      if (K > 1) {
        double frac = 0.5;
        if (height > 1 && width > 1) {
          frac = static_cast<double>(p / width) / (height - 1);
        } else if (width > 1) {
          frac = static_cast<double>(p % width) / (width - 1);
        } else {
          const double yaw = std::atan2(
            static_cast<double>(iy), static_cast<double>(ix));
          frac = (yaw + M_PI) / (2.0 * M_PI);   // [0,1)
        }
        size_t kb = static_cast<size_t>(frac * (K - 1) + 0.5);
        k = std::min(kb, K - 1);
      }
      const BinTF & b = bins[k];
      const tf2::Vector3 v =
        b.R * tf2::Vector3(ix, iy, iz) +
        tf2::Vector3(b.tx, b.ty, b.tz);
      float ox = static_cast<float>(v.x());
      float oy = static_cast<float>(v.y());
      float oz = static_cast<float>(v.z());
      std::memcpy(out.data.data() + p * ps + xoff, &ox, 4);
      std::memcpy(out.data.data() + p * ps + yoff, &oy, 4);
      std::memcpy(out.data.data() + p * ps + zoff, &oz, 4);
    }
  }

  bool fit_ground(const Cloud & c, double & a, double & b, double & d)
  {
    int xoff = -1, yoff = -1, zoff = -1;
    for (const auto & f : c.fields) {
      if (f.name == "x") { xoff = static_cast<int>(f.offset); }
      else if (f.name == "y") { yoff = static_cast<int>(f.offset); }
      else if (f.name == "z") { zoff = static_cast<int>(f.offset); }
    }
    if (xoff < 0 || yoff < 0 || zoff < 0 || c.point_step == 0) {
      return false;
    }
    const size_t ps = c.point_step;
    const size_t n = std::min(
      static_cast<size_t>(c.height) * c.width,
      c.data.size() / ps);
    constexpr size_t MAX_PTS = 4000;
    size_t stride = 1;
    if (n > MAX_PTS * 4) {
      stride = n / (MAX_PTS * 4) + 1;
    }
    // 最小二乘: z = a*x + b*y + d (两遍: 先粗拟合, 剔除深点异常后再拟合)
    std::vector<float> px, py, pz;
    px.reserve(MAX_PTS); py.reserve(MAX_PTS); pz.reserve(MAX_PTS);
    for (size_t p = 0; p < n; p += stride) {
      float ix, iy, iz;
      std::memcpy(&ix, c.data.data() + p * ps + xoff, 4);
      std::memcpy(&iy, c.data.data() + p * ps + yoff, 4);
      std::memcpy(&iz, c.data.data() + p * ps + zoff, 4);
      if (!std::isfinite(ix) || !std::isfinite(iy) || !std::isfinite(iz) ||
          iz > 0.25) {
        continue;
      }
      px.push_back(ix); py.push_back(iy); pz.push_back(iz);
      if (px.size() >= MAX_PTS) {
        break;
      }
    }
    auto solve = [&](double & aa, double & bb, double & dd) -> bool {
      double qx = 0, qy = 0, qz = 0, qxx = 0, qyy = 0, qxy = 0;
      double qxz = 0, qyz = 0;
      const size_t m = px.size();
      for (size_t i = 0; i < m; ++i) {
        const double x = px[i], y = py[i], z = pz[i];
        qx += x; qy += y; qz += z;
        qxx += x*x; qyy += y*y; qxy += x*y;
        qxz += x*z; qyz += y*z;
      }
      double mm[3][4] = {
        {qxx, qxy, qx, qxz},
        {qxy, qyy, qy, qyz},
        {qx, qy, static_cast<double>(m), qz},
      };
      for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int r = col + 1; r < 3; ++r) {
          if (std::abs(mm[r][col]) > std::abs(mm[pivot][col])) {
            pivot = r;
          }
        }
        if (std::abs(mm[pivot][col]) < 1e-12) {
          return false;
        }
        if (pivot != col) {
          for (int cc = 0; cc < 4; ++cc) {
            std::swap(mm[pivot][cc], mm[col][cc]);
          }
        }
        for (int r = 0; r < 3; ++r) {
          if (r == col) { continue; }
          const double f = mm[r][col] / mm[col][col];
          for (int cc = col; cc < 4; ++cc) {
            mm[r][cc] -= f * mm[col][cc];
          }
        }
      }
      aa = mm[0][3] / mm[0][0];
      bb = mm[1][3] / mm[1][1];
      dd = mm[2][3] / mm[2][2];
      return true;
    };
    if (!solve(a, b, d)) {
      return false;
    }
    // 剔除 |残差|>5cm 的点 (深点异常/障碍物), 再拟一遍
    std::vector<float> fx, fy, fz;
    for (size_t i = 0; i < px.size(); ++i) {
      const double res = pz[i] - (a*px[i] + b*py[i] + d);
      if (std::abs(res) < 0.05) {
        fx.push_back(px[i]); fy.push_back(py[i]); fz.push_back(pz[i]);
      }
    }
    if (fx.size() < static_cast<size_t>(align_min_pts_)) {
      return false;
    }
    px.swap(fx); py.swap(fy); pz.swap(fz);
    if (!solve(a, b, d)) {
      return false;
    }
    // 倾斜过大视为拟合不可靠 (> ~11°)
    return std::abs(a) < 0.2 && std::abs(b) < 0.2;
  }

  void apply_align(Cloud & c, double rx, double ry, double tz)
  {
    int xoff = -1, yoff = -1, zoff = -1;
    for (const auto & f : c.fields) {
      if (f.name == "x") { xoff = static_cast<int>(f.offset); }
      else if (f.name == "y") { yoff = static_cast<int>(f.offset); }
      else if (f.name == "z") { zoff = static_cast<int>(f.offset); }
    }
    if (xoff < 0 || yoff < 0 || zoff < 0 || c.point_step == 0) {
      return;
    }
    // 小角度旋转矩阵 (绕 x 转 rx, 绕 y 转 ry) + z 平移
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double R[3][3] = {
      {cy, sy*sx, sy*cx},
      {0.0, cx, -sx},
      {-sy, cy*sx, cy*cx},
    };
    const size_t ps = c.point_step;
    const size_t n = std::min(
      static_cast<size_t>(c.height) * c.width,
      c.data.size() / ps);
    for (size_t p = 0; p < n; ++p) {
      float ix, iy, iz;
      std::memcpy(&ix, c.data.data() + p * ps + xoff, 4);
      std::memcpy(&iy, c.data.data() + p * ps + yoff, 4);
      std::memcpy(&iz, c.data.data() + p * ps + zoff, 4);
      const double x = ix, y = iy, z = iz;
      float ox = static_cast<float>(R[0][0]*x + R[0][1]*y + R[0][2]*z);
      float oy = static_cast<float>(R[1][0]*x + R[1][1]*y + R[1][2]*z);
      float oz = static_cast<float>(
        R[2][0]*x + R[2][1]*y + R[2][2]*z + tz);
      std::memcpy(c.data.data() + p * ps + xoff, &ox, 4);
      std::memcpy(c.data.data() + p * ps + yoff, &oy, 4);
      std::memcpy(c.data.data() + p * ps + zoff, &oz, 4);
    }
  }

  void filter_below_plane(Cloud & c, double a, double b, double d)
  {
    int xoff = -1, yoff = -1, zoff = -1;
    for (const auto & f : c.fields) {
      if (f.name == "x") { xoff = static_cast<int>(f.offset); }
      else if (f.name == "y") { yoff = static_cast<int>(f.offset); }
      else if (f.name == "z") { zoff = static_cast<int>(f.offset); }
    }
    if (xoff < 0 || yoff < 0 || zoff < 0 || c.point_step == 0) {
      return;
    }
    const size_t ps = c.point_step;
    const size_t n = std::min(
      static_cast<size_t>(c.height) * c.width,
      c.data.size() / ps);
    size_t w = 0;
    for (size_t p = 0; p < n; ++p) {
      float ix, iy, iz;
      std::memcpy(&ix, c.data.data() + p * ps + xoff, 4);
      std::memcpy(&iy, c.data.data() + p * ps + yoff, 4);
      std::memcpy(&iz, c.data.data() + p * ps + zoff, 4);
      const double plane_z = a*ix + b*iy + d;
      if (std::isfinite(ix) && std::isfinite(iy) && std::isfinite(iz) &&
          iz >= plane_z - outlier_below_) {
        if (w != p) {
          std::memmove(c.data.data() + w * ps,
                       c.data.data() + p * ps, ps);
        }
        ++w;
      }
    }
    c.height = 1;
    c.width = w;
    c.row_step = c.point_step * w;
    c.data.resize(w * ps);
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
    const double tz = tfs.transform.translation.z + 0.345;  // world->base_link (球半径)

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
      ++n_cloud1_;
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
      ++n_cloud2_;
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

  void status()
  {
    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (have1_ && have2_) {
      dt = std::abs((a1_ - a2_).seconds()) * 1000.0;
    }
    RCLCPP_INFO(
      get_logger(),
      "status: clouds=%zu/%zu tf=%zu pub=%zu |a1-a2|=%.0fms "
      "tf1=%zu tf2=%zu deskew=%d align=%zu skip=%zu (rx=%.2f ry=%.2f z=%.2fcm)",
      n_cloud1_, n_cloud2_, n_tf_, n_pub_, dt,
      tf_hist_[0].size(), tf_hist_[1].size(), deskew_ ? 1 : 0,
      n_align_, n_align_skip_,
      align_rx_ * 180.0 / M_PI, align_ry_ * 180.0 / M_PI,
      align_z_ * 100.0);
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
      const rclcpp::Time mid1 = a1_ - off;
      const rclcpp::Time mid2 = a2_ - off;
      const TFStamped * tf1 = lookup(0, mid1);
      const TFStamped * tf2 = lookup(1, mid2);
      if (tf1 == nullptr || tf2 == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF history empty, skip");
        return;
      }

      Cloud t1, t2, merged;
      std::vector<BinTF> bins1, bins2;
      if (deskew_) {
        bins1 = build_bins(0, mid1);
        bins2 = build_bins(1, mid2);
      }
      if (!bins1.empty()) {
        transform_cloud_deskew(*c1_, bins1, t1);
      } else {
        transform_cloud(*c1_, *tf1, t1);
      }
      if (!bins2.empty()) {
        transform_cloud_deskew(*c2_, bins2, t2);
      } else {
        transform_cloud(*c2_, *tf2, t2);
      }
      if (t1.data.empty() && t2.data.empty()) {
        return;
      }

      double g_a = 0.0, g_b = 0.0, g_d = 0.0;
      bool have_g = false;
      // 融合前地面共面校正: 把 lidar2 的地面平面对齐到 lidar1
      if (plane_align_) {
        double a1, b1, d1, a2, b2, d2;
        const bool ok1 = fit_ground(t1, a1, b1, d1);
        const bool ok2 = fit_ground(t2, a2, b2, d2);
        if (ok1 && ok2) {
          g_a = a1; g_b = b1; g_d = d1;
          have_g = true;
          double n1x = -a1, n1y = -b1, n1z = 1.0;
          double n2x = -a2, n2y = -b2, n2z = 1.0;
          const double nn1 = std::sqrt(n1x*n1x + n1y*n1y + n1z*n1z);
          const double nn2 = std::sqrt(n2x*n2x + n2y*n2y + n2z*n2z);
          n1x /= nn1; n1y /= nn1; n1z /= nn1;
          n2x /= nn2; n2y /= nn2; n2z /= nn2;
          const double dot = std::max(-1.0, std::min(1.0,
            n1x*n2x + n1y*n2y + n1z*n2z));
          // 旋转轴 = n2 × n1 (把 n2 转到 n1)
          double ax = n2y*n1z - n2z*n1y;
          double ay = n2z*n1x - n2x*n1z;
          double az = n2x*n1y - n2y*n1x;
          const double ac = std::sqrt(ax*ax + ay*ay + az*az);
          const double angle = std::atan2(ac, dot);
          double rx_t = 0.0, ry_t = 0.0;
          if (ac > 1e-6 && angle > 0.01) {
            ax /= ac; ay /= ac; az /= ac;
            rx_t = ax * angle;
            ry_t = ay * angle;
          }
          double dz_t = d1 - d2;
          dz_t = std::max(-0.20, std::min(0.20, dz_t));
          rx_t = std::max(-0.10, std::min(0.10, rx_t));
          ry_t = std::max(-0.10, std::min(0.10, ry_t));
          if (!align_ready_) {
            align_rx_ = rx_t;
            align_ry_ = ry_t;
            align_z_ = dz_t;
            align_ready_ = true;
          } else {
            align_rx_ += align_gain_ * (rx_t - align_rx_);
            align_ry_ += align_gain_ * (ry_t - align_ry_);
            align_z_ += align_gain_ * (dz_t - align_z_);
          }
          apply_align(t2, align_rx_, align_ry_, align_z_);
          ++n_align_;
        } else {
          ++n_align_skip_;
        }
      }

      // 深点畸变保护: 剔除低于地面平面 outlier_below_ 的异常点
      // (高速翻滚时单帧点云畸变产生的"穿地"假点, 离线验证: 穿透 0.22->0.11%,
      //  最差帧 4.63->2.54%, 每帧仅滤掉 ~0.14% 点)
      if (outlier_filter_) {
        if (!have_g) {
          have_g = fit_ground(t1, g_a, g_b, g_d);
          if (!have_g) {
            have_g = fit_ground(t2, g_a, g_b, g_d);
          }
        }
        if (have_g) {
          filter_below_plane(t1, g_a, g_b, g_d);
          filter_below_plane(t2, g_a, g_b, g_d);
        }
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
      ++n_pub_;

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
