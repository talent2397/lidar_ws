// pointcloud_lite_node.cpp
// 轻量显示降采样节点:
//   大点云(融合/BEV) -> VoxelGrid 体素降采样 -> 限频发布
//   给 RViz / 浏览器 WebGL 显示, 大幅降低 Jetson CPU / 网络带宽
//
// 典型用法 (web_view.launch.py 中会同时起两个实例):
//   /merged_points      -> /merged_points_lite      (0.1m, ~3Hz)
//   /merged_points_bev  -> /merged_points_bev_lite  (0.1m, ~3Hz)

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudLite : public rclcpp::Node
{
public:
  PointCloudLite()
  : Node("pointcloud_lite"), last_pub_(std::chrono::steady_clock::now())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/merged_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/merged_points_lite");
    leaf_size_ = declare_parameter<double>("leaf_size", 0.1);
    min_interval_ms_ = declare_parameter<int>("min_interval_ms", 300);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudLite::cloud_cb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());

    RCLCPP_INFO(get_logger(),
      "轻量显示: %s -> %s (leaf=%.2fm, 最高 %.1fHz)",
      input_topic_.c_str(), output_topic_.c_str(), leaf_size_,
      1000.0 / static_cast<double>(min_interval_ms_));
  }

private:
  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_pub_ < std::chrono::milliseconds(min_interval_ms_)) {
      return;  // 限频: 丢弃中间帧, 只保留最新
    }

    pcl::PCLPointCloud2 in;
    pcl_conversions::toPCL(*msg, in);

    pcl::PCLPointCloud2::Ptr in_ptr(new pcl::PCLPointCloud2(in));
    pcl::PCLPointCloud2 out;
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud(in_ptr);
    voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel.filter(out);

    auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::moveFromPCL(out, *out_msg);
    out_msg->header = msg->header;  // 保留原始 frame_id / 时间戳
    pub_->publish(*out_msg);
    last_pub_ = now;

    if (++publish_count_ % 10 == 1) {
      RCLCPP_INFO(get_logger(), "%s: %zu -> %zu 点",
        input_topic_.c_str(),
        static_cast<size_t>(msg->width * msg->height),
        static_cast<size_t>(out_msg->width * out_msg->height));
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double leaf_size_ = 0.1;
  int min_interval_ms_ = 300;
  std::chrono::steady_clock::time_point last_pub_;
  uint64_t publish_count_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudLite>());
  rclcpp::shutdown();
  return 0;
}
