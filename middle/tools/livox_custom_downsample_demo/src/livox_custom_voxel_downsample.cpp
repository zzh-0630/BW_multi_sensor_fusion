#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"

class LivoxCustomVoxelDownsampleNode : public rclcpp::Node {
 public:
  LivoxCustomVoxelDownsampleNode()
      : Node("livox_custom_voxel_downsample_node") {
    this->declare_parameter<double>("leaf_size", 0.2);

    this->declare_parameter<std::string>("input_topic", "/livox/lidar");

    this->declare_parameter<std::string>("raw_output_topic",
                                         "/livox/lidar_raw_pointcloud2");

    this->declare_parameter<std::string>("downsampled_output_topic",
                                         "/livox/lidar_downsampled");

    this->declare_parameter<std::string>("default_frame_id", "livox_frame");

    leaf_size_ = this->get_parameter("leaf_size").as_double();
    input_topic_ = this->get_parameter("input_topic").as_string();
    raw_output_topic_ = this->get_parameter("raw_output_topic").as_string();
    downsampled_output_topic_ =
        this->get_parameter("downsampled_output_topic").as_string();
    default_frame_id_ = this->get_parameter("default_frame_id").as_string();

    pub_raw_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        raw_output_topic_, rclcpp::SensorDataQoS());

    pub_downsampled_cloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            downsampled_output_topic_, rclcpp::SensorDataQoS());

    sub_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &LivoxCustomVoxelDownsampleNode::livoxCallback,
          this,
          std::placeholders::_1
        )
    );

    RCLCPP_INFO(this->get_logger(),
                "Livox CustomMsg voxel downsample node started.");
    RCLCPP_INFO(this->get_logger(), "Input topic              : %s",
                input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Raw PointCloud2 topic    : %s",
                raw_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Downsampled topic        : %s",
                downsampled_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Leaf size                : %.3f m",
                leaf_size_);
    RCLCPP_INFO(this->get_logger(), "Default frame_id         : %s",
                default_frame_id_.c_str());
  }

 private:
  std_msgs::msg::Header makeOutputHeader(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    std_msgs::msg::Header header = msg->header;

    if (header.frame_id.empty()) {
      header.frame_id = default_frame_id_;
    }

    /*
     * 有些情况下 msg->header.stamp 可能不是你期望的 ROS 时间。
     * 为了 RViz2 显示更稳，这里直接使用当前 ROS 时间。
     * 如果你想严格保留 Livox 原始时间戳，可以改成：
     *
     * header.stamp = msg->header.stamp;
     */
    header.stamp = this->now();

    return header;
  }

  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    using PointType = pcl::PointXYZI;

    pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_downsampled(
        new pcl::PointCloud<PointType>);

    cloud_raw->reserve(msg->points.size());

    for (const auto& p : msg->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      PointType point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.intensity = static_cast<float>(p.reflectivity);

      cloud_raw->push_back(point);
    }

    cloud_raw->width = static_cast<uint32_t>(cloud_raw->size());
    cloud_raw->height = 1;
    cloud_raw->is_dense = false;

    if (cloud_raw->empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Received empty Livox point cloud.");
      return;
    }

    std_msgs::msg::Header output_header = makeOutputHeader(msg);

    /*
     * 1. 发布原始转换后的 PointCloud2
     * 这个话题没有下采样，只是把 Livox CustomMsg 转成 RViz2 能显示的
     * PointCloud2。
     */
    sensor_msgs::msg::PointCloud2 raw_msg;
    pcl::toROSMsg(*cloud_raw, raw_msg);
    raw_msg.header = output_header;
    pub_raw_cloud_->publish(raw_msg);

    /*
     * 2. 体素下采样
     */
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(cloud_raw);
    voxel_filter.setLeafSize(static_cast<float>(leaf_size_),
                             static_cast<float>(leaf_size_),
                             static_cast<float>(leaf_size_));
    voxel_filter.filter(*cloud_downsampled);

    cloud_downsampled->width = static_cast<uint32_t>(cloud_downsampled->size());
    cloud_downsampled->height = 1;
    cloud_downsampled->is_dense = false;

    /*
     * 3. 发布下采样后的 PointCloud2
     */
    sensor_msgs::msg::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*cloud_downsampled, downsampled_msg);
    downsampled_msg.header = output_header;
    pub_downsampled_cloud_->publish(downsampled_msg);

    frame_count_++;

    if (frame_count_ % 30 == 0) {
      double ratio = 100.0 * static_cast<double>(cloud_downsampled->size()) /
                     static_cast<double>(cloud_raw->size());

      RCLCPP_INFO(
          this->get_logger(),
          "Raw: %zu points, Downsampled: %zu points, Remaining: %.2f %%",
          cloud_raw->size(), cloud_downsampled->size(), ratio);
    }
  }

 private:
  double leaf_size_;

  std::string input_topic_;
  std::string raw_output_topic_;
  std::string downsampled_output_topic_;
  std::string default_frame_id_;

  size_t frame_count_ = 0;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_downsampled_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LivoxCustomVoxelDownsampleNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}