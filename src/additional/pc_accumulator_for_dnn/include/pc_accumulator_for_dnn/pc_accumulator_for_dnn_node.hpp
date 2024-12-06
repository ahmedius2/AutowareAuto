// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PC_ACC_HPP_  // NOLINT
#define PC_ACC_HPP_  // NOLINT

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <valo_msgs/msg/float32_multi_array_stamped.hpp>
#include <deque>

//#include <tf2/LinearMath/Pose.h>
//#include <tf2/convert.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>

namespace pc_acc_for_dnn
{
class PCloudAccForDnnComponent : public rclcpp::Node
{
public:
//  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PCloudAccForDnnComponent(const rclcpp::NodeOptions & options);

private:
  double accumulation_time_sec_;
  int point_cloud_buf_sz_;
  bool publish_debug_pc_;

  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointcloud_buffer_; // each PC2 msg has timestamp in it
  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> ego_poses_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr egopose_sub_;
  rclcpp::Publisher<valo_msgs::msg::Float32MultiArrayStamped>::SharedPtr floatarr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pc_pub_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
  void ego_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr ego_pose);
};
}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // PC_ACC_HPP_  // NOLINT
// clang-format on
