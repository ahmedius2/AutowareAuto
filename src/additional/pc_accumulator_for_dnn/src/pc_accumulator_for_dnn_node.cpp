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

#include "pc_accumulator_for_dnn/pc_accumulator_for_dnn_node.hpp"
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rcpputils/asserts.hpp>

namespace pc_acc_for_dnn
{
PCloudAccForDnnComponent::PCloudAccForDnnComponent(const rclcpp::NodeOptions & options)
  : Node("PointcloudAccumulatorForDNN", options)
{
  accumulation_time_sec_ = declare_parameter<double>("accumulation_time_sec", 0.5);
  point_cloud_buf_sz_ = declare_parameter<int>("pointcloud_buffer_size", 10);
  publish_debug_pc_ = declare_parameter<bool>("debug", true);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&PCloudAccForDnnComponent::pointcloud_callback, this, std::placeholders::_1));

  egopose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose_estimator/pose", 10,
    std::bind(&PCloudAccForDnnComponent::ego_pose_callback, this, std::placeholders::_1));

  floatarr_pub_ = create_publisher<valo_msgs::msg::Float32MultiArrayStamped>(
    "dnn_inp_float_arr", 10);

  if(publish_debug_pc_){
    debug_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "pc_acc_for_dnn_debug_pc", 10);
  }
}

void PCloudAccForDnnComponent::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr input) 
{
  pointcloud_buffer_.push_back(input);

  // Lookup the transform from BASE_LINK to MAP
//  auto target_frame_id = "map";
//  auto source_frame_id = input->header.frame_id;
//
//  const rclcpp::Time measurement_time =
//    rclcpp::Time(input->header.stamp, this->now().get_clock_type());
//  geometry_msgs::msg::PoseStamped transform_stamped = tf_buffer_.lookupPose(
//    target_frame_id, source_frame_id, measurement_time,
//    rclcpp::Duration::from_seconds(0.5)); // what if timeout?
//  ego_poses_.push_back(transform_stamped);

  // Remove point clouds that is beyond time lim
  rclcpp::Time most_recent_pc_time = input->header.stamp;
  double tdiff = (most_recent_pc_time - pointcloud_buffer_.front()->header.stamp).seconds();
  while(pointcloud_buffer_.size() > 1 && tdiff > accumulation_time_sec_){
    pointcloud_buffer_.pop_front();
    tdiff = (most_recent_pc_time - pointcloud_buffer_.front()->header.stamp).seconds();
  }

  while(pointcloud_buffer_.size() > point_cloud_buf_sz_){
    pointcloud_buffer_.pop_front();
  }

//  pcl::PointCloud<pcl::PointXYZI> pcl_input;
//  pcl::PointCloud<pcl::PointXYZI> pcl_output;
//  for(auto pc_ptr : pointcloud_buffer_){
//    // I have to do the transformation
//    pcl::fromROSMsg(*pc_ptr, pcl_input);
//    pcl_output += pcl_input;
//  }
//
//  pcl::toROSMsg(pcl_output, output);
//  output.header = input->header;
}


void PCloudAccForDnnComponent::ego_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr ego_pose) 
{
  // push the transform from map to base_link (right?)
  ego_poses_.push_back(ego_pose);
  if(pointcloud_buffer_.empty())
    return;

  auto most_recent_pc = pointcloud_buffer_.back();
  rclcpp::Time most_recent_pc_time = most_recent_pc->header.stamp;
  double tdiff = (most_recent_pc_time - ego_poses_.front()->header.stamp).seconds();
  while(ego_poses_.size() > 1 && tdiff > accumulation_time_sec_){
    ego_poses_.pop_front();
    tdiff = (most_recent_pc_time - ego_poses_.front()->header.stamp).seconds();
  }

  // Find the point cloud of each egopose
  // the head of the queue has the most recent paired cloud
  std::deque<std::pair<sensor_msgs::msg::PointCloud2::ConstSharedPtr, 
    geometry_msgs::msg::PoseStamped::ConstSharedPtr>> pairs;
  unsigned num_points = 0;
  for(auto pc : pointcloud_buffer_){
    rclcpp::Time pc_ts = pc->header.stamp;
    for(auto pose : ego_poses_){
      rclcpp::Time pose_ts = pose->header.stamp;
      if(pc_ts == pose_ts){ // it ok to compare like this
        pairs.push_front(std::make_pair(pc, pose));
        num_points += pc->width;
        break;
      }
    }
  }

  if(pairs.empty() || (pairs.front().first->header.stamp != most_recent_pc_time)){
    // Publish the most recent point cloud only
    num_points = most_recent_pc->width;
    pairs.clear();
    pairs.push_front(std::make_pair(most_recent_pc, nullptr));
  }
  else{
    //I am going to apply affine transformations
    auto most_recent_pose = pairs.front().second;
  }

  valo_msgs::msg::Float32MultiArrayStamped multarr;
  multarr.header.frame_id = "base_link";
  multarr.header.stamp = pairs.front().first->header.stamp;
  auto num_fields = 6; // batch_id, x, y, z, i ,t
  auto dim1 = std_msgs::msg::MultiArrayDimension();
  dim1.label = "num_points";
  dim1.size = num_points;
  dim1.stride = num_points * num_fields;
  auto dim2 = std_msgs::msg::MultiArrayDimension();
  dim2.label = "bxyzit";
  dim2.size = num_fields;
  dim2.stride = num_fields;
  multarr.array.layout.dim.push_back(dim1);
  multarr.array.layout.dim.push_back(dim2);
  multarr.array.layout.data_offset = 0;
  multarr.array.data.resize(num_points * num_fields);

  sensor_msgs::msg::PointCloud2 debug_pc;
  if(publish_debug_pc_){
    debug_pc = *most_recent_pc;
    debug_pc.width = num_points;
    debug_pc.data.resize(num_points * debug_pc.point_step); // bytes array
  }

  Eigen::Quaternion<float> q1(
      ego_pose->pose.orientation.w,
      ego_pose->pose.orientation.x,
      ego_pose->pose.orientation.y,
      ego_pose->pose.orientation.z);
  q1.normalize();
  Eigen::Transform<float,3,Eigen::Affine> map_to_bl_t = Eigen::Translation3f(
      ego_pose->pose.position.x,
      ego_pose->pose.position.y,
      ego_pose->pose.position.z) * q1;

  int point_counter = 0;
  for (auto p : pairs){ // from newest to oldest
    auto pc_msg = *(p.first);
    auto pose = p.second; // this can be nullptr!

    float tdiff = (most_recent_pc_time - pc_msg.header.stamp).seconds();
    sensor_msgs::PointCloud2ConstIterator<float> 
      x_itr(pc_msg, "x"), y_itr(pc_msg, "y"), z_itr(pc_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> 
      i_itr(pc_msg, "intensity"), rt_itr(pc_msg, "return_type");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> c_itr(pc_msg, "channel"); 
  
//    Eigen::Transform<float,3,Eigen::Affine> t;
    Eigen::MatrixXf points(4, num_points);
    for (size_t idx = 0; x_itr != x_itr.end(); ++x_itr, ++y_itr, ++z_itr, ++idx) {
        points(0, idx) = *x_itr;
        points(1, idx) = *y_itr;
        points(2, idx) = *z_itr;
    }
    points.row(3).setOnes();

    if(tdiff != .0){
      Eigen::Quaternion<float> q2(
          pose->pose.orientation.w,
          pose->pose.orientation.x,
          pose->pose.orientation.y,
          pose->pose.orientation.z);
      q2.normalize();
      Eigen::Transform<float,3,Eigen::Affine> map_to_bl_t2= Eigen::Translation3f(
          pose->pose.position.x,
          pose->pose.position.y,
          pose->pose.position.z) * q2;

      points = map_to_bl_t.matrix() * map_to_bl_t2.inverse().matrix() * points;
      //std::cout << "Rows: " << transformed_points.rows() << std::endl;
     //std::cout << "Columns: " << transformed_points.cols() << std::endl;
    }

    for(; i_itr != i_itr.end(); ++i_itr, ++point_counter){
      //NOTE still need to do the transformation and timing
      auto idx = point_counter * num_fields;
      multarr.array.data[idx] = 0; // batch size always 0
      multarr.array.data[idx+1] = points(0, point_counter);
      multarr.array.data[idx+2] = points(1, point_counter);
      multarr.array.data[idx+3] = points(2, point_counter);
      multarr.array.data[idx+4] = (float) *i_itr; // should be between 0 and 255
      multarr.array.data[idx+5] = tdiff; // save it as sec
      if(publish_debug_pc_){
        // ignore rettype and channel?
        uint8_t* addr = &debug_pc.data[point_counter * debug_pc.point_step];
        ((float*)addr)[0] = multarr.array.data[idx+1];
        ((float*)addr)[1] = multarr.array.data[idx+2];
        ((float*)addr)[2] = multarr.array.data[idx+3];
        addr += sizeof(float)*3;
        addr[0] = *i_itr;
        addr[1] = *rt_itr;
        addr += sizeof(uint8_t)*2;
        ((uint16_t*)addr)[0] = *c_itr;
        ++rt_itr; ++c_itr;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Published %d points from %d point clouds.", point_counter,
      pairs.size());

  floatarr_pub_->publish(multarr);

  if(publish_debug_pc_){
    debug_pc_pub_->publish(debug_pc);
  }
}
}  // namespace pc_acc_for_dnn

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pc_acc_for_dnn::PCloudAccForDnnComponent)
