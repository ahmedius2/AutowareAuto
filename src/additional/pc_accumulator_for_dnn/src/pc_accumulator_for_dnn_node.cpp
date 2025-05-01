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
#include <chrono>
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
    accumulation_time_sec_ = declare_parameter<double>("accumulation_time_sec", 0.55);
    point_cloud_buf_sz_ = declare_parameter<int>("pcloud_queue_size", 5);
    debug_mode_ = declare_parameter<bool>("debug", false);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/pointcloud", rclcpp::SensorDataQoS(),
        std::bind(&PCloudAccForDnnComponent::pointcloud_callback, this, std::placeholders::_1));

    pc_notify_pub_ = create_publisher<std_msgs::msg::Header>("accumulated_pc_as_arr", rclcpp::SensorDataQoS());

//if(debug_mode_){
    debug_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "debug/accumulated_pointcloud", 10);
//    }

    stop_watch_ = 
      std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
    process_time_pub_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
        "~/exec_time_ms", 10);
   //pipeline_time_pub_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
   //     "~/debug/pipeline_time_ms", 10);
   //
    // In your node's initialization
    shm_data_size_ = 500000 * 4 * sizeof(float); // Around 8 MB
    shm_fd_ = shm_open(shm_name_, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_, shm_data_size_);
    shared_pc_data_ = (float*)mmap(nullptr, shm_data_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);

    shm_mutex_ = sem_open(sem_name_, O_CREAT, 0666, 1);
  }

  void PCloudAccForDnnComponent::pointcloud_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    debug_mode_ = this->get_parameter("debug").as_bool();

    auto tp = std::chrono::system_clock::now();
    rclcpp::Time pc_arrival_sys_time(tp.time_since_epoch().count());
    stop_watch_->tic("processing_time");
    pcloud_queue_.push_back(input);

    // Remove point clouds that is beyond time lim
    rclcpp::Time most_recent_pc_time = input->header.stamp;
    double tdiff = (most_recent_pc_time - pcloud_queue_.front()->header.stamp).seconds();
    while(pcloud_queue_.size() > 1 && tdiff > accumulation_time_sec_){
      pcloud_queue_.pop_front();
      tdiff = (most_recent_pc_time - pcloud_queue_.front()->header.stamp).seconds();
    }

    while(pcloud_queue_.size() > point_cloud_buf_sz_){
      pcloud_queue_.pop_front();
    }

    geometry_msgs::msg::TransformStamped::ConstSharedPtr tf;
    rclcpp::Time tf_time;

    try {
      rclcpp::Duration dur(0, 100000000); // Timout 100 milliseconds, but it should just take 1 ms
      auto tf_res = tf_buffer_->lookupTransform("base_link", "map", most_recent_pc_time, dur);
      tf_time = tf_res.header.stamp;
      tdiff = std::abs((most_recent_pc_time - tf_time).seconds());
      if(tdiff*1000 > 20 || debug_mode_){
        RCLCPP_INFO(this->get_logger(), "point cloud and tf tdiff: %.1f ms", tdiff * 1000);
      }

      tf_res.header.stamp = input->header.stamp; // Just to make sure they are same
      tf = std::make_shared<geometry_msgs::msg::TransformStamped>(tf_res);
      tf_queue_.push_back(tf);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          "base_link", "map", ex.what());
    }

    if(pcloud_queue_.empty() || tf_queue_.empty())
      return;

    tdiff = (most_recent_pc_time - tf_queue_.front()->header.stamp).seconds();
    while(tf_queue_.size() > 1 && tdiff > accumulation_time_sec_){
      tf_queue_.pop_front();
      tdiff = (most_recent_pc_time - tf_queue_.front()->header.stamp).seconds();
    }

    // Find the point cloud of each egopose
    // the head of the queue has the most recent paired cloud
    std::deque<std::pair<sensor_msgs::msg::PointCloud2::ConstSharedPtr,
      geometry_msgs::msg::TransformStamped::ConstSharedPtr>> pairs;
    unsigned num_all_points = 0;
    for(auto pc : pcloud_queue_){
      rclcpp::Time pc_ts = pc->header.stamp;
      for(auto tf_sel : tf_queue_){
        rclcpp::Time tf_ts = tf_sel->header.stamp;
        if(pc_ts == tf_ts){ // it ok to compare like this
          pairs.push_front(std::make_pair(pc, tf_sel));
          num_all_points += pc->width;
          break;
        }
      }
    }

    if(pairs.empty() || (pairs.front().first->header.stamp != most_recent_pc_time)){
      // Publish the most recent point cloud only
      num_all_points = input->width;
      pairs.clear();
      pairs.push_front(std::make_pair(input, nullptr));
    }

    auto num_fields = 4; // x, y, z, t
    std_msgs::msg::Header new_pc_notify_msg;
    new_pc_notify_msg.frame_id = "velodyne_top";
    new_pc_notify_msg.stamp = pairs.front().first->header.stamp;

    // after these, put the data

    sensor_msgs::msg::PointCloud2 debug_pc;
    if(debug_mode_){
      debug_pc = *input;
      debug_pc.header.frame_id = "velodyne_top";
      debug_pc.width = num_all_points;
      debug_pc.data.resize(num_all_points * debug_pc.point_step); // bytes array
    }

    Eigen::Quaternion<float> q1(
        tf->transform.rotation.w,
        tf->transform.rotation.x,
        tf->transform.rotation.y,
        tf->transform.rotation.z);
    Eigen::Transform<float,3,Eigen::Affine> map_to_bl_t = Eigen::Translation3f(
        tf->transform.translation.x,
        tf->transform.translation.y,
        tf->transform.translation.z) * q1;

    // This is a static transformation
    auto baselink_to_velotop_t = tf_buffer_->lookupTransform("velodyne_top",
        "base_link", tf2::TimePointZero);

    Eigen::Quaternion<float> q3(
        baselink_to_velotop_t.transform.rotation.w,
        baselink_to_velotop_t.transform.rotation.x,
        baselink_to_velotop_t.transform.rotation.y,
        baselink_to_velotop_t.transform.rotation.z);
    Eigen::Transform<float,3,Eigen::Affine> bl_to_vlt_t = Eigen::Translation3f(
        baselink_to_velotop_t.transform.translation.x,
        baselink_to_velotop_t.transform.translation.y,
        baselink_to_velotop_t.transform.translation.z) * q3;

    sem_wait(shm_mutex_);
    shared_pc_data_[0] = num_all_points;
    shared_pc_data_[1] = num_fields;

    int point_counter = 0;
    for (auto p : pairs){ // from newest to oldest
      auto pc_msg = *(p.first);
      auto tf_sel = p.second; // this can be nullptr!

      float tdiff = (most_recent_pc_time - pc_msg.header.stamp).seconds();
      sensor_msgs::PointCloud2ConstIterator<float>
        x_itr(pc_msg, "x"), y_itr(pc_msg, "y"), z_itr(pc_msg, "z");
      sensor_msgs::PointCloud2ConstIterator<uint8_t>
        i_itr(pc_msg, "intensity"), rt_itr(pc_msg, "return_type");
      sensor_msgs::PointCloud2ConstIterator<uint16_t> c_itr(pc_msg, "channel");

      Eigen::MatrixXf points(4, pc_msg.width);
      for (size_t i = 0; x_itr != x_itr.end(); ++x_itr, ++y_itr, ++z_itr, ++i) {
        points(0, i) = *x_itr;
        points(1, i) = *y_itr;
        points(2, i) = *z_itr;
      }
      points.row(3).setOnes();

      if(tdiff != .0){
        Eigen::Quaternion<float> q2(
            tf_sel->transform.rotation.w,
            tf_sel->transform.rotation.x,
            tf_sel->transform.rotation.y,
            tf_sel->transform.rotation.z);
        Eigen::Transform<float,3,Eigen::Affine> map_to_bl_t2= Eigen::Translation3f(
            tf_sel->transform.translation.x,
            tf_sel->transform.translation.y,
            tf_sel->transform.translation.z) * q2;

        points = bl_to_vlt_t.matrix() * map_to_bl_t.matrix() * \
                 map_to_bl_t2.inverse().matrix() * points;
      }
      else{
        points = bl_to_vlt_t.matrix() * points;
      }

      float* data_ptr = &shared_pc_data_[2];
      for(size_t i=0; i_itr != i_itr.end(); ++i, ++i_itr, ++point_counter){
        //NOTE still need to do the transformation and timing
        auto idx = point_counter * num_fields;
        data_ptr[idx] = points(0, i);
        data_ptr[idx+1] = points(1, i);
        data_ptr[idx+2] = points(2, i);
        data_ptr[idx+3] = tdiff; // save it as sec

        if(debug_mode_){
          // ignore rettype and channel?
          uint8_t* addr = &debug_pc.data[point_counter * debug_pc.point_step];
          ((float*)addr)[0] = points(0, i);
          ((float*)addr)[1] = points(1, i);
          ((float*)addr)[2] = points(2, i);
          addr += sizeof(float)*3;
          addr[0] = *i_itr; // its just 0 for awsim but save it anyway
          addr[1] = *rt_itr;
          addr += sizeof(uint8_t)*2;
          ((uint16_t*)addr)[0] = *c_itr;
          ++rt_itr; ++c_itr;
        }
      }
    }
    sem_post(shm_mutex_);

    pc_notify_pub_->publish(new_pc_notify_msg);

    const auto processing_time_ms = stop_watch_->toc("processing_time");
    tier4_debug_msgs::msg::Float64Stamped time_msg;
    time_msg.stamp = pc_arrival_sys_time; // input->header.stamp;
    time_msg.data = processing_time_ms;
    process_time_pub_->publish(time_msg);

    //auto cur_time = this->get_clock()->now();
    //time_msg.data = (cur_time - most_recent_pc_time).nanoseconds() * 1e-6;
    //pipeline_time_pub_->publish(time_msg);

    if(debug_mode_){
      debug_pc_pub_->publish(debug_pc);
      RCLCPP_INFO(this->get_logger(), "Published %d points from %d point clouds. Took %.1f ms.",
          point_counter, pairs.size(), processing_time_ms);
    }
  }
}  // namespace pc_acc_for_dnn

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pc_acc_for_dnn::PCloudAccForDnnComponent)
