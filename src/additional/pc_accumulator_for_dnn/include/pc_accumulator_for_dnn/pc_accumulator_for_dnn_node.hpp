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
#include <std_msgs/msg/header.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <deque>

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <semaphore.h>

namespace pc_acc_for_dnn
{
  class PCloudAccForDnnComponent : public rclcpp::Node
  {
    public:
      //  PCL_MAKE_ALIGNED_OPERATOR_NEW
      explicit PCloudAccForDnnComponent(const rclcpp::NodeOptions & options);

      ~PCloudAccForDnnComponent() {
        cleanup_shared_memory();
      }

    private:
      double accumulation_time_sec_;
      int point_cloud_buf_sz_;
      bool debug_mode_;

      const char* shm_name_ = "/valor_pointcloud_data";
      float* shared_pc_data_ = nullptr;
      int shm_fd_;
      size_t shm_data_size_;
      const char* sem_name_ = "/valor_data_mutex";
      sem_t* shm_mutex_ = nullptr;

      std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pcloud_queue_; // each PC2 msg has timestamp in it
      std::deque<geometry_msgs::msg::TransformStamped::ConstSharedPtr> tf_queue_;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr egopose_sub_;
      rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pc_notify_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pc_pub_;
      rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr process_time_pub_;

      std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_;

      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

      void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
      void cleanup_shared_memory() {
        if (shared_pc_data_ != nullptr) {
          // Unmap the memory
          munmap(shared_pc_data_, shm_data_size_);
          shared_pc_data_ = nullptr;

          // Close the file descriptor
          close(shm_fd_);

          // Remove the shared memory object
          shm_unlink(shm_name_);

          RCLCPP_INFO(get_logger(), "Shared memory resources cleaned up");
        }

        if(shm_mutex_ != nullptr){
          sem_close(shm_mutex_);
          sem_unlink(sem_name_);
        }
      }
  };
}

// clang-format off
#endif  // PC_ACC_HPP_  // NOLINT
        // clang-format on
