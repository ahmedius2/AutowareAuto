#include <string>
#include <sstream>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <list>
#include <mutex>
#include <atomic>
#include <map>
#include <malloc.h>
#include <sys/mman.h>

#include <rclcpp/rclcpp.hpp>
//#include <ros/callback_queue.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include <assert.h>
#include <unistd.h>
#include <iostream>

#include "node_profiler/node_profiler.h"
#include "node_profiler/rapidcsv.h"


static builtin_interfaces::msg::Time convert_to_ros2_time(std::chrono::system_clock::time_point time_point)
{
    builtin_interfaces::msg::Time ros2_time;

    // Get the duration since the epoch
    auto duration = time_point.time_since_epoch();

    // Convert the duration to seconds and nanoseconds
    ros2_time.sec = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    ros2_time.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;

    return ros2_time;
}


NodeProfiler::NodeProfiler(rclcpp::Node* node_handle, std::vector<std::string> callback_names)
  : cb_prof_nh(node_handle), period_id(0), profiling_now(false)
{
  for(std::string name : callback_names){
    auto cbProfPublisher = cb_prof_nh->create_publisher<callback_profile::msg::CallbackProfile>(
        std::string(cb_prof_nh->get_name()) + "/" + name + "_cb_profile", 10);
    publishers_map.emplace(name, cbProfPublisher);
  }

  auto full_node_name = std::string(cb_prof_nh->get_namespace()) + "/" + std::string(cb_prof_nh->get_name());
  try{
      rapidcsv::Document conf(config_file_path, rapidcsv::LabelParams(-1, 0));
      std::vector<std::string> row = conf.GetRow<std::string>(full_node_name);
      auto sched_policy_s = row[0];
      int sched_priority = std::stoi(row[1]);
      auto cpu_affinity_s = row[2];

      int sched_policy;
      if(sched_policy_s.compare("SCHED_FIFO") == 0){
          sched_policy = SCHED_FIFO;
      }
      else if(sched_policy_s.compare("SCHED_RR") == 0){
          sched_policy = SCHED_RR;
      }
      else if(sched_policy_s.compare("SCHED_OTHER") == 0){
          sched_policy = SCHED_OTHER;
      }
      else{
          sched_policy = SCHED_OTHER;
          sched_policy_s = "SCHED_OTHER";
          RCLCPP_WARN(cb_prof_nh->get_logger(), "Scheduling policy is undefined, defaulting to SCHED_OTHER");
      }

      struct sched_param sp;
      sp.sched_priority= (sched_policy == SCHED_OTHER) ? 0 : sched_priority;
      int ret = sched_setscheduler(0, sched_policy, &sp);
      if(ret){
          RCLCPP_ERROR(cb_prof_nh->get_logger(), "sched_setscheduler returned error %s, desired priority was %d\n",
                  std::strerror(errno), sched_priority);
      }

      if(sched_policy == SCHED_OTHER){
          // Set niceness
          ret = setpriority(PRIO_PROCESS, 0, sched_priority);
          if(ret == -1){
              RCLCPP_ERROR_STREAM(cb_prof_nh->get_logger(), "setpriority returned error: " <<
                  std::strerror(errno) << ", desired niceness was " << sched_priority << std::endl);
          }
      }

      std::stringstream ss_hex;
      unsigned mask;
      ss_hex << std::hex << cpu_affinity_s;
      ss_hex >> mask;

      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      for(unsigned i=0; mask != 0; ++i, mask = mask>>1)
          if(mask & 0x1){
              CPU_SET(i,&cpuset);
          }


      ret = sched_setaffinity(0, sizeof(cpuset), &cpuset);
      if(ret){
          RCLCPP_ERROR_STREAM(cb_prof_nh->get_logger(),  "sched_setaffinity returned error: "
                    << std::strerror(errno) << std::endl);
      }

      RCLCPP_INFO(cb_prof_nh->get_logger(), "Scheduling node %s with policy %s, priority %d and cpuset %s\n",
                  full_node_name.c_str(), sched_policy_s.c_str(), sched_priority, cpu_affinity_s.c_str());

  }
  catch(...){
      RCLCPP_ERROR(cb_prof_nh->get_logger(), "Couln't open the configuration file or"
              " node %s is not listed in the sched configuration file.", full_node_name.c_str());
  }

  RCLCPP_INFO(cb_prof_nh->get_logger(), "\nNode %d being profiled: %s\n", getpid(), full_node_name.c_str());

  perfProfileInit();


}

void NodeProfiler::perfProfileInit()
{
  auto event_id = -1;
  struct perf_event_attr pe;
  long fd;

  std::memset(&pe, 0, sizeof(struct perf_event_attr));
  pe.size = sizeof(struct perf_event_attr);
  pe.disabled = 1;
  pe.exclude_kernel = 1;
  pe.exclude_hv = 1;
  // pe.pinned = 0;
  pe.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;

  // HARDWARE EVENTS
  pe.type = PERF_TYPE_HARDWARE;

  pe.config = PERF_COUNT_HW_CPU_CYCLES;
  perfGroupFd = perf_event_open(&pe, 0, -1, -1, 0);
  if(perfGroupFd == -1) {
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Problem opening perf hw cpu cycles: %s", std::strerror(errno));
    // Consider throwing an exception or returning an error code here...
  }
  ioctl(perfGroupFd, PERF_EVENT_IOC_ID, &event_id);
  perfEventIDsMap.emplace(event_id, "cpu_cycles");

  //	pe.config = PERF_COUNT_HW_INSTRUCTIONS;
  //	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //	if(fd == -1)
  //		perror("Problem opening perf hw instructions");
  //	perfFileDescriptors.push_back(fd);
  //	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //	perfEventIDsMap.emplace(event_id, "instructions");
  //
  //	pe.config = PERF_COUNT_HW_CACHE_REFERENCES;
  //	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //	if(fd == -1)
  //		perror("Problem opening perf hw cache ref");
  //	perfFileDescriptors.push_back(fd);
  //	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //	perfEventIDsMap.emplace(event_id, "cache_references");
  //
  //	pe.config = PERF_COUNT_HW_CACHE_MISSES;
  //	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //	if(fd == -1)
  //		perror("Problem opening perf hw cache miss");
  //	perfFileDescriptors.push_back(fd);
  //	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //	perfEventIDsMap.emplace(event_id, "cache_misses");

  // SOFTWARE EVENTS
  pe.type = PERF_TYPE_SOFTWARE;

  pe.config = PERF_COUNT_SW_CPU_CLOCK;
  fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  if(fd == -1) {
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Problem opening perf sw cpu clock: %s", std::strerror(errno));
    // Consider handling the error...
  }
  perfFileDescriptors.push_back(fd);
  ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  perfEventIDsMap.emplace(event_id, "cpu_clock_time_ns");

  pe.config = PERF_COUNT_SW_TASK_CLOCK;
  fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  if(fd == -1) {
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Problem opening perf sw task clock: %s", std::strerror(errno));
    // Consider handling the error...
  }
  perfFileDescriptors.push_back(fd);
  ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  perfEventIDsMap.emplace(event_id, "task_clock_time_ns");

  //  pe.config = PERF_COUNT_SW_PAGE_FAULTS;
  //  fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //  if(fd == -1) {
  //    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Problem opening perf sw page faults: %s", std::strerror(errno));
  //    // Consider handling the error...
  //  }
  //  perfFileDescriptors.push_back(fd);
  //  ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //  perfEventIDsMap.emplace(event_id, "page_faults");

  // These can be added as well as SW:
  // PERF_COUNT_SW_CONTEXT_SWITCHES
  // PERF_COUNT_SW_CPU_MIGRATIONS

  //    // HARDWARE CACHE EVENTS
  //    pe.type = PERF_TYPE_HW_CACHE;
  //    pe.config = (PERF_COUNT_HW_CACHE_LL) |
  //            (PERF_COUNT_HW_CACHE_OP_READ << 8) |
  //            (PERF_COUNT_HW_CACHE_RESULT_MISS << 16);
  //    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //    if(fd == -1)
  //        perror("Problem opening perf hw llc cache read miss");
  //    perfFileDescriptors.push_back(fd);
  //    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //    perfEventIDsMap.emplace(event_id, "LL_cache_r_misses");

  //    pe.config = (PERF_COUNT_HW_CACHE_LL) |
  //            (PERF_COUNT_HW_CACHE_OP_WRITE << 8) |
  //            (PERF_COUNT_HW_CACHE_RESULT_MISS << 16);
  //    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
  //    if(fd == -1)
  //        perror("Problem opening perf hw llc cache write miss");
  //    perfFileDescriptors.push_back(fd);
  //    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
  //    perfEventIDsMap.emplace(event_id, "LL_cache_w_misses");

  assert(NUM_PERF_EVENTS == (perfFileDescriptors.size()+1));
  //PERF_FLAG_FD_OUTPUT
}

void NodeProfiler::profileStart(std::string callback_name){
  if(publishers_map.count(callback_name) == 0){
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Trying to profile unregistered callback: %s", callback_name.c_str());
    return;
  }

  if(profiling_now){
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Cannot start another profiling while one is running");
    return;
  }

  ioctl(perfGroupFd, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
  ioctl(perfGroupFd, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);

  last_cb_prof_info.start_stamp = convert_to_ros2_time(std::chrono::system_clock::now());

  profiling_now=true;
}

void NodeProfiler::profileEnd(std::string callback_name){


  if(publishers_map.count(callback_name) == 0){
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "Trying to profile unregistered callback: %s", callback_name.c_str());
    return;
  }

  last_cb_prof_info.end_stamp = convert_to_ros2_time(std::chrono::system_clock::now());

  ioctl(perfGroupFd, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
  struct read_format rf;
  auto ret = read(perfGroupFd, &rf, sizeof(rf));
  if(ret == -1)
    RCLCPP_ERROR(cb_prof_nh->get_logger(), "perf read error: %s", std::strerror(errno));
  else{
    for (auto i = 0; i < rf.nr; i++) {
      std::string str_of_id = perfEventIDsMap[rf.values[i].id];

      if (str_of_id.compare("cpu_cycles") == 0) {
        last_cb_prof_info.cpu_cycles = rf.values[i].value;
      }
//      else if (str_of_id.compare("instructions") == 0) {
//        cb_prof_info.instructions = rf.values[i].value;
//      }
      else if (str_of_id.compare("cpu_clock_time_ns") == 0) {
        last_cb_prof_info.cpu_clock_time_ns = rf.values[i].value;
      } else if (str_of_id.compare("task_clock_time_ns") == 0) {
        last_cb_prof_info.task_clock_time_ns = rf.values[i].value;
      } 
//      else if (str_of_id.compare("page_faults") == 0) {
//        cb_prof_info.page_faults = rf.values[i].value;
//      } else if (str_of_id.compare("cache_references") == 0) {
//        cb_prof_info.cache_references = rf.values[i].value;
//      } else if (str_of_id.compare("cache_misses") == 0) {
//        cb_prof_info.cache_misses = rf.values[i].value;
//      }
    }
  }

  last_cb_prof_info.period_id = period_id++;
  publishers_map[callback_name]->publish(last_cb_prof_info);
  profiling_now=false;
}


NodeProfiler::~NodeProfiler(){
  for(long fd : perfFileDescriptors)
    close(fd);
  close(perfGroupFd);
}
