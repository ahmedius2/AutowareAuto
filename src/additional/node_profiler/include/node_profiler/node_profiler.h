// header
//

#ifndef NODE_PROFILER_H 
#define NODE_PROFILER_H

#include <list>
#include <chrono>
#include <map>
#include <functional>
#include <atomic>
#include <chrono>
#include <memory>
#include <semaphore.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "callback_profile/msg/callback_profile.hpp"

#define NUM_PERF_EVENTS 3

class NodeProfiler
{
  public:
    NodeProfiler(rclcpp::Node* node_handle, std::vector<std::string> callback_names);

    void profileStart(std::string callback_name);
    void profileEnd(std::string callback_name); // publishes callback profile automatically

    ~NodeProfiler();

    // the config file must be a comma seperated csv file
    const char* config_file_path="/home/humble/aw_sched_conf.csv";

  private:
    //OperationMode op_mode_;
    void perfProfileInit();

    static long
      perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
          int cpu, int group_fd, unsigned long flags)
      {
        return syscall(__NR_perf_event_open, hw_event, pid, cpu,
            group_fd, flags);
      }
    struct read_format {
      uint64_t nr;
      struct {
        uint64_t value;
        uint64_t id;
      } values[NUM_PERF_EVENTS];
    };

    rclcpp::Node* cb_prof_nh;
    long perfGroupFd;
    int period_id;
    std::vector<long> perfFileDescriptors;
    std::map<uint64_t, std::string> perfEventIDsMap;
    std::map<std::string, std::shared_ptr<rclcpp::Publisher<callback_profile::msg::CallbackProfile>>> publishers_map;
    bool profiling_now;
    callback_profile::msg::CallbackProfile last_cb_prof_info;
//    std::chrono::time_point<std::chrono::steady_clock> last_start_tp;
};

#endif 

