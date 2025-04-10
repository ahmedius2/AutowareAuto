#!/bin/bash
set -m  # Ensure the script uses job control
set -x

printf "Make sure AWSIM is running before executing this script!\n"

run_aw()
{
  AW_PARAMS="vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit
    map_path:=/home/humble/shared/autoware/src/additional/shinjuku_map/map
    rviz:=false perception_mode:=lidar_dnn_only"

#  AW_PARAMS="$AW_PARAMS planning:=false control:=false"

  rm -rf ~/.ros/log/* aw_logs.txt
  chrt -r 90 ros2 launch autoware_launch e2e_simulator.launch.xml $AW_PARAMS | tee aw_logs.txt &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  ros2_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
  echo "autoware started with PID $ros2_pid and PGID $ros2_pgid"
}

run_bag_record()
{
  rm -rf awsim_exp_rosbag2
  TOPICS="/awsim/ground_truth/on_collision
    /awsim/object_recognition/matched_objects
    /perception/object_recognition/objects
    /vehicle/status/velocity_status
    /system/operation_mode/state
    /perception/object_recognition/detection/obstacle_pointcloud_based_validator_node/exec_time_ms
    /perception/object_recognition/detection/valor/lidar_objdet_valor_dnn/execution_time_ms
    /perception/object_recognition/detection/valor/pc_acc_for_dnn_node/exec_time_ms
    /perception/object_recognition/prediction/map_based_prediction/exec_time_ms
    /perception/object_recognition/tracking/multi_object_tracker/exec_time_ms"
  #/perception/object_recognition/detection/objects
  #/awsim/ground_truth/perception/object_recognition/detection/objects
  chrt -r 90 ros2 bag record -o awsim_exp_rosbag2 $TOPICS < /dev/null &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  ros2_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
}

send_goal_pose()
{
  printf "*********************************************************************\n"
  printf "************************ SENDING GOAL POSE **************************\n"
  printf "*********************************************************************\n"
  chrt -f 91 ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped "{
      header: {
          stamp: {sec: 0, nanosec: 0},
          frame_id: 'map'
      },
      pose: {
          position: {x: 81562.796875, y: 50591.6171875, z: 0},
          orientation: {x: 0, y: 0, z: 0.08980559531591707, w: 0.9959593139531121}
      }
  }"
#          position: {x: 81376.3, y: 50128.1 , z: 0},
#          orientation: {x: 0, y: 0, z: 0.76063, w: 0.649186}
}

#perception evaluator config file:
#ros2 launch perception_online_evaluator perception_online_evaluator.launch.xml config_file_path:=/home/humble/shared/AutowareAuto/src/universe/autoware.universe/evaluator/perception_online_evaluator/param/perception_online_evaluator.awsim.yaml

run_aw
AW_PGID=$ros2_pgid

# Reset the simulation
chrt -f 91 ros2 topic pub -1 /awsim/traffic_reset std_msgs/msg/Header "{frame_id: 'reload'}"

# wait for autoware to initialize, it should be done one the message comes
chrt -r 91 ros2 topic echo /perception/object_recognition/detection/objects \
  autoware_perception_msgs/msg/DetectedObjects --once

send_goal_pose 
sleep 20 # wait for planning

chrt -r 91 python scripts/reset_and_engage.py # syncronizes to be done in future

run_bag_record
RECORD_PGID=$ros2_pgid

set +x

# if the mode field of /api/operation_mode/state becomes 1, it reached the destination 
#sleep 360
#chrt -91 kill -SIGINT -$RECORD_PGID
#chrt -91 kill -SIGINT -$AW_PGID

# wait pid?
#sleep 20
#printf "Done!"

#KILL:
#ps -ef | grep -i ros | awk {'print $2'} | xargs chrt -r 91 kill -9
