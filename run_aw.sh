#!/bin/bash
set -m  # Ensure the script uses job control
set -x

printf "Make sure AWSIM is running before executing this script!\n"

run_aw()
{
  AW_PARAMS="vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit
    map_path:=/home/humble/shared/autoware/src/additional/shinjuku_map/map
    rviz:=false"

  rm -rf ~/.ros/log/* aw_logs.txt
  ros2 launch autoware_launch e2e_simulator.launch.xml $AW_PARAMS | tee aw_logs.txt &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  ros2_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
  echo "autoware started with PID $ros2_pid and PGID $ros2_pgid"
}

run_bag_record()
{
  rm -rf awsim_exp_rosbag2
  TOPICS="/awsim/ground_truth/on_collision /awsim/ground_truth/vehicle/pose"
  ros2 bag record -o awsim_exp_rosbag2 $TOPICS < /dev/null &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  ros2_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
}

send_goal_pose()
{
  printf "*********************************************************************\n"
  printf "************************ SENDING GOAL POSE **************************\n"
  printf "*********************************************************************\n"
  ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped "{
      header: {
          stamp: {sec: 0, nanosec: 0},
          frame_id: 'map'
      },
      pose: {
          position: {x: 81562.796875, y: 50591.6171875, z: 0},
          orientation: {x: 0, y: 0, z: 0.08980559531591707, w: 0.9959593139531121}
      }
  }"
}


# 
run_aw
AW_PGID=$ros2_pgid

# Reset the simulation
ros2 topic pub -1 /awsim/traffic_reset std_msgs/msg/String "data: 'reload'"

#sleep 200
# wait for autoware to initialize, it should be done one the message comes
ros2 topic echo "/perception/object_recognition/detection/valor/validation/objects" \
  autoware_perception_msgs/msg/DetectedObjects --once

send_goal_pose 
sleep 20 # wait for planning

#Ideal is to do the Engage immadiately after reset, but it can take a few seconds
chrt -f 90 ros2 topic pub -1 /awsim/traffic_reset std_msgs/msg/String "data: '150_150'"
chrt -f 90 ros2 topic pub -1 /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}'
run_bag_record
RECORD_PGID=$ros2_pgid

set +x
sleep 300
# if the mode field of /api/operation_mode/state becomes 1, it reached the destination 

kill -SIGINT -$RECORD_PGID
kill -SIGINT -$AW_PGID

# wait pid?
sleep 20
printf "Done!"
