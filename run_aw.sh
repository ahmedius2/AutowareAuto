#!/bin/bash
set -m  # Ensure the script uses job control

export PMODE="pmode_0002"
#export LIDAR_DNN_MODEL="MURAL_Pillarnet"
#export LIDAR_DNN_MODEL="Pillarnet0100"
export LIDAR_DNN_DELEGATE_GT=0 # The model won't matter, NOTE that prediction is not working well
unset FIXED_RES_IDX
#export FIXED_RES_IDX=4
export IGNORE_DL_MISS=0 # used by lidar dnn, make it 0 when using mural!
export ASSUME_ALL_TLIGHTS_ARE_GREEN=1

TARGET_DIR=bags/$LIDAR_DNN_MODEL
mkdir -p $TARGET_DIR
TARGET_DIR=$TARGET_DIR/exp"$1"
if test -d $TARGET_DIR; then
  printf "Skipping $TARGET_DIR\n"
  return
fi
#rm -rf $TARGET_DIR

. cleanup_shm.sh

printf "Make sure AWSIM is running before executing this script!\n"

run_aw()
{
  AW_PARAMS="vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit
    map_path:=/home/a249s197/work/AutowareAuto/src/additional/shinjuku_map/map
    rviz:=true perception_mode:=lidar_dnn_only use_sim_time:=true"

#  AW_PARAMS="$AW_PARAMS planning:=false control:=false"

  rm -rf ~/.ros/log/* aw_logs.txt
  #DONT_LAUNCH_VALOR="true" 
  #nice --20 taskset 00ff ros2 launch autoware_launch e2e_simulator.launch.xml $AW_PARAMS | tee aw_logs.txt &
  nice --20 taskset 00ff ros2 launch autoware_launch e2e_simulator.launch.xml $AW_PARAMS | tee aw_logs.txt &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  aw_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
  echo "autoware started with PID $ros2_pid and PGID $ros2_pgid"
}

run_bag_record()
{
  rm -rf awsim_exp_rosbag2
  TOPICS="/awsim/object_recognition/matched_objects
    /vehicle/status/velocity_status
    /control/command/control_cmd
    /system/operation_mode/state
    /perception/object_recognition/detection/obstacle_pointcloud_based_validator_node/exec_time_ms
    /perception/object_recognition/detection/valor/pc_acc_for_dnn_node/exec_time_ms
    /perception/object_recognition/detection/valor/lidar_objdet_valor_dnn/exec_time_ms
    /perception/object_recognition/prediction/map_based_prediction/exec_time_ms
    /perception/object_recognition/tracking/multi_object_tracker/exec_time_ms
    /perception/object_recognition/detection/valor/lidar_objdet_valor_dnn/selected_res_idx
    /awsim/object_distances
    /awsim/trigger_events"

  #/control/autonomous_emergency_braking/info/markers
  #/perception/object_recognition/objects
  #/control/command/emergency_cmd
  #/awsim/ground_truth/on_collision
  #/perception/object_recognition/detection/objects
  #/awsim/ground_truth/perception/object_recognition/detection/objects
  #nice --20 taskset 00ff ros2 bag record -o $TARGET_DIR $TOPICS < /dev/null &
  nice --20 taskset 00ff ros2 bag record -o $TARGET_DIR $TOPICS < /dev/null &
  ros2_pid=$!  # Capture the PID of the ros2 launch process
  rosbag_pgid=$(ps -o pgid= -p $ros2_pid | grep -o '[0-9]*')
}

send_goal_pose()
{
  printf "*********************************************************************\n"
  printf "************************ SENDING GOAL POSE **************************\n"
  printf "*********************************************************************\n"
  nice --20 ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped "{
      header: {
          stamp: {sec: 0, nanosec: 0},
          frame_id: 'map'
      },
      pose: {
          position: {x: $1, y: $2, z: 0},
          orientation: {x: 0, y: 0, z: $3, w: $4}
      }
  }"
}

source_ros2
run_aw
AW_PGID=$ros2_pgid

# Reset the simulation
nice --20 ros2 topic pub -1 /awsim/traffic_reset std_msgs/msg/Header "{frame_id: 'reload'}"

# wait for autoware to initialize, it should be done one the message comes
#ros2 topic echo /perception/object_recognition/detection/objects \
#  autoware_perception_msgs/msg/DetectedObjects --once
sleep 60 # wait for autoware to initialize

#send_goal_pose 81597.9 50207.1 0.0600405 0.998196 # end to end
run_bag_record $1
printf "Sending goal pose 1\n"
send_goal_pose 81403.1 49968.5 0.758414 0.651773 # after crowded area
sleep 2
nice --20 python scripts/reset_and_engage.py engage 2
sleep 10 # this might be needed
nice --20 python scripts/reset_and_engage.py waitstop

printf "Sending goal pose 2\n"
send_goal_pose 81400.1 50170.6 0.068659 0.99764 # after truck
sleep 2
nice --20 python scripts/reset_and_engage.py engage 2
sleep 10
nice --20 python scripts/reset_and_engage.py waitstop

printf "Sending goal pose 3\n"
send_goal_pose 81597.9 50207.1 0.0600405 0.998196 # after jaycar
sleep 2
nice --20 python scripts/reset_and_engage.py engage 2
sleep 10
nice --20 python scripts/reset_and_engage.py waitstop

printf "Sending goal pose 4\n"
send_goal_pose 81717.4 50229.2 0.0626653 0.998035 # after jaywalker
sleep 2
nice --20 python scripts/reset_and_engage.py engage 2
sleep 10
nice --20 python scripts/reset_and_engage.py waitstop

# stop bag recorder and aw
kill -SIGINT -$rosbag_pgid
sleep 5
kill -SIGINT -$aw_pgid
sleep 10

kill %1
kill %2
kill %3
kill %4
pkill ros

#KILLALL:
#ps -ef | grep -i ros | awk {'print $2'} | xargs chrt -r 91 kill -9
