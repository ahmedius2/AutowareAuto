#!/bin/bash

unset FIXED_RES_IDX
NUM=10

for S in $(seq 1 $NUM)
do
  #export LIDAR_DNN_MODEL="Pillarnet0100"
  #. run_aw.sh $S
  #export LIDAR_DNN_MODEL="MURAL_Pillarnet"
  #. run_aw.sh $S
  #export LIDAR_DNN_MODEL="Pillarnet0200"
  #. run_aw.sh $S
  export LIDAR_DNN_MODEL="Pillarnet0128"
  . run_aw.sh $S
done

#for RES_IDX in $(seq 0 2)
#do
#  export FIXED_RES_IDX=$RES_IDX
#  for S in $(seq 1 15); do . run_aw.sh $S; done
#  mv bags/MURAL_Pillarnet "bags/MURAL_Pillarnet_fixedres"${RES_IDX}
#done

unset FIXED_RES_IDX
