#!/bin/bash
# L1: two simulated runs of a 1.5 m square driven twice, with an unknown start (particles spread over
# the room) and a known start (particles around the true pose). Writes data/a4_l1_sim/*.csv.
# Runs on its own ROS domain (63) so that it does not disturb a robot session on domain 20.
source /opt/ros/jazzy/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_mujoco
source ~/FOR26/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OUT=~/FOR26/ros_ws/data/a4_l1_sim; mkdir -p $OUT
cd ~/FOR26/Assignment4/Localization/scripts
for run in unknown_start known_start; do
  echo "=== $run"
  setsid ros2 launch aruco_localization localization.launch.py rviz:=false viewer:=false > $OUT/$run.launch.log 2>&1 &
  LAUNCH=$!
  sleep 15
  if [ $run = known_start ]; then
    ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.003]}}" > /dev/null
    sleep 1
  fi
  python record_localization.py $OUT/$run.csv --duration 115 > $OUT/$run.record.log 2>&1 &
  REC=$!
  sleep 2
  python drive_pattern.py square --size 1.5 --laps 2 > $OUT/$run.drive.log 2>&1
  wait $REC
  kill -INT -- -$LAUNCH; sleep 8; kill -KILL -- -$LAUNCH 2>/dev/null; sleep 2
  echo "done $run: $(wc -l < $OUT/$run.csv) rows"
done
# Plot: python ~/FOR26/ros_ws/tools/plot_a4.py l1 a4_l1_sim
