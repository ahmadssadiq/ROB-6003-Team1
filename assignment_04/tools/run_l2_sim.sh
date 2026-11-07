#!/bin/bash
# L2 in simulation: marker errors against distance while driving a 1.5 m square twice, after one turn
# on the spot so that the filter has converged. Writes data/a4_l2_sim/marker_errors.csv.
# Runs on its own ROS domain (63) so that it does not disturb a robot session on domain 20.
source /opt/ros/jazzy/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_mujoco
source ~/FOR26/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OUT=~/FOR26/ros_ws/data/a4_l2_sim; mkdir -p $OUT
cd ~/FOR26/Assignment4/Localization/scripts
setsid ros2 launch aruco_localization localization.launch.py rviz:=false viewer:=false > $OUT/launch.log 2>&1 &
LAUNCH=$!
sleep 15
python drive_pattern.py spin > $OUT/converge.log 2>&1
python record_marker_errors.py $OUT/marker_errors.csv --duration 115 > $OUT/record.log 2>&1 &
REC=$!
sleep 2
python drive_pattern.py square --size 1.5 --laps 2 > $OUT/drive.log 2>&1
wait $REC
kill -INT -- -$LAUNCH; sleep 8; kill -KILL -- -$LAUNCH 2>/dev/null
echo "done: $(wc -l < $OUT/marker_errors.csv) rows"
# Plot: python ~/FOR26/ros_ws/tools/plot_a4.py l2 a4_l2_sim
