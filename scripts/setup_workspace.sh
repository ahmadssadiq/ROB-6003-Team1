#!/bin/bash
# Link the RoomRunner packages into the course workspace and build them.
#   bash scripts/setup_workspace.sh
# Assumes the course workspace at ~/FOR26/ros_ws (Assignments 03 and 04) and the conda environment ros_mujoco.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="${ROS_WS:-$HOME/FOR26/ros_ws}"

ln -sfn "$REPO/roomrunner" "$WS/src/roomrunner"
echo "Linked $WS/src/roomrunner -> $REPO/roomrunner"

source /opt/ros/jazzy/setup.bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_mujoco
cd "$WS"
colcon build --packages-up-to roomrunner_bringup
echo
echo "Done. In every new terminal: conda activate ros_mujoco && source $WS/install/setup.bash"
echo "Try it:                     ros2 launch roomrunner_bringup stubs.launch.py"
