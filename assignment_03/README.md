# Assignment 3: kinematics and keyboard typing on the LeKiwi

Forward and inverse kinematics for the LeKiwi arm as ROS 2 nodes, used to type
"pool" on a real keyboard. Built on the course repository
[FOR26](https://github.com/Rooholla-KhorramBakht/FOR26), which provides the
simulator (`lekiwi_sim`), the real-robot driver (`lekiwi_ros`) and the starter package.

## Contents

| Path | What it is |
| --- | --- |
| `arm_control/` | ROS 2 package: `kinematics.py` (FK and damped-least-squares IK), the `forward_kinematics`, `inverse_kinematics` (EEControl) and `keyboard_typer` nodes, launch files for simulation, the real robot and RViz |
| `hello_ros/` | the leader/follower exercise from the ROS guide |
| `ee` | sends the fingertip to a named position (`./ee home`, `./ee p`, `./ee o`, `./ee l`) |
| `typeword` | types a word with `ee`: `./typeword pool` |
| `tools/record.py` | records the topics to `data/<run>/*.csv` |
| `tools/measure_keys.py` | measures the key positions with the robot's own FK and writes them into `ee` |
| `tools/plot.py` | makes the report plots from `data/` |
| `data/` | the recorded runs and their plots |
| `report/` | the IEEE report (LaTeX; upload the folder to Overleaf) |

## Running it

Copy `arm_control` (and `hello_ros`) into `~/FOR26/ros_ws/src/`, and `ee`,
`typeword` and `tools/` into `~/FOR26/ros_ws/`. Then build:

```bash
conda activate ros_mujoco
cd ~/FOR26/ros_ws && colcon build && source install/setup.bash
```

Simulation (type `a`, `s` or `d`):

```bash
ros2 launch arm_control typing_sim.launch.py
ros2 topic pub --once /key std_msgs/msg/String "{data: a}"
```

Real robot:

```bash
ros2 launch arm_control typing_real.launch.py port:=/dev/serial/by-id/<board> enable_motors:=true
./typeword pool
```

The key positions in `ee` are for one placement of the robot and keyboard. After
moving either, run `python tools/measure_keys.py` with the motors off.

Plots (system Python with matplotlib):

```bash
/usr/bin/python3 tools/plot.py fk    sim_fk_validation
/usr/bin/python3 tools/plot.py keys  sim_typing
/usr/bin/python3 tools/plot.py speed sim_speed
/usr/bin/python3 tools/plot.py real  real_pool_1
```
