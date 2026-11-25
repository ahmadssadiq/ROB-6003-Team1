# ROB-GY 6003 Team 1: RoomRunner and assignments

This repository holds our assignments for ROB-GY 6003 Foundations of Robotics (NYU) and our final
project, **RoomRunner**: an LLM-planned fetch-and-place robot built on the LeKiwi mobile manipulator.

| Folder | What it is |
| --- | --- |
| [`roomrunner/`](roomrunner/) | final project: ROS 2 packages (see below) |
| [`docs/`](docs/) | project architecture, node and topic diagram, frames and transforms |
| [`experiments/`](experiments/) | loggers and analysis scripts for experiments E1, E2 and pickup |
| [`scripts/`](scripts/) | workspace setup |
| [`tests/`](tests/) | repository-level tests (keeps the assignment tests running) |
| `assignment_02.py` | Assignment 02: SO-101 forward and inverse kinematics |
| [`assignment_03/`](assignment_03/) | Assignment 03: ROS 2 forward kinematics, EEControl (IK) node, keyboard typing |
| [`assignment_04/`](assignment_04/) | Assignment 04: ArUco detection and particle filter localization |

## RoomRunner

A user types a request such as "bring the red block to the tray". The robot turns it into a task with an
LLM, plans a path with A* on a map from an overhead camera, drives there while localizing with a particle
filter (wheel odometry and ArUco markers on the walls), picks up the block with the SO-101 arm, drives to
the drop-off, releases it and reports success.

### Hardware

- LeKiwi mobile base (three omniwheels) with an SO-101 follower arm and a front camera, driven through
  LeRobot from a BeeLink mini PC (Ubuntu 24.04, ROS 2 Jazzy)
- SO-101 leader arm (for teleoperation and recording demonstrations)
- an overhead camera fixed above the room, looking down
- ArUco markers on the walls (localization) and on the objects (blocks, tray)

### Packages and owners

| Package | Owner | Contents |
| --- | --- | --- |
| `roomrunner_interfaces` | shared | messages, services and actions |
| `roomrunner_bringup` | shared | launch files (stubs, simulation, real robot) and configuration |
| `roomrunner_planning` | Ahmad Sadiq | LLM task parser and validator, LLM+P, A*, mission state machine |
| `roomrunner_perception` | Aditya Mourya | overhead camera calibration, occupancy grid, object poses |
| `roomrunner_localization` | Aditya Mourya | particle filter reuse (Assignment 04), ArUco map to planning map transform |
| `roomrunner_control` | Aditya Mourya | waypoint follower for the LeKiwi base |
| `roomrunner_manipulation` | Aditya Sarvanagupta | arm interface (Assignment 03 EEControl), scripted pick and place, demonstrations, imitation learning |

See [docs/architecture.md](docs/architecture.md) for every node, topic and frame.

### Install

The packages build in the course workspace `~/FOR26/ros_ws` (ROS 2 Jazzy, conda environment `ros_mujoco`):

```bash
bash scripts/setup_workspace.sh
```

It links `roomrunner/` into `~/FOR26/ros_ws/src/` and builds. After that, in every new terminal:

```bash
conda activate ros_mujoco && source ~/FOR26/ros_ws/install/setup.bash
```

### Run

```bash
ros2 launch roomrunner_bringup stubs.launch.py      # everything with fake data, no hardware
```

Simulation and real-robot launch files are added as the modules are built.

### LLM key

Copy `.env.example` to `.env` and fill in the key. `.env` is ignored by git; never commit a key.
Without a key the task parser runs in mock mode.

### Working together

See [CONTRIBUTING.md](CONTRIBUTING.md): one branch per owner, merged into `main` through pull requests.

---

## Assignment 02 (SO-101 kinematics)

`assignment_02.py` implements forward kinematics and numerical position inverse
kinematics. Its only external Python dependencies are NumPy and SciPy; `math`
is part of Python. The script includes its own test runner, so pytest is not
required.

### Create the environment once

Open **Anaconda Prompt**, then run:

```bat
cd /d C:\Projects\S101_arm
conda env create -f environment.yml
```

The environment is named `s101-arm`. The definition selects Python 3.11 and
pins NumPy 1.26.4 and SciPy 1.11.4 to the versions checked with this assignment.

### Run whenever needed

```bat
cd /d C:\Projects\S101_arm
conda activate s101-arm
python assignment_02.py
```

Expected result: `FK: 5/5 passed` and `IK: 5/5 passed`, with exit code 0.
When finished, use `conda deactivate`.

You can also run without activating first:

```bat
conda run -n s101-arm python assignment_02.py
```

For an editor or IDE, select the Python interpreter belonging to `s101-arm`.
If you change dependencies later, edit `environment.yml` and apply them with:

```bat
conda env update -n s101-arm -f environment.yml --prune
```

These commands follow the [Conda environment documentation](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html).

### Verification and model inputs

All 10 bundled tests passed using the existing Anaconda installation with
Python 3.11.7, NumPy 1.26.4, and SciPy 1.11.4. Creating a separate environment
in the automated session was blocked by network/proxy access; the offline
package cache also lacked the NumPy package record. Run the creation command
above from a terminal with access to the Conda package repositories.

Tests use synthetic dimensions. To evaluate your robot, set `ROBOT_LENGTHS`
to the 11 measured lengths in metres, or pass `lengths=` explicitly. Joint
angles are in radians. The model follows the supplied frame assumptions and
has not been validated against a physical SO-101 robot.