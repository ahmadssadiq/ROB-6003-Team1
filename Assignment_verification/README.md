# SO-101 assignment verification

Run in an extracted folder, not inside the ZIP:

```bat
python -m pip install -r requirements.txt
python assignment_verification.py
```

The folder includes a complete snapshot of your latest assignment_02.py with
confirmed T23=[0.1126,0,0.0280] and T67=[-0.0432,0,0.0752]. The verification
script imports it. To test later edits, replace that copy with your edited
assignment_02.py. Do not paste partial preview text into either Python file.
The verification script does not modify your assignment.

## What was actually executed

The robot designer's official SO-101 URDF is evaluated by the independently
installed open-source yourdfpy 0.0.60 FK engine. No assignment transformation
function computes the reference FK. Mesh loading is disabled; no robot,
meshes, ROS, or motor connection is needed. NumPy and SciPy are also required.

Hugging Face's official LeRobot kinematics.py was downloaded and inspected.
An unmodified copy is included for provenance, but is NOT imported or executed
by this verifier. It uses Placo and accepts DEGREES, whereas this verification
script and your assignment use RADIANS. yourdfpy provides the runnable FK
reference here. This is not a claim that we executed the LeRobot/Placo backend.

## Results and meaning

The included verification_report.txt records this run; CSV has all inputs and
positions, and JSON records the mapping and summary. 57 configurations:
* Maximum matching-point position error: 0.246103 mm.
* Maximum orientation error: 0.001108 degrees.
* 57/57 meet the explicitly chosen 0.5 mm and 0.01 degree tolerances.

These results are CONDITIONAL geometric agreement, not proof of hardware
calibration, joint-zero choices, collision-free motion, or fingertip location.
The reference model itself is CAD-derived, not ground-truth measurements.

## Why the same numbers cannot simply be used

The assignment assumes adjacent axes coincide at zero; the official URDF
contains fixed rotations and a different zero convention. The candidate map is

q_urdf = [-1,+1,+1,+1,-1] * q_assignment + b

b in degrees = [0, 62.06765567, -149.86016406, -2.20749161, 0].

These offsets were derived from the link directions, NOT fitted by minimizing
end-effector errors over the tested poses:

beta = atan2(0.1126,0.0280) - atan2(0.0280,0.11257)
gamma = -atan2(0.1349,0.0052)
b = [0,beta,gamma-beta,-pi/2-gamma,0].

The assignment's zero pose maps to an elbow angle outside the URDF limits.
That does not invalidate a mathematical frame-zero definition, but it means
zero cannot be assumed to be a realizable physical configuration. Actual test
cases are chosen INSIDE official URDF joint limits and converted to assignment
angles. No motor calibration is inferred from these equations.

A constant tool-axis rotation is obtained at the nominal zero pose and tested
at the other poses. No constant POSITION offset is fitted to the outputs.
For base alignment we request the assignment's base_position=(0,0,0), psi=0,
so both are expressed relative to the robot mounting base.

## Endpoint distinction

Your fixed final translations cancel their x offsets and put frame 7 at
(0,0,+0.0981) relative to assignment frame 5, on the wrist-roll axis.
The reference compares the nominal equivalent virtual point
(0,0,-0.0981) expressed in URDF gripper_link. The sign is due to different axes.

The official gripper_frame_link has translation
(-0.0079,-0.000218121,-0.0981274) in gripper_link. It is about 7.9 mm off the
roll axis and is NOT your on-axis point. Its 7.68--8.13 mm distance from your
prediction is reported separately; it is not a same-point error and should
not be interpreted as an FK implementation failure. To verify the physical
fingertip, first identify its exact pose relative to gripper_link.

## Remaining model differences

The assignment rounds dimensions and drops small lateral offsets. The URDF
forearm vector includes a 5.2 mm sideways-in-plane offset. The angle mapping
absorbs its direction into the joint-zero convention but not its length:
sqrt(0.1349^2+0.0052^2) differs from 0.1349 m. Those approximations account for
small residual errors. The derivation assumes near-orthogonal CAD rotations;
the URDF's rounded pi values contribute micro-radian orientation residuals.

## Sources and reproducibility

Official URDF (TheRobotStudio; Apache-2.0):
https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101/so101_new_calib.urdf

Official model documentation:
https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101/README.md

Official LeRobot code (Hugging Face; Apache-2.0; included, not executed):
https://github.com/huggingface/lerobot/blob/8c920c4270460851cedd2737657584586d3dc66f/src/lerobot/model/kinematics.py

Executed external FK library:
https://github.com/clemense/yourdfpy
https://yourdfpy.readthedocs.io/en/latest/api/yourdfpy.html

The reference files retain their original contents; Apache license text is
included under reference/LICENSE_SO_ARM100 (the same license applies to the
included LeRobot source). The run writes URDF SHA-256 to the summary JSON.
The verification script and this explanation are newly written for this task.
