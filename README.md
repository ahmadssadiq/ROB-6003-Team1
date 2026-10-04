# S101 arm assignment

`assignment_02.py` implements forward kinematics and numerical position inverse
kinematics. Its only external Python dependencies are NumPy and SciPy; `math`
is part of Python. The script includes its own test runner, so pytest is not
required.

## Create the environment once

Open **Anaconda Prompt**, then run:

```bat
cd /d C:\Projects\S101_arm
conda env create -f environment.yml
```

The environment is named `s101-arm`. The definition selects Python 3.11 and
pins NumPy 1.26.4 and SciPy 1.11.4 to the versions checked with this assignment.

## Run whenever needed

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

## Verification and model inputs

All 10 bundled tests passed using the existing Anaconda installation with
Python 3.11.7, NumPy 1.26.4, and SciPy 1.11.4. Creating a separate environment
in the automated session was blocked by network/proxy access; the offline
package cache also lacked the NumPy package record. Run the creation command
above from a terminal with access to the Conda package repositories.

Tests use synthetic dimensions. To evaluate your robot, set `ROBOT_LENGTHS`
to the 11 measured lengths in metres, or pass `lengths=` explicitly. Joint
angles are in radians. The model follows the supplied frame assumptions and
has not been validated against a physical SO-101 robot.
