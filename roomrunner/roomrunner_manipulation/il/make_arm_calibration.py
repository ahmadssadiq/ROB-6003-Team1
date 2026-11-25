"""Make an SO-101 follower calibration from the LeKiwi calibration, for recording arm demonstrations.

lerobot-record can record the arm alone (robot.type=so101_follower), but looks for the calibration under
so_follower/<id>.json with joint names without the "arm_" prefix. The arm's calibration already exists inside
lekiwi.json (from Assignment 01). This copies the six arm joints over; the wheels are left out.

    python make_arm_calibration.py            # writes .../robots/so_follower/lekiwi_arm.json
Then use --robot.id=lekiwi_arm in lerobot-teleoperate and lerobot-record. Existing files are never overwritten.
"""
import json
import sys
from pathlib import Path

ROOT = Path.home() / ".cache/huggingface/lerobot/calibration/robots"
source = ROOT / "lekiwi" / "lekiwi.json"
target = ROOT / "so_follower" / "lekiwi_arm.json"

if target.exists():
    sys.exit(f"{target} already exists; not overwriting.")
lekiwi = json.loads(source.read_text())
arm = {name.removeprefix("arm_"): values for name, values in lekiwi.items() if name.startswith("arm_")}
if len(arm) != 6:
    sys.exit(f"expected 6 arm joints in {source}, found {sorted(arm)}")
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(arm, indent=4))
print(f"Wrote {target} with joints {list(arm)}")
