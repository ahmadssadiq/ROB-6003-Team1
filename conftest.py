"""pytest setup for the whole repository: run `python3 -m pytest` in the repository root.

Makes the RoomRunner packages importable without building them, so their pure-Python modules can be
tested directly. Use the system Python (/usr/bin/python3), which has pytest, NumPy and SciPy.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for package in sorted((ROOT / "roomrunner").glob("roomrunner_*")):
    if (package / package.name / "__init__.py").exists():
        sys.path.insert(0, str(package))
