"""Workspace entry point; run_door.sh selects the project interpreter."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "LeWM_Door_Handle_Manipulation"))

from door_task.demo import main


if __name__ == "__main__":
    main()
