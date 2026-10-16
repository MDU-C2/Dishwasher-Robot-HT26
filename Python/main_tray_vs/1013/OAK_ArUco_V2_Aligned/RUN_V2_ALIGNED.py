"""Explicit entry point for the V2 alignment repair (avoids launching stale V2/V3)."""
from pathlib import Path
import runpy

if __name__ == "__main__":
    filename = Path(__file__).resolve().with_name("aruco_center_sampling.py")
    print("RUN_V2_ALIGNED.py ->", filename, flush=True)
    runpy.run_path(str(filename), run_name="__main__")
