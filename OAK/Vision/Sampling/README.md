# OAK ArUco Sampling

Collect camera XYZ coordinates from the centre of an ArUco marker for later pairing with robot coordinates and fitting an HT matrix. This program does not connect to or move the robot.

## Requirements

- Python 3.11
- DepthAI **2.30.0.0** (keep this version; do not upgrade to 3.x)
- NumPy 2.2.6
- opencv-python 4.12.0.88 (OpenCV 4.12.0 with ArUco support)
- A connected OAK-D Pro and a printed **DICT_6X6_250, ID 23** marker

## Run

Activate an environment with the dependencies above, open a terminal in this folder, and run:

```powershell
python .\aruco_center_sampling.py
```

Keep all three Python files in this folder:

- `aruco_center_sampling.py`: automatic marker-centre selection and sample saving.
- `mouse_depth_sampling.py`: shared camera, depth and recording functions; no separate launch is needed.
- `aruco_rgb.py`: ArUco and camera-parameter helpers; no separate launch is needed.

With the image window focused, use lowercase **s** to save, **Space** to freeze/resume, and **q** to quit. Keep the robot stationary while recording each camera/robot pair. Missing or ambiguous target markers and invalid depth are rejected.

## Data

New runs create a timestamped folder under `aruco_center_sessions/`. The included 16-sample dataset is currently stored directly inside that directory.

Each sample contains:
- `sample.json`: camera XYZ in millimetres and measurement details.
- `rgb.png` and `marked.png`: original and annotated images.
- `depth.npy`: the saved depth array in millimetres; zero indicates invalid depth.

`session.json` records camera settings, intrinsics and distortion coefficients read from the connected OAK. No separate hardcoded camera-parameter file is required. `index.csv` indexes the samples. For the included dataset, `points.txt` contains robot targets paired as sample 1 = HOMEpos and samples 2–16 = s1–s15. Robot coordinates are supplied separately, not automatically read by this sampling program.

Primary XYZ comes from stereo depth and back-projection using distortion-corrected image directions, not from ArUco PnP translation. Included HT/diagnostic files are results from a separate fitting step, not inputs to sampling. Sampling has been run on hardware; relocation/import checks passed, but this does not certify calibration accuracy. The marker-centre/tool-point offset remains to be resolved.
