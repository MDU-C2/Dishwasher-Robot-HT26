# OAK detection and coordinate preview

## Contents

- detect_cups.py: OAK RGB/depth acquisition, YOLOv5 detection, target selection, display and recording.
- robot_coordinate_preview.py: load HT and calculate coordinate estimates. 
- models/: MARC YOLOv5 blob and matching JSON configuration.
- calibration/ht_diagnostic_preview.json: historical 25 September 16-point fit.
- test_robot_coordinate_preview.py: existing offline coordinate tests.

## Run

Use Python 3.11 with the versions in requirements.txt. Keep DepthAI 2.30.0.0.
From this folder in an activated environment:

```
python ./detect_cups.py
```

This opens OAK RGB and OAK Robot Coordinates windows. It does not connect to YuMi.
The red box selects the valid upright target with the smallest camera X (leftmost), not the smallest Z.
Purple marks project returned XYZ into RGB; they are not established grasp points.
Press lowercase q to quit, or r to request paired selected-target/image saving.
Mouse clicks save a separate depth probe. CSV files and images/ are saved beside the script.

## Calibration and limitations

Sampling -> HT fitting -> this module's coordinate preview.
The matrix is a local historical snapshot. New calibration results do not replace it automatically.
To preview a new result, explicitly change MATRIX_PATH in robot_coordinate_preview.py to the intended JSON containing T_diagnostic.
The existing regression test checks historical per-sample residuals and must use its matching dataset.

Marker-to-tool offset and coordinate-frame consistency still require verification.
RGB/detection sequence matching does not guarantee strict synchronization with displayed depth.
No detection, target selection, camera or projection algorithm was changed during packaging.
The only script change is the relocated matrix path.

## Offline tests

```
python -m unittest test_robot_coordinate_preview -v
```

The model comes from MARC-HT25: [https://github.com/MDU-C2/MARC-HT25](https://github.com/MDU-C2/MARC-HT25) .
This local copy does not establish model redistribution permissions. No GitHub upload was performed.
Historical images, CSV logs and backup scripts were not copied.