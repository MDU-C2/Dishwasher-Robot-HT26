# Dishwasher Robot Vision — Initial Experimental Release

OAK-D Pro sampling, HT estimation and YOLOv5 detection/coordinate preview for the ABB YuMi dishwasher robot project.

## Modules

1. [Sampling](Sampling/README.md): acquire ArUco-centre camera coordinates and save paired RGB/depth records. The included campaign contains 16 samples and separately supplied robot targets.
2. [Calibration](Calibration/README.md): fit a rigid transform from those pairs and export camera_to_robot.json plus HT_matrix.txt.
3. [Detection](Detection/README.md): detect cups, select a target and display camera XYZ and robot-coordinate estimates.

Workflow: Sampling -> Calibration -> HT -> Detection coordinate preview. Detection includes a historical matrix snapshot; a new calibration result is not loaded automatically. Follow its README to select another result.

Use Python 3.11. Keep DepthAI 2.30.0.0; the module READMEs and requirements files specify dependencies and commands. No module in this package commands robot motion.

## Validation status

The original sampling and fitting workflow was used in initial physical grasp trials. This packaged release passed Python syntax and JSON checks, sampling imports, two coordinate-preview tests, depth-array/data completeness checks, and offline reproduction of the 16-point matrix. JSON and TXT exports agree with the matrix bundled in Detection.

The relocated detection and sampling programs have not been rerun on hardware during packaging. Marker-to-tool offset, independent calibration accuracy and repeatable grasp performance remain unresolved. This is an experimental release, not a validated autonomous pick-and-place system.

## Included data and attribution

Sampling includes 16 RGB/annotated image pairs, depth arrays, metadata and robot-point definitions. Uploading this directory will share those experiment records. They are intentionally not excluded by .gitignore.

The YOLOv5 model and its configuration originate from [MARC-HT25](https://github.com/MDU-C2/MARC-HT25). Attribution does not establish redistribution permission; the upstream model licensing status has not been verified here. No new license is assigned by this README.

The root .gitignore excludes Python caches, local virtual environments and generated detection recordings. Each module README provides the detailed instructions.
