"""Fit a camera-to-robot transform from one confirmed ArUco sampling session.

Maps camera marker centres to the recorded robot tool positions. The marker
centre and the tool origin are still different physical points, so the fit
includes that unmeasured offset.
Usage: python estimate_transform.py SESSION_DIRECTORY
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np


def fit(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = b - rotation @ a
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def predict(transform, points):
    return points @ transform[:3, :3].T + transform[:3, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    records = re.findall(r'robtarget\s+(\w+)\s*:=\s*(\[.*?\])\s*;',
                         (args.session / 'points.txt').read_text(encoding='utf-8-sig'), re.S)
    targets = {name: json.loads(value) for name, value in records}
    names = ['HOMEpos'] + [f's{i}' for i in range(1, 16)]
    folders = sorted(args.session.glob('sample_[0-9][0-9][0-9][0-9]'))
    if len(folders) != 16 or set(targets) != set(names):
        raise ValueError('Expected confirmed HOMEpos + s1..s15 sequence and 16 samples')
    samples = [json.loads((p / 'sample.json').read_text(encoding='utf-8-sig')) for p in folders]
    for sample in samples:
        selection = sample['aruco_selection']
        if selection['target_id'] != 23 or selection['target_count'] != 1:
            raise ValueError('Expected unique ID 23 in every sample')
    camera = np.array([[s['measurement']['camera_xyz_mm'][k] for k in ('x', 'y', 'z')] for s in samples])
    robot = np.array([targets[name][0] for name in names])
    if not np.isfinite(camera).all() or not np.isfinite(robot).all():
        raise ValueError('Non-finite coordinates')
    transform = fit(camera, robot)
    errors = np.linalg.norm(predict(transform, camera) - robot, axis=1)
    held_out = []
    for i in range(16):
        mask = np.arange(16) != i
        held_out.append(float(np.linalg.norm(predict(fit(camera[mask], robot[mask]), camera[i:i+1])[0] - robot[i])))
    result = {
        'status': 'fitted_marker_center_to_recorded_tool_position',
        'not_for_robot_motion': True,
        'mapping_assumption': 'sample 1 HOMEpos, samples 2..16 s1..s15; operator must confirm correspondence and controller values for each session',
        'source': 'camera frame, ArUco ID23 centre, depth backprojection',
        'destination': 'recorded robot tool position frame; actual tool/workobject definitions not confirmed',
        'equation': 'robot_position_approx_mm = R @ camera_marker_center_mm + t_mm',
        'limitations': ['Marker centre and robot tool origin are different physical points; orientation-dependent offset ignored',
                        'User reports 80 mm vertical marker-to-gripper-centre distance, not a measured 3D tool-frame offset',
                        'RGB/depth pixel geometry still pending verification',
                        'Leave-one-out errors use this same campaign, not independent physical validation'],
        'T_diagnostic': transform.tolist(),
        'fit_rmse_mm': float(np.sqrt(np.mean(errors**2))),
        'fit_max_mm': float(errors.max()),
        'leave_one_out_rmse_mm': float(np.sqrt(np.mean(np.array(held_out)**2))),
        'leave_one_out_max_mm': max(held_out),
        'camera_spread_singular_values_mm': np.linalg.svd(camera-camera.mean(axis=0), compute_uv=False).tolist(),
        'samples': [{'sample': folder.name, 'robot_target': name,
                     'camera_xyz_mm': cam.tolist(), 'robot_xyz_mm': rob.tolist(),
                     'fit_error_mm': float(error), 'leave_one_out_error_mm': loo,
                     'depth_range_mm': sample['measurement']['depth_range_mm']}
                    for folder, name, cam, rob, error, loo, sample in zip(folders, names, camera, robot, errors, held_out, samples)]
    }
    output = args.session / 'camera_to_robot.json'
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'limitations')}, indent=2))
    # Export the same fitted matrix; no second fit is performed.
    matrix_output = args.session / 'HT_matrix.txt'
    np.savetxt(
        matrix_output, transform, fmt='%.17g',
        header=('HT matrix: p_robot = R @ p_camera + t; XYZ in mm.\n'
                'Marker-to-tool offset remains unresolved; see camera_to_robot.json.')
    )
    print('JSON output:', output)
    print('Matrix output:', matrix_output)


if __name__ == '__main__':
    main()
