"""Independent FK comparison: assignment_02 vs official SO-101 URDF + yourdfpy.
Run: python assignment_verification.py
Dependencies: python -m pip install numpy scipy yourdfpy==0.0.60

Does NOT modify assignment_02.py. Radians/metres internally.
Angle mapping is an analytically derived CONVENTION CANDIDATE, not measured
motor calibration. Results are conditional on adopting this zero convention.
An official-TCP comparison uses a different endpoint and is diagnostic only.
The matching-point comparison uses a virtual point on the wrist-roll axis,
matching the endpoint assumption in the assignment, not a verified fingertip.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from yourdfpy import URDF
import assignment_02 as assignment

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / 'reference' / 'so101_new_calib.urdf'
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
SIGNS = np.array([-1., 1., 1., 1., -1.])
# Derived from link directions, NOT optimized against sampled endpoint errors:
# At URDF zero, upper-arm displacement is approximately (+0.028,0,+0.11257).
# At assignment zero it is (+0.1126,0,+0.028). Their polar-angle difference
# defines shoulder zero. Forearm displacement (+0.1349,0,+0.0052) is rotated
# onto +z; wrist pitch then brings the roll axis onto the assignment's +z.
BETA = math.atan2(0.1126, 0.0280) - math.atan2(0.0280, 0.11257)
GAMMA = -math.atan2(0.1349, 0.0052)
OFFSETS = np.array([0., BETA, GAMMA - BETA, -math.pi/2 - GAMMA, 0.])
# Assignment's fixed final translations sum to (0,0,0.0981) in frame 5.
# URDF gripper_link has its +z pointing back along the wrist, hence minus.
ON_AXIS_TOOL_IN_GRIPPER = np.array([0., 0., -0.0981, 1.])


def assignment_to_urdf(q):
    return SIGNS * np.asarray(q, dtype=float) + OFFSETS


def urdf_to_assignment(q):
    return (np.asarray(q, dtype=float) - OFFSETS) / SIGNS


def load_reference():
    # External library computes every URDF transform; no assignment helpers
    # are used to compute reference FK. No meshes or robot hardware required.
    return URDF.load(str(URDF_PATH), load_meshes=False,
                     load_collision_meshes=False, build_scene_graph=True)


def set_reference(robot, q):
    robot.update_cfg(dict(zip(NAMES, q)))


def reference_poses(robot, q, orientation_correction):
    set_reference(robot, q)
    tcp = robot.get_transform('gripper_frame_link', 'base_link').copy()
    wrist = robot.get_transform('gripper_link', 'base_link').copy()
    on_axis = tcp.copy()
    on_axis[:3, 3] = (wrist @ ON_AXIS_TOOL_IN_GRIPPER)[:3]
    # Change only endpoint axis labels. NO position alignment/fitting.
    tcp[:3, :3] = tcp[:3, :3] @ orientation_correction
    on_axis[:3, :3] = on_axis[:3, :3] @ orientation_correction
    return tcp, on_axis


def errors(T, U):
    position_mm = 1000 * np.linalg.norm(T[:3, 3] - U[:3, 3])
    angle_deg = np.rad2deg(Rotation.from_matrix(T[:3, :3].T @ U[:3, :3]).magnitude())
    return float(position_mm), float(angle_deg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--random-count', type=int, default=50)
    parser.add_argument('--position-tol-mm', type=float, default=0.5)
    parser.add_argument('--rotation-tol-deg', type=float, default=0.01)
    args = parser.parse_args()
    if args.random_count < 1 or args.position_tol_mm <= 0 or args.rotation_tol_deg <= 0:
        parser.error('Count and tolerances must be positive.')
    robot = load_reference()
    set_reference(robot, OFFSETS)
    # At assignment q=0 its orientation is I. This fixed right-side change
    # maps reference TCP axis labels to those labels. Later poses validate it.
    # q=0 in the assignment is not necessarily inside the physical joint limits.
    orientation_correction = robot.get_transform('gripper_frame_link', 'base_link')[:3, :3].T
    root = ET.parse(URDF_PATH).getroot()
    bounds = np.array([[float(root.find(f"joint[@name='{n}']/limit").get(k))
                        for k in ('lower', 'upper')] for n in NAMES])
    print('REFERENCE: TheRobotStudio SO-101 URDF; independent yourdfpy FK')
    print('CANDIDATE mapping: q_urdf = signs * q_assignment + offsets (radians)')
    print('Signs:', SIGNS.tolist())
    print('Offsets in degrees:', np.round(np.rad2deg(OFFSETS), 6).tolist())
    print('Mapping derives model zero conventions; physical motor zeros are not verified.')
    print('Base comparison: assignment FK uses base_position=(0,0,0), psi=0.')
    print('On-axis reference tool: gripper_link point (0,0,-0.0981) metres.')
    print('Official TCP differs from this point: its error is NOT a same-point FK error.\n')

    # Diagnostic ONLY: same numbers without converting frame/angle conventions.
    q0 = np.zeros(5)
    set_reference(robot, q0)
    raw_ref = robot.get_transform('gripper_frame_link', 'base_link')
    raw_own = assignment.forward_kinematics(*q0, base_position=(0,0,0), psi=0)
    ep, er = errors(raw_own, raw_ref)
    print(f'UNALIGNED zero-angle diagnostic: {ep:.3f} mm, {er:.3f} deg (not a verdict)\n')

    # Cases are defined in official URDF coordinates so they remain inside
    # its limits, then converted to assignment coordinates for the same pose.
    cases = [('urdf_zero', np.zeros(5))]
    for i, name in enumerate(NAMES):
        q = np.zeros(5); q[i] = math.radians(30)
        cases.append((name + '_30deg', q))
    cases.append(('mixed', np.deg2rad([20, -30, 40, -25, 60])))
    rng = np.random.default_rng(2026)
    for i in range(args.random_count):
        q = rng.uniform(bounds[:, 0]*0.8, bounds[:, 1]*0.8)
        cases.append((f'random_{i+1:02d}', q))

    rows = []
    print(f'{"Case":25s} {"same point mm":>14s} {"rotation deg":>14s} {"other TCP mm":>14s}')
    for label, q_ref in cases:
        if not np.all((q_ref >= bounds[:, 0]) & (q_ref <= bounds[:, 1])):
            raise ValueError(f'{label} is outside URDF joint limits.')
        q = urdf_to_assignment(q_ref)
        T = assignment.forward_kinematics(*q, base_position=(0,0,0), psi=0)
        tcp, axis = reference_poses(robot, q_ref, orientation_correction)
        ep, er = errors(T, axis)
        other_ep, _ = errors(T, tcp)
        row = {'case': label, 'same_point_error_mm': ep, 'rotation_error_deg': er,
               'different_tcp_distance_mm': other_ep}
        for i in range(5):
            row[f'assignment_q{i+1}_rad'] = q[i]
            row[f'urdf_q{i+1}_rad'] = q_ref[i]
        for i, name in enumerate('xyz'):
            row[f'assignment_{name}_m'] = T[i, 3]
            row[f'reference_on_axis_{name}_m'] = axis[i, 3]
            row[f'reference_tcp_{name}_m'] = tcp[i, 3]
        rows.append(row)
        if not label.startswith('random_'):
            print(f'{label:25s} {ep:14.6f} {er:14.6f} {other_ep:14.6f}')

    p = np.array([r['same_point_error_mm'] for r in rows])
    rot = np.array([r['rotation_error_deg'] for r in rows])
    other = np.array([r['different_tcp_distance_mm'] for r in rows])
    ok = (p <= args.position_tol_mm) & (rot <= args.rotation_tol_deg)
    print(f'\n{len(rows)} cases: same-point error mean={p.mean():.6f}, max={p.max():.6f} mm')
    print(f'Orientation error max={rot.max():.6f} deg')
    print(f'Different official TCP distance range={other.min():.6f}..{other.max():.6f} mm')
    print(f'Within stated {args.position_tol_mm} mm / {args.rotation_tol_deg} deg tolerances: '
          f'{ok.sum()}/{len(ok)}')
    print('This is conditional geometric agreement, NOT motor calibration or fingertip validation.')
    print('Expected residuals: rounded lengths, omitted 5.2 mm forearm offset, small lateral offsets.')
    print('Forearm offset is partly absorbed by the derived angular-zero convention;')
    print('its actual length sqrt(0.1349^2+0.0052^2) still differs from assignment L6.')
    with (HERE / 'verification_results.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    summary = {'status': 'conditional_geometric_agreement' if np.all(ok) else 'mismatch',
               'cases': len(rows), 'within_tolerance': int(ok.sum()),
               'max_same_point_error_mm': float(p.max()),
               'max_orientation_error_deg': float(rot.max()),
               'angle_signs': SIGNS.tolist(), 'angle_offsets_rad': OFFSETS.tolist(),
               'tool_axis_rotation_correction': orientation_correction.tolist(),
               'urdf_sha256': hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
               'physical_calibration_verified': False}
    (HERE / 'verification_summary.json').write_text(json.dumps(summary, indent=2))
    return 0 if np.all(ok) else 1


if __name__ == '__main__':
    sys.exit(main())
