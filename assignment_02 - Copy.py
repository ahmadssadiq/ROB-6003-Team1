"""Assignment 02: FK and position IK for the student's SO-101 frame model.

Install: conda env create -f environment.yml
Activate: conda activate s101-arm
Run tests: python assignment_02.py

Angles: signed radians, right-hand rule. Lengths: metres.
T_AB maps coordinates in B into A. End effector K is on the moving finger.
Frame chain: S -> B -> D -> F -> G -> H -> J -> K.

This implements the supplied drawing, NOT a verified SO-101 URDF model.
Adjacent axes are assumed aligned at zero joint angle. Add fixed orientation
and calibration offsets if your physical zero configuration differs.
Set ROBOT_LENGTHS to your measured l1,...,l11 before using real-robot FK/IK.
TEST_LENGTHS are arbitrary test data, NOT SO-101 measurements.

IK follows Lecture 4A, slide 23: FK -> squared position error -> minimize.
It returns one candidate, not all solutions or a proof of uniqueness.
No collision checking. Supply measured joint limits for physical feasibility.
"""

import math
import numpy as np
from scipy.optimize import minimize

# Replace None with (l1, l2, ..., l11), all in metres.
ROBOT_LENGTHS = None
POSITION_TOLERANCE = 1e-5  # metres
TEST_LENGTHS = (0.01, 0.02, 0.03, 0.04, 0.05, 0.06,
                0.07, 0.08, 0.09, 0.10, 0.11)


def HomogeneousTranRot_X(theta_x):
    """Right-handed rotation around x; input in radians."""
    c, s = math.cos(theta_x), math.sin(theta_x)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0],
                     [0, s, c, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranRot_Y(theta_y):
    """Right-handed rotation around y; input in radians."""
    c, s = math.cos(theta_y), math.sin(theta_y)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0],
                     [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranRot_Z(theta_z):
    """Right-handed rotation around z; input in radians."""
    c, s = math.cos(theta_z), math.sin(theta_z)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0],
                     [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranTranslation(x, y, z):
    """Translation with no rotation."""
    return np.array([[1, 0, 0, x], [0, 1, 0, y],
                     [0, 0, 1, z], [0, 0, 0, 1]], dtype=float)


def _get_lengths(lengths):
    if lengths is None:
        lengths = ROBOT_LENGTHS
    if lengths is None:
        raise ValueError('Set ROBOT_LENGTHS to measured l1,...,l11 in metres, '
                         'or pass lengths= explicitly. Test dimensions are not robot dimensions.')
    values = np.asarray(lengths, dtype=float)
    if values.shape != (11,) or not np.all(np.isfinite(values)) or np.any(values < 0):
        raise ValueError('lengths must contain 11 finite, nonnegative values.')
    return values


def forward_kinematics(theta_1, theta_2, theta_3, theta_4, theta_5,
                       theta_6=0.0, *, lengths=None):
    """Return 4x4 T_SK. theta_6 is the moving-finger angle.

    Each translation column is expressed in the starting frame of that step.
    Translation @ Rotation therefore gives the block matrix [R, p; 0, 1].
    """
    q = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
    if not np.all(np.isfinite(q)):
        raise ValueError('Joint angles must be finite.')
    l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11 = _get_lengths(lengths)

    T_SB = HomogeneousTranTranslation(l2, l1, 0) @ HomogeneousTranRot_Y(theta_1)
    T_BD = HomogeneousTranTranslation(l3, l4, 0) @ HomogeneousTranRot_Z(theta_2)
    T_DF = HomogeneousTranTranslation(l6, l5, 0) @ HomogeneousTranRot_Z(theta_3)
    T_FG = HomogeneousTranTranslation(l7, 0, 0) @ HomogeneousTranRot_Z(theta_4)
    T_GH = HomogeneousTranTranslation(l8, 0, 0) @ HomogeneousTranRot_X(theta_5)
    T_HJ = HomogeneousTranTranslation(l9, l10, 0) @ HomogeneousTranRot_Z(theta_6)
    T_JK = HomogeneousTranTranslation(l11, 0, 0)

    T_SK = T_SB @ T_BD @ T_DF @ T_FG @ T_GH @ T_HJ @ T_JK
    return T_SK


def get_position_error(theta, desired_position, *, lengths=None):
    """Lecture-style scalar objective: error.dot(error), in square metres."""
    position = forward_kinematics(*theta, lengths=lengths)[:3, 3]
    error = np.asarray(desired_position, dtype=float) - position
    return float(error.dot(error))


def inverse_kinematics_position(x_des, y_des, z_des, *, lengths=None,
                                initial_guess=None, joint_limits=None,
                                active_joints=(0, 1, 2, 3, 4, 5),
                                tolerance=POSITION_TOLERANCE, restarts=8):
    """Return (theta, solution_found), theta = six signed angles in radians.

    Uses scipy.optimize.minimize on squared position error, as in Lecture 4A.
    Optional joint_limits: six (lower, upper) pairs in radians.
    Optional active_joints: zero-based indices of joints to optimize. Others
    stay at initial_guess (zero by default). To hold the gripper fixed, use
    active_joints=(0,1,2,3,4) and set initial_guess[5] to its desired angle.
    With no joint_limits, this is an unconstrained geometric model.

    False means no candidate within tolerance was found; numerical failure
    does not prove unreachability. On failure, returns the best candidate.
    Position only: orientation is unconstrained.
    """
    lengths = _get_lengths(lengths)
    target = np.array([x_des, y_des, z_des], dtype=float)
    if not np.all(np.isfinite(target)) or not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Target must be finite and tolerance positive.')
    q0 = np.zeros(6) if initial_guess is None else np.asarray(initial_guess, dtype=float).copy()
    if q0.shape != (6,) or not np.all(np.isfinite(q0)):
        raise ValueError('initial_guess must contain six finite angles.')
    active = np.asarray(active_joints, dtype=int)
    if (active.ndim != 1 or len(set(active.tolist())) != len(active)
            or np.any(active < 0) or np.any(active > 5)):
        raise ValueError('active_joints must contain distinct indices from 0 to 5.')
    if not isinstance(restarts, int) or restarts < 0:
        raise ValueError('restarts must be a nonnegative integer.')
    limits = None
    if joint_limits is not None:
        limits = np.asarray(joint_limits, dtype=float)
        if (limits.shape != (6, 2) or not np.all(np.isfinite(limits))
                or np.any(limits[:, 0] >= limits[:, 1])):
            raise ValueError('joint_limits must contain six finite lower < upper pairs.')
        if np.any(q0 < limits[:, 0]) or np.any(q0 > limits[:, 1]):
            raise ValueError('initial_guess must be within joint_limits.')

    def expand(active_angles):
        theta = q0.copy()
        theta[active] = active_angles
        return theta

    def objective(active_angles):
        return get_position_error(expand(active_angles), target, lengths=lengths)

    best_theta = q0.copy()
    best_error = get_position_error(best_theta, target, lengths=lengths)
    if best_error <= tolerance**2 or len(active) == 0:
        return best_theta, bool(best_error <= tolerance**2)

    # Deterministic restarts help escape poor initial guesses/local minima.
    rng = np.random.default_rng(42)
    bounds = None if limits is None else limits[active].tolist()
    for attempt in range(restarts + 1):
        if attempt == 0:
            start = q0[active]
        elif limits is None:
            start = rng.uniform(-math.pi, math.pi, size=len(active))
        else:
            start = rng.uniform(limits[active, 0], limits[active, 1])
        method = 'BFGS' if bounds is None else 'L-BFGS-B'
        options = {'maxiter': 2000, 'gtol': 1e-10}
        if bounds is not None:
            options['ftol'] = 1e-15
        result = minimize(objective, start, method=method, bounds=bounds, options=options)
        theta = expand(result.x)
        error = get_position_error(theta, target, lengths=lengths)
        if np.isfinite(error) and error < best_error:
            best_theta, best_error = theta, error
        # Check actual task error, not just the optimizer's success flag.
        if best_error <= tolerance**2:
            return best_theta, True
    return best_theta, False


# Five FK tests: expected poses are calculated independently by hand using
# TEST_LENGTHS, NOT by calling FK to generate the expected answers.
def _check_fk(angles, expected_position, expected_rotation):
    T = forward_kinematics(*angles, lengths=TEST_LENGTHS)
    np.testing.assert_allclose(T[:3, 3], expected_position, atol=1e-12, rtol=0)
    np.testing.assert_allclose(T[:3, :3], expected_rotation, atol=1e-12, rtol=0)
    np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-12, rtol=0)


def test_fk_zero():
    # x=l2+l3+l6+l7+l8+l9+l11; y=l1+l4+l5+l10.
    _check_fk([0]*6, [0.46, 0.20, 0], np.eye(3))


def test_fk_base_rotation():
    # Base pivot remains at (0.02,0.01,0); downstream +x becomes -z.
    _check_fk([math.pi/2, 0, 0, 0, 0, 0], [0.02, 0.20, -0.44],
              [[0, 0, 1], [0, 1, 0], [-1, 0, 0]])


def test_fk_shoulder_rotation():
    # Pivot D=(0.05,0.05,0); downstream (0.41,0.15) becomes (-0.15,0.41).
    _check_fk([0, math.pi/2, 0, 0, 0, 0], [-0.10, 0.46, 0],
              [[0, -1, 0], [1, 0, 0], [0, 0, 1]])


def test_fk_elbow_and_wrist_flex():
    # Opposite rotations cancel orientation; FG's 0.07 shifts from x to y.
    _check_fk([0, 0, math.pi/2, -math.pi/2, 0, 0], [0.39, 0.27, 0], np.eye(3))


def test_fk_wrist_roll_and_gripper():
    # Roll maps H's +y into +z; gripper maps its tip's +x into H's +y.
    _check_fk([0, 0, 0, 0, math.pi/2, math.pi/2], [0.35, 0.10, 0.21],
              [[0, -1, 0], [0, 0, -1], [1, 0, 0]])


def _check_ik(target, **kwargs):
    q, found = inverse_kinematics_position(*target, lengths=TEST_LENGTHS, **kwargs)
    assert found, 'IK did not find a solution.'
    actual = forward_kinematics(*q, lengths=TEST_LENGTHS)[:3, 3]
    assert np.linalg.norm(actual - target) <= POSITION_TOLERANCE
    return q


def test_ik_zero_pose():
    _check_ik(np.array([0.46, 0.20, 0]))


def test_ik_reachable_position():
    _check_ik(np.array([0.35, 0.10, 0.21]))


def test_ik_no_solution():
    # 10 m lies beyond the sum of every offset length (<1 m for this fixture).
    _, found = inverse_kinematics_position(10, 10, 10, lengths=TEST_LENGTHS, restarts=1)
    assert not found


def test_ik_one_solution_restricted():
    # Explicitly restricted to ONE active joint, theta1 in [-pi,pi].
    # The target requires theta1=pi/2 uniquely in this interval.
    limits = [(-math.pi, math.pi)] * 6
    q = _check_ik(np.array([0.02, 0.20, -0.44]), active_joints=(0,), joint_limits=limits)
    np.testing.assert_allclose(q, [math.pi/2, 0, 0, 0, 0, 0], atol=1e-4, rtol=0)


def test_ik_many_solutions():
    # Find two distinct configurations for the same target using different seeds.
    # Verify both with FK and distinguish them modulo 2*pi. This demonstrates
    # multiple solutions; it does not enumerate every solution.
    target = np.array([0.35, 0.10, 0.21])
    q_a = _check_ik(target, initial_guess=[0, 0, 0, 0, math.pi/2, math.pi/2])
    q_b = _check_ik(target, initial_guess=[0.8, -0.5, 0.7, -0.4, -0.7, 0.2])
    difference = (q_a - q_b + math.pi) % (2*math.pi) - math.pi
    assert np.linalg.norm(difference) > 1e-2, 'Need distinct solutions modulo 2*pi.'


def run_tests():
    groups = [('FK', [test_fk_zero, test_fk_base_rotation, test_fk_shoulder_rotation,
                       test_fk_elbow_and_wrist_flex, test_fk_wrist_roll_and_gripper]),
              ('IK', [test_ik_zero_pose, test_ik_reachable_position, test_ik_no_solution,
                       test_ik_one_solution_restricted, test_ik_many_solutions])]
    failures = 0
    print('Using synthetic test dimensions, NOT measured SO-101 dimensions.\n')
    for label, tests in groups:
        passed = 0
        for test in tests:
            try:
                test()
                passed += 1
                print('PASS:', test.__name__)
            except AssertionError as error:
                failures += 1
                print('FAIL:', test.__name__, error)
        print(f'{label}: {passed}/{len(tests)} passed\n')
    return failures


if __name__ == '__main__':
    raise SystemExit(1 if run_tests() else 0)
