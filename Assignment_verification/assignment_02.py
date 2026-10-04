"""Assignment 02: coordinate-frame PDF model, B -> 0 -> ... -> 7.

Install: python -m pip install numpy scipy
Run: python assignment_02.py
All angles are signed radians; lengths and positions are metres.
T_AB maps coordinates in B into A. FK returns T_B7.

User-confirmed translations (metres):
* T_23: [L4,0,L5] = [0.1126,0,0.0280].
* T_67: [-L10,0,L11] = [-0.0432,0,0.0752].
These confirmed values supersede the differing entries in the PDF.

The two final transforms are fixed: there is no theta_6 in this model.
Base pose (Lx,Ly,Lz,psi) defaults to (0.5,0.1,0.1,0). Psi is configurable;
the PDF specifies no numeric yaw. IK holds this base pose fixed.

IK follows Lecture 4A slide 23: FK -> squared position error -> minimize.
It returns one candidate, not all solutions or proof of uniqueness.
No collision checking. Supply measured joint limits for physical feasibility.
These tests verify the written mathematical model, not hardware calibration.
"""

import math
import numpy as np
from scipy.optimize import minimize

# L0,...,L11 in metres, including the user-confirmed L11=0.0752.
ROBOT_LENGTHS = (0.0388, 0.0624, 0.0304, 0.0542, 0.1126, 0.0280,
                 0.1349, 0.0611, 0.0432, 0.0229, 0.0432, 0.0752)
BASE_POSITION = (0.5, 0.1, 0.1)
BASE_YAW = 0.0
POSITION_TOLERANCE = 1e-5


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
    values = np.asarray(lengths, dtype=float)
    if values.shape != (12,) or not np.all(np.isfinite(values)) or np.any(values < 0):
        raise ValueError('lengths must contain 12 finite, nonnegative values, L0,...,L11.')
    return values


def forward_kinematics(theta_1, theta_2, theta_3, theta_4, theta_5,
                       *, lengths=None, base_position=BASE_POSITION, psi=BASE_YAW):
    """Return T_B7 using the PDF model with user-confirmed T23 and T67.

    Put each rotation in the upper-left block and each parent-frame position
    in the last column directly. Multiply only the completed transforms.
    """
    q = np.asarray([theta_1, theta_2, theta_3, theta_4, theta_5, psi])
    base = np.asarray(base_position, dtype=float)
    if not np.all(np.isfinite(q)) or base.shape != (3,) or not np.all(np.isfinite(base)):
        raise ValueError('Angles and the three base coordinates must be finite.')
    L0, L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11 = _get_lengths(lengths)

    T_B0 = HomogeneousTranRot_Z(psi)
    T_B0[:3, 3] = base

    T_01 = HomogeneousTranRot_Z(theta_1)
    T_01[:3, 3] = [L0, 0, L1]

    T_12 = HomogeneousTranRot_Y(theta_2)
    T_12[:3, 3] = [L2, 0, L3]

    T_23 = HomogeneousTranRot_Y(theta_3)
    T_23[:3, 3] = [L4, 0, L5]  # Confirmed: [0.1126, 0, 0.0280].

    T_34 = HomogeneousTranRot_Y(theta_4)
    T_34[:3, 3] = [0, 0, L6]

    T_45 = HomogeneousTranRot_Z(theta_5)
    T_45[:3, 3] = [0, 0, L7]

    T_56 = HomogeneousTranTranslation(L8, 0, L9)
    T_67 = HomogeneousTranTranslation(-L10, 0, L11)

    T_B7 = T_B0 @ T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ T_67
    return T_B7


def get_position_error(theta, desired_position, *, lengths=None,
                       base_position=BASE_POSITION, psi=BASE_YAW):
    """Lecture-style scalar objective: error.dot(error), in square metres."""
    position = forward_kinematics(*theta, lengths=lengths,
                                  base_position=base_position, psi=psi)[:3, 3]
    error = np.asarray(desired_position, dtype=float) - position
    return float(error.dot(error))


def inverse_kinematics_position(x_des, y_des, z_des, *, lengths=None,
                                initial_guess=None, joint_limits=None,
                                base_position=BASE_POSITION, psi=BASE_YAW,
                                active_joints=(0, 1, 2, 3, 4),
                                tolerance=POSITION_TOLERANCE, restarts=8):
    """Return (theta, solution_found), theta = five signed angles in radians.

    Uses scipy.optimize.minimize on squared position error, as in Lecture 4A.
    Optional joint_limits: five (lower, upper) pairs in radians.
    Optional active_joints: zero-based indices of joints to optimize. Others
    stay at initial_guess (zero by default). The base pose is held fixed.
    With L8=L10 and fixed final rotations, theta5 changes orientation but
    not frame 7 position, so position-only IK cannot determine it uniquely.
    With no joint_limits, this is an unconstrained geometric model.

    False means no candidate within tolerance was found; numerical failure
    does not prove unreachability. On failure, returns the best candidate.
    Position only: orientation is unconstrained.
    """
    lengths = _get_lengths(lengths)
    target = np.array([x_des, y_des, z_des], dtype=float)
    if not np.all(np.isfinite(target)) or not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Target must be finite and tolerance positive.')
    q0 = np.zeros(5) if initial_guess is None else np.asarray(initial_guess, dtype=float).copy()
    if q0.shape != (5,) or not np.all(np.isfinite(q0)):
        raise ValueError('initial_guess must contain five finite angles.')
    active = np.asarray(active_joints, dtype=int)
    if (active.ndim != 1 or len(set(active.tolist())) != len(active)
            or np.any(active < 0) or np.any(active > 4)):
        raise ValueError('active_joints must contain distinct indices from 0 to 4.')
    if not isinstance(restarts, int) or restarts < 0:
        raise ValueError('restarts must be a nonnegative integer.')
    limits = None
    if joint_limits is not None:
        limits = np.asarray(joint_limits, dtype=float)
        if (limits.shape != (5, 2) or not np.all(np.isfinite(limits))
                or np.any(limits[:, 0] >= limits[:, 1])):
            raise ValueError('joint_limits must contain five finite lower < upper pairs.')
        if np.any(q0 < limits[:, 0]) or np.any(q0 > limits[:, 1]):
            raise ValueError('initial_guess must be within joint_limits.')

    def expand(active_angles):
        theta = q0.copy()
        theta[active] = active_angles
        return theta

    def objective(active_angles):
        return get_position_error(expand(active_angles), target, lengths=lengths,
                                  base_position=base_position, psi=psi)

    best_theta = q0.copy()
    best_error = get_position_error(best_theta, target, lengths=lengths,
                                  base_position=base_position, psi=psi)
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
        error = get_position_error(theta, target, lengths=lengths,
                                  base_position=base_position, psi=psi)
        if np.isfinite(error) and error < best_error:
            best_theta, best_error = theta, error
        # Check actual task error, not just the optimizer's success flag.
        if best_error <= tolerance**2:
            return best_theta, True
    return best_theta, False


# Five FK tests: independently hand-calculated poses for the confirmed geometry.
# Keep their geometry fixed so changing ROBOT_LENGTHS cannot silently change
# the model under test. These values include the user-confirmed corrections.
TEST_LENGTHS = (0.0388, 0.0624, 0.0304, 0.0542, 0.1126, 0.0280,
                0.1349, 0.0611, 0.0432, 0.0229, 0.0432, 0.0752)


def _check_fk(angles, expected_position, expected_rotation, *, psi=0.0):
    T = forward_kinematics(*angles, lengths=TEST_LENGTHS,
                           base_position=(0.5, 0.1, 0.1), psi=psi)
    np.testing.assert_allclose(T[:3, 3], expected_position, atol=1e-12, rtol=0)
    np.testing.assert_allclose(T[:3, :3], expected_rotation, atol=1e-12, rtol=0)
    np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-12, rtol=0)


def test_fk_zero():
    # x=0.5+L0+L2+L4+L8-L10; z=0.1+L1+L3+L5+L6+L7+L9+L11.
    _check_fk([0]*5, [0.6818, 0.1, 0.5387], np.eye(3))


def test_fk_shoulder_pan():
    # theta1 rotates only downstream x=0.1430 into +y.
    _check_fk([math.pi/2, 0, 0, 0, 0], [0.5388, 0.2430, 0.5387],
              [[0, -1, 0], [1, 0, 0], [0, 0, 1]])


def test_fk_shoulder_pitch():
    # Frame 2 origin=(0.5692,0.1,0.2166); (0.1126,0,0.3221) -> (0.3221,0,-0.1126).
    _check_fk([0, math.pi/2, 0, 0, 0], [0.8913, 0.1, 0.1040],
              [[0, 0, 1], [0, 1, 0], [-1, 0, 0]])


def test_fk_elbow_and_wrist_pitch():
    # Opposite pitches cancel orientation; L6 shifts from +z to +x.
    _check_fk([0, 0, math.pi/2, -math.pi/2, 0], [0.8167, 0.1, 0.4038], np.eye(3))


def test_fk_base_yaw_and_wrist_roll():
    # Wrist roll cannot move frame 7 because L8-L10=0. Base yaw does move it.
    _check_fk([0, 0, 0, 0, math.pi/2], [0.5, 0.2818, 0.5387],
              [[-1, 0, 0], [0, -1, 0], [0, 0, 1]], psi=math.pi/2)


def _check_ik(target, **kwargs):
    q, found = inverse_kinematics_position(*target, lengths=TEST_LENGTHS,
                                          base_position=(0.5, 0.1, 0.1), psi=0, **kwargs)
    assert found, 'IK did not find a solution.'
    actual = forward_kinematics(*q, lengths=TEST_LENGTHS,
                                base_position=(0.5, 0.1, 0.1), psi=0)[:3, 3]
    assert np.linalg.norm(actual - target) <= POSITION_TOLERANCE
    return q


def test_ik_zero_pose():
    _check_ik(np.array([0.6818, 0.1, 0.5387]))


def test_ik_reachable_position():
    _check_ik(np.array([0.8913, 0.1, 0.1040]))


def test_ik_no_solution():
    # Target is far beyond the sum of offsets from the base.
    _, found = inverse_kinematics_position(10, 10, 10, lengths=TEST_LENGTHS,
                                          base_position=(0.5, 0.1, 0.1), psi=0, restarts=1)
    assert not found


def test_ik_one_solution_restricted():
    # Explicitly ONE active joint, theta1 in [-pi,pi], others fixed to zero.
    # Target requires theta1=pi/2 uniquely within this interval.
    q = _check_ik(np.array([0.5388, 0.2430, 0.5387]), active_joints=(0,),
                  joint_limits=[(-math.pi, math.pi)]*5)
    np.testing.assert_allclose(q, [math.pi/2, 0, 0, 0, 0], atol=3e-4, rtol=0)


def test_ik_many_solutions():
    # Wrist roll is position-invariant in this specific PDF model. Two
    # different wrist angles therefore give exactly the same target position.
    target = np.array([0.6818, 0.1, 0.5387])
    q_a = _check_ik(target, initial_guess=[0, 0, 0, 0, 0])
    q_b = _check_ik(target, initial_guess=[0, 0, 0, 0, math.pi/2])
    difference = (q_a - q_b + math.pi) % (2*math.pi) - math.pi
    assert np.linalg.norm(difference) > 1e-2


def run_tests():
    groups = [('FK', [test_fk_zero, test_fk_shoulder_pan, test_fk_shoulder_pitch,
                       test_fk_elbow_and_wrist_pitch, test_fk_base_yaw_and_wrist_roll]),
              ('IK', [test_ik_zero_pose, test_ik_reachable_position, test_ik_no_solution,
                       test_ik_one_solution_restricted, test_ik_many_solutions])]
    failures = 0
    print('Model: confirmed T23=[0.1126,0,0.0280], T67=[-0.0432,0,0.0752] m.\n')
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
