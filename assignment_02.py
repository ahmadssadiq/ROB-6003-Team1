import math
import numpy as np
from scipy.optimize import minimize

ROBOT_LENGTHS = (0.0388, 0.0624, 0.0304,0.0542, 0.1126, 0.0280,0.1349, 0.0611, 0.0432, 0.0229, 0.0432, 0.0752)
BASE_POSITION = (0.5, 0.1, 0.1)
BASE_YAW = 0.0
POSITION_TOLERANCE = 1e-5

def HomogeneousTranRot_X(theta_x):
    c, s = math.cos(theta_x), math.sin(theta_x)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0],
                     [0, s, c, 0], [0, 0, 0, 1]], dtype=float)

def HomogeneousTranRot_Y(theta_y):
    c, s = math.cos(theta_y), math.sin(theta_y)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0],
                     [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)

def HomogeneousTranRot_Z(theta_z):
    c, s = math.cos(theta_z), math.sin(theta_z)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0],
                     [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)

def HomogeneousTranTranslation(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y],
                     [0, 0, 1, z], [0, 0, 0, 1]], dtype=float)

def forward_kinematics(theta_1, theta_2, theta_3, theta_4, theta_5,lengths=ROBOT_LENGTHS, base_position=BASE_POSITION, psi=BASE_YAW):
    L0, L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11 = lengths
    T_B0 = HomogeneousTranRot_Z(psi)
    T_B0[:3, 3] = base_position
    T_01 = HomogeneousTranRot_Z(theta_1)
    T_01[:3, 3] = [L0, 0, L1]
    T_12 = HomogeneousTranRot_Y(theta_2)
    T_12[:3, 3] = [L2, 0, L3]
    T_23 = HomogeneousTranRot_Y(theta_3)
    T_23[:3, 3] = [L4, 0, L5]
    T_34 = HomogeneousTranRot_Y(theta_4)
    T_34[:3, 3] = [0, 0, L6]
    T_45 = HomogeneousTranRot_Z(theta_5)
    T_45[:3, 3] = [0, 0, L7]
    T_56 = HomogeneousTranTranslation(L8, 0, L9)
    T_67 = HomogeneousTranTranslation(-L10, 0, L11)
    T_B7 = T_B0 @ T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ T_67
    return T_B7

def get_position_error(theta, desired_position, lengths=ROBOT_LENGTHS,base_position=BASE_POSITION, psi=BASE_YAW):
    T = forward_kinematics(theta[0], theta[1], theta[2], theta[3], theta[4],
                          lengths=lengths,
                          base_position=base_position, psi=psi)
    position = T[:3, 3]
    error = desired_position - position
    return error.dot(error)

def inverse_kinematics_position(x_des, y_des, z_des, initial_guess=None,lengths=ROBOT_LENGTHS, base_position=BASE_POSITION,psi=BASE_YAW, joint_limits=None):
    desired_position = np.array([x_des, y_des, z_des])
    if initial_guess is None:
        initial_guess = [0, 0, 0, 0, 0]
    x_0 = np.array(initial_guess, dtype=float)
    res = minimize(get_position_error, x_0,args=(desired_position, lengths, base_position, psi),bounds=joint_limits,options={'maxiter': 1000, 'gtol': 1e-10})
    error = get_position_error(res.x, desired_position, lengths, base_position, psi)
    found = bool(error <= POSITION_TOLERANCE**2)
    return res.x, found

TOLERANCE = 0.001

def unit_test_fk(test_number, angles, ee_position, ee_R, psi):
    FK = forward_kinematics(angles[0], angles[1], angles[2], angles[3], angles[4],lengths=ROBOT_LENGTHS, base_position=(0.5, 0.1, 0.1), psi=psi)
    error = np.linalg.norm(FK[0:3, 3] - ee_position)
    if  error > TOLERANCE:
        print('Failed FK test', test_number, 'position error:', error)
        return False

    eye_FK = FK[0:3, 0:3] @ np.linalg.inv(ee_R)
    error = np.linalg.norm(np.identity(3) - eye_FK)
    if  error > TOLERANCE:
        print('Failed FK test', test_number, 'rotation error:', error)
        return False
    error = np.linalg.norm(FK[3] - np.array([0, 0, 0, 1]))
    if  error > TOLERANCE:
        print('Failed FK test', test_number, 'last row error:', error)
        return False
    print('Passed FK test', test_number)
    return True


def check_ik_position(angles, target):
    FK = forward_kinematics(angles[0], angles[1], angles[2], angles[3], angles[4], lengths=ROBOT_LENGTHS, base_position=(0.5, 0.1, 0.1), psi=0)
    error = np.linalg.norm(FK[0:3, 3] - target)
    if  error > POSITION_TOLERANCE:
        return False
    return True


def unit_test_ik(test_number, target, expected_found, initial_guess,joint_limits, expected_angles, second_guess):
    angles, found = inverse_kinematics_position(target[0], target[1], target[2], initial_guess=initial_guess,lengths=ROBOT_LENGTHS, base_position=(0.5, 0.1, 0.1), psi=0,joint_limits=joint_limits)
    if found != expected_found:
        print('Failed IK test', test_number, 'expected found:', expected_found,
              'received:', found)
        return False

    if expected_found:
        if not check_ik_position(angles, target):
            print('Failed IK test', test_number, 'incorrect position')
            return False

    if expected_angles is not None:
        error = np.linalg.norm(angles - expected_angles)
        if  error > TOLERANCE:
            print('Failed IK test', test_number, 'incorrect joint angles')
            return False

    if second_guess is not None:
        angles_b, found_b = inverse_kinematics_position(target[0], target[1], target[2], initial_guess=second_guess,lengths=ROBOT_LENGTHS, base_position=(0.5, 0.1, 0.1), psi=0,joint_limits=joint_limits)
        if not found_b or not check_ik_position(angles_b, target):
            print('Failed IK test', test_number, 'second solution did not reach target')
            return False

        difference = (angles - angles_b + math.pi) % (2 * math.pi) - math.pi
        if np.linalg.norm(difference) < TOLERANCE:
            print('Failed IK test', test_number, 'solutions are the same')
            return False
    print('Passed IK test', test_number)
    return True


def run_tests():
    fk_tests = [
        [[0, 0, 0, 0, 0], np.array([0.6818, 0.1, 0.5387]), np.identity(3), 0],
        [[math.pi/2, 0, 0, 0, 0], np.array([0.5388, 0.2430, 0.5387]),
         np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), 0],
        [[0, math.pi/2, 0, 0, 0], np.array([0.8913, 0.1, 0.1040]),
         np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), 0],
        [[0, 0, math.pi/2, -math.pi/2, 0], np.array([0.8167, 0.1, 0.4038]),
         np.identity(3), 0],
        [[0, 0, 0, 0, math.pi/2], np.array([0.5, 0.2818, 0.5387]),
         np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), math.pi/2]
    ]

    ik_tests = [
        [np.array([0.6818, 0.1, 0.5387]), True, [0.1, 0.1, 0.1, 0.1, 0],
         None, None, None],
        [np.array([0.8913, 0.1, 0.1040]), True, [0, 0, 0, 0, 0],
         None, None, None],
        [np.array([10, 10, 10]), False, [0, 0, 0, 0, 0], None, None, None],
        [np.array([0.5388, 0.2430, 0.5387]), True, [0, 0, 0, 0, 0],
         [(-math.pi, math.pi), (0, 0), (0, 0), (0, 0), (0, 0)],
         np.array([math.pi/2, 0, 0, 0, 0]), None],
        [np.array([0.6818, 0.1, 0.5387]), True, [0.1, 0.1, 0.1, 0.1, 0],
         None, None, [0.1, 0.1, 0.1, 0.1, math.pi/2]]
    ]
    print('Running FK tests:')
    fk_successes = 0
    test_number = 1
    for row in fk_tests:
        if unit_test_fk(test_number, row[0], row[1], row[2], row[3]):
            fk_successes += 1
        test_number += 1
    print('Successful FK tests:', fk_successes, '/', len(fk_tests))
    print('\nRunning IK tests:')
    ik_successes = 0
    test_number = 1
    for row in ik_tests:
        if unit_test_ik(test_number, row[0], row[1], row[2], row[3], row[4], row[5]):
            ik_successes += 1
        test_number += 1
    print('Successful IK tests:', ik_successes, '/', len(ik_tests))
    return len(fk_tests) + len(ik_tests) - fk_successes - ik_successes

if __name__ == '__main__':
    failures = run_tests()
    if failures > 0:
        raise SystemExit(1)
