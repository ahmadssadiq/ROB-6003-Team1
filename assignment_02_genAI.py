"""
Assignment 02 (GenAI version) - Forward & Inverse Kinematics of the SO-101 arm.
Kinematic chain from the so101_new_calib URDF:

  base_link --shoulder_pan--> shoulder_link --shoulder_lift--> upper_arm_link
  --elbow_flex--> lower_arm_link --wrist_flex--> wrist_link
  --wrist_roll--> gripper_link --(fixed)--> gripper_frame_link   (end effector)

Each revolute joint turns about its own local z axis. Each URDF <origin> is a
translation followed by R = Rz(yaw) Ry(pitch) Rx(roll); then Rz(q) is applied:
    T_parent_child = Trans(xyz) * Rz(yaw) Ry(pitch) Rx(roll) * Rz(q)
Units: metres, radians. Stationary base frame = URDF base_link.

NOTE: frames, zero angles and the end-effector point follow the URDF. A hand-drawn
model with different frames/zeros will give different numbers for the same angles
unless you convert conventions.
"""
import numpy as np
import scipy.optimize


# ---------------- basic homogeneous transforms ----------------
def HomogeneousTranRot_X(theta_x):
    c, s = np.cos(theta_x), np.sin(theta_x)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranRot_Y(theta_y):
    c, s = np.cos(theta_y), np.sin(theta_y)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranRot_Z(theta_z):
    c, s = np.cos(theta_z), np.sin(theta_z)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def HomogeneousTranTranslation(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


# ---------------- URDF geometry ----------------
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINT_ORIGINS = {
    "shoulder_pan":  ((0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159)),
    "shoulder_lift": ((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0)),
    "elbow_flex":    ((-0.11257, -0.028, 1.73763e-16),   (-3.63608e-16, 8.74301e-16, 1.5708)),
    "wrist_flex":    ((-0.1349, 0.0052, 3.62355e-17),    (4.02456e-15, 8.67362e-16, -1.5708)),
    "wrist_roll":    ((5.55112e-17, -0.0611, 0.0181),    (1.5708, 0.0486795, 3.14159)),
}
GRIPPER_FRAME_ORIGIN = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))  # fixed joint
JOINT_LIMITS = [(-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69),
                (-1.65806, 1.65806), (-2.74385, 2.84121)]


def _origin_transform(xyz, rpy):
    roll, pitch, yaw = rpy
    return (HomogeneousTranTranslation(*xyz) @ HomogeneousTranRot_Z(yaw)
            @ HomogeneousTranRot_Y(pitch) @ HomogeneousTranRot_X(roll))


# ---------------- forward kinematics ----------------
def forward_kinematics(theta_pan, theta_lift, theta_elbow, theta_wrist_flex, theta_wrist_roll):
    """4x4 pose of gripper_frame_link in the stationary base_link frame."""
    T_01 = _origin_transform(*JOINT_ORIGINS["shoulder_pan"])  @ HomogeneousTranRot_Z(theta_pan)
    T_12 = _origin_transform(*JOINT_ORIGINS["shoulder_lift"]) @ HomogeneousTranRot_Z(theta_lift)
    T_23 = _origin_transform(*JOINT_ORIGINS["elbow_flex"])    @ HomogeneousTranRot_Z(theta_elbow)
    T_34 = _origin_transform(*JOINT_ORIGINS["wrist_flex"])    @ HomogeneousTranRot_Z(theta_wrist_flex)
    T_45 = _origin_transform(*JOINT_ORIGINS["wrist_roll"])    @ HomogeneousTranRot_Z(theta_wrist_roll)
    T_5E = _origin_transform(*GRIPPER_FRAME_ORIGIN)
    return T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_5E


def _fk_position(theta):
    return forward_kinematics(*theta)[:3, 3]


# ---------------- inverse kinematics (position only, Lecture 04A style) ----------------
def get_error(theta, desired_position):
    e = _fk_position(theta) - np.asarray(desired_position, dtype=float)
    return e.dot(e)


def inverse_kinematics_position(x_des, y_des, z_des, initial_guess=None, n_restarts=30,
                                seed=0, tol=1e-4, joint_limits=None):
    """
    Returns (theta[5], found). found=True iff the end effector reaches the target
    within `tol` metres (default 0.1 mm).
    - initial_guess: first starting point (default all zeros). With n_restarts=0 the
      search starts ONLY there, so different guesses can expose different solutions.
    - joint_limits: optional override of the 5 (lo, hi) bounds. Setting lo == hi locks
      a joint (used to turn the redundant 5-joint problem into a smaller one).
    """
    limits = JOINT_LIMITS if joint_limits is None else joint_limits
    target = np.array([x_des, y_des, z_des], dtype=float)
    rng = np.random.default_rng(seed)
    first = np.zeros(5) if initial_guess is None else np.asarray(initial_guess, dtype=float)
    guesses = [first] + [np.array([rng.uniform(lo, hi) for lo, hi in limits]) for _ in range(n_restarts)]
    best_theta, best_err = None, np.inf
    for x0 in guesses:
        x0 = np.clip(x0, [l[0] for l in limits], [l[1] for l in limits])
        res = scipy.optimize.minimize(get_error, x0, args=(target,), method="L-BFGS-B",
                                      bounds=limits, options={"ftol": 1e-15, "gtol": 1e-11})
        if res.fun < best_err:
            best_theta, best_err = res.x, res.fun
        if np.sqrt(best_err) < tol * 1e-2:
            break
    return best_theta, bool(np.sqrt(best_err) <= tol)


# =====================================================================
# Unit tests
# FK expected poses are HARD-CODED. They were produced by an independent chain built
# with scipy.spatial.transform.Rotation (not by forward_kinematics above), rounded to 5 dp.
# =====================================================================
def _close(a, b, atol):
    return np.allclose(a, b, atol=atol)

FK_CASES = [
    ((0, 0, 0, 0, 0),
     [[0.00001, -0.00001, 1.0, 0.39136], [0.04866, 0.99882, 0.00001, -0.00001],
      [-0.99882, 0.04866, 0.00001, 0.22647], [0, 0, 0, 1]]),
    ((0.5, -0.6, 0.9, 0.3, 0.4),
     [[-0.63023, 0.27961, 0.72431, 0.25466], [-0.04785, 0.91714, -0.39568, -0.12144],
      [-0.77493, -0.28403, -0.56464, 0.09445], [0, 0, 0, 1]]),
    ((-0.8, 0.7, -1.0, -0.5, 1.2),
     [[0.85872, 0.16429, 0.48539, 0.29834], [-0.42671, 0.75375, 0.49979, 0.25646],
      [-0.28375, -0.6363, 0.71736, 0.34162], [0, 0, 0, 1]]),
    ((1.2, -1.0, 1.3, 0.6, -1.5),
     [[0.92553, 0.30439, 0.22525, 0.11121], [0.37842, -0.7219, -0.57937, -0.16486],
      [-0.01375, 0.62146, -0.78332, 0.04111], [0, 0, 0, 1]]),
    ((0, 1.0, -1.2, 0.8, 0),
     [[-0.56397, 0.02747, 0.82534, 0.43722], [0.04866, 0.99882, 0.00001, -0.00001],
      [-0.82436, 0.04017, -0.56463, 0.08933], [0, 0, 0, 1]]),
]


def _fk_check(i):
    q, expected = FK_CASES[i]
    assert _close(forward_kinematics(*q), expected, 5e-5), forward_kinematics(*q)

def test_fk_1_zero_pose():        _fk_check(0)
def test_fk_2_mixed_pose():       _fk_check(1)
def test_fk_3_negative_angles():  _fk_check(2)
def test_fk_4_large_pan_roll():   _fk_check(3)
def test_fk_5_planar_lift_elbow(): _fk_check(4)


# ---- helpers for the provable 0 / 1 / many IK tests -------------------------------
# Lock pan, wrist_flex, wrist_roll. Lift & elbow axes are parallel (both +z of the arm plane),
# so the tip is  L0 * Rz(lift) * [ e + Rz(pi/2+elbow) * w ]  : a planar 2-link problem with
# fixed height. Max planar radius = |e| + |w| (triangle inequality, attained only when collinear).
PAN, FLEX, ROLL = 0.3, 0.0, 0.0
LOCK = [(PAN, PAN), JOINT_LIMITS[1], JOINT_LIMITS[2], (FLEX, FLEX), (ROLL, ROLL)]


def _lift_frame():
    return (_origin_transform(*JOINT_ORIGINS["shoulder_pan"]) @ HomogeneousTranRot_Z(PAN)
            @ _origin_transform(*JOINT_ORIGINS["shoulder_lift"]))        # lift frame at lift = 0


def _tip_in_upper_arm(elbow):
    T = (_origin_transform(*JOINT_ORIGINS["elbow_flex"]) @ HomogeneousTranRot_Z(elbow)
         @ _origin_transform(*JOINT_ORIGINS["wrist_flex"]) @ HomogeneousTranRot_Z(FLEX)
         @ _origin_transform(*JOINT_ORIGINS["wrist_roll"]) @ HomogeneousTranRot_Z(ROLL)
         @ _origin_transform(*GRIPPER_FRAME_ORIGIN))
    return T[:3, 3]


def _tip_world(lift, elbow):
    p = _lift_frame() @ HomogeneousTranRot_Z(lift) @ np.append(_tip_in_upper_arm(elbow), 1.0)
    return p[:3]


def _extension():
    """(R_max, elbow*).
    R_max = |e| + |w| is computed ANALYTICALLY (triangle inequality).
    elbow* is found NUMERICALLY (scan + 1-D minimisation of -radius), then checked to attain R_max."""
    e_xy = np.array(JOINT_ORIGINS["elbow_flex"][0][:2])
    w_len = np.linalg.norm(_tip_in_upper_arm(0.0)[:2] - e_xy)    # constant for any elbow angle
    r_max = np.linalg.norm(e_xy) + w_len
    grid = np.linspace(*JOINT_LIMITS[2], 400001)
    r = np.array([np.linalg.norm(_tip_in_upper_arm(g)[:2]) for g in grid[::50]])
    g0 = grid[::50][r.argmax()]
    res = scipy.optimize.minimize_scalar(lambda g: -np.linalg.norm(_tip_in_upper_arm(g)[:2]),
                                         bracket=(g0 - 0.01, g0, g0 + 0.01), tol=1e-14)
    assert abs(-res.fun - r_max) < 1e-7           # collinear config attains the triangle bound
    return r_max, res.x


# ---- IK ----
def test_ik_0_solutions():
    # far outside the workspace and far below the base: no solution
    assert inverse_kinematics_position(2.0, 0.0, 0.2)[1] is False
    assert inverse_kinematics_position(0.0, 0.0, -0.5)[1] is False
    # 1 mm beyond the analytic max reach (locked joints): provably unreachable
    r_max, _ = _extension()
    L0 = _lift_frame()
    tip_L = np.linalg.inv(L0) @ np.append(_tip_world(-0.5, _extension()[1]), 1.0)
    scale = (r_max + 1e-3) / np.linalg.norm(tip_L[:2])
    beyond = L0 @ np.array([tip_L[0] * scale, tip_L[1] * scale, tip_L[2], 1.0])
    th, ok = inverse_kinematics_position(*beyond[:3], joint_limits=LOCK, n_restarts=20)
    assert ok is False
    assert np.linalg.norm(_fk_position(th) - beyond[:3]) > 5e-4      # best effort misses by ~1 mm


def test_ik_1_solution_full_extension():
    # Planar arm fully extended: triangle inequality is an equality, so lift/elbow are unique.
    r_max, elbow_star = _extension()
    lift_star = -0.5
    assert JOINT_LIMITS[2][0] < elbow_star < JOINT_LIMITS[2][1]
    target = _tip_world(lift_star, elbow_star)
    sols = []
    for seed in range(15):
        th, ok = inverse_kinematics_position(*target, joint_limits=LOCK, n_restarts=10,
                                             seed=seed, tol=1e-6)
        assert ok and np.linalg.norm(_fk_position(th) - target) <= 1e-6
        sols.append(th)
    expected = np.array([PAN, lift_star, elbow_star, FLEX, ROLL])
    for th in sols:
        # Singular pose: position error ~ 0.035*dq^2, so 1e-6 m allows dq ~ 5e-3 rad -> tolerance 1e-2.
        assert _close(th, expected, 1e-2), th        # every restart returns the same joint vector


def test_ik_many_solutions_distinct():
    # All five joints FREE (no locking): a 3-D target with 5 joints is redundant, so interior points
    # have a continuum of solutions. Start from different guesses and require that the returned
    # joint vectors are genuinely different while each one reaches the same target.
    q_true = np.array([0.3, -0.5, 0.8, 0.2, 0.0])
    target = _fk_position(q_true)
    guesses = [q_true + [0, 0, 0, 0, 0], [0.3, 0.8, -0.8, 0.2, 0.0],
               [-0.5, 0.2, 0.5, -0.4, 1.0], [0.9, -1.0, 1.2, 0.9, -1.0]]
    sols = []
    for g in guesses:
        th, ok = inverse_kinematics_position(*target, initial_guess=g, n_restarts=0)
        if ok:
            assert np.linalg.norm(_fk_position(th) - target) <= 1e-4    # each truly reaches target
            sols.append(th)
    assert len(sols) >= 3
    spread = max(np.linalg.norm(a - b) for a in sols for b in sols)
    assert spread > 0.1, f"solutions not distinct (max joint-space distance {spread:.4f})"


def test_ik_round_trip_hardcoded_targets():
    targets = [FK_CASES[i][1] for i in range(5)]       # reuse the hard-coded FK positions
    for T in targets:
        pos = [T[0][3], T[1][3], T[2][3]]
        th, ok = inverse_kinematics_position(*pos)
        assert ok and np.linalg.norm(_fk_position(th) - pos) <= 1e-4


def test_ik_joint_limits_respected():
    th, ok = inverse_kinematics_position(0.3, 0.1, 0.2)
    assert ok
    for t, (lo, hi) in zip(th, JOINT_LIMITS):
        assert lo - 1e-9 <= t <= hi + 1e-9


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for t in tests:
        try:
            t(); print(f"PASS  {t.__name__}"); passed += 1
        except Exception as e:
            print(f"FAIL  {t.__name__}: {e!r}")
    print(f"\n{passed}/{len(tests)} tests passed")

