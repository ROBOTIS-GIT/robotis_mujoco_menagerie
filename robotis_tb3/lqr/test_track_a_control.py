"""Part D checks: no MuJoCo, no rendering. Run from the repo root:
    python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (shared modules, `lqr` package)
import unittest
import numpy as np
from turtlebot_model import wrap_angle
from track_a_navigation import OccupancyPlanner, StateFeedbackTracker
from lqr.track_a_control import LQRPathTracker, ReferencePath, project_on_segment
from lqr.track_a_control_design import bryson_weights, lqr_discrete, V_NOM

ANGLES = np.linspace(-np.pi, np.pi, 360, endpoint=False)
FAR = np.full(360, 3.5)          # no LiDAR returns near the robot


def blocked_planner():
    planner = OccupancyPlanner()
    planner.occupied[12:23, 10:20] = True      # same box as the A* test
    return planner


def kinematic_run(tracker, start, seconds=40, noise=0.0, seed=0, planner=None):
    """Unicycle kinematics + real planner/reference/tracker. Estimate = truth + noise."""
    rng = np.random.default_rng(seed)
    planner = planner or OccupancyPlanner()
    reference = ReferencePath(planner)
    truth = np.array(start, dtype=float)
    xte = []
    for _ in range(int(seconds * 10)):
        est = truth + rng.normal(0, noise, 3)
        path = planner.plan(est[:2], tracker.goal)
        if not tracker.docking:
            tracker.reference = reference.update(est[:2], path, tracker.goal)
        v, w, status = tracker.command(est, path, ANGLES, FAR, True)
        if tracker.docking:
            break
        if len(reference.points) >= 2:
            xte.append(reference.project(truth)[2])
        truth = truth + 0.1 * np.array([v * np.cos(truth[2]), v * np.sin(truth[2]), w])
    return truth, np.array(xte), reference


class ControlTests(unittest.TestCase):
    def test_gain_is_the_design_file_lqr_gain(self):
        t = LQRPathTracker()
        Q, R = bryson_weights()
        np.testing.assert_allclose(t.K, lqr_discrete(V_NOM, Q, R)[0])
        self.assertTrue(np.all(t.K > 0))

    def test_projection_sign_left_positive(self):
        s, length, psi, ey = project_on_segment(np.zeros(2), np.array([2., 0.]), np.array([1., .3]))
        self.assertAlmostEqual(ey, .3)
        self.assertAlmostEqual(s, 1.)

    def test_straighten_reduces_corners_and_stays_free(self):
        planner = blocked_planner()
        path = planner.plan(np.array([0., 0.]))
        ref = ReferencePath(planner)
        pts = ref.straighten(path)
        self.assertLess(len(pts), len(path))
        for a, b in zip(pts[:-1], pts[1:]):
            self.assertTrue(ref._segment_free(a, b))

    def test_reference_is_latched_when_estimate_moves(self):
        planner = OccupancyPlanner()
        ref = ReferencePath(planner)
        goal = np.array([1.95, 0.])
        p0 = ref.update(np.array([0., 0.]), planner.plan(np.array([0., 0.]), goal), goal)
        p1 = ref.update(np.array([.3, .04]), planner.plan(np.array([.3, .04]), goal), goal)
        self.assertIs(p0, p1)
        self.assertEqual(ref.adoptions, 1)

    def test_reference_replans_after_persistent_block(self):
        planner = OccupancyPlanner()
        ref = ReferencePath(planner)
        goal = np.array([1.95, 0.])
        est = np.array([0., 0.])
        first = ref.update(est, planner.plan(est, goal), goal)
        planner.occupied[12:23, 10:20] = True            # obstacle appears on the route
        for _ in range(ref.blocked_frames):
            current = ref.update(est, planner.plan(est, goal), goal)
        self.assertIsNot(first, current)
        for a, b in zip(current[:-1], current[1:]):
            self.assertTrue(ref._segment_free(a, b, skip_near=est))

    def test_left_offset_turns_right_and_right_offset_turns_left(self):
        t = LQRPathTracker()
        t.reference = [np.array([0., 0.]), np.array([2., 0.])]
        v, w, _ = t.follow_path(np.array([.5, .1, 0.]), [])
        self.assertGreater(v, 0)
        self.assertLess(w, 0)
        v, w, _ = t.follow_path(np.array([.5, -.1, 0.]), [])
        self.assertGreater(w, 0)

    def test_rotate_in_place_ignores_lateral_error_and_has_hysteresis(self):
        t = LQRPathTracker()
        t.reference = [np.array([0., 0.]), np.array([2., 0.])]
        v, w, _ = t.follow_path(np.array([.5, .3, .6]), [])   # |e_theta|>0.4
        self.assertEqual(v, 0.0)
        self.assertAlmostEqual(w, -t.w_max)                   # saturated heading term
        t2 = LQRPathTracker()
        t2.reference = t.reference
        _, wa, _ = t2.follow_path(np.array([.5, .3, .45]), [])
        t3 = LQRPathTracker()
        t3.reference = t.reference
        _, wb, _ = t3.follow_path(np.array([.5, -.3, .45]), [])
        self.assertAlmostEqual(wa, wb)                        # e_y has no effect at v=0
        v, _, _ = t.follow_path(np.array([.5, .3, .3]), [])   # between 0.25 and 0.4: keep rotating
        self.assertEqual(v, 0.0)
        v, _, _ = t.follow_path(np.array([.5, .3, .2]), [])   # below 0.25: drive
        self.assertGreater(v, 0.0)

    def test_segment_switch_wraps_heading_error(self):
        t = LQRPathTracker()
        t.reference = [np.array([0., 0.]), np.array([1., 0.]), np.array([1., 1.])]
        t.follow_path(np.array([.95, 0., 0.]), [])             # within 10 cm of vertex
        self.assertEqual(t.seg, 1)
        t.follow_path(np.array([1., .5, -np.pi + .1]), [])
        self.assertLessEqual(abs(t.last_error[1]), np.pi)

    def test_closed_loop_step_against_fixed_line_converges(self):
        t = LQRPathTracker()
        t.reference = [np.array([0., 0.]), np.array([1.95, 0.])]
        truth = np.array([0., .15, 0.])                        # 15 cm lateral step
        ey = []
        for _ in range(250):
            v, w, _ = t.follow_path(truth, [])                 # estimate == truth here
            truth = truth + 0.1 * np.array([v * np.cos(truth[2]), v * np.sin(truth[2]), w])
            ey.append(truth[1])
            if truth[0] > 1.8:
                break
        ey = np.array(ey)
        self.assertLess(abs(ey[-1]), .01)
        self.assertLess(np.max(-ey), .03)                      # small overshoot
        self.assertLess(abs(ey[int(8 / .1)]), .03)             # settled within 8 s

    def test_closed_loop_reaches_dock_handoff(self):
        t = LQRPathTracker()
        truth, xte, ref = kinematic_run(t, [0., .15, 0.], seconds=25)
        self.assertTrue(t.docking)

    def test_closed_loop_with_estimate_noise_stays_bounded(self):
        t = LQRPathTracker()
        truth, xte, ref = kinematic_run(t, [0., 0., 0.], seconds=25, noise=.03, seed=1,
                                        planner=blocked_planner())
        self.assertLess(np.max(np.abs(xte)), .30)

    def test_baseline_tracker_behaviour_unchanged(self):
        t = StateFeedbackTracker()
        path = [np.array([0., 0.]), np.array([.1, 0.]), np.array([.3, 0.]), np.array([.5, 0.])]
        v, w, status = t.command(np.array([0., 0., 0.]), path, ANGLES, FAR, True)
        self.assertAlmostEqual(v, .18)
        self.assertAlmostEqual(w, 0.0)
        self.assertEqual(status, 'LIDAR PATH / EKF TRACK')


if __name__ == '__main__':
    unittest.main()
