"""Sim-free checks for the opt-in no-path recovery and the rotate-threshold option."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/
import unittest
import numpy as np
from track_a_estimation import marker_measurement
from track_a_navigation import StateFeedbackTracker, RECOVERY_FRAMES, SEARCH_TURN
from lqr.track_a_control import LQRPathTracker, ReferencePath

ANGLES = np.linspace(-np.pi, np.pi, 360, endpoint=False)
FAR = np.full(360, 3.5)
# The stall pose from the shifted_obstacle failures: heading estimate 15 degrees too high.
STALL = np.array([.251, .292, 1.203])
GOOD_PATH = [np.array([.25, .29]), np.array([.5, .29]), np.array([1.95, 0.])]


def step(tracker, pose, path):
    return tracker.command(pose, path, ANGLES, FAR, False)


class RecoveryTests(unittest.TestCase):
    def test_default_behaviour_is_unchanged_a_permanent_stop(self):
        t = StateFeedbackTracker()
        for _ in range(3 * RECOVERY_FRAMES):
            v, w, status = step(t, STALL, [])
        self.assertEqual((v, w, status), (0.0, 0.0, 'NO SAFE PATH - STOP'))
        self.assertEqual(t.search_frames, 0)
        self.assertEqual(t.no_path_frames, 3 * RECOVERY_FRAMES)       # metric counts even when off

    def test_recovery_waits_then_turns_toward_the_estimated_marker_bearing(self):
        t = StateFeedbackTracker(recovery=True)
        for _ in range(RECOVERY_FRAMES - 1):
            self.assertEqual(step(t, STALL, [])[2], 'NO SAFE PATH - STOP')
        v, w, status = step(t, STALL, [])
        self.assertEqual(v, 0.0)
        self.assertEqual(status, 'NO PATH - SEARCH FOR MARKER')
        bearing = marker_measurement(STALL)[1]
        self.assertLess(bearing, 0)                                      # marker is to the right
        self.assertAlmostEqual(w, -SEARCH_TURN)                          # so turn right (clockwise)

    def test_search_keeps_its_direction_when_the_marker_is_near_dead_ahead(self):
        t = StateFeedbackTracker(recovery=True)
        t.no_path_run = RECOVERY_FRAMES
        t._search_dir = -1.0
        ahead = np.array([.25, .29, np.arctan2(-.29, 2.75)])             # facing the marker
        self.assertAlmostEqual(step(t, ahead, [])[1], -SEARCH_TURN)      # keeps scanning, no flip

    def test_normal_tracking_resumes_as_soon_as_a_path_returns(self):
        t = StateFeedbackTracker(recovery=True)
        for _ in range(RECOVERY_FRAMES + 3):
            step(t, STALL, [])
        self.assertGreater(t.search_frames, 0)
        status = step(t, np.array([.25, .29, .0]), GOOD_PATH)[2]
        self.assertEqual(status, 'LIDAR PATH / EKF TRACK')
        self.assertEqual(t.no_path_run, 0)

    def test_short_planner_gaps_do_not_trigger_the_search(self):
        t = StateFeedbackTracker(recovery=True)
        for _ in range(5):
            step(t, STALL, [])
        step(t, np.array([.25, .29, .0]), GOOD_PATH)
        for _ in range(5):
            step(t, STALL, [])                                           # run restarted, still < 10
        self.assertEqual(t.search_frames, 0)

    def test_lqr_tracker_stops_following_a_stale_reference_when_recovering(self):
        t = LQRPathTracker(recovery=True)
        t.reference = [np.array([.25, .29]), np.array([1.95, 0.])]       # old route still latched
        status = [step(t, np.array([.25, .29, .0]), [])[2] for _ in range(RECOVERY_FRAMES + 1)]
        self.assertEqual(status[0], 'LIDAR PATH / LQR TRACK')            # before: follows stale reference
        self.assertEqual(status[-1], 'NO PATH - SEARCH FOR MARKER')      # after 1 s: searches instead

    def test_search_is_never_used_while_docking(self):
        t = StateFeedbackTracker(recovery=True)
        t.docking = True
        t.no_path_run = 10 * RECOVERY_FRAMES
        status = t.command(np.array([2.3, 0.0, 0.0]), [], ANGLES, FAR, True)[2]
        self.assertNotEqual(status, 'NO PATH - SEARCH FOR MARKER')


class RotateThresholdTests(unittest.TestCase):
    def _speed(self, enter, e_theta):
        t = LQRPathTracker(rotate_enter=enter)
        t.reference = [np.array([0., 0.]), np.array([2., 0.])]
        return t.follow_path(np.array([.5, 0., e_theta]), [])[0]

    def test_higher_threshold_keeps_driving_through_larger_heading_error(self):
        self.assertEqual(self._speed(0.40, 0.5), 0.0)                    # default rotates at 0.5 rad
        self.assertGreater(self._speed(0.60, 0.5), 0.0)                  # raised threshold drives on

    def test_default_threshold_and_hysteresis_ratio_match_the_original_tracker(self):
        t = LQRPathTracker()
        self.assertEqual(t.rotate_enter, 0.40)
        self.assertAlmostEqual(t.rotate_exit, 0.25)
        t2 = LQRPathTracker(rotate_enter=0.8)
        self.assertAlmostEqual(t2.rotate_exit, 0.5)                      # same 0.625 ratio

    def test_out_of_range_threshold_is_rejected(self):
        for bad in (0.0, 0.05, 1.5):
            with self.assertRaises(ValueError):
                LQRPathTracker(rotate_enter=bad)


if __name__ == '__main__':
    unittest.main()
