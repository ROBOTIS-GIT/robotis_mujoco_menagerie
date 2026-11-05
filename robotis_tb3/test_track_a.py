"""Numerical, estimator integrity and LiDAR planning checks (no GUI)."""
import unittest
import numpy as np
from turtlebot_model import load_scene
from track_a_estimation import PoseEKF, marker_measurement
from track_a_navigation import OccupancyPlanner, StateFeedbackTracker


class TrackATests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _,cls.spec=load_scene()

    def test_camera_update_reduces_actual_prior_error(self):
        truth=np.array([.4,.2,.1])
        ekf=PoseEKF(truth+[.03,-.03,.04],self.spec)
        before=np.linalg.norm(ekf.pose-truth)
        z=np.r_[marker_measurement(truth),truth[2]]
        self.assertTrue(ekf.correct(z))
        self.assertLess(np.linalg.norm(ekf.pose-truth),before)
        self.assertTrue(np.all(np.linalg.eigvalsh(ekf.P)>0))

    def test_rejected_outlier_does_not_move_state(self):
        ekf=PoseEKF([0,0,0],self.spec)
        before=ekf.pose.copy()
        self.assertFalse(ekf.correct([20,2,2]))
        np.testing.assert_array_equal(ekf.pose,before)
        self.assertFalse(ekf.correct(None))

    def test_dropout_increases_uncertainty(self):
        ekf=PoseEKF([0,0,0],self.spec)
        before=np.trace(ekf.P)
        for _ in range(20):
            ekf.predict(np.array([.4,.4]))
        self.assertGreater(np.trace(ekf.P),before)

    def test_astar_routes_around_obstacle(self):
        planner=OccupancyPlanner()
        planner.occupied[12:23,10:20]=True
        path=planner.plan(np.array([0.,0.]))
        self.assertTrue(path)
        self.assertTrue(all(not planner.occupied[planner.cell(p)] for p in path[1:]))
        self.assertGreater(max(abs(p[1]) for p in path),.4)

    def test_lidar_blocks_forward_command(self):
        tracker=StateFeedbackTracker()
        angles=np.linspace(-np.pi,np.pi,181,endpoint=False)
        scan=np.full(181,4.)
        scan[np.abs(angles)<.4]=.2
        v,w,status=tracker.command(np.zeros(3),[np.array([.5,0])],angles,scan,True)
        self.assertEqual((v,w),(0,0))
        self.assertIn('SAFETY STOP',status)

    def test_docking_requires_camera_and_estimate(self):
        tracker=StateFeedbackTracker()
        angles=np.linspace(-np.pi,np.pi,181,endpoint=False)
        scan=np.full(181,4.)
        tracker.docking=True
        self.assertEqual(tracker.command(np.array([2.36,0,0]),[],angles,scan,False)[:2],(0,0))
        self.assertEqual(tracker.command(np.array([2.0,0,-1.0]),[],angles,scan,False)[0],0)
        for _ in range(5):
            tracker.command(np.array([2.36,0,0]),[],angles,scan,True)
        self.assertTrue(tracker.arrived)


    def docking_fixture(self):
        tracker=StateFeedbackTracker()
        tracker.docking=True
        angles=np.linspace(-np.pi,np.pi,181,endpoint=False)
        return tracker,angles,np.full(181,4.)

    def test_estimator_shift_releases_position_capture(self):
        tracker,angles,scan=self.docking_fixture()
        tracker.command(np.array([2.331,0,0]),[],angles,scan,True)
        self.assertTrue(tracker.position_captured)
        speed,_,_=tracker.command(np.array([2.317,0,0]),[],angles,scan,True)
        self.assertFalse(tracker.position_captured)
        self.assertGreater(speed,0)
        self.assertEqual(tracker.arrival_count,0)

    def test_short_camera_gaps_preserve_real_confirmations(self):
        tracker,angles,scan=self.docking_fixture()
        pose=np.array([2.34,0,.02])
        for _ in range(4):
            tracker.command(pose,[],angles,scan,True)
            self.assertEqual(tracker.command(pose,[],angles,scan,False)[:2],(0,0))
            self.assertFalse(tracker.arrived)
        tracker.command(pose,[],angles,scan,True)
        self.assertTrue(tracker.arrived)

    def test_long_camera_gap_expires_confirmation(self):
        tracker,angles,scan=self.docking_fixture()
        pose=np.array([2.34,0,.02])
        for _ in range(4):
            tracker.command(pose,[],angles,scan,True)
        for _ in range(3):
            tracker.command(pose,[],angles,scan,False)
        self.assertEqual(tracker.arrival_count,0)
        tracker.command(pose,[],angles,scan,True)
        self.assertFalse(tracker.arrived)
        self.assertEqual(tracker.arrival_count,1)

    def test_out_of_tolerance_pose_clears_confirmation_during_dropout(self):
        tracker,angles,scan=self.docking_fixture()
        for _ in range(4):
            tracker.command(np.array([2.34,0,0]),[],angles,scan,True)
        tracker.command(np.array([2.34,0,.2]),[],angles,scan,False)
        self.assertEqual(tracker.arrival_count,0)
        self.assertFalse(tracker.arrived)

    def test_q_r_scales_reject_invalid_values(self):
        for scale in (0,-1,np.nan,np.inf):
            with self.assertRaises(ValueError):
                PoseEKF([0,0,0],self.spec,q_scale=scale)
            with self.assertRaises(ValueError):
                PoseEKF([0,0,0],self.spec,r_scale=scale)

    def test_reconstructed_encoders_match_a_curved_encoder_path(self):
        from study_ekf import encoder_increments
        from turtlebot_model import step_pose_from_encoder
        before=np.array([.7,-.3,3.13])
        wheels=np.array([.3,.8])
        after=step_pose_from_encoder(before,*wheels,self.spec)
        np.testing.assert_allclose(encoder_increments(before,after,self.spec),wheels,atol=1e-10)

if __name__=='__main__':
    unittest.main()
