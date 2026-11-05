"""Controller safety checks and optional rendered-camera integration tests."""
import contextlib
import io
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np

from camera_navigation import DockingController, MarkerCamera, run


class CameraTests(unittest.TestCase):
    def test_missing_marker_stops_after_motion(self):
        controller = DockingController()
        self.assertGreater(controller.command((2.0, .1))[0], 0)
        self.assertEqual(controller.command(None)[:2], (0, 0))
        self.assertFalse(controller.arrived)

    def test_turn_directions_and_large_error(self):
        self.assertGreater(DockingController().command((2, .2))[1], 0)
        self.assertLess(DockingController().command((2, -.2))[1], 0)
        self.assertEqual(DockingController().command((2, .5))[0], 0)

    def test_blank_and_wrong_marker_are_rejected(self):
        detector = MarkerCamera()
        image = np.full((480, 640, 3), 255, np.uint8)
        self.assertIsNone(detector.observe(image)[0])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        wrong_marker = cv2.aruco.generateImageMarker(dictionary, 7, 200)
        image[140:340, 220:420] = wrong_marker[:, :, None]
        self.assertIsNone(detector.observe(image)[0])

    def test_arrival_requires_consecutive_frames(self):
        controller = DockingController()
        for _ in range(4):
            controller.command((.56, .01))
        self.assertFalse(controller.arrived)
        controller.command(None)
        for _ in range(5):
            controller.command((.56, .01))
        self.assertTrue(controller.arrived)
        self.assertEqual(controller.command((2, .2))[:2], (0, 0))


class RenderedCameraTests(unittest.TestCase):
    def test_dock_left_and_right_and_stop_without_marker(self):
        # Measurements come from rendered pixels. Ground truth is used only
        # for the final independent distance/speed assertions.
        with tempfile.TemporaryDirectory() as temporary:
            for y in (.4, -.4, 3.0):
                with self.subTest(marker_y=y), contextlib.redirect_stdout(io.StringIO()):
                    args = Namespace(marker_y=y, headless=True, seconds=35 if abs(y)<1 else 2,
                                     output=str(Path(temporary) / str(y)))
                    result = run(args)
                    self.assertLess(result['evaluation_stopped_speed_m_s'], .005)
                    if abs(y) < 1:
                        self.assertTrue(result['arrived'])
                        self.assertGreater(result['detections'], 10)
                        self.assertAlmostEqual(result['evaluation_true_distance_m'], .55, delta=.04)
                        self.assertAlmostEqual(result['final_measurement'][0],
                                               result['evaluation_true_distance_m'], delta=.025)
                    else:
                        self.assertFalse(result['arrived'])
                        self.assertEqual(result['detections'], 0)
                        import json
                        records = json.loads((Path(args.output)/'navigation-trace.json').read_text())
                        self.assertTrue(all(r['wheel_commands'] == [0, 0] for r in records))


if __name__ == '__main__':
    unittest.main()
