"""Sim-free check of the tracking figure script on a synthetic trace."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/
import gzip, json, tempfile, unittest
import numpy as np
from lqr import plot_tracking as pt


def synthetic(n=60, with_reference=True):
    recs = []
    for i in range(n):
        x = 0.03 * i
        recs.append({'time_s': i * .1, 'estimate': [x + .01, .02, .0], 'truth': [x, .0, .01 * i / n],
                     'covariance': np.diag([.01, .01, .002]).tolist(), 'camera_accepted': i % 7 == 0,
                     'status': 'LIDAR PATH / LQR TRACK', 'speed_cmd': 0.0 if i < 10 else .18,
                     'turn_cmd': .8 if i < 10 else -.1,
                     'track': {'e_y_truth': .02, 'e_y_est': .01, 'e_theta_truth': .0, 'e_theta_est': .0},
                     'reference': ([[0, 0], [1.8, 0]] if (with_reference and i == 0) else None)})
    return recs


class PlotTests(unittest.TestCase):
    def test_series_shapes_and_yaw_rate(self):
        d = pt.series(synthetic())
        self.assertEqual(len(d['t']), 60)
        self.assertAlmostEqual(d['achieved'][0], 0.01 / 60 / .1 * 1.0, places=6)    # d(heading)/dt from truth
        self.assertTrue(np.isnan(d['achieved'][-1]))
        self.assertEqual(len(d['references']), 1)
        self.assertTrue(d['tracking'].all())

    def test_figure_is_written_with_and_without_reference_or_occupancy(self):
        with tempfile.TemporaryDirectory() as tmp:
            for flag in (True, False):
                fig = pt.make_figure(synthetic(with_reference=flag), occupancy=np.zeros((41, 29), bool) if flag else None, title='t')
                out = Path(tmp) / f'f{flag}.png'
                fig.savefig(out, dpi=60)
                self.assertGreater(out.stat().st_size, 5000)

    def test_gzip_traces_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / 'trace.json.gz'
            with gzip.open(f, 'wt') as h:
                json.dump(synthetic(5), h)
            self.assertEqual(len(pt.load_trace(f)), 5)


if __name__ == '__main__':
    unittest.main()
