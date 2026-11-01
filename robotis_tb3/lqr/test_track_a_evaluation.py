"""Sim-free checks for the Part D evaluation tooling (fake run(), synthetic results)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (shared modules, `lqr` package)
import json, tempfile, unittest
from unittest import mock
import numpy as np
for _lib in ('machinevisiontoolbox', 'spatialmath'):   # tooling under test never calls them
    try:
        __import__(_lib)
    except ImportError:
        sys.modules[_lib] = mock.MagicMock()
from lqr import evaluate_controllers as ev
from lqr import analyse_controllers as an
from lqr import part_d_summary as ps
from lqr import diagnose_no_path as dn


def fake_result(args):
    rng = np.random.default_rng(args.seed + (0 if args.controller == 'lqr' else 100))
    scale = 1.0 if args.controller == 'lqr' else 2.0
    return {'success': bool(args.seed % 7), 'collision': False, 'final_status': 'ARRIVED',
            'simulation_seconds': 45 + rng.normal(),
            'final_lateral_error_truth_m': .02,
            'tracking': {'tracking_seconds': 30, 'xte_rms_truth_m': .06 * scale * (1 + .1 * rng.normal()),
                         'xte_max_truth_m': .15 * scale, 'xte_rms_driving_m': .05 * scale,
                         'xte_est_minus_truth_rms_m': .04,
                         'effort_w2_driving': 1.5 / np.sqrt(args.lqr_r_scale) * scale,
                         'rotate_seconds': 8, 'reference_adoptions': 7}}


class EvalToolTests(unittest.TestCase):
    def test_parse_spec(self):
        self.assertEqual(ev.parse_spec('heuristic'), ('heuristic', 1.0, 'heuristic', 0.4, False))
        self.assertEqual(ev.parse_spec('lqr'), ('lqr', 1.0, 'lqr', 0.4, False))
        self.assertEqual(ev.parse_spec('lqr:0.25'), ('lqr', 0.25, 'lqr_r0.25', 0.4, False))
        self.assertEqual(ev.parse_spec('lqr@0.6'), ('lqr', 1.0, 'lqr_rot0.6', 0.6, False))
        self.assertEqual(ev.parse_spec('heuristic+recovery'), ('heuristic', 1.0, 'heuristic_recovery', 0.4, True))
        self.assertEqual(ev.parse_spec('lqr:2@0.8+recovery'), ('lqr', 2.0, 'lqr_r2_rot0.8_recovery', 0.8, True))
        for bad in ('pid', 'heuristic:2', 'heuristic@0.6', 'lqr:0', 'lqr@0.05', 'lqr@2', 'lqr+recover', 'lqr:'):
            with self.assertRaises(ValueError):
                ev.parse_spec(bad)

    def test_wilson_interval_is_sane(self):
        lo, hi = an.wilson(10, 10)
        self.assertLess(lo, 1.0); self.assertEqual(hi, 1.0)       # 10/10 is not "certain"
        lo, hi = an.wilson(0, 10)
        self.assertEqual(lo, 0.0); self.assertGreater(hi, 0.0)
        lo, hi = an.wilson(5, 10)
        self.assertLess(lo, .5); self.assertGreater(hi, .5)

    def test_runner_is_resumable_and_writes_summary(self):
        with tempfile.TemporaryDirectory() as d:
            calls = []

            def fake_run(args):
                calls.append((args.controller, args.seed, args.lqr_r_scale, args.pixel_noise,
                              args.lqr_rotate_enter, args.recovery))
                out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
                r = fake_result(args)
                (out / 'result.json').write_text(json.dumps(r))
                return r

            argv = ['x', '--controllers', 'heuristic', 'lqr:0.5@0.6+recovery', '--conditions', 'nominal',
                    'camera_noise_dropout', '--trials', '3', '--output', d]
            with mock.patch.object(ev, 'run', fake_run), mock.patch.object(sys, 'argv', argv):
                ev.main()
                self.assertEqual(len(calls), 2 * 2 * 3)
                self.assertIn(('lqr', 1, 0.5, 5, 0.6, True), calls)        # condition + variant params applied
                self.assertIn(('heuristic', 1, 1.0, 5, 0.4, False), calls)
                ev.main()                                                   # second pass: all cached
                self.assertEqual(len(calls), 12)
            summary = json.loads((Path(d) / 'summary.json').read_text())
            self.assertEqual(len(summary), 12)
            self.assertEqual({r['controller_tag'] for r in summary}, {'heuristic', 'lqr_r0.5_rot0.6_recovery'})
            self.assertTrue(all('recovery' in r and 'lqr_rotate_enter' in r for r in summary))

    def test_report_and_plots_on_synthetic_sweep(self):
        results = []
        for tag, name, rs in [('heuristic', 'heuristic', 1.0), ('lqr_r0.25', 'lqr', .25), ('lqr_r1', 'lqr', 1.0),
                              ('lqr_r4', 'lqr', 4.0), ('lqr', 'lqr', 1.0)]:
            for cond in ('nominal', 'slippery_floor'):
                for seed in range(8):
                    ns = type('A', (), dict(seed=seed, controller=name, lqr_r_scale=rs))()
                    r = fake_result(ns)
                    r.update(condition=cond, controller_tag=tag, controller_name=name, lqr_r_scale=rs, seed=seed)
                    results.append(r)
        md = an.report(results)
        self.assertIn('Wilcoxon p=', md)
        self.assertIn('## Failures', md)
        self.assertIn('seed 0:', md)                                        # seed % 7 == 0 fails
        with tempfile.TemporaryDirectory() as d:
            made = an.plots(results, Path(d))
            self.assertEqual(made, ['tracking_boxplot.png', 'tradeoff.png'])
            for m in made:
                self.assertGreater((Path(d) / m).stat().st_size, 5000)

    def test_variants_pair_against_their_base_and_sweep_plot_ignores_them(self):
        tags = ['heuristic', 'lqr', 'heuristic_recovery', 'lqr_rot0.6', 'lqr_r2']
        self.assertEqual(an.comparison_pairs(tags),
                         [('heuristic', 'lqr'), ('heuristic', 'heuristic_recovery'),
                          ('lqr', 'lqr_rot0.6'), ('lqr', 'lqr_r2')])
        self.assertEqual(sorted(['lqr', 'heuristic_recovery', 'heuristic'], key=an.tag_order),
                         ['heuristic', 'heuristic_recovery', 'lqr'])

    def test_failed_trials_are_excluded_from_tracking_metrics_and_pairs(self):
        def trial(tag, seed, ok, xte):
            return {'condition': 'c', 'controller_tag': tag, 'controller_name': tag, 'lqr_r_scale': 1.0,
                    'seed': seed, 'success': ok, 'simulation_seconds': 50, 'final_status': 'x',
                    'collision': False, 'final_lateral_error_truth_m': .02,
                    'tracking': {'tracking_seconds': 6 if not ok else 30, 'xte_rms_truth_m': xte}}
        res = [trial('heuristic', 0, False, .02), trial('heuristic', 1, True, .14),   # stalled run looks "great"
               trial('lqr', 0, True, .13), trial('lqr', 1, True, .13)]
        md = an.report(res)
        row = [l for l in md.splitlines() if l.startswith('| heuristic')][0]
        self.assertIn('1/2', row)                      # success rate still counts the failure
        self.assertIn('14.00', row)                    # mean over the successful trial only
        pr = an.paired(res, 'c', 'heuristic', 'lqr', 'xte_rms_truth_m')
        self.assertEqual(pr['n'], 1)                   # only seed 1 succeeded for both
        self.assertAlmostEqual(pr['median'], -.01)
        self.assertEqual(an.paired(res, 'c', 'heuristic', 'lqr', 'xte_rms_truth_m', False)['n'], 2)
        self.assertIn('14.00', md)

    def test_paired_success_uses_mcnemar_and_reports_both_tests(self):
        def trial(tag, seed, ok):
            return {'condition': 'shifted_obstacle', 'controller_tag': tag, 'controller_name': tag, 'lqr_r_scale': 1.0,
                    'seed': seed, 'success': ok, 'simulation_seconds': 50, 'final_status': 'x', 'collision': False,
                    'final_lateral_error_truth_m': .03, 'tracking': {'tracking_seconds': 30, 'xte_rms_truth_m': .15,
                    'effort_w2': 5., 'effort_w2_driving': 4.}}
        res = [trial('heuristic', i, i % 2 == 1) for i in range(10)] + [trial('heuristic_recovery', i, True) for i in range(10)]
        text = ps.comparison_table(res, 'heuristic', 'heuristic_recovery')
        self.assertIn('exact McNemar p = 0.062', text)          # 5 discordant seeds, all one way
        self.assertIn('5 seeds favour heuristic_recovery, 0 favour heuristic', text)
        self.assertIn('Fisher exact p = 0.033', text)

    def test_no_path_episodes_are_found_with_search_frames_and_heading_error(self):
        def rec(i, path, status='LIDAR PATH / EKF TRACK', acc=False, est_h=0.2):
            return {'time_s': i * .1, 'path': path, 'status': status, 'camera_accepted': acc,
                    'estimate': [0, 0, est_h], 'truth': [0, 0, 0.0]}
        recs = [rec(i, [[0, 0]]) for i in range(5)]
        recs += [rec(5 + i, [], 'NO SAFE PATH - STOP', est_h=.26) for i in range(10)]
        recs += [rec(15 + i, [], 'NO PATH - SEARCH FOR MARKER', est_h=.24) for i in range(3)]
        recs += [rec(18 + i, [[0, 0]], acc=(i == 1), est_h=.1) for i in range(30)]
        eps = dn.episodes(recs)
        self.assertEqual(len(eps), 1)
        e = eps[0]
        self.assertEqual((e['frames'], e['search_frames'], e['ended']), (13, 3, True))
        self.assertAlmostEqual(e['start_s'], .5)
        self.assertEqual((e['accepted_during'], e['accepted_after']), (0, 1))
        self.assertAlmostEqual(e['heading_err_start_deg'], np.degrees(.26), places=4)
        self.assertEqual(dn.episodes(recs[:5]), [])                # no empty paths: no episodes

    def test_missing_tracking_is_not_counted_as_zero_error(self):
        r = {'tracking': {'tracking_seconds': 0.0, 'controller': 'lqr'}, 'simulation_seconds': 3}
        self.assertIsNone(an.metric(r, 'xte_rms_truth_m'))


if __name__ == '__main__':
    unittest.main()