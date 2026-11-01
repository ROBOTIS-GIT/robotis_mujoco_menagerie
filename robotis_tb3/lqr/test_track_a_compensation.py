"""Sim-free checks for the opt-in actuator compensation wiring (specs, runner, analysis filters, modes)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (shared modules, `lqr` package)
import json, tempfile, unittest
from types import SimpleNamespace
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
import track_a_project as tp
import actuator_compensation as ac


class SpecTests(unittest.TestCase):
    def test_split_comp(self):
        self.assertEqual(ev.split_comp('lqr'), ('lqr', 'off'))
        self.assertEqual(ev.split_comp('lqr+comp'), ('lqr', 'path'))
        self.assertEqual(ev.split_comp('lqr+compall'), ('lqr', 'all'))
        self.assertEqual(ev.split_comp('lqr:2@0.6+recovery+comp'), ('lqr:2@0.6+recovery', 'path'))

    def test_earlier_specs_keep_their_tags(self):
        for spec in ('heuristic', 'lqr', 'lqr:0.25', 'lqr@0.6', 'heuristic+recovery', 'lqr:2@0.8+recovery'):
            *old, = ev.parse_spec(spec)
            new = ev.parse_full_spec(spec)
            self.assertEqual(list(new[:5]), old)
            self.assertEqual(new[5], 'off')

    def test_compensated_tags(self):
        self.assertEqual(ev.parse_full_spec('lqr+comp'), ('lqr', 1.0, 'lqr_comp', 0.4, False, 'path'))
        self.assertEqual(ev.parse_full_spec('heuristic+compall')[2:], ('heuristic_compall', 0.4, False, 'all'))
        self.assertEqual(ev.parse_full_spec('lqr:2+recovery+comp')[2], 'lqr_r2_recovery_comp')

    def test_bad_specs_still_rejected(self):
        for bad in ('lqr+comp+recovery', 'lqr+compfoo', 'heuristic:2+comp', '+comp'):
            with self.assertRaises(ValueError):
                ev.parse_full_spec(bad)


class RunnerTests(unittest.TestCase):
    def test_runner_passes_mode_tags_results_and_caches(self):
        with tempfile.TemporaryDirectory() as d:
            calls = []

            def fake_run(args):
                calls.append((args.controller, args.compensate, args.seed))
                out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
                r = {'success': True, 'collision': False, 'final_status': 'ARRIVED', 'simulation_seconds': 40.0,
                     'final_lateral_error_truth_m': .02,
                     'tracking': {'tracking_seconds': 30, 'xte_rms_truth_m': .05, 'compensate': args.compensate}}
                (out / 'result.json').write_text(json.dumps(r))
                return r

            argv = ['x', '--controllers', 'lqr', 'lqr+comp', 'lqr+compall', '--conditions', 'nominal',
                    '--trials', '2', '--output', d]
            with mock.patch.object(ev, 'run', fake_run), mock.patch.object(sys, 'argv', argv):
                ev.main()
                self.assertEqual(len(calls), 6)
                self.assertEqual({(c[1]) for c in calls}, {'off', 'path', 'all'})
                ev.main()
                self.assertEqual(len(calls), 6)                 # cached
            summary = json.loads((Path(d) / 'summary.json').read_text())
            self.assertEqual({r['controller_tag'] for r in summary}, {'lqr', 'lqr_comp', 'lqr_compall'})
            self.assertEqual({r['compensate'] for r in summary}, {'off', 'path', 'all'})
            for tag in ('lqr', 'lqr_comp', 'lqr_compall'):      # separate cache folders per variant
                self.assertTrue((Path(d) / tag / 'nominal' / 'seed-0' / 'result.json').exists())

    def test_project_parser_flag(self):
        p = tp.parser()
        self.assertEqual(p.parse_args([]).compensate, 'off')            # default keeps all earlier results valid
        self.assertEqual(p.parse_args(['--compensate', 'path']).compensate, 'path')
        with self.assertRaises(SystemExit):
            with mock.patch('sys.stderr'):
                p.parse_args(['--compensate', 'always'])


class AnalysisFilterTests(unittest.TestCase):
    def test_comp_variants_pair_with_their_base(self):
        tags = ['heuristic', 'lqr', 'heuristic_comp', 'lqr_comp', 'lqr_compall']
        self.assertEqual(an.comparison_pairs(tags),
                         [('heuristic', 'lqr'), ('heuristic', 'heuristic_comp'),
                          ('lqr', 'lqr_comp'), ('lqr', 'lqr_compall')])

    def test_sweep_table_ignores_compensated_runs(self):
        def res(tag, comp, scale, seed):
            return {'condition': 'nominal', 'controller_tag': tag, 'controller_name': 'lqr', 'lqr_r_scale': scale,
                    'lqr_rotate_enter': .4, 'recovery': False, 'compensate': comp, 'seed': seed, 'success': True,
                    'collision': False, 'final_status': 'ARRIVED', 'simulation_seconds': 40.0,
                    'final_lateral_error_truth_m': .02,
                    'tracking': {'tracking_seconds': 30, 'xte_rms_driving_m': .05, 'effort_w2_driving': 1.0,
                                 'effort_w2': 2.0, 'rotate_seconds': 3.0}}
        results = [res('lqr', 'off', 1.0, s) for s in range(3)] + [res('lqr_comp', 'path', 1.0, s) for s in range(3)]
        table = ps.sweep_table(results)
        self.assertIn('| lqr |', table)
        self.assertNotIn('lqr_comp', table)

    def test_old_results_without_the_field_count_as_uncompensated(self):
        old = {'condition': 'nominal', 'controller_tag': 'lqr', 'controller_name': 'lqr', 'lqr_r_scale': 1.0,
               'lqr_rotate_enter': .4, 'seed': 0, 'success': True, 'collision': False, 'final_status': 'ARRIVED',
               'simulation_seconds': 40.0, 'final_lateral_error_truth_m': .02,
               'tracking': {'tracking_seconds': 30, 'xte_rms_driving_m': .05, 'effort_w2_driving': 1.0,
                            'effort_w2': 2.0, 'rotate_seconds': 3.0}}
        self.assertIn('| lqr |', ps.sweep_table([old]))


class ModeTests(unittest.TestCase):
    def test_use_compensation_truth_table(self):
        self.assertEqual([ac.use_compensation('off', d) for d in (False, True)], [False, False])
        self.assertEqual([ac.use_compensation('path', d) for d in (False, True)], [True, False])
        self.assertEqual([ac.use_compensation('all', d) for d in (False, True)], [True, True])
        with self.assertRaises(ValueError):
            ac.use_compensation('sometimes', False)

    def test_build_compensator_uses_scene_limit_and_is_feedforward_only(self):
        comp = ac.build_compensator(SimpleNamespace(max_wheel_rate_rad_s=7.88))
        self.assertFalse(comp.use_trim)
        self.assertEqual(comp.model.cmd_limit, 7.88)
        self.assertLessEqual(float(np.max(np.abs(comp.command(0.18, 0.8)))), 7.88 + 1e-9)
        small = ac.build_compensator(SimpleNamespace(max_wheel_rate_rad_s=6.0))
        self.assertLessEqual(float(np.max(np.abs(small.command(0.18, 0.0)))), 6.0 + 1e-9)

    def test_uncompensated_command_is_below_the_friction_band_compensated_is_not(self):
        # 0.03 m/s -> 0.9 rad/s: far below the ~2 rad/s friction offset (does nothing on the real plant);
        # the compensator adds the offset so the wheel actually turns at the requested rate.
        comp = ac.build_compensator(SimpleNamespace(max_wheel_rate_rad_s=7.88))
        cmd = comp.command(0.03, 0.0)
        self.assertGreater(cmd[0], 2.0)
        np.testing.assert_allclose(cmd[0], 0.03 / comp.model.kv + comp.model.c_ff)


if __name__ == '__main__':
    unittest.main()
