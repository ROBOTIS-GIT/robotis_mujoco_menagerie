"""Part D evaluation: controller x condition x seed grid. Resumable.

A controller spec is  heuristic | lqr  with optional modifiers, in this order:

  lqr:<r_scale>      R multiplied by r_scale (larger = gentler turning)   e.g. lqr:0.5
  lqr@<rotate>       rotate-in-place threshold [rad] (default 0.40)        e.g. lqr@0.6
  <spec>+recovery    rotate to search for the marker when the planner has no route for 1 s
                     e.g. heuristic+recovery, lqr:2@0.6+recovery

The same spec list covers every study:

  comparison  python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 10 --output eval_cmp
  R sweep     python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25 lqr:0.5 lqr lqr:2 lqr:4 \\
                  --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep
  rotate      python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr@0.6 lqr@0.8 \\
                  --conditions nominal slippery_floor --trials 10 --output eval_rotate
  recovery    python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic heuristic+recovery \\
                  --conditions shifted_obstacle --trials 10 --output eval_recovery

Layout: <output>/<tag>/<condition>/seed-<n>/result.json (+ trace). summary.json is rewritten
after every trial; a trial whose result.json already exists is loaded, not re-run, so an
interrupted batch continues where it stopped. Failures stay in the summary.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (shared modules, `lqr` package)
import json
import re
from track_a_project import parser, run

# Same conditions as evaluate_track_a.py so results are comparable with the baseline.
CONDITIONS = {
    'nominal': {},
    'camera_noise_dropout': {'pixel_noise': 5, 'dropout': .30},
    'shifted_obstacle': {'obstacle_shift': .15, 'encoder_bias': .004},
    'slippery_floor': {'friction': .6},
}


SPEC = re.compile(r'^(heuristic|lqr)(?::([0-9]*\.?[0-9]+))?(?:@([0-9]*\.?[0-9]+))?(\+recovery)?$')


def parse_spec(spec):
    """'lqr:0.5@0.6+recovery' -> ('lqr', 0.5, 'lqr_r0.5_rot0.6_recovery', 0.6, True).

    Returns (name, r_scale, tag, rotate_enter, recovery). Tags for the older specs are
    unchanged ('lqr', 'lqr_r0.25', 'heuristic'), so cached trials are still found."""
    m = SPEC.match(spec)
    if not m:
        raise ValueError(f'bad controller spec {spec!r}')
    name, scale, rotate, recovery = m.group(1), m.group(2), m.group(3), bool(m.group(4))
    if name == 'heuristic' and (scale or rotate):
        raise ValueError('heuristic has no R scale or rotate threshold')
    r = float(scale) if scale else 1.0
    rot = float(rotate) if rotate else 0.40
    if r <= 0:
        raise ValueError('R scale must be positive')
    if not 0.1 <= rot <= 1.2:
        raise ValueError('rotate threshold must be between 0.1 and 1.2 rad')
    tag = name + (f'_r{scale}' if scale else '') + (f'_rot{rotate}' if rotate else '') + ('_recovery' if recovery else '')
    return name, r, tag, rot, recovery


def main():
    p = parser()
    p.add_argument('--trials', type=int, default=10, help='Seeds per controller and condition')
    p.add_argument('--controllers', nargs='+', default=['heuristic', 'lqr'])
    p.add_argument('--conditions', nargs='+', default=list(CONDITIONS), choices=list(CONDITIONS))
    args = p.parse_args()
    specs = [parse_spec(s) for s in args.controllers]
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=True)
    summary_path = root / 'summary.json'
    results = []
    for name, r_scale, tag, rotate, recovery in specs:
        for condition in args.conditions:
            for seed in range(args.trials):
                out = root / tag / condition / f'seed-{seed}'
                done = out / 'result.json'
                if done.exists():
                    result = json.loads(done.read_text())
                else:
                    trial = parser().parse_args([])
                    trial.headless = True
                    trial.seconds = args.seconds
                    trial.seed = seed
                    trial.output = str(out)
                    trial.controller = name
                    trial.lqr_r_scale = r_scale
                    trial.lqr_rotate_enter = rotate
                    trial.recovery = recovery
                    for key, value in CONDITIONS[condition].items():
                        setattr(trial, key, value)
                    result = run(trial)
                result.update(condition=condition, controller_tag=tag, controller_name=name,
                              lqr_r_scale=r_scale, lqr_rotate_enter=rotate, recovery=recovery, seed=seed)
                results.append(result)
                summary_path.write_text(json.dumps(results, indent=2) + '\n')
    n_ok = sum(r['success'] for r in results)
    print(f'Successes {n_ok}/{len(results)}  ->  {summary_path}')


if __name__ == '__main__':
    main()
