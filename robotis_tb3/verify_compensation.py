"""Part D, Step 2c: does the compensator make the plant deliver the twist we ask for?

Bench test, 10 Hz control loop, encoder-quantised wheel rates (1024 ticks/rev, as in
track_a_project.py). The compensator sees ENCODER rates only. Ground-truth base
velocity is read here only to score what the plant actually did.

Methods
  baseline : current mapping  twist_to_wheel_rates(v, w) with nominal r, track
  ff       : feedforward only (kinematic gains + friction offset)
  ff+pi    : feedforward + per-wheel PI trim on encoder rate error

Run:  python3 verify_compensation.py [--output verify_out]
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from identify_actuators import Bench
from turtlebot_model import twist_to_wheel_rates, limit_wheel_rates
from actuator_compensation import WheelCompensator, ActuatorModel, wheel_rates_for_twist

MODE_KW = ActuatorModel(kw_drive=0.054)   # tried: speed-blended yaw gain (rejected)

DT = 0.1
SETTLE_S, HOLD_S = 3.0, 2.0
TICKS = 1024


def run_case(bench, method, v, w, comp=None):
    """Hold desired twist (v, w) for SETTLE_S + HOLD_S; return achieved (v, w) over the hold."""
    m, d = bench.model, bench.data
    bench.reset()
    qa = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
          for n in ('wheel_left', 'wheel_right')]
    quant = lambda a: np.round(a * TICKS / (2 * np.pi)) * (2 * np.pi / TICKS)
    prev = quant(d.qpos[qa].copy())
    if comp is not None:
        comp.reset()
    steps = int(round(DT / m.opt.timestep))
    vs, ws = [], []
    n_total = int(round((SETTLE_S + HOLD_S) / DT))
    rate = None
    for k in range(n_total):
        ang = quant(d.qpos[qa].copy())
        rate = (ang - prev) / DT if k > 0 else None
        prev = ang
        if method == 'baseline':
            cmd = limit_wheel_rates(twist_to_wheel_rates(v, w, bench.spec), bench.spec)
        else:
            cmd = comp.command(v, w, rate)
        d.ctrl[bench.act[0]], d.ctrl[bench.act[1]] = cmd
        for _ in range(steps):
            mujoco.mj_step(m, d)
            if k * DT >= SETTLE_S:
                R = d.xmat[bench.base].reshape(3, 3)
                vs.append(float(np.dot(R[:2, 0], d.qvel[0:2])))
                ws.append(float(d.qvel[5]))
    return float(np.mean(vs)), float(np.mean(ws))


CASES = [(0.05, 0.0), (0.10, 0.0), (0.14, 0.0),
         (0.10, 0.2), (0.10, -0.2), (0.10, 0.4), (0.10, -0.4),
         (0.0, 0.3), (0.0, -0.3), (0.0, 0.6), (0.0, -0.6)]
# Held out: not used to fit kw_drive (which was fit on v = 0.10 m/s, w = +-0.2, +-0.4)
HOLDOUT = [(0.05, 0.3), (0.05, -0.3), (0.14, 0.3), (0.14, -0.3), (0.07, 0.5), (0.12, -0.15)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='verify_out')
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    model = ActuatorModel()                  # recommended: single yaw gain, no trim
    methods = {'baseline': None,
               'ff': WheelCompensator(model, DT, use_trim=False),
               'ff_modekw': WheelCompensator(MODE_KW, DT, use_trim=False),
               'ff_modekw+pi': WheelCompensator(MODE_KW, DT, use_trim=True)}
    results = []
    for friction in (1.0, 0.6):
        bench = Bench(friction)
        for method, comp in methods.items():
            for v, w in CASES + HOLDOUT:
                va, wa = run_case(bench, method, v, w, comp)
                # twist the compensator can actually ask for once the actuator range is spent
                vl, wl = (None, None) if comp is None else wheel_rates_for_twist(v, w, comp.model)[1:]
                results.append(dict(friction=friction, method=method, v=v, w=w,
                                    v_ach=va, w_ach=wa, v_lim=vl, w_lim=wl,
                                    holdout=(v, w) in HOLDOUT))
    (out / 'verify_compensation.json').write_text(json.dumps(results, indent=2))

    print(f"{'fric':>4} {'method':9} {'v_des':>6} {'w_des':>6} | {'v_ach':>7} {'w_ach':>7} | "
          f"{'v err':>7} {'w err':>7}")
    for r in results:
        print(f"{r['friction']:4.1f} {r['method']:9} {r['v']:6.2f} {r['w']:6.2f} | "
              f"{r['v_ach']:7.4f} {r['w_ach']:7.4f} | {r['v_ach']-r['v']:7.4f} {r['w_ach']-r['w']:7.4f}")

    print('\nSummary at friction 1.0 (floor friction 0.6 gave the same numbers, see note)')
    print('"vs request" includes loss from actuator saturation; "vs feasible" scores only whether the')
    print('plant delivers what the compensator promised once the actuator range is spent.')
    print(f"{'method':13s} {'set':8s} | {'v vs request':>12} {'w vs request':>12} | {'v vs feasible':>13} {'w vs feasible':>13} | {'cases speed-limited':>19}")
    for method in methods:
        for name, ho in (('fit', False), ('holdout', True)):
            rs = [r for r in results if r['friction'] == 1.0 and r['method'] == method and r['holdout'] == ho]
            rq_v = np.sqrt(np.mean([(r['v_ach'] - r['v'])**2 for r in rs]))
            rq_w = np.sqrt(np.mean([(r['w_ach'] - r['w'])**2 for r in rs]))
            if rs[0]['v_lim'] is None:
                fe_v = fe_w = float('nan'); lim = 'n/a'
            else:
                fe_v = np.sqrt(np.mean([(r['v_ach'] - r['v_lim'])**2 for r in rs]))
                fe_w = np.sqrt(np.mean([(r['w_ach'] - r['w_lim'])**2 for r in rs]))
                lim = f"{sum(r['v_lim'] < r['v'] - 1e-6 for r in rs)}/{len(rs)}"
            print(f"{method:13s} {name:8s} | {rq_v:12.4f} {rq_w:12.4f} | {fe_v:13.4f} {fe_w:13.4f} | {lim:>19}")
    worst = max(abs(a['v_ach'] - b['v_ach']) + abs(a['w_ach'] - b['w_ach'])
                for a in results for b in results
                if a['friction'] == 1.0 and b['friction'] == 0.6 and a['method'] == b['method']
                and a['v'] == b['v'] and a['w'] == b['w'])
    print(f"\nLargest difference between friction 1.0 and 0.6 outcomes: {worst:.4f}")

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for method, mk in (('baseline', 'o'), ('ff', 's'), ('ff_modekw', '^')):
        rs = [r for r in results if r['friction'] == 1.0 and r['method'] == method]
        ax[0].plot([r['v'] for r in rs], [r['v_ach'] for r in rs], mk, label=method)
        ax[1].plot([r['w'] for r in rs], [r['w_ach'] for r in rs], mk, label=method)
    for a, lim, name in ((ax[0], .16, 'forward speed v [m/s]'), (ax[1], .65, 'yaw rate w [rad/s]')):
        a.plot([-lim if name.startswith('yaw') else 0, lim], [-lim if name.startswith('yaw') else 0, lim],
               ':', color='gray', label='ideal')
        a.set(xlabel='desired ' + name, ylabel='achieved ' + name)
        a.grid(alpha=.3); a.legend()
    fig.suptitle('Steady-state achieved vs desired twist (friction 1.0)')
    fig.tight_layout(); fig.savefig(out / 'verify_compensation.png', dpi=150)
    print(f'\nsaved to {out}/')


if __name__ == '__main__':
    main()
