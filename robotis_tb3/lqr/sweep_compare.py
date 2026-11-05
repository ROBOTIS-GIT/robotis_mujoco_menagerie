"""R-sweep trade-off, plain LQR vs compensated LQR (Part D).

usage: python robotis_tb3/lqr/sweep_compare.py eval_sweep/summary.json eval_comp/summary.json eval_sweep_comp/summary.json \
           [--out figures] [--conditions nominal camera_noise_dropout]
Several summary files are merged (duplicates by tag/condition/seed are dropped). Only successful trials
are used. Plain = compensate 'off', compensated = 'path'; rotate threshold 0.4 and no recovery only.
Writes <out>/sweep_comp_table.md and <out>/sweep_comp_tradeoff.png.
"""
import argparse, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument('summaries', nargs='+')
ap.add_argument('--out', default='.')
ap.add_argument('--conditions', nargs='+', default=['nominal', 'camera_noise_dropout'])
a = ap.parse_args()

seen, res = set(), []
for f in a.summaries:
    for r in json.loads(Path(f).read_text()):
        k = (r['controller_tag'], r['condition'], r['seed'])
        if k not in seen:
            seen.add(k); res.append(r)

def keep(r):
    return (r['controller_name'] == 'lqr' and not r.get('recovery') and r.get('success')
            and r.get('compensate', 'off') in ('off', 'path') and abs(r.get('lqr_rotate_enter', .4) - .4) < 1e-9)
res = [r for r in res if keep(r)]
M = lambda r, k: r['tracking'].get(k)
def ms(v):
    v = [x for x in v if x is not None]
    return (np.mean(v), np.std(v, ddof=1) if len(v) > 1 else 0.0) if v else (np.nan, np.nan)

lines = ['| condition | controller | R scale | n | XTE RMS driving [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] | final lat [cm] |',
         '|---|---|---|---|---|---|---|---|---|---|']
fig, axs = plt.subplots(1, len(a.conditions), figsize=(5.5 * len(a.conditions), 4.4), squeeze=False)
for ax, cond in zip(axs[0], a.conditions):
    for comp, name, style in (('off', 'plain LQR', 'o-'), ('path', 'LQR + compensator', 's--')):
        scales = sorted({r['lqr_r_scale'] for r in res if r['condition'] == cond and r.get('compensate', 'off') == comp})
        X, Y, XE, YE = [], [], [], []
        for s in scales:
            rs = [r for r in res if r['condition'] == cond and r.get('compensate', 'off') == comp and r['lqr_r_scale'] == s]
            e, ee = ms([M(r, 'effort_w2_driving') for r in rs]); x, xe = ms([100 * M(r, 'xte_rms_driving_m') for r in rs])
            ts, _ = ms([M(r, 'effort_w2') for r in rs]); rot, _ = ms([M(r, 'rotate_seconds') for r in rs])
            tm, _ = ms([r['simulation_seconds'] for r in rs]); fl, _ = ms([100 * r['final_lateral_error_truth_m'] for r in rs])
            n = len(rs)
            X.append(e); Y.append(x); XE.append(ee / np.sqrt(n)); YE.append(xe / np.sqrt(n))
            lines.append(f'| {cond} | {name} | {s:g} | {n} | {x:.1f} ± {xe:.1f} | {e:.2f} ± {ee:.2f} | {ts:.2f} | {rot:.1f} | {tm:.1f} | {fl:.1f} |')
        if X:
            ax.errorbar(X, Y, xerr=XE, yerr=YE, fmt=style, capsize=3, label=name)
            for x, y, s in zip(X, Y, scales):
                ax.annotate(f'R×{s:g}', (x, y), fontsize=7, xytext=(3, 3), textcoords='offset points')
    ax.set(xlabel='control effort, driving ∫w² dt', ylabel='cross-track RMS, driving [cm]', title=cond)
    ax.grid(alpha=.3); ax.legend(fontsize=8)
fig.suptitle('R sweep: tracking error vs control effort (mean ± s.e., successful trials)')
fig.tight_layout()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
(out / 'sweep_comp_table.md').write_text('\n'.join(lines) + '\n')
fig.savefig(out / 'sweep_comp_tradeoff.png', dpi=150)
print('\n'.join(lines)); print('wrote', out / 'sweep_comp_table.md', 'and', out / 'sweep_comp_tradeoff.png')