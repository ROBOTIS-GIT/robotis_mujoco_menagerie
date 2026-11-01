"""Diagnostic plots for one trial's trace.json (Part D).

usage: python plot_trace.py eval_comp/lqr_comp/shifted_obstacle/seed-1 [more trial dirs ...] [-o out.png]
Several dirs are overlaid for comparison (e.g. the lqr and lqr_comp run of the same seed).
Panels: XTE est vs truth, applied vs commanded speed, turn cmd, NEES (3-state 95% bound), xy path.
Ground truth is used for evaluation plots only.
"""
import argparse, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument('trials', nargs='+')
ap.add_argument('-o', '--out', default='trace_diagnostics.png')
args = ap.parse_args()

fig, ax = plt.subplots(3, 2, figsize=(13, 10))
ax = ax.ravel()
for i, d in enumerate(args.trials):
    c = f'C{i}'
    tr = json.loads((Path(d) / 'trace.json').read_text())
    label = '/'.join(Path(d).parts[-3:])
    t = np.array([r['time_s'] for r in tr])
    trk = [r.get('track') or {} for r in tr]
    nan = float('nan')
    ey_t = np.array([k.get('e_y_truth', nan) for k in trk]) * 100
    ey_e = np.array([k.get('e_y_est', nan) for k in trk]) * 100
    v_cmd = np.array([r.get('speed_cmd') if r.get('speed_cmd') is not None else nan for r in tr])
    v_app = np.array([r.get('speed_applied') if r.get('speed_applied') is not None else nan for r in tr])
    w = np.array([r.get('turn_cmd') if r.get('turn_cmd') is not None else nan for r in tr])
    nees = np.array([r.get('nees') if r.get('nees') is not None else nan for r in tr])
    est = np.array([r['estimate'] for r in tr]); tru = np.array([r['truth'] for r in tr])
    ax[0].plot(t, ey_t, color=c, label=f'{label} truth'); ax[0].plot(t, ey_e, '--', color=c, label=f'{label} est')
    ax[1].plot(t, v_cmd, ':', color=c, label=f'{label} cmd'); ax[1].plot(t, v_app, color=c, label=f'{label} applied')
    ax[2].plot(t, w, color=c, label=label)
    ra = np.array([r.get('reference_adoptions') or 0 for r in tr]); jump = np.flatnonzero(np.diff(ra) > 0)
    ax[2].plot(t[jump + 1], np.full(len(jump), 0.9 - 0.05 * i), '|', color=c, ms=8)
    ax[3].plot(t, nees, color=c, label=label)
    ax[4].plot(t, np.hypot(*(est[:, :2] - tru[:, :2]).T) * 100, color=c, label=label)
    ax[5].plot(tru[:, 0], tru[:, 1], color=c, label=f'{label} truth')
ax[0].set_title('cross-track error [cm]'); ax[1].set_title('speed [m/s]')
ax[2].set_title('turn command [rad/s]  (ticks on top = new reference adopted)'); ax[3].set_title('NEES (3-state 95% bound = 7.81)')
ax[3].axhline(7.81, color='k', lw=.8); ax[3].set_yscale('log')
ax[4].set_title('position error |est - truth| [cm]'); ax[5].set_title('truth path'); ax[5].axis('equal')
for a in ax[:5]: a.set_xlabel('t [s]'); a.grid(alpha=.3)
ax[0].legend(fontsize=7); ax[1].legend(fontsize=7)
fig.tight_layout(); fig.savefig(args.out, dpi=130)
print('wrote', args.out)