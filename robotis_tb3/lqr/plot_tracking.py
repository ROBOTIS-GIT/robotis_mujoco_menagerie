"""Closed-loop tracking figure from one trial's trace.json (or .json.gz).

    python robotis_tb3/lqr/plot_tracking.py eval_cmp/lqr/nominal/seed-0/trace.json \
        --occupancy eval_cmp/lqr/nominal/seed-0/occupancy.npy --out tracking_lqr_nominal.png

Four panels: (a) path in the world frame, truth against estimate and every latched reference;
(b) cross-track error from the truth and from the estimate (rotate-in-place shaded);
(c) heading estimate error against the EKF's own 2-sigma band (marker updates ticked);
(d) commanded yaw rate against the yaw rate the robot actually achieved. Ground truth is
read here for evaluation only. No simulator imports.
"""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np


def load_trace(path):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as f:
        return json.load(f)


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def series(records):
    """Arrays used by the figure. Missing keys (older traces) degrade gracefully."""
    t = np.array([r['time_s'] for r in records])
    est = np.array([r['estimate'] for r in records])
    tru = np.array([r['truth'] for r in records])
    cmd = np.array([r.get('turn_cmd', np.nan) for r in records])
    spd = np.array([r.get('speed_cmd', np.nan) for r in records])
    sigma = np.array([np.sqrt(max(r['covariance'][2][2], 0.0)) for r in records])
    track = [r.get('track') for r in records]
    ey_t = np.array([x['e_y_truth'] if x else np.nan for x in track])
    ey_e = np.array([x['e_y_est'] if x else np.nan for x in track])
    tracking = np.array([bool(x) and r['status'].startswith('LIDAR PATH') for x, r in zip(track, records)])
    dt = np.diff(t).mean() if len(t) > 1 else 0.1
    achieved = np.r_[wrap(np.diff(tru[:, 2])) / dt, np.nan]
    return dict(t=t, est=est, tru=tru, cmd=cmd, spd=spd, sigma=sigma, ey_t=ey_t, ey_e=ey_e,
                tracking=tracking, achieved=achieved, dt=dt,
                accepted=np.array([r['camera_accepted'] for r in records]),
                references=[r['reference'] for r in records if r.get('reference')])


def shade_rotation(ax, d):
    rot = d['tracking'] & (d['spd'] == 0)
    edges = np.flatnonzero(np.diff(np.r_[0, rot.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        ax.axvspan(d['t'][a], d['t'][min(b, len(d['t']) - 1)], color='0.85', lw=0)


def make_figure(records, occupancy=None, title=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d = series(records)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    a = ax[0, 0]
    if occupancy is not None:
        res, lower = 0.10, np.array([-.5, -1.4])
        xs, ys = np.nonzero(occupancy)
        a.scatter(lower[0] + xs * res, lower[1] + ys * res, s=14, c='0.8', marker='s', label='occupied (final map)')
    for k, ref in enumerate(d['references']):
        ref = np.array(ref)
        a.plot(ref[:, 0], ref[:, 1], '-', color='tab:green', lw=1, alpha=.35, label='latched reference' if k == 0 else None)
    a.plot(d['tru'][:, 0], d['tru'][:, 1], color='k', lw=1.6, label='truth')
    a.plot(d['est'][:, 0], d['est'][:, 1], color='tab:red', lw=1.2, ls='--', label='EKF estimate')
    a.plot(*d['tru'][0, :2], 'ko'); a.plot(2.36, 0, 'b*', ms=10, label='dock goal')
    a.set(title='(a) path', xlabel='x [m]', ylabel='y [m]'); a.set_aspect('equal'); a.legend(fontsize=7, loc='upper left')
    b = ax[0, 1]
    shade_rotation(b, d)
    b.plot(d['t'], 100 * d['ey_t'], color='k', label='truth')
    b.plot(d['t'], 100 * d['ey_e'], color='tab:red', ls='--', label='estimate (what the controller sees)')
    b.axhline(0, color='0.5', lw=.5)
    b.set(title='(b) cross-track error (grey = rotating in place)', xlabel='t [s]', ylabel='e_y, left positive [cm]'); b.legend(fontsize=7)
    c = ax[1, 0]
    err = np.degrees(wrap(d['est'][:, 2] - d['tru'][:, 2]))
    band = 2 * np.degrees(d['sigma'])
    c.fill_between(d['t'], -band, band, color='tab:red', alpha=.15, label='EKF ±2σ')
    c.plot(d['t'], err, color='tab:red', label='estimate − truth')
    c.plot(d['t'][d['accepted']], np.full(d['accepted'].sum(), -band.max() * 1.05), '|', color='tab:blue', ms=6, label='marker update accepted')
    c.axhline(0, color='0.5', lw=.5)
    c.set(title='(c) heading estimate error and filter uncertainty', xlabel='t [s]', ylabel='[deg]'); c.legend(fontsize=7)
    e = ax[1, 1]
    e.plot(d['t'], d['cmd'], color='tab:blue', label='commanded')
    e.plot(d['t'], d['achieved'], color='k', label='achieved (from truth)')
    e.axhline(0, color='0.5', lw=.5)
    e.set(title='(d) yaw rate: command against what the robot did', xlabel='t [s]', ylabel='[rad/s]'); e.legend(fontsize=7)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace')
    ap.add_argument('--occupancy')
    ap.add_argument('--out', default='tracking.png')
    ap.add_argument('--title', default='')
    a = ap.parse_args()
    occ = np.load(a.occupancy) if a.occupancy else None
    fig = make_figure(load_trace(a.trace), occ, a.title)
    fig.savefig(a.out, dpi=150)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
