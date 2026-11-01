"""Summarise evaluate_controllers.py output: tables, paired tests, trade-off plots.

    python robotis_tb3/lqr/analyse_controllers.py eval_cmp/summary.json --out eval_cmp
    python robotis_tb3/lqr/analyse_controllers.py eval_sweep/summary.json --out eval_sweep

Writes controller_results.md, tracking_boxplot.png and (if the data has an R sweep)
tradeoff.png. No simulator imports.

Reading the numbers (also stated in the report): cross-track error is measured against
the LATCHED reference polyline, which is replaced on blockage (see reference_adoptions),
and the heuristic follows a per-frame replanned path rather than that line. Differences
are therefore "closeness to the latched reference", not a controller ranking in general.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import numpy as np

METRICS = [
    ('xte_rms_truth_m', 'XTE RMS truth [cm]', 100),
    ('xte_max_truth_m', 'XTE max truth [cm]', 100),
    ('xte_rms_driving_m', 'XTE RMS driving [cm]', 100),
    ('xte_est_minus_truth_rms_m', 'est-truth XTE [cm]', 100),
    ('effort_w2_driving', 'effort w^2 driving', 1),
    ('effort_w2', 'effort w^2 total', 1),
    ('rotate_seconds', 'rotate [s]', 1),
    ('simulation_seconds', 'time [s]', 1),
    ('final_lateral_error_truth_m', 'final lat [cm]', 100),
    ('reference_adoptions', 'ref adoptions', 1),
    ('no_path_frames', 'no-path frames', 1),
]
PAIRED_KEYS = ('xte_rms_truth_m', 'xte_max_truth_m', 'xte_rms_driving_m', 'effort_w2_driving',
               'effort_w2', 'simulation_seconds')


def tag_order(tag):
    """heuristic first, then lqr variants; plain controller before its modified variants."""
    return (not tag.startswith('heuristic'), tag)


def comparison_pairs(tags):
    """(reference, other) tag pairs worth a paired test: LQR vs heuristic, and each modified
    variant against its plain controller."""
    pairs = []
    if 'heuristic' in tags and 'lqr' in tags:
        pairs.append(('heuristic', 'lqr'))
    for t in tags:
        if t in ('heuristic', 'lqr'):
            continue
        base = 'heuristic' if t.startswith('heuristic') else 'lqr'
        if base in tags and ('_rot' in t or t.endswith(('_recovery', '_comp', '_compall')) or t.startswith('lqr_r')):
            pairs.append((base, t))
    return pairs


def wilson(k, n, z=1.96):
    """Wilson score interval for a success rate (honest at small n and 0/n, n/n)."""
    if n == 0:
        return (np.nan, np.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def metric(result, key):
    t = result.get('tracking') or {}
    v = t.get(key, result.get(key))
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return None
    if t.get('tracking_seconds', 1) == 0 and key.startswith(('xte', 'effort')):
        return None
    return float(v)


def group(results):
    g = defaultdict(list)
    for r in results:
        g[(r['condition'], r['controller_tag'])].append(r)
    return g


def mean_sd(values):
    a = np.array(values, dtype=float)
    if a.size == 0:
        return 'n/a'
    return f'{a.mean():.2f} ± {a.std(ddof=1) if a.size > 1 else 0:.2f}'


def paired(results, cond, tag_a, tag_b, key, successes_only=True):
    """Per-seed paired difference b - a with a Wilcoxon signed-rank p (needs >= 6 pairs).

    With successes_only, a seed counts only if BOTH controllers succeeded on it: a trial that
    stalled after 6 s has a tiny, meaningless tracking error and must not enter the comparison."""
    keep = (lambda r: r['success']) if successes_only else (lambda r: True)
    A = {r['seed']: metric(r, key) for r in results if r['condition'] == cond and r['controller_tag'] == tag_a and keep(r)}
    B = {r['seed']: metric(r, key) for r in results if r['condition'] == cond and r['controller_tag'] == tag_b and keep(r)}
    seeds = [s for s in A if s in B and A[s] is not None and B[s] is not None]
    if not seeds:
        return None
    d = np.array([B[s] - A[s] for s in seeds])
    p = None
    if len(d) >= 6 and np.any(d != 0):
        from scipy.stats import wilcoxon
        p = float(wilcoxon(d).pvalue)
    return dict(n=len(d), median=float(np.median(d)), mean=float(d.mean()), p=p,
                b_better=int(np.sum(d < 0)))


def report(results, successes_only=True):
    g = group(results)
    conds = sorted({r['condition'] for r in results})
    tags = sorted({r['controller_tag'] for r in results}, key=tag_order)
    lines = ['# Controller evaluation', '',
             f'{len(results)} trials, {len(conds)} conditions, controllers: {", ".join(tags)}.', '',
             'Mean ± sd over seeds. Success rate (all trials) shows the 95% Wilson interval. ' +
             ('Tracking metrics use SUCCESSFUL trials only (n ok shown): a trial that stalls early has a '
              'tiny tracking error that says nothing about the controller. Failures are listed below.'
              if successes_only else 'Tracking metrics include failed trials (--include-failed).'), '']
    for cond in conds:
        lines += [f'## {cond}', '',
                  '| controller | success | 95% CI | n ok | ' + ' | '.join(m[1] for m in METRICS) + ' |',
                  '|---|---|---|---|' + '---|' * len(METRICS)]
        for tag in tags:
            rs = g.get((cond, tag), [])
            if not rs:
                continue
            k = sum(r['success'] for r in rs)
            lo, hi = wilson(k, len(rs))
            cells = []
            used = [r for r in rs if r['success']] if successes_only else rs
            for key, _, scale in METRICS:
                vals = [metric(r, key) for r in used]
                cells.append(mean_sd([v * scale for v in vals if v is not None]))
            lines.append(f'| {tag} | {k}/{len(rs)} | {lo:.2f}–{hi:.2f} | {len(used)} | ' + ' | '.join(cells) + ' |')
        lines.append('')
        labels = {k: (lab, sc) for k, lab, sc in METRICS}
        for ref, other in comparison_pairs(tags):
            for key in PAIRED_KEYS:
                pr = paired(results, cond, ref, other, key, successes_only)
                if pr:
                    lab, sc = labels[key]
                    ptxt = f', Wilcoxon p={pr["p"]:.3f}' if pr['p'] is not None else ', too few pairs for a test'
                    lines.append(f'- {other} − {ref}, {lab}: median {pr["median"] * sc:+.2f}, '
                                 f'{other} lower in {pr["b_better"]}/{pr["n"]} paired seeds{ptxt}')
            lines.append('')
    failed = [r for r in results if not r['success']]
    lines += ['## Failures', '']
    if not failed:
        lines.append('None.')
    for r in failed:
        lines.append(f'- {r["controller_tag"]} / {r["condition"]} / seed {r["seed"]}: '
                     f'{r.get("final_status")} (collision={r.get("collision")}, '
                     f'final lateral {r.get("final_lateral_error_truth_m", float("nan")):.3f} m)')
    return '\n'.join(lines) + '\n'


def plots(results, out, successes_only=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if successes_only:
        results = [r for r in results if r['success']]
    g = group(results)
    conds = sorted({r['condition'] for r in results})
    tags = sorted({r['controller_tag'] for r in results}, key=tag_order)
    fig, ax = plt.subplots(figsize=(1.6 * len(conds) * max(1, len(tags)) / 1.5 + 2, 4))
    data, labels = [], []
    for cond in conds:
        for tag in tags:
            v = [100 * metric(r, 'xte_rms_truth_m') for r in g.get((cond, tag), [])
                 if metric(r, 'xte_rms_truth_m') is not None]
            if v:
                data.append(v)
                labels.append(f'{cond}\n{tag}')
    try:
        ax.boxplot(data, tick_labels=labels)       # matplotlib >= 3.9
    except TypeError:
        ax.boxplot(data, labels=labels)
    ax.set(ylabel='cross-track RMS vs latched reference [cm]', title='Tracking error per trial')
    ax.tick_params(axis='x', labelsize=7, rotation=45)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(out / 'tracking_boxplot.png', dpi=150)
    plt.close(fig)

    def plain(r):
        return (r['controller_name'] == 'lqr' and not r.get('recovery') and r.get('compensate', 'off') == 'off'
                and abs(r.get('lqr_rotate_enter', .4) - .4) < 1e-9)
    sweep = sorted({r['lqr_r_scale'] for r in results if plain(r)})
    if len(sweep) < 3:
        return ['tracking_boxplot.png']
    fig, ax = plt.subplots(figsize=(6, 4.2))
    for cond in conds:
        xs, ys, xe, ye, rr = [], [], [], [], []
        for r_scale in sweep:
            rs = [r for r in results if r['condition'] == cond and plain(r)
                  and r['lqr_r_scale'] == r_scale]
            e = [metric(r, 'effort_w2_driving') for r in rs]
            x = [100 * metric(r, 'xte_rms_driving_m') for r in rs if metric(r, 'xte_rms_driving_m') is not None]
            e = [v for v in e if v is not None]
            if e and x:
                xs.append(np.mean(e)); ys.append(np.mean(x))
                xe.append(np.std(e, ddof=1) / np.sqrt(len(e)) if len(e) > 1 else 0)
                ye.append(np.std(x, ddof=1) / np.sqrt(len(x)) if len(x) > 1 else 0)
                rr.append(r_scale)
        ax.errorbar(xs, ys, xerr=xe, yerr=ye, marker='o', capsize=3, label=cond)
        for x, y, r_scale in zip(xs, ys, rr):
            ax.annotate(f'R×{r_scale:g}', (x, y), fontsize=7, xytext=(3, 3), textcoords='offset points')
    ax.set(xlabel='control effort, driving frames ∫w² dt', ylabel='cross-track RMS, driving [cm]',
           title='LQR Q/R trade-off (mean ± s.e.)')
    ax.grid(alpha=.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / 'tradeoff.png', dpi=150)
    plt.close(fig)
    return ['tracking_boxplot.png', 'tradeoff.png']


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('summary')
    p.add_argument('--out', default='.')
    p.add_argument('--include-failed', action='store_true',
                   help='include failed trials in tracking metrics (default: successful trials only)')
    a = p.parse_args()
    results = json.loads(Path(a.summary).read_text())
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ok_only = not a.include_failed
    md = report(results, ok_only)
    (out / 'controller_results.md').write_text(md)
    made = plots(results, out, ok_only)
    print(md)
    print('wrote', out / 'controller_results.md', *[out / m for m in made])


if __name__ == '__main__':
    main()