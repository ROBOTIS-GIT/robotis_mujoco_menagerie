"""Generate the Part D result tables for PART_D_CONTROL.md from summary.json files.

    python robotis_tb3/lqr/part_d_summary.py robotis_tb3/evidence/part-d/eval_cmp/summary.json
    python robotis_tb3/lqr/part_d_summary.py eval_sweep/summary.json --sweep
    python robotis_tb3/lqr/part_d_summary.py eval_recovery/summary.json --recovery

Numbers in the write-up come from this script, not from hand copying. No simulator imports.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.stats import binomtest, fisher_exact
try:
    from lqr.analyse_controllers import metric, paired, wilson, tag_order
except ImportError:                      # run as a plain script
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from lqr.analyse_controllers import metric, paired, wilson, tag_order

CONDITIONS = ('nominal', 'camera_noise_dropout', 'shifted_obstacle', 'slippery_floor')


def rows(results, cond, tag, successes_only=True):
    rs = [r for r in results if r['condition'] == cond and r['controller_tag'] == tag]
    return rs, [r for r in rs if r['success']] if successes_only else rs


def fmt(values, scale=1.0, nd=1):
    v = np.array([x * scale for x in values if x is not None], dtype=float)
    if v.size == 0:
        return 'n/a'
    return f'{v.mean():.{nd}f} ± {v.std(ddof=1) if v.size > 1 else 0:.{nd}f}'


def comparison_table(results, ref='heuristic', other='lqr'):
    """Per-condition table, ref vs other: success, tracking, effort, rotating, time, paired tests."""
    out = [f'| condition | controller | success (95% CI) | XTE RMS [cm] | XTE max [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] | final lat [cm] |',
           '|---|---|---|---|---|---|---|---|---|---|']
    notes = []
    for cond in CONDITIONS:
        for tag in (ref, other):
            allr, ok = rows(results, cond, tag)
            if not allr:
                continue
            lo, hi = wilson(len(ok), len(allr))
            g = lambda key, sc=1.0, nd=1: fmt([metric(r, key) for r in ok], sc, nd)
            out.append(f'| {cond} | {tag} | {len(ok)}/{len(allr)} ({lo:.2f}–{hi:.2f}) | {g("xte_rms_truth_m", 100)} | '
                       f'{g("xte_max_truth_m", 100)} | {g("effort_w2_driving", 1, 2)} | {g("effort_w2", 1, 2)} | '
                       f'{g("rotate_seconds")} | {g("simulation_seconds")} | {g("final_lateral_error_truth_m", 100)} |')
        ra, oa = rows(results, cond, ref)[0], rows(results, cond, other)[0]
        if ra and oa:
            a_ok, b_ok = sum(r['success'] for r in ra), sum(r['success'] for r in oa)
            if a_ok != len(ra) or b_ok != len(oa):
                p = fisher_exact([[b_ok, len(oa) - b_ok], [a_ok, len(ra) - a_ok]])[1]
                A = {r['seed']: r['success'] for r in ra}
                B = {r['seed']: r['success'] for r in oa}
                both = sorted(set(A) & set(B))
                fav_other = sum(B[i] and not A[i] for i in both)      # discordant seeds, other succeeded only
                fav_ref = sum(A[i] and not B[i] for i in both)
                pm = binomtest(fav_other, fav_other + fav_ref, 0.5).pvalue if fav_other + fav_ref else 1.0
                notes.append(f'- {cond}: success {other} {b_ok}/{len(oa)} vs {ref} {a_ok}/{len(ra)}; '
                             f'Fisher exact p = {p:.3f} (treats the groups as independent), '
                             f'exact McNemar p = {pm:.3f} (paired by seed: {fav_other} seeds favour {other}, {fav_ref} favour {ref})')
            for key, label, sc in (('xte_rms_truth_m', 'XTE RMS [cm]', 100), ('effort_w2', 'total ∫w²', 1),
                                   ('simulation_seconds', 'time [s]', 1)):
                pr = paired(results, cond, ref, other, key)
                if pr:
                    ptxt = f'Wilcoxon p = {pr["p"]:.3f}' if pr['p'] is not None else 'too few pairs for a test'
                    notes.append(f'- {cond}: {other} − {ref}, {label}: median {pr["median"] * sc:+.2f}, '
                                 f'{other} lower in {pr["b_better"]}/{pr["n"]} paired seeds ({ptxt})')
    return '\n'.join(out) + '\n\nPaired differences use seeds where both controllers succeeded:\n\n' + '\n'.join(notes) + '\n'


def estimator_share(results, tag='lqr'):
    """How much of the true cross-track error is the estimator's (est-truth gap / truth XTE)."""
    out = ['| condition | est-truth XTE [cm] | truth XTE [cm] | ratio |', '|---|---|---|---|']
    for cond in CONDITIONS:
        _, ok = rows(results, cond, tag)
        if not ok:
            continue
        gap = [metric(r, 'xte_est_minus_truth_rms_m') for r in ok]
        tru = [metric(r, 'xte_rms_truth_m') for r in ok]
        out.append(f'| {cond} | {fmt(gap, 100)} | {fmt(tru, 100)} | {np.mean(np.array(gap) / np.array(tru)):.2f} |')
    nees = [r['mean_nees'] for r in results if r['controller_tag'] == tag and r['success']]
    return '\n'.join(out) + f'\n\nMean NEES over {tag} successes: {np.mean(nees):.1f} (3 expected for a consistent 3-state filter).\n'


def failures(results):
    bad = [r for r in results if not r['success']]
    if not bad:
        return 'No failed trials.\n'
    out = ['| controller | condition | seed | final status | final lateral [m] | camera updates | EKF pos RMSE [cm] |', '|---|---|---|---|---|---|---|']
    for r in sorted(bad, key=lambda r: (r['controller_tag'], r['condition'], r['seed'])):
        out.append(f'| {r["controller_tag"]} | {r["condition"]} | {r["seed"]} | {r["final_status"]} | '
                   f'{r["final_lateral_error_truth_m"]:.3f} | {r["camera_updates"]} | {100 * r["ekf_position_rmse_m"]:.1f} |')
    return '\n'.join(out) + '\n'


def sweep_table(results):
    out = ['| tag | R scale | condition | success | XTE RMS driving [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] |', '|---|---|---|---|---|---|---|---|---|']
    plain = [r for r in results if r['controller_name'] == 'lqr' and not r.get('recovery')
             and abs(r.get('lqr_rotate_enter', .4) - .4) < 1e-9]
    for cond in CONDITIONS:
        for scale in sorted({r['lqr_r_scale'] for r in plain}):
            rs = [r for r in plain if r['condition'] == cond and r['lqr_r_scale'] == scale]
            ok = [r for r in rs if r['success']]
            if not rs:
                continue
            g = lambda key, sc=1.0, nd=1: fmt([metric(r, key) for r in ok], sc, nd)
            out.append(f'| {rs[0]["controller_tag"]} | {scale:g} | {cond} | {len(ok)}/{len(rs)} | {g("xte_rms_driving_m", 100)} | '
                       f'{g("effort_w2_driving", 1, 2)} | {g("effort_w2", 1, 2)} | {g("rotate_seconds")} | {g("simulation_seconds")} |')
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('summary')
    ap.add_argument('--sweep', action='store_true', help='R-sweep table instead of the controller comparison')
    ap.add_argument('--recovery', action='store_true', help='compare heuristic vs heuristic_recovery (and lqr vs lqr_recovery)')
    a = ap.parse_args()
    results = json.loads(Path(a.summary).read_text())
    tags = sorted({r['controller_tag'] for r in results}, key=tag_order)
    print(f'{len(results)} trials; controllers: {", ".join(tags)}\n')
    if a.sweep:
        print(sweep_table(results))
    elif a.recovery:
        for base in ('heuristic', 'lqr'):
            if base in tags and f'{base}_recovery' in tags:
                print(f'### {base} vs {base}_recovery\n')
                print(comparison_table(results, base, f'{base}_recovery'))
    else:
        print('### Controller comparison\n')
        print(comparison_table(results))
        print('### Estimator share of the LQR tracking error\n')
        print(estimator_share(results))
        print('### Failed trials\n')
        print(failures(results))


if __name__ == '__main__':
    main()
