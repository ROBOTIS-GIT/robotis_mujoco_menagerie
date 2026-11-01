"""Why did the planner lose (and regain) its route? Report every no-path episode in one trace.

    python robotis_tb3/lqr/diagnose_no_path.py eval_recovery/heuristic_recovery/shifted_obstacle/seed-0/trace.json

For each run of consecutive frames with an empty planner path: when it started, how long it
lasted, how many frames were recovery search turns, whether a marker update was accepted
during it or in the 2 s after, and the heading estimate error (estimate - truth) at its start
and end. This separates two explanations for a recovery that works:
  * the marker came back into view and the EKF corrected the heading, or
  * a small rotation changed the tilted LiDAR map enough to reopen a marginally blocked route.
Ground truth is used here for evaluation only. No simulator imports.
"""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np


def load(path):
    path = Path(path)
    with (gzip.open if path.suffix == '.gz' else open)(path, 'rt') as f:
        return json.load(f)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def episodes(records, after_s=2.0):
    """List of dicts, one per run of frames whose planner path is empty."""
    dt = records[1]['time_s'] - records[0]['time_s'] if len(records) > 1 else 0.1
    empty = np.array([not r['path'] for r in records])
    edges = np.flatnonzero(np.diff(np.r_[0, empty.astype(int), 0]))
    out = []
    for a, b in zip(edges[::2], edges[1::2]):                 # [a, b) is one empty run
        seg = records[a:b]
        tail = records[b:b + int(after_s / dt)]
        err = lambda r: float(np.degrees(wrap(r['estimate'][2] - r['truth'][2])))
        out.append(dict(
            start_s=records[a]['time_s'], frames=b - a, duration_s=(b - a) * dt,
            search_frames=sum(r['status'].startswith('NO PATH - SEARCH') for r in seg),
            accepted_during=sum(bool(r['camera_accepted']) for r in seg),
            accepted_after=sum(bool(r['camera_accepted']) for r in tail),
            heading_err_start_deg=err(records[a]),
            heading_err_end_deg=err(records[min(b, len(records) - 1)]),
            ended=b < len(records)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace')
    a = ap.parse_args()
    recs = load(a.trace)
    eps = episodes(recs)
    print(f'{len(recs)} frames; {len(eps)} no-path episode(s); final status: {recs[-1]["status"]}')
    if not eps:
        return
    print(' start[s]  length[s]  search frames  marker updates (during / next 2 s)  heading err start -> end [deg]  route returned')
    for e in eps:
        print(f' {e["start_s"]:7.1f}  {e["duration_s"]:9.1f}  {e["search_frames"]:13d}  '
              f'{e["accepted_during"]:>12d} / {e["accepted_after"]:<4d}                {e["heading_err_start_deg"]:7.1f} -> {e["heading_err_end_deg"]:6.1f}'
              f'          {"yes" if e["ended"] else "NO (still stalled at the end)"}')


if __name__ == '__main__':
    main()
