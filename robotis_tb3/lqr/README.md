# Part D: LQR path tracking (Track A)

Everything specific to the LQR controller lives in this folder. The shared pipeline stays in
`robotis_tb3/` and imports it as `from lqr.track_a_control import ...`.

| File | Purpose |
|---|---|
| `track_a_control_design.py` | Error model, controllability, Bryson weights, discrete/continuous LQR, actuator-lag robustness. No simulator imports. |
| `track_a_control.py` | `ReferencePath` (latched, straightened reference) and `LQRPathTracker` (`w = -K [e_y, e_theta]`, rotate-in-place with hysteresis, configurable threshold). |
| `evaluate_controllers.py` | Resumable controller x condition x seed runner. Specs: `heuristic`, `lqr`, `lqr:<r_scale>`, `lqr@<rotate_rad>`, any of them with `+recovery`, and last of all `+comp` (actuator compensation off while docking) or `+compall` (docking included). |
| `analyse_controllers.py` | Tables, paired Wilcoxon tests, box plot and Q/R trade-off plot from `summary.json` (successful trials only by default). |
| `part_d_summary.py` | Generates the comparison, estimator-share, failure and sweep tables used in `PART_D_CONTROL.md`. |
| `diagnose_no_path.py` | Lists every no-path episode in one trace: length, search frames, marker updates, heading error. Used to explain why recovery works. |
| `plot_tracking.py` | Four-panel closed-loop figure from one `trace.json`: path, cross-track error, heading error against the EKF's 2-sigma band, yaw command against achieved. |
| `test_track_a_control.py`, `test_track_a_control_design.py`, `test_track_a_recovery.py`, `test_track_a_evaluation.py`, `test_track_a_plot.py`, `test_track_a_compensation.py` | Sim-free tests. |
| `VIVA_NOTES.md` | Short answers and numbers for the Part D viva. |

Shared files that Part D touches (outside this folder):

- `track_a_navigation.py`: path following is the overridable `StateFeedbackTracker.follow_path()`; opt-in no-path recovery (`recovery=True`).
- `track_a_project.py`: `--controller {heuristic,lqr}`, `--lqr-r-scale`, `--lqr-rotate-enter`, `--recovery`, `--compensate {off,path,all}`, tracking metrics (including delivered driving speed) in `result.json`,
  the latched reference logged in `trace.json`, and the source snapshot keeps the `lqr/` subfolder.

Run from the repository root:

```sh
python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v
python robotis_tb3/track_a_project.py --headless --controller lqr --output out_lqr
python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 10 --output eval_cmp
python robotis_tb3/lqr/analyse_controllers.py eval_cmp/summary.json --out eval_cmp
python robotis_tb3/lqr/part_d_summary.py eval_cmp/summary.json
python robotis_tb3/lqr/plot_tracking.py out_lqr/trace.json --occupancy out_lqr/occupancy.npy --out tracking.png
python robotis_tb3/lqr/track_a_control_design.py        # design study, writes step1_lqr_vs_heuristic.png
```

Cross-track error is measured against the latched reference polyline, which is replaced when it
is blocked (`reference_adoptions` in `result.json`). Ground truth is used only to evaluate it.
