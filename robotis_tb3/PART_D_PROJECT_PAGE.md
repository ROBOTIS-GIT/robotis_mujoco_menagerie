# Part D: Control (project page section)

## Summary
The path tracker is state-feedback LQR on the cross-track error model `x = [e_y, e_theta]`, `w = -K x`, `K = [6.94, 2.54]` from Bryson-rule weights, computed from the EKF pose only. The reference path is a straightened A* route held in the EKF frame (`ReferencePath`) and replaced only when blocked. When `|e_theta| > 0.4` rad the robot rotates in place on heading error alone, because the error model loses controllability at zero speed.

Compared with the baseline bearing-law tracker over 80 trials (4 conditions × 10 seeds × 2 controllers), the LQR keeps the robot closer to the latched reference in every condition (cross-track RMS 6.3 against 13.6 cm nominal; lower in 10/10 paired seeds, Wilcoxon p = 0.002), drives with about 7× less effort in driving frames, and completed 10/10 shifted-obstacle trials against 5/10 for the baseline (Fisher p = 0.033, paired exact McNemar p = 0.0625: favours LQR, borderline at ten seeds). An opt-in no-path recovery rescued all five stalled baseline seeds (10/10 against 5/10). It costs time: 28–31% of its tracking time is spent rotating in place, and docking takes 6–9 s longer. Most of its remaining error comes from the estimator (69–73% of the true cross-track error), not from the controller.

**Actuator compensation.** The simulated wheels have a friction dead zone of about 2 rad/s and deliver 0.85 of the nominal ground speed (Section 2.1 of the write-up), so the uncompensated robot drives at 0.094 m/s for a 0.18 m/s command and rotates in place for about 30% of its tracking time. A feedforward wheel-command compensator (`actuator_compensation.py`, identified from open-loop bench runs) was evaluated over 120 trials (3 controllers × 4 conditions × 10 seeds, all arrived). With it, rotate-in-place time falls from 10–13 s to 2–5 s per run, total control effort falls by 62–76%, and time to dock by 28–35% (paired p ≤ 0.004 in every condition). Cross-track error is not significantly changed (p = 0.105 to 0.557; about 1 cm higher in nominal and dropout, 0.5 cm lower in shifted-obstacle and slippery). Compensating the docking phase as well (`compall`) is faster but docks 0.7–1.5 cm less accurately, so it is reported as an ablation only.

**Gain sweep.** Plain LQR works for R×0.25 to R×1 but fails at R×2 (7/10 nominal, 5/10 with camera noise) and R×4 (0/10), because the weaker turn command falls into the actuator dead band during rotate-in-place (traced in two failed runs). With the compensator all 80 sweep trials arrived, at R×0.25 to R×4. At a matched R the compensator trades about 70% less effort for somewhat higher cross-track error; R×1 gives the lowest compensated error in the nominal condition.

**Open items.** One compensated shifted-obstacle seed (seed 1) rotates for 17 s with 51 reference replacements. **Ethan did not have time to finish investigating this issue**

Full write-up: [`PART_D_CONTROL.md`](robotis_tb3/PART_D_CONTROL.md). Results, plots and raw files: [`evidence/part-d/`](robotis_tb3/evidence/part-d/). Code: [`robotis_tb3/lqr/`](robotis_tb3/lqr/).

![Cross-track error per trial](robotis_tb3/evidence/part-d/eval_cmp/tracking_boxplot.png)

## Parameters
Control period 0.1 s; `V_NOM = 0.18` m/s; yaw saturation ±0.8 rad/s; `Q = diag(100, 8.2)`, `R = 1.56`; rotate-in-place enter 0.40 rad, exit 0.25 rad; segment switch 0.10 m before the vertex; reference replaced after 3 blocked frames, when the goal moves, or when the robot is 0.4 m off it. Actuator compensator (`--compensate path`): ground speed per wheel rate 0.0281 m/rad, yaw per wheel-rate difference 0.075, friction offset 2.0 rad/s, lag 0.11 s, command limit ±7.88 rad/s, not used while docking.

## Limits, stated plainly
The comparison against the baseline (80 trials) was run on the uncompensated robot; the compensator was evaluated separately on the LQR only (120 trials), and there is no compensated baseline. The estimator, not the controller, limits cross-track error (est−truth gap about as large as the true error), and the EKF is overconfident (Part C). Seeds vary noise, not the scene layout. Cross-track error is measured against a latched reference that the baseline does not track. Docking is the shared relay law. Simulation only; no hardware result is claimed.

## Run it
```sh
python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v
python robotis_tb3/track_a_project.py --headless --controller lqr --output out_lqr
python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 10 --output eval_cmp
python robotis_tb3/lqr/analyse_controllers.py eval_cmp/summary.json --out eval_cmp
python robotis_tb3/lqr/plot_tracking.py out_lqr/trace.json --occupancy out_lqr/occupancy.npy --out tracking.png
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr+comp lqr+compall --trials 10 --output eval_comp
python robotis_tb3/lqr/part_d_summary.py eval_comp/summary.json --comp
python robotis_tb3/lqr/sweep_compare.py eval_sweep/summary.json eval_comp/summary.json eval_sweep_comp/summary.json --out figures
```
The full command list, including the sweeps, is under *Reproduce* in the write-up. The `eval_*` folders are about 1 MB per trial and are not committed; `summary.json` and the generated tables are.

## Generative AI declaration (Part D) 

> Claude (Anthropic, via claude.ai) was used for Part D to: review the path-error model and LQR design; write and test `lqr/track_a_control.py` (the latched reference path and the LQR tracker), `lqr/evaluate_controllers.py`, `lqr/analyse_controllers.py`, `lqr/part_d_summary.py`, `lqr/plot_tracking.py` and their tests; `lqr/plot_trace.py` and `lqr/sweep_compare.py` (diagnostic plots and the compensated R-sweep comparison); the analysis of the compensator and sweep results; add the opt-in no-path recovery and the rotate-threshold option; diagnose the baseline stall from the saved traces and occupancy grids; and draft sections of `PART_D_CONTROL.md`. **Ethan** ran all simulations personally, parsed through, expanded and corrected generated code base.