# Part D viva notes

Short answers to the likely questions, with the numbers. Check every figure against `PART_D_CONTROL.md` before the viva and
re-derive anything you cannot explain. Part of this code was written with AI assistance (see the declaration in `PART_D_PROJECT_PAGE.md`),
so you must be able to explain every file you claim: if you cannot, say so honestly instead of guessing.

## Model and design
- **Error model.** `e_y' = v sin(e_theta)`, `e_theta' = w`; linearised `A = [[0, v], [0, 0]]`, `B = [[0], [1]]`. Frames: world pose from the EKF → project onto the active segment → `(e_y, e_theta)`, `e_y` left positive.
- **Controllability.** `[B, AB]` has rank 2 only if `v ≠ 0`. A stationary robot cannot remove lateral error by turning, which is why the tracker rotates on heading error alone (ignoring `e_y`) when `|e_theta| > 0.4` rad.
- **Where K comes from.** Bryson's rule: weight = 1 / (largest acceptable value)². `e_y` 0.10 m → 100; `e_theta` 0.35 rad → 8.2; `w` 0.8 rad/s → R = 1.56. Discrete LQR at 10 Hz and 0.18 m/s gives `K = [6.94, 2.54]` (continuous would give [8.0, 2.85]; the sample-and-hold loop is slower, so the discrete one is correct).
- **What limits the weights.** Yaw saturation (0.8 rad/s), the 0.4 rad rotate threshold (the `e_theta` weight sits just under it), the 2 rad/s wheel dead zone (Section 2.1) and the wheel-rate envelope (speed and yaw share about 5.9 rad/s of usable rate).
- **"What if R were ten times smaller?"** For the LQR effort weight: the gain rises (R×0.25 already gives `K = [12.4, 4.1]`) but the yaw command saturates at 0.8 anyway, so the model predicts little speed-up and more effort. For the EKF measurement covariance (Part C): the filter trusts the camera more, the estimate gets noisier, and NIS rises.
- **Why a fixed gain?** K_ey changes about 2% and K_etheta about 14% across 0.06–0.18 m/s, and Section 1.7 shows a single gain stays stable (gain margin about 8× on yaw gain). Scheduling could still help performance; it was not tested.

## Pipeline
- **Latched reference.** The planner replans from the estimated pose every frame, so a path that starts at the robot has near-zero cross-track error by construction. `ReferencePath` keeps a straightened polyline and replaces it only when blocked for 3 frames, when the goal moves, or when the robot is more than 0.4 m away.
- **Straightening.** Greedy line-of-sight on the inflated occupancy grid (string pulling), so segments are long and straight, matching the straight-segment model.
- **Segment switching.** Advance when within 0.10 m of the vertex; `e_theta` is wrapped, and a large jump triggers rotate-in-place with hysteresis (enter 0.40, exit 0.25).
- **Ground truth.** The controller receives only `ekf.pose`. Truth is read in `run()` after the command is computed, for logging.

## Results
- **LQR against baseline.** Cross-track RMS 6.3 against 13.6 cm nominal, 8.4 against 15.1 dropout, 8.2 against 10.9 slippery; lower in 10/10 paired seeds each. Driving effort about 7× lower. But 28–31% of its time rotating in place, 6–9 s slower to dock and higher total effort.
- **Why most of the remaining error is not the controller.** Estimate-to-truth cross-track gap is 69–73% of the LQR's true error; the EKF is overconfident (mean NEES 6.7 against 3).
- **Why p = 0.002.** It is the smallest p a Wilcoxon signed-rank test can give with ten pairs (10/10 in one direction); it does not mean "more significant" than that.
- **Docking.** Unchanged relay docking; final lateral error 2.2–2.4 cm (LQR) against 2.6–3.2 cm (baseline). The difference is small and the docking code is shared.

## Failures
- **The stall (baseline, shifted obstacle, 5/10 seeds).** Marker leaves view at 2.7 s → wheels turn more than the body (dead zone and slip) → 15° heading error → LiDAR map tilts → goal cell occupied → no path → `NO SAFE PATH - STOP` commands zero motion → no new marker view → permanent deadlock. Evidence: the map rebuilt from true geometry at the estimated pose matches the saved grid in 99.9% of cells.
- **The recovery result.** Heuristic with recovery: 10/10 against 5/10 on the shifted-obstacle seeds; the four non-stalling seeds are bit-identical. It needed only 2–9 search frames (about 2–10° of real turning), and the camera was still off, so "the marker came back into view" is *not* the explanation. Likelier: a marginally blocked route reopened after a small rotation. Say it is unexplained and name the check (`diagnose_no_path.py`).
- **Which p-value?** Fisher (p = 0.033) treats the groups as independent; the seeds are shared, so the paired exact McNemar (p = 0.0625, five discordant seeds all one way) is the more defensible test. Report both and say "borderline at ten seeds".
- **What you cannot claim.** That LQR avoids the stall by design (untested; it may keep following a stale reference when the planner has no path), that recovery fixes the cause (it does not change the heading drift), or why recovery works.

## Limits and what you would change
Uncompensated actuators (dead zone not integrated); fixed layout within a condition; ten seeds per cell; XTE measured against a latched reference that the baseline does not track; no hardware.
Next: integrate the dead-zone compensation and a speed schedule `v(|w|)`, add integral action for a constant bias, curvature feedforward at corners, a no-path recovery, and test whether gain scheduling in speed helps.
