# Part D: Control (Track A, TurtleBot3 Waffle Pi)

Code, all under `robotis_tb3/lqr/` unless noted: `track_a_control_design.py` (model, LQR, robustness, no simulator imports), `track_a_control.py` (latched reference path and the LQR tracker), `evaluate_controllers.py` and `analyse_controllers.py` (batch runs and statistics), `part_d_summary.py` (generates the tables below). Actuator work, in `robotis_tb3/`: `identify_actuators.py`, `actuator_compensation.py`, `verify_compensation.py`. Figures and raw results: `evidence/part-d/`.

## 1. Controller design

### 1.1 What the controller must do
Track the planned path using the **estimated** pose only (the EKF output, never simulator ground truth).
The controller outputs a forward speed `v` and a yaw rate `w`, which `twist_to_wheel_rates` converts to wheel
commands at 10 Hz (control period `dt = 0.1 s`).

### 1.2 Error coordinates and frames
The path is a polyline in the world frame. On the active segment with tangent heading `psi_p`:

- `e_y` = signed lateral offset of the robot from the segment, **left positive** [m]
- `e_theta = wrap(theta - psi_p)` [rad]

Both are computed from the EKF pose `(x, y, theta)` in the world frame, so the chain from sensor to controller input is
camera/encoders → EKF (world frame) → projection onto the segment → `(e_y, e_theta)`.

### 1.3 Model and assumptions
Unicycle kinematics in error coordinates on a straight segment (curvature 0):

```
e_y'     = v sin(e_theta)
e_theta' = w
```

Linearising about `e = 0` at speed `v`, with `x = [e_y, e_theta]^T` and input `w`:

```
A = [[0, v], [0, 0]],   B = [[0], [1]]
```

Assumptions, stated so they can be challenged:

1. Locally straight segments. Corners are handled by switching segments and rotating in place, not by curvature feedforward.
2. Small errors (`sin e ≈ e`).
3. `v` is treated as constant at `V_NOM = 0.18 m/s`. Section 1.6 checks what happens when it is not.
4. The commanded yaw rate is achieved; see 1.6 and 1.7.

### 1.4 Controllability
`[B, AB] = [[0, v], [1, 0]]` has rank 2 if and only if `v ≠ 0`. This is the same rank drop at zero speed found in Part A, now for the error model: a stationary robot cannot remove lateral error by turning alone. This is why the tracker rotates in place on heading error only (gain `K_theta`) when `|e_theta|` is large, rather than letting `e_y` drive a spin it cannot fix.

### 1.5 LQR design
Discretised exactly at `dt = 0.1 s` (A is nilpotent): `Ad = [[1, v dt], [0, 1]]`, `Bd = [[v dt²/2], [dt]]`. Control law
`w = -K [e_y, e_theta]`, clipped to ±0.8 rad/s.

Weights follow Bryson's rule, weight = 1 / (largest acceptable value)²:

| Quantity | Largest acceptable | Reason | Weight |
|---|---|---|---|
| `e_y` | 0.10 m | half the 0.20 m obstacle-inflation margin | Q11 = 100 |
| `e_theta` | 0.35 rad | just under the 0.4 rad rotate-in-place threshold | Q22 = 8.2 |
| `w` | 0.8 rad/s | the existing yaw saturation | R = 1.56 |

Result at `v = 0.18 m/s`: **K = [6.94, 2.54]** (discrete), against [8.0, 2.85] from the continuous design. The two differ because a 10 Hz sample-and-hold loop is slower than the continuous ideal, so the discrete gain is the one to use. 

Across `v` = 0.06 to 0.18 m/s, `K_ey` changes by about 2% (7.07 to 6.94) and `K_etheta` by about 14% (2.22 to 2.54).
The robustness check in 1.7 shows a single fixed gain stays stable and well damped over that range, so speed scheduling is not needed for stability; it could still be tested for performance.

### 1.6 Comparison with the baseline controller
The baseline law `w = 2.8 · bearing` to a point `L = 0.20 m` ahead is equivalent, for small errors, to `K = [2.8/L, 2.8] = [14, 2.8]`. On the same linear model it is a second-order system with `zeta = 2.8 / (2 sqrt(0.18 · 14)) = 0.88`, so it is well damped, not poorly.

The LQR is not an improvement in stability; it is a **different point on the speed/effort trade-off**, chosen from stated tolerances rather than tuned by hand.

Nonlinear step test (0.20 m lateral offset, saturated yaw rate, `step1_lqr_vs_heuristic.png`):

| | K (e_y, e_theta) | Settle to 2 cm | Peak heading swing | Effort ∫w² |
|---|---|---|---|---|
| Baseline | 14, 2.8 | 2.5 s | 39° | 0.74 |
| LQR | 6.94, 2.54 | 4.2 s | 25° | 0.34 |

### 1.7 Robustness of the design to the real actuator
The LQR was derived for an ideal actuator. To check it against a non-ideal one, the loop was re-analysed with a first-order yaw lag
(`tau = 0.11 s`, the value measured in the actuator step tests) and a plant yaw gain `g` (1 = ideal), keeping K fixed
(`closed_loop_robustness`, `max_stable_yaw_gain`):

- Stable for every combination tested: `g` from 0.3 to 2.0 and `v` from 0.03 to 0.18 m/s (spectral radius ≤ 0.992).
- Worst damping ratio 0.47 (at `g = 2`); 0.61 at `g = 0.3`, `v = 0.18`.
- Gain margin on the yaw gain: **about 8.6× at 0.18 m/s and 8.9× at 0.096 m/s**. With an ideal actuator it is 9.9×.
- Cost of low speed: the slowest pole moves toward 1 as `v` falls (ρ = 0.938, 0.971, 0.992 at v = 0.18, 0.096, 0.03 m/s), because
  lateral error can only be removed by driving while turned. At 0.03 m/s the slowest closed-loop time constant is about 12 s.

### 1.8 Limitations
- No curvature feedforward; corners rely on segment switching and rotate-in-place.
- A single fixed gain; no integral action, so a constant lateral bias from the plant would not be removed.
- Controller states come from the EKF, so estimator error enters directly (separation is assumed, not proved, for this nonlinear system).
- Weights are a defensible starting point from stated tolerances, not an optimum. Section 3 sweeps `R` to quantify the trade-off.

### 1.9 Docking is not state feedback (a decision, with its reason)
Only the path-following phase is LQR. The final approach (`EKF VISUAL DOCK`) is inherited unchanged from the baseline tracker: a relay turn of ±0.8 rad/s above a 0.06 rad heading error, and translation at `max(0.10, min(0.18, 0.8·distance))` m/s.

It was left as it is for three reasons. (1) Section 2.1 shows wheel commands below about 2 rad/s do nothing, so a proportional law that shrinks its output near the goal would stall short of it; the relay and the 0.10 m/s floor exist for that reason. (2) Docking accuracy is already well inside the success criteria and is **similar for both controllers**: final lateral error 2.2–2.4 cm for the LQR and 2.6–3.2 cm for the baseline, heading about 4° for both
(Section 4.2). The sub-centimetre difference is small against the 8 cm criterion, and the docking code is shared, so a better path tracker does not meaningfully change it. (3) Replacing it properly needs the actuator compensation of Section 2, which is now available as an opt-in flag (Section 2.4) but has not been evaluated at docking over repeated trials.

This is a limitation, not a claim that relay docking is optimal.

## 2. Actuator identification and compensation

Code: `identify_actuators.py`, `actuator_compensation.py`, `verify_compensation.py`
(tests: `test_track_a_actuator_compensation.py`). All numbers are from open-loop bench runs on the project's
Waffle Pi model, using the same 500-step settle and the same 10 Hz / 1024-tick encoder timing as `track_a_project.py`.
Ground-truth base velocity is read only to score what the plant did.

### 2.1 What the plant actually does
| Measurement | Value | Notes |
|---|---|---|
| Wheel lag | τ ≈ 0.11 s | about one control period |
| Friction offset, straight | ≈ 2.0 rad/s | wheel runs ~2 rad/s slower than commanded; commands below ≈ 2.1 rad/s do nothing |
| Friction offset, in place | ≈ 2.3 rad/s | differs per wheel when turning on the move (1.6 to 2.6 seen) |
| Ground speed per wheel rate | 0.0281 m/rad (nominal 0.0330) | ratio 0.85; independent of floor friction, so not slip |
| Yaw per wheel-rate difference | ≈ 0.075 in place (nominal 0.115) | lower again while driving, ≈ 0.70–0.75 of the in-place value |

Consequences for the controller as it currently runs (command → achieved):
- cruise 0.18 m/s → **0.096 m/s**; the 0.10 m/s minimum speed → **0.03 m/s**
- in-place yaw 0.8 rad/s (the relay value) → **0.19 rad/s**, and 0.6 rad/s → 0.05 rad/s

The in-place numbers imply a 45° turn takes about 4 s. That is *consistent with* the long rotate-in-place times seen in the LQR trials (≈ 11 s per run) and Section 2.5 tests it in closed loop with compensation on and off: rotate-in-place time falls from 11.4 s to 2.2 s in the nominal condition, which supports it.

**Note for Parts A–C.** The 0.85 ratio also appears in the project's own traces: encoder odometry path length is 17% longer than the true path (true/odometry = 0.83–0.86 in three traces). The odometry model assumes r = 0.033 m. The EKF corrects this with the camera.

### 2.2 Compensator
Desired twist (v, w) → wheel rates using the measured gains → add a signed friction offset (c_ff = 2.0 rad/s) → clip to ±7.88 rad/s.

Yaw has priority: when speed and yaw together exceed the actuator range, speed is cut first.

Open-loop check at 10 Hz, steady state over 2 s, 11 fit twists and 6 **held-out** twists (RMS error, friction 1.0):

| Method | v error (fit / held-out) [m/s] | w error (fit / held-out) [rad/s] |
|---|---|---|
| Current mapping | 0.054 / 0.064 | 0.295 / 0.249 |
| **Feedforward (recommended)** | **0.008 / 0.028** | **0.057 / 0.058** |
| Feedforward, speed-blended yaw gain | 0.027 / 0.043 | 0.062 / 0.089 |
| + PI trim on encoder rates | 0.027 / 0.051 | 0.068 / 0.107 |

The floor-friction 0.6 condition changed the compensated results by at most 0.016 (combined twist), so these tests cannot show whether the trim would help on a slippery floor.

Remaining residuals: yaw while driving is still under-delivered by roughly 15–25%, and low-speed turns lose speed because the inner wheel sits near the friction band. The outer loop absorbs both; Section 1.7 showed the LQR keeps a gain margin of about 8× on yaw gain.

### 2.3 Actuator envelope (a design constraint)
The 7.88 rad/s limit minus the friction offset leaves about 5.9 rad/s of usable wheel rate, shared between speed and yaw:

| \|w\| [rad/s] | 0 | 0.2 | 0.4 | 0.6 | 0.8 |
|---|---|---|---|---|---|
| max feasible v [m/s] | 0.165 | 0.128 | 0.090 | 0.053 | 0.015 |

So `V_NOM = 0.18` m/s and `W_MAX = 0.8` rad/s cannot both be reached. The compensator applies this limit itself (yaw has priority, speed is cut first), so no separate schedule is added to the tracker; the trace records what was asked (`speed_cmd`) and what was allowed (`speed_applied`).

### 2.4 Integration (opt-in; off by default)
`track_a_project.py --compensate {off,path,all}`:

| mode | which frames use the compensator |
|---|---|
| `off` (default) | none: the original `twist_to_wheel_rates` mapping, so every earlier result and cached trial stays valid |
| `path` | path following and the no-path search turn; the docking relay keeps the original mapping |
| `all` | every frame, docking included |

`path` exists because the docking relay (±0.8 rad/s above a 0.06 rad error, creep at 0.10 m/s or more) was tuned on the weak plant. Compensated, the same relay command delivers about four times more yaw per step (≈ 0.08 rad against a 0.06 rad dead band), so `all` may chatter or overshoot. Whether it does is a question for the
ten-seed batch, not something this section claims.

Evaluation specs take `+comp` (= `path`) or `+compall` (= `all`) as the **last** modifier, for example `lqr+comp` or `lqr+recovery+compall`; results are tagged
`lqr_comp` / `lqr_compall` and stored in their own folders. The compensator is the feedforward-only one of Section 2.2. It uses no ground truth.

New logged fields: per frame `speed_applied` and `compensated` (in `trace.json`); per trial `compensate` and `mean_driving_speed_{cmd,applied,truth}_mps` (in `result.json`).
The truth speed is computed from consecutive true poses and is for evaluation only.

Checks made when it was integrated (one environment, one seed, **mechanism checks, not results**):
- With `--compensate off`, a 15 s LQR trace (150 frames) is exactly equal to the trace from the code before the change (estimate, truth, wheel commands, commands).
- Delivered driving speed in a closed-loop LQR trial: about 0.093 m/s uncompensated for a 0.18 m/s request, and about 0.139 m/s compensated, against 0.145 m/s predicted by the compensator model.

Study run (results in Section 2.5), from a fresh output folder with the same seeds for every mode:
```sh
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr+comp lqr+compall \
    --conditions nominal camera_noise_dropout shifted_obstacle slippery_floor --trials 10 --output eval_comp
python robotis_tb3/lqr/part_d_summary.py eval_comp/summary.json --comp
```

### 2.5 Closed-loop evaluation of the compensator
**Protocol.** Three controllers (`lqr`, `lqr+comp`, `lqr+compall`), four conditions, the same ten seeds in each cell: 120 trials, all arrived with no collision (120/120). Conditions and success criteria as in Section 4.1.
Tables below are mean ± sd over the ten trials of each cell, generated by `part_d_summary.py --comp` from `evidence/part-d/eval_comp/summary.json`.

The compensator's parameters (Section 2.2) come from the open-loop bench runs of Sections 2.1 and 2.2, not from these trials. **[EDIT: confirm this is true of your process; if you adjusted anything after seeing closed-loop results, say so here.]**

`lqr` against `lqr+comp` (`path` mode):

| condition | controller | success (95% CI) | XTE RMS [cm] | XTE max [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] | final lat [cm] |
|---|---|---|---|---|---|---|---|---|---|
| nominal | lqr | 10/10 (0.72–1.00) | 6.3 ± 1.3 | 15.2 ± 1.2 | 0.55 ± 0.08 | 7.62 ± 1.48 | 11.4 ± 2.3 | 52.6 ± 2.3 | 2.2 ± 0.3 |
| nominal | lqr_comp | 10/10 (0.72–1.00) | 7.2 ± 0.3 | 16.7 ± 0.6 | 0.44 ± 0.03 | 1.82 ± 0.03 | 2.2 ± 0.1 | 34.8 ± 0.3 | 2.1 ± 0.3 |
| camera_noise_dropout | lqr | 10/10 (0.72–1.00) | 8.4 ± 2.0 | 17.1 ± 2.3 | 0.58 ± 0.21 | 6.79 ± 1.04 | 10.0 ± 1.6 | 56.6 ± 2.5 | 2.4 ± 0.1 |
| camera_noise_dropout | lqr_comp | 10/10 (0.72–1.00) | 9.1 ± 1.3 | 16.9 ± 1.3 | 0.38 ± 0.19 | 2.04 ± 0.53 | 2.6 ± 0.7 | 40.6 ± 1.2 | 2.0 ± 0.1 |
| shifted_obstacle | lqr | 10/10 (0.72–1.00) | 12.9 ± 1.1 | 21.6 ± 1.1 | 1.41 ± 0.38 | 9.44 ± 1.78 | 13.0 ± 2.5 | 58.5 ± 2.7 | 2.7 ± 0.2 |
| shifted_obstacle | lqr_comp | 10/10 (0.72–1.00) | 12.4 ± 1.5 | 22.4 ± 2.2 | 0.42 ± 0.17 | 3.55 ± 2.75 | 4.9 ± 4.3 | 39.6 ± 4.5 | 2.4 ± 0.3 |
| slippery_floor | lqr | 10/10 (0.72–1.00) | 8.2 ± 1.0 | 17.0 ± 2.1 | 0.78 ± 0.39 | 7.50 ± 0.97 | 10.8 ± 1.3 | 53.0 ± 2.6 | 2.2 ± 0.2 |
| slippery_floor | lqr_comp | 10/10 (0.72–1.00) | 7.7 ± 0.2 | 18.6 ± 0.2 | 0.43 ± 0.11 | 1.85 ± 0.04 | 2.2 ± 0.1 | 34.5 ± 0.3 | 1.7 ± 0.3 |

Paired differences (`lqr_comp` − `lqr`, same seeds), Wilcoxon signed-rank:

| condition | XTE RMS [cm], median (lower in n/10) | p | total ∫w², median (lower in n/10) | p | time [s], median (lower in n/10) | p |
|---|---|---|---|---|---|---|
| nominal | +1.34 (2/10) | 0.105 | −6.06 (10/10) | 0.002 | −18.30 (10/10) | 0.002 |
| camera_noise_dropout | +1.39 (3/10) | 0.557 | −4.83 (10/10) | 0.002 | −15.95 (10/10) | 0.002 |
| shifted_obstacle | −0.53 (7/10) | 0.492 | −6.61 (9/10) | 0.004 | −20.30 (10/10) | 0.002 |
| slippery_floor | −0.43 (6/10) | 0.193 | −5.29 (10/10) | 0.002 | −17.70 (10/10) | 0.002 |

(p = 0.002 is the smallest value a Wilcoxon test can give with ten pairs; it does not mean "more significant" than p = 0.004.)

**What the data support.**
1. **The dead zone dominates rotate-in-place time**, as Section 2.1 predicted. Rotating falls from 10.0–13.0 s to 2.2–4.9 s per run (nominal: 11.4 to 2.2 s, from about 30% of tracking time to about 12%). Total effort falls by 62–76% and time to dock by 28–35% in every condition, in 9 or 10 of 10 paired seeds.
2. **Delivered speed now matches the request.** Mean true driving speed rises from 0.094 to 0.145 m/s (nominal), against a 0.18 m/s command uncompensated; the compensator limits itself to about 0.15 m/s while turning (the envelope of Section 2.3) and the plant delivers about 0.14–0.15 m/s of that.
3. **Cross-track error is not significantly changed** (p = 0.105 to 0.557). The medians are about +1.4 cm in nominal and dropout (compensated higher in 8/10 and 7/10 paired seeds) and about −0.5 cm in shifted-obstacle and slippery. The honest reading is "no accuracy gain; possibly a small loss in two conditions".
   Two untested contributors: the robot now drives about 55% faster, and the estimator's cross-track gap grows (est−truth XTE 4.3 to 5.7 cm nominal, 5.9 to 7.5 cm dropout).
4. **Docking accuracy is unchanged or slightly better** (final lateral error 1.7–2.4 cm against 2.2–2.7 cm; final heading error about 3.8° in both). Docking is the shared relay law, so this is not evidence for a better docking law.

**`compall` (compensation during docking too).** Section 2.4 raised the possibility that the relay would chatter or overshoot when compensated. It did not fail (all 40 trials arrived), but it docks less accurately than `path`:

| condition | time [s] lqr_comp / lqr_compall | final lat [cm] | final heading [°] | camera updates | mean NEES |
|---|---|---|---|---|---|
| nominal | 34.8 / 24.2 | 2.1 / 3.5 | 3.8 / 1.7 | 165 / 118 | 6.2 / 8.4 |
| camera_noise_dropout | 40.6 / 27.9 | 2.0 / 2.7 | 3.8 / 2.6 | 161 / 114 | 5.2 / 7.5 |
| shifted_obstacle | 39.6 / 28.7 | 2.4 / 3.7 | 4.1 / 2.0 | 142 / 96 | 7.4 / 9.8 |
| slippery_floor | 34.5 / 24.1 | 1.7 / 3.2 | 3.7 / 2.6 | 168 / 125 | 6.5 / 8.9 |

It saves another 10–13 s and ends with a smaller heading error, but the lateral error is 0.7–1.5 cm larger, the filter gets about 30% fewer camera updates, and mean NEES is 2.3–2.5 higher. The tracking metrics are identical to `lqr_comp` because the phases before docking are identical.
Why docking is worse was not examined (it skips the camera-reacquire phase, which is the obvious candidate). **`path` is the recommended mode**, and `compall` is reported as an ablation.

**Limits of this evidence.** Cells have ten seeds; in the slippery-floor cells of both compensated controllers only 7 of 10 trials differ in cross-track RMS (the seeds change noise, not the layout), so the effective sample is smaller than ten there.
In the nominal condition the compensated runs are almost identical to each other (sd 0.3 cm), so the small sd does not mean the estimate of the mean is more reliable than a noisier cell. The estimator's overconfidence (Section 4.3) is unchanged by compensation.

## 3. Gain sweep and trade-offs

### 3.1 What is varied
The Bryson weights give `K = [6.94, 2.54]`. The one free scalar is the ratio between state and effort weights, varied here as `R × s` with
`s ∈ {0.25, 0.5, 1, 2, 4}` (`--lqr-r-scale`, or the `lqr:<s>` controller spec). Larger `s` means gentler turning. `Q` is not varied separately
because only the ratio `Q/R` changes the optimal gain.

### 3.2 Model prediction (no simulator; generated from `track_a_control_design.py`)
Nonlinear step, 0.20 m lateral offset, yaw saturated at ±0.8 rad/s, ideal actuator:

| R scale | K_ey | K_etheta | slowest pole radius | step overshoot | settle to 2 cm [s] | step ∫w² | peak yaw cmd [rad/s] |
|---|---|---|---|---|---|---|---|
| 0.25 | 12.38 | 4.12 | 0.938 | 0% | 4.2 | 0.43 | 0.80 |
| 0.5 | 9.36 | 3.24 | 0.938 | 0% | 4.2 | 0.39 | 0.80 |
| 1 | 6.94 | 2.54 | 0.936 | 0% | 4.2 | 0.34 | 0.80 |
| 2 | 5.08 | 1.98 | 0.932 | 0% | 4.3 | 0.27 | 0.80 |
| 4 | 3.68 | 1.56 | 0.921 | 0% | 4.5 | 0.19 | 0.74 |

The model predicts a nearly free trade: a 0.20 m step saturates the yaw command for every `s` up to 2, so settling time hardly changes (4.2 to 4.5 s) while effort falls by more than half. **This is a hypothesis for the closed-loop sweep to test, not a result.** It ignores the actuator dead zone (Section 2.1): gentler commands are exactly the ones the dead zone swallows, so the real loop may degrade faster than the model says at large `s`. **It did: see Section 3.3.**

### 3.3 Closed-loop sweep
```sh
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25 lqr:0.5 lqr lqr:2 lqr:4 \
    --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep
python robotis_tb3/lqr/analyse_controllers.py eval_sweep/summary.json --out eval_sweep      # tradeoff.png, markdown tables
python robotis_tb3/lqr/part_d_summary.py eval_sweep/summary.json --sweep                    # table to paste here
```
Report: cross-track RMS (driving frames) against driving effort `∫w² dt`, mean ± s.e. over seeds, with rotate time and time-to-dock alongside. Compare the measured curve with the prediction in 3.2. Copy `summary.json`, the plot and the table into `evidence/part-d/eval_sweep/`.

The same sweep with the compensator (`path` mode); R×1 is taken from `eval_comp`, the other four scales were run separately (`spec` suffix `+comp` goes last):
```sh
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25+comp lqr:0.5+comp lqr:2+comp lqr:4+comp \
    --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep_comp
python robotis_tb3/lqr/sweep_compare.py eval_sweep/summary.json eval_comp/summary.json eval_sweep_comp/summary.json --out figures
```
`sweep_compare.py` merges the three summaries (the standard sweep tables ignore compensated runs) and writes `sweep_comp_table.md` and `sweep_comp_tradeoff.png`.

**Results** (successful trials only; mean ± sd; the plot shows mean ± s.e.). Plain LQR at R×4 had 0/10 successes in both conditions, so it has no row; every compensated cell is 10/10.

| condition | controller | R scale | n | XTE RMS driving [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] | final lat [cm] |
|---|---|---|---|---|---|---|---|---|---|
| nominal | plain LQR | 0.25 | 10 | 7.9 ± 1.1 | 1.43 ± 0.41 | 8.33 | 10.8 | 53.2 | 2.2 |
| nominal | plain LQR | 0.5 | 10 | 7.0 ± 1.2 | 0.94 ± 0.22 | 6.61 | 8.9 | 50.5 | 2.4 |
| nominal | plain LQR | 1 | 10 | 6.6 ± 1.0 | 0.55 ± 0.08 | 7.62 | 11.4 | 52.6 | 2.2 |
| nominal | plain LQR | 2 | 7 | 6.8 ± 1.3 | 0.65 ± 0.08 | 19.28 | 57.5 | 99.0 | 2.4 |
| nominal | LQR + compensator | 0.25 | 10 | 8.9 ± 0.5 | 0.66 ± 0.09 | 2.28 | 2.5 | 35.4 | 1.9 |
| nominal | LQR + compensator | 0.5 | 10 | 8.5 ± 1.1 | 0.58 ± 0.17 | 2.11 | 2.4 | 35.5 | 1.9 |
| nominal | LQR + compensator | 1 | 10 | 7.1 ± 0.3 | 0.44 ± 0.03 | 1.82 | 2.2 | 34.8 | 2.1 |
| nominal | LQR + compensator | 2 | 10 | 8.8 ± 0.4 | 0.25 ± 0.09 | 1.74 | 2.5 | 35.0 | 2.1 |
| nominal | LQR + compensator | 4 | 10 | 10.4 ± 1.0 | 0.33 ± 0.16 | 1.72 | 2.8 | 35.4 | 1.7 |
| camera_noise_dropout | plain LQR | 0.25 | 10 | 7.6 ± 1.4 | 1.28 ± 0.23 | 7.23 | 9.3 | 56.8 | 2.3 |
| camera_noise_dropout | plain LQR | 0.5 | 10 | 7.8 ± 1.4 | 1.08 ± 0.47 | 7.03 | 9.3 | 56.5 | 2.3 |
| camera_noise_dropout | plain LQR | 1 | 10 | 8.2 ± 1.8 | 0.58 ± 0.21 | 6.79 | 10.0 | 56.6 | 2.4 |
| camera_noise_dropout | plain LQR | 2 | 5 | 6.6 ± 0.7 | 0.68 ± 0.24 | 19.86 | 58.7 | 105.2 | 2.6 |
| camera_noise_dropout | LQR + compensator | 0.25 | 10 | 9.6 ± 1.6 | 0.78 ± 0.29 | 2.35 | 2.5 | 41.4 | 2.0 |
| camera_noise_dropout | LQR + compensator | 0.5 | 10 | 8.9 ± 1.4 | 0.44 ± 0.19 | 1.98 | 2.4 | 40.9 | 1.9 |
| camera_noise_dropout | LQR + compensator | 1 | 10 | 9.1 ± 1.3 | 0.38 ± 0.19 | 2.04 | 2.6 | 40.6 | 2.0 |
| camera_noise_dropout | LQR + compensator | 2 | 10 | 8.9 ± 1.4 | 0.24 ± 0.09 | 1.66 | 2.4 | 40.2 | 1.9 |
| camera_noise_dropout | LQR + compensator | 4 | 10 | 8.6 ± 0.9 | 0.19 ± 0.06 | 1.56 | 2.6 | 40.3 | 2.0 |

Success of plain LQR by R scale: R×0.25, ×0.5, ×1: 10/10 in both conditions; R×2: 7/10 nominal and 5/10 dropout; R×4: 0/10 in both.

![R sweep: cross-track RMS against driving effort, plain and compensated](evidence/part-d/sweep_comp_tradeoff.png)

**Reading the sweep.**
1. **Against the model (3.2), up to R×1 the prediction holds for plain LQR:** raising R from 0.25 to 1 cuts driving effort by 62% (1.43 to 0.55) with no loss of accuracy (7.9 to 6.6 cm). **At R×2 and above it fails, as the dead-zone caveat in 3.2 warned.**
2. **Why plain LQR fails at high R (diagnosed from two traces).** In the failed runs examined (`eval_sweep`, R×2 nominal seed 8 and R×4 nominal seed 0) the robot has zero speed command in 80% and 90% of frames. During those frames, yaw commands of 0.3–0.7 rad/s produced a median true yaw rate of about 0.002 rad/s, i.e. nothing,
   while 0.7–0.8 rad/s produced about 0.17–0.19 rad/s. The rotate-in-place law is `w = −K_etheta · e_theta`, so at the 0.25 rad exit threshold the command is 0.64 rad/s at R×1, 0.50 at R×2 and 0.39 at R×4: the larger R is, the further the final approach falls into the dead band, and the robot cannot finish rotating.
   This agrees with the identified dead zone (Section 2.1, 0.6 rad/s gives 0.05 rad/s). Only two traces were opened, and R×1 is itself marginal (about 0.085 rad/s achieved at 0.5–0.7 rad/s commanded), which is consistent with its 11 s of rotating.
3. **The compensator removes this failure mode:** 10/10 at R×2 and R×4 in both conditions, with 2.4–2.8 s of rotating. It widens the usable R range from about [0.25, 1] to at least [0.25, 4].
4. **At a matched R below 2 the compensator is a trade, not a dominant improvement.** In the plot the compensated curve is to the left (less effort) and above (more cross-track error): nominal R×1 is 7.1 against 6.6 cm and R×0.5 is 8.5 against 7.0 cm, while total effort is about 70% lower.
5. **The best R depends on the condition.** Nominal: the compensated curve has its minimum at R×1 (7.1 ± 0.3 cm); R×4 costs about 3 cm for a 25% effort reduction. With camera noise and dropout, XTE does not rise with R (8.6 cm at R×4, 9.6 at R×0.25; differences are inside the spread) while effort falls from 0.78 to 0.19. No significance test was run on the sweep.

**Choice of gain - optional based on effort.** R×1 with the compensator: the lowest compensated cross-track error in the nominal condition, low effort, and the value the Bryson weights were derived for (Section 1.5). R×2 is a defensible alternative if effort matters more than about 1.7 cm of nominal tracking accuracy (about 40% less driving effort, no change under noise).

### 3.4 Rotate-in-place threshold (to run)
In the 80-trial comparison the LQR spends 28–31% of its tracking time rotating in place (Section 4.2), against 1–5% for the baseline. The threshold is 0.40 rad with exit at 0.25 rad, inherited from the baseline; the `e_theta` weight in Section 1.5 was chosen just under it, so raising the threshold weakens that justification. The option `--lqr-rotate-enter` (spec `lqr@<rad>`) keeps the 0.625 exit ratio. Rotation time is dominated
by the actuator dead zone rather than by the threshold (a 45° turn takes about 4 s at the relay value, Section 2.1), so the expected gain is uncertain.
```sh
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr@0.6 lqr@0.8 \
    --conditions nominal slippery_floor --trials 10 --output eval_rotate
python robotis_tb3/lqr/analyse_controllers.py eval_rotate/summary.json --out eval_rotate
```
Report rotate time, time to dock and cross-track error together: a higher threshold saves time only if the larger transient offset stays acceptable.

## 4. Repeated trials and failure diagnosis

### 4.1 Protocol
Ten seeds for each of four conditions and two controllers: 80 trials. Conditions (defined in `evaluate_controllers.py`, the same as `evaluate_track_a.py`):
nominal; camera noise and 30% dropout; shifted obstacle (0.15 m) with doubled encoder bias; floor friction 0.6. A seed changes the LiDAR noise, camera noise and
dropout draws. **The scene layout is fixed within a condition**, so these seeds test robustness to noise, not to different environments.
A trial succeeds under the criteria in `track_a_project.py` (arrived, no collision, within 0.08 m of the stop distance, lateral error under 0.08 m, heading under 0.15 rad).
Tracking metrics use successful trials only; a trial that stalls early has a tiny tracking error that says nothing about the controller.

Metric definitions:
- **XTE** is the signed distance of the *true* pose from the latched reference polyline (ground truth is read only here, for evaluation). The reference is held
  in the EKF frame and replaced only when blocked, when the goal moves, or when the robot is lost (`ReferencePath`), so it is not the path that is replanned every frame.
  Each replacement restarts the line at the robot, which resets the apparent error; `reference adoptions` (7 to 13 per run) is reported for that reason.
- **Driving ∫w²** counts frames with forward speed above zero; **total ∫w²** adds the rotate-in-place frames. Rotating dominates the LQR's effort, so both are reported.
- The baseline tracker does not follow the latched reference, so XTE measures "closeness to the latched reference", not a ranking of controllers in general.

### 4.2 Results (generated by `part_d_summary.py` from `evidence/part-d/eval_cmp/summary.json`)
Mean ± sd over successful trials. Success shows the 95% Wilson interval over all ten trials.

| condition | controller | success (95% CI) | XTE RMS [cm] | XTE max [cm] | driving ∫w² | total ∫w² | rotate [s] | time [s] | final lat [cm] |
|---|---|---|---|---|---|---|---|---|---|
| nominal | heuristic | 10/10 (0.72–1.00) | 13.6 ± 2.1 | 24.9 ± 2.2 | 3.87 ± 0.16 | 4.23 ± 0.21 | 0.6 ± 0.1 | 44.9 ± 0.2 | 2.9 ± 0.1 |
| nominal | lqr | 10/10 (0.72–1.00) | 6.3 ± 1.3 | 15.2 ± 1.2 | 0.55 ± 0.08 | 7.62 ± 1.48 | 11.4 ± 2.3 | 52.6 ± 2.3 | 2.2 ± 0.3 |
| camera_noise_dropout | heuristic | 10/10 (0.72–1.00) | 15.1 ± 2.1 | 25.7 ± 4.1 | 3.99 ± 0.29 | 4.58 ± 0.39 | 0.9 ± 0.4 | 50.5 ± 0.8 | 2.6 ± 0.2 |
| camera_noise_dropout | lqr | 10/10 (0.72–1.00) | 8.4 ± 2.0 | 17.1 ± 2.3 | 0.58 ± 0.21 | 6.79 ± 1.04 | 10.0 ± 1.6 | 56.6 ± 2.5 | 2.4 ± 0.1 |
| shifted_obstacle | heuristic | 5/10 (0.24–0.76) | 15.5 ± 2.1 | 29.9 ± 4.4 | 4.26 ± 0.37 | 5.32 ± 0.47 | 1.7 ± 0.2 | 53.2 ± 8.5 | 3.2 ± 0.2 |
| shifted_obstacle | lqr | 10/10 (0.72–1.00) | 12.9 ± 1.1 | 21.6 ± 1.1 | 1.41 ± 0.38 | 9.44 ± 1.78 | 13.0 ± 2.5 | 58.5 ± 2.7 | 2.7 ± 0.2 |
| slippery_floor | heuristic | 10/10 (0.72–1.00) | 10.9 ± 1.9 | 19.6 ± 3.3 | 3.71 ± 0.09 | 3.80 ± 0.11 | 0.1 ± 0.3 | 44.3 ± 0.4 | 2.8 ± 0.1 |
| slippery_floor | lqr | 10/10 (0.72–1.00) | 8.2 ± 1.0 | 17.0 ± 2.1 | 0.78 ± 0.39 | 7.50 ± 0.97 | 10.8 ± 1.3 | 53.0 ± 2.6 | 2.2 ± 0.2 |

Paired differences (seeds where both succeeded), Wilcoxon signed-rank:
- XTE RMS, LQR − heuristic: nominal −8.43 cm, dropout −7.52 cm, slippery −2.23 cm; LQR lower in 10/10 paired seeds each, p = 0.002 (the smallest p possible with ten pairs).
  Shifted obstacle: −2.25 cm, 5/5 paired seeds, too few pairs for a test.
- Total ∫w², LQR − heuristic (nominal, dropout, slippery): +3.57, +2.00, +3.33; LQR higher in every paired seed. Time to dock: +8.0 s, +6.6 s, +8.1 s; LQR slower in every paired seed.
- Success in the shifted-obstacle condition: LQR 10/10 against heuristic 5/10. Fisher exact p = 0.033 (treats the two groups as independent); exact McNemar p = 0.0625
  (paired by seed: five seeds favour the LQR, none favour the baseline). The Wilson intervals (0.72–1.00 and 0.24–0.76) barely separate. Report the result as "favours LQR, borderline at ten seeds", not as a clear significant difference.

![Per-trial cross-track RMS](evidence/part-d/eval_cmp/tracking_boxplot.png)

One closed-loop trial in detail (`lqr/plot_tracking.py`; LQR, nominal, seed 0 from an earlier run, so the latched reference lines, which newer traces record, are absent from panel a):

![LQR nominal trial: path, cross-track error, heading uncertainty, yaw command against achieved](evidence/part-d/tracking_lqr_nominal_seed0.png)

Panel (b) shows the controller regulating its own estimate close to zero while the true error is larger wherever the estimate is biased (about 10 to 14 s and 20 to 28 s, when the marker is out of view). Panel (c) shows the heading error staying inside the filter's own ±2σ band, and the band growing while no marker update is accepted.
Panel (d) shows the actuator dead zone directly: a 0.65 rad/s command during the camera-reacquire turn produces about 0.08 rad/s of actual yaw, and the 0.8 rad/s relay about 0.19 rad/s.

### 4.3 What the results support, and what they do not
Supported:
1. The LQR keeps the robot closer to the latched reference in all four conditions: 54% lower in nominal, 45% in dropout, 25% on the slippery floor, and about 15% in shifted-obstacle (paired median −2.25 cm on the five paired successes).
2. It drives more smoothly: driving ∫w² is about 7× lower in nominal and dropout, and 3–5× lower in the slippery and shifted-obstacle conditions.
3. That smoothness has a price: total effort is higher and docking takes 6–9 s longer, because 28–31% of tracking time is rotating in place.

Not supported:
- **Materially better docking.** Final lateral error is 0.5–0.7 cm smaller for the LQR and heading is about 4° for both (Section 1.9). That is real but small against the 8 cm criterion, and the docking code is shared, so the gain is not attributable to a better docking law.
- **A controller-limited error.** The estimator's error is 69–73% of the LQR's true cross-track error (table below). The controller regulates the estimated state to a few centimetres, so
  what remains is what the EKF cannot see. Retuning the gains cannot remove it. The EKF is also overconfident (mean NEES 6.7 against 3 expected), which is a Part C finding.
- **Robustness to different environments**, or any hardware claim (none run).

| condition | est−truth XTE [cm] | truth XTE [cm] | ratio |
|---|---|---|---|
| nominal | 4.3 ± 0.9 | 6.3 ± 1.3 | 0.69 |
| camera_noise_dropout | 5.9 ± 1.5 | 8.4 ± 2.0 | 0.71 |
| shifted_obstacle | 9.4 ± 0.4 | 12.9 ± 1.1 | 0.73 |
| slippery_floor | 5.9 ± 0.8 | 8.2 ± 1.0 | 0.73 |

### 4.4 Failure diagnosis: the baseline's no-path stall
Five of ten baseline trials in the shifted-obstacle condition (seeds 0, 2, 4, 6, 7) ended in `NO SAFE PATH - STOP` with a final lateral error of 0.243–0.247 m and only 27–28 camera updates.
Traces and occupancy grids were examined for seeds 0 and 2; the other three show the same signature but were not opened. The chain, with the evidence for each link:

1. **The marker leaves view at 2.7 s** (last accepted camera update), while the robot turns left to avoid the obstacle.
2. **The heading estimate drifts.** Between 2.7 and 6.1 s the body turned 19.9° but the encoders imply 33.2°. Removing the 0.4% encoder bias changes that by only 0.6°, so it is not the bias: it is the wheel-versus-body mismatch of Section 2.1.
   By 8 s the heading error is 15.3° (estimate 1.203 rad, truth 0.935 rad) while the EKF's own 1σ is 8.5°, so the filter is overconfident.
3. **The occupancy map tilts.** `OccupancyPlanner.update` places LiDAR returns using the estimated pose. Rebuilding the scan from the true room geometry and placing it with the estimated pose reproduces the saved occupancy grid in 99.9% of cells (seed 0; 99.2% for seed 2).
4. **Deadlock.** `NO SAFE PATH - STOP` commands zero velocity and no turn. A stationary robot sees no marker (0 detections in about 1,040 stalled frames), so the heading estimate and the map never change.
   Seed 1, which survived, had the same 14° error at 6 s but kept moving and had reacquired the marker by 20 s, after which the filter's 1σ fell to 1.6°.

`ReferencePath.update` keeps following the previous reference when the planner returns no path, so LQR never reaches the deadlock state. The second can be checked by counting empty-path frames; `tracking.no_path_frames` is now recorded in every `result.json`:
```sh
python -c "import json,sys; r=json.load(open(sys.argv[1])); print(sum(1 for x in r if not x['path']))" eval_cmp/lqr/shifted_obstacle/seed-0/trace.json
```

### 4.5 Recovery behaviour
`StateFeedbackTracker(recovery=True)`, enabled with `--recovery` or the `+recovery` controller spec, rotates in place at the relay value (0.8 rad/s) toward the estimated marker bearing after ten consecutive frames
(1 s) without a planner route, and returns to normal tracking as soon as a route exists. It is never used while docking, and it is off by default, so the recorded baseline failures are unchanged.
Unit tests: `lqr/test_track_a_recovery.py`.

**Result** (shifted-obstacle condition, ten seeds each, `eval_recovery/`; table from `part_d_summary.py --recovery`):

| controller | success (95% CI) | XTE RMS [cm] | XTE max [cm] | total ∫w² | time [s] | final lat [cm] |
|---|---|---|---|---|---|---|
| heuristic | 5/10 (0.24–0.76) | 15.5 ± 2.1 | 29.9 ± 4.4 | 5.32 ± 0.47 | 53.2 ± 8.5 | 3.2 ± 0.2 |
| heuristic + recovery | 10/10 (0.72–1.00) | 15.8 ± 2.0 | 31.1 ± 5.6 | 5.48 ± 0.43 | 50.9 ± 1.6 | 3.1 ± 0.1 |

- All five previously stalled seeds (0, 2, 4, 6, 7) now succeed; no seed got worse. Exact McNemar p = 0.0625 (five discordant seeds, all favouring recovery); Fisher exact p = 0.033; one-sided sign test p = 0.031.
  The evidence favours recovery but is borderline at ten seeds.
- It does not change runs that never stall: seeds 1, 3, 5 and 8 are bit-identical with and without it (no-path runs shorter than the ten-frame trigger). Seed 9 stalled for 179 frames (17.9 s) and recovered by itself in the baseline, taking 68.4 s; with recovery it stalled 12 frames and took 50.7 s.
- No-path frames in the stalled seeds fall from 1038–1039 (the rest of the run) to 13–39, with only **2 to 9 search frames** each (0.2–0.9 s).
- Tracking quality is unchanged (XTE 15.8 against 15.5 cm), and the estimator gap is unchanged (est−truth XTE 11.4 against 11.0 cm): this treats the symptom, not the heading drift behind it.

```sh
for s in 0 2 4 6 7; do python robotis_tb3/lqr/diagnose_no_path.py eval_recovery/heuristic_recovery/shifted_obstacle/seed-$s/trace.json; done
```
For each no-path episode the script prints the length, the number of search frames, any marker updates accepted during it or in the next 2 s, and the heading estimate error at its start and end.
Marker updates during the search would support the first account; none, with the route returning after a few degrees, would support the second.

### 4.6 One slow compensated seed: shifted obstacle, seed 1
In the shifted-obstacle condition nine of ten `lqr_comp` seeds rotate for 2.6–4.5 s and finish in 37–39 s. **Seed 1 rotates for 17.0 s and takes 52.2 s**, which is what produces the ±4.3 s spread in Section 2.5. It is a different failure from the baseline stall of 4.4: it arrives.

| `lqr_comp`, shifted obstacle | seed 1 | other nine seeds |
|---|---|---|
| reference adoptions | **51** | 7 to 13 |
| rotate-in-place time [s] | 17.0 | 2.6 to 4.5 |
| saturated fraction | 0.47 | 0.12 to 0.19 |

Evidence from the traces (`lqr/plot_trace.py`, seed 1 against seed 0 and against plain `lqr` seed 1; figure `seed1_diagnosis.png`):
- Between about 8 s and 24 s the yaw command flips between +0.8 and −0.8 rad/s many times and the applied speed keeps dropping to zero. The truth path is the same shape as in the other runs, so the lost time is spent chattering, not detouring.
- Over the same interval the estimate-to-truth position error rises to about 22 cm and stays there; the estimated cross-track error stays near zero while the true one drifts to about −20 cm. The controller regulates a state that is not where the robot is.
- NEES is above the 7.81 bound (95%, 3 states) for long stretches in all three trials examined, with peaks of 30 to 45 in the first three seconds, and the position error at t = 0 is already 3.5 to 7 cm. That is a Part C observation (initial covariance, and overconfidence under the 0.15 m shift and 0.004 encoder bias); it is passed to Part C, not claimed here.

**Mechanism: a hypothesis, not established.** The latched reference is replaced when the route is blocked for three frames, and a new reference starts at the *estimated* pose, so its estimated cross-track error is near zero whatever the truth is (docstring of `track_a_control.py`).
Each adoption also resets the rotating flag and the segment index. A 0.15 m obstacle shift together with a 20 cm position error would make the LiDAR scan disagree with the stored map, so the path could look blocked every few frames, giving 51 adoptions and new reference headings that flip the yaw command.
The check is to mark each adoption on the yaw-command panel of `plot_trace.py` (it does: ticks at the top of the turn-command panel) and see whether the ticks coincide with the sign flips. **[EDIT: run this and replace this paragraph with what you see.]**
If confirmed, one candidate change (not implemented, not tested) is a minimum hold time between adoptions, or more than three blocked frames; it should be reported with its effect on seed 1 and on the other nine seeds.

**Consequence for the metric.** XTE is measured against the current latched reference, which is replaced during a run (7 to 13 times typically, 51 here). On seeds with many adoptions the reported XTE understates the real deviation, because each adoption resets the line to the robot. State this beside every shifted-obstacle XTE value.

### 4.7 Limitations of this evidence
- Ten seeds per cell with a fixed layout; noise robustness only. Some compensated cells have fewer than ten distinct outcomes (Section 2.5).
- The latched-reference metric flatters neither controller exactly, but the baseline does not track it (Section 4.1), and it is reset by each reference adoption (Section 4.6).
- One parameter set per controller in Sections 4.2 to 4.5, which use **uncompensated** actuators; the compensated results are in Sections 2.5 and 3.3. There is no compensated baseline (`heuristic+comp` was not run).
- The rotate-threshold study (3.4) is not run. The R sweep covers two conditions (nominal and dropout) and has no significance tests.
- The plain-R×2/R×4 diagnosis rests on two traces; the seed-1 diagnosis is a hypothesis pending the adoption-tick check.
- The project's simulated Waffle Pi only. No hardware was used.

## Reproduce
From the repository root:
```sh
python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v      # design, actuator, controller, recovery and evaluation tests
python robotis_tb3/lqr/track_a_control_design.py                         # Section 1 numbers + step1_lqr_vs_heuristic.png
python robotis_tb3/track_a_project.py --headless --controller lqr --output out_lqr     # one closed-loop trial
python robotis_tb3/lqr/evaluate_controllers.py --controllers heuristic lqr --trials 10 --output eval_cmp   # Section 4 (80 trials)
python robotis_tb3/lqr/analyse_controllers.py eval_cmp/summary.json --out eval_cmp
python robotis_tb3/lqr/part_d_summary.py eval_cmp/summary.json                                        # the tables above
cd robotis_tb3
python identify_actuators.py --output evidence/part-d                    # Section 2.1
python verify_compensation.py --output evidence/part-d                   # Section 2.2
cd ..
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr lqr+comp lqr+compall \
    --conditions nominal camera_noise_dropout shifted_obstacle slippery_floor --trials 10 --output eval_comp   # Section 2.5 (120 trials)
python robotis_tb3/lqr/part_d_summary.py eval_comp/summary.json --comp
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25 lqr:0.5 lqr lqr:2 lqr:4 \
    --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep                                  # Section 3.3 (plain sweep)
python robotis_tb3/lqr/evaluate_controllers.py --controllers lqr:0.25+comp lqr:0.5+comp lqr:2+comp lqr:4+comp \
    --conditions nominal camera_noise_dropout --trials 10 --output eval_sweep_comp                              # Section 3.3 (compensated sweep)
python robotis_tb3/lqr/sweep_compare.py eval_sweep/summary.json eval_comp/summary.json eval_sweep_comp/summary.json --out figures
python robotis_tb3/lqr/plot_trace.py eval_comp/lqr_comp/shifted_obstacle/seed-1 eval_comp/lqr_comp/shifted_obstacle/seed-0 \
    eval_comp/lqr/shifted_obstacle/seed-1 -o seed1_diagnosis.png                                                  # Section 4.6
```