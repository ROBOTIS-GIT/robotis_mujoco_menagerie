# Controller evaluation

80 trials, 4 conditions, controllers: heuristic, lqr.

Mean ± sd over seeds. Success rate (all trials) shows the 95% Wilson interval. Tracking metrics use SUCCESSFUL trials only (n ok shown): a trial that stalls early has a tiny tracking error that says nothing about the controller. Failures are listed below.

## camera_noise_dropout

| controller | success | 95% CI | n ok | XTE RMS truth [cm] | XTE max truth [cm] | XTE RMS driving [cm] | est-truth XTE [cm] | effort w^2 driving | rotate [s] | time [s] | ref adoptions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heuristic | 10/10 | 0.72–1.00 | 10 | 15.11 ± 2.15 | 25.70 ± 4.14 | 15.13 ± 2.17 | 9.40 ± 0.94 | 3.99 ± 0.29 | 0.92 ± 0.36 | 50.52 ± 0.82 | 9.00 ± 2.62 |
| lqr | 10/10 | 0.72–1.00 | 10 | 8.35 ± 2.05 | 17.12 ± 2.34 | 8.20 ± 1.82 | 5.93 ± 1.50 | 0.58 ± 0.21 | 10.00 ± 1.59 | 56.64 ± 2.49 | 9.60 ± 1.58 |

- LQR − heuristic, XTE RMS truth [cm]: median -7.52, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, XTE max truth [cm]: median -9.24, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, XTE RMS driving [cm]: median -7.77, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, effort w^2 driving: median -3.39, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002

## nominal

| controller | success | 95% CI | n ok | XTE RMS truth [cm] | XTE max truth [cm] | XTE RMS driving [cm] | est-truth XTE [cm] | effort w^2 driving | rotate [s] | time [s] | ref adoptions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heuristic | 10/10 | 0.72–1.00 | 10 | 13.64 ± 2.10 | 24.92 ± 2.18 | 13.60 ± 2.14 | 7.68 ± 0.36 | 3.87 ± 0.16 | 0.57 ± 0.15 | 44.88 ± 0.23 | 7.50 ± 0.97 |
| lqr | 10/10 | 0.72–1.00 | 10 | 6.29 ± 1.28 | 15.23 ± 1.17 | 6.56 ± 0.99 | 4.31 ± 0.94 | 0.55 ± 0.08 | 11.40 ± 2.32 | 52.57 ± 2.34 | 9.20 ± 0.92 |

- LQR − heuristic, XTE RMS truth [cm]: median -8.43, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, XTE max truth [cm]: median -10.85, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, XTE RMS driving [cm]: median -8.06, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, effort w^2 driving: median -3.27, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002

## shifted_obstacle

| controller | success | 95% CI | n ok | XTE RMS truth [cm] | XTE max truth [cm] | XTE RMS driving [cm] | est-truth XTE [cm] | effort w^2 driving | rotate [s] | time [s] | ref adoptions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heuristic | 5/10 | 0.24–0.76 | 5 | 15.45 ± 2.14 | 29.94 ± 4.43 | 15.84 ± 2.17 | 10.98 ± 0.55 | 4.26 ± 0.37 | 1.66 ± 0.18 | 53.24 ± 8.53 | 9.00 ± 2.55 |
| lqr | 10/10 | 0.72–1.00 | 10 | 12.94 ± 1.10 | 21.65 ± 1.13 | 12.45 ± 0.82 | 9.40 ± 0.44 | 1.41 ± 0.38 | 13.00 ± 2.51 | 58.52 ± 2.71 | 13.40 ± 1.43 |

- LQR − heuristic, XTE RMS truth [cm]: median -2.25, LQR lower in 5/5 paired seeds, too few pairs for a test
- LQR − heuristic, XTE max truth [cm]: median -7.81, LQR lower in 5/5 paired seeds, too few pairs for a test
- LQR − heuristic, XTE RMS driving [cm]: median -2.91, LQR lower in 5/5 paired seeds, too few pairs for a test
- LQR − heuristic, effort w^2 driving: median -2.91, LQR lower in 5/5 paired seeds, too few pairs for a test

## slippery_floor

| controller | success | 95% CI | n ok | XTE RMS truth [cm] | XTE max truth [cm] | XTE RMS driving [cm] | est-truth XTE [cm] | effort w^2 driving | rotate [s] | time [s] | ref adoptions |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heuristic | 10/10 | 0.72–1.00 | 10 | 10.87 ± 1.92 | 19.61 ± 3.27 | 10.86 ± 1.92 | 7.89 ± 0.33 | 3.71 ± 0.09 | 0.14 ± 0.28 | 44.28 ± 0.37 | 7.40 ± 1.71 |
| lqr | 10/10 | 0.72–1.00 | 10 | 8.19 ± 0.97 | 16.96 ± 2.10 | 8.12 ± 0.84 | 5.95 ± 0.82 | 0.78 ± 0.39 | 10.81 ± 1.33 | 53.05 ± 2.56 | 10.00 ± 1.33 |

- LQR − heuristic, XTE RMS truth [cm]: median -2.23, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, XTE max truth [cm]: median -2.38, LQR lower in 9/10 paired seeds, Wilcoxon p=0.037
- LQR − heuristic, XTE RMS driving [cm]: median -2.20, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002
- LQR − heuristic, effort w^2 driving: median -3.04, LQR lower in 10/10 paired seeds, Wilcoxon p=0.002

## Failures

- heuristic / shifted_obstacle / seed 0: NO SAFE PATH - STOP (collision=False, final lateral 0.245 m)
- heuristic / shifted_obstacle / seed 2: NO SAFE PATH - STOP (collision=False, final lateral 0.247 m)
- heuristic / shifted_obstacle / seed 4: NO SAFE PATH - STOP (collision=False, final lateral 0.245 m)
- heuristic / shifted_obstacle / seed 6: NO SAFE PATH - STOP (collision=False, final lateral 0.247 m)
- heuristic / shifted_obstacle / seed 7: NO SAFE PATH - STOP (collision=False, final lateral 0.243 m)
