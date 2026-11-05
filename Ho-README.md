# TurtleBot3 visual waypoint navigation - review draft

Track A simulation: LiDAR obstacle navigation, encoder/camera EKF and estimated-state feedback.
This branch contains a tested development pipeline and project-page drafts for team review.

- [Project page and run instructions](robotis_tb3/TRACK_A_PROJECT.md)
- [Part C: estimation equations, tuning and results](robotis_tb3/PART_C_ESTIMATION.md)
- [Viva practice notes](robotis_tb3/PART_C_VIVA_NOTES.txt)
- [Evidence and reproduction instructions](robotis_tb3/evidence/README.md)

The baseline completed **8/8 simulation trials** with no recorded collisions.
The Part C study compared **nine Q/R settings on eight fixed sensor streams**;
these are 72 estimator replays, not navigation trials. A conservative candidate
also completed two separate fresh navigation checks. **17 regression tests pass.**
Default tuning remains Q x1,R x1; the candidate is selected with `--q-scale 10`.

Team/group details, actual human contribution statements, calibration validation,
and the final five-minute video link still need team completion/review. Codex
assistance with implementation, experiments, tests and drafting is declared in
the project notes. This is not a claimed finished assessment submission.

## Inherited ROBOTIS model collection

# robotis_mujoco_menagerie
A collection of models for the MuJoCo physics engine from ROBOTIS

## AI Worker (FFW-SH5)
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_ffw/ffw_sh5.png" width="300">

## AI Worker (FFW-SG2)
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_ffw/ffw_sg2.png" width="300">

## AI Worker (FFW-BG2)
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_ffw/ffw_bg2.png" width="300">

## OpenMANIPULATOR-Y
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_omy/omy.png" width="300">

## OMX
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_omx/omx.png" width="300">

## OpenMANIPULATOR-X
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_open_manipulator_x/open_manipulator_x.png" width="300">

## OP3
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_op3/op3.png" width="300">

## TurtleBot3 Burger
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_tb3/tb3_burger.png" width="300">

## TurtleBot3 Waffle Pi
<img src="https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_mujoco_menagerie/main/robotis_tb3/tb3_waffle_pi.png" width="300">

# Applications
The models in this repository are designed for use with the MuJoCo physics engine. They can be utilized in various applications, including but not limited to:

- [LeRobot Tutorial with MuJoCo](https://github.com/jeongeun980906/lerobot-mujoco-tutorial)
: Collecting demonstration data and training (or fine-tuning) vision language action models on custom datasets.

  <img src="./media/RILAB-OMY-Tutorial.gif" width="600">


# Acknowledgments

This repository and its associated codebase were made possible through the dedicated efforts of the developers at [ROBOTIS](https://en.robotis.com/). The asset files for OP3 were developed by ROBOTIS, and we gratefully acknowledge the referenced XML model files utilized in the publicly available [OP3 model](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotis_op3).
Additionally, we extend our sincere appreciation to Professor Sungjoon Choi, Researcher Jihwan Yoon and Jeongeun Park from the [Robot Intelligence Lab](https://sites.google.com/view/sungjoon-choi/home) at Korea University for their valuable contributions and insightful collaboration.

# License
XML and asset files contained within each specific model directory of this repository are governed by their own distinct licensing conditions. Please refer to the LICENSE documentation located within each respective model's directory for comprehensive license terms and associated copyright details.

All other materials are the intellectual property of ROBOTIS CO., LTD., © 2025, and are made available under the Apache License, Version 2.0. The full text of this license is included in the main LICENSE file at the repository's root. Additionally, it is accessible online at https://www.apache.org/licenses/LICENSE-2.0.
