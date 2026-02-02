^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for robotis_mujoco_menagerie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.1.0 (2026-02-02)
------------------
* Added FFW-SH5 model (FFW-S base with HX5-D20 5-finger hand)
* Reorganized asset folder structure for clarity (ffw_sg2 → ffw_s, ffw_bg2 → ffw_b)
* Fixed head joint movement by adding contact excludes between head_link1, head_link2, and arm_base_link
* Fixed gripper joint movement by adding contact excludes between arm_link7 and gripper fingertip parts
* Fixed gripper mimic joint configuration by adding gripper_mimic_pos class for positive range joints
* Added damping to gripper joints for improved stability
* Contributors: Taehyeong Kim

1.0.0 (2026-01-16)
------------------
* Tuned simulation parameters for all robot models (FFW-BG2, FFW-SG2, OMY, OMX, TurtleBot3)
* Fixed gripper mimic joint configuration to match OMY gripper behavior
* Corrected body inertial values for realistic physics simulation
* Updated material colors to black for consistent appearance
* Contributors: Taehyeong Kim

0.4.0 (2025-12-12)
------------------
* Added Model files for FFW-SG2
* Fixed joint limit bug in the FFW-BG2 arm model.
* Replaced the FFW model base collision geometry with a simplified cube for more stable collision behavior.
* Contributors: Taehyeong Kim, jinw00-1, gahyun0425

0.3.0 (2025-10-17)
------------------
* Added Model files for OMX
* Contributors: Pyo, Taehyeong Kim

0.2.1 (2025-09-19)
------------------
* Changed the gripper part of omy to black
* Renamed robot identifier from omx to open_manipulator_x.
* Contributors: Pyo, Taehyeong Kim

0.2.0 (2025-06-02)
------------------
* Added Model files for FFW-BG2
* Removed Model files for FFW-1
* Contributors: Pyo, Woojin Wie

0.1.0 (2025-03-21)
------------------
* Added Model files for FFW-1, OMY, OMX, OP3, TurtleBot3 Burger and Waffle Pi
* Contributors: Pyo, Woojin Wie, Sungjoon Choi
