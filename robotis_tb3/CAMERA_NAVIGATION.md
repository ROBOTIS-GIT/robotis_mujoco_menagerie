# Simulated camera navigation

This demo adds a fixed forward-facing 640 × 480 RGB camera to the Waffle Pi,
renders an ArUco DICT_4X4_50 marker (ID 0), and uses OpenCV detection and
solvePnP to estimate the target's horizontal distance and bearing. A visual
servo controller commands wheel velocities using the existing TurtleBot model.
It approaches the marker and stops around 0.55 m from the camera (within 0.025 m).
The marker's black square is 0.22 m, with a white border on a 0.30 m board.
Camera focal length comes from its 60 degree vertical field of view. The
simulated pinhole camera has zero lens distortion.

## Run

From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-camera.txt
.venv/bin/mjpython robotis_tb3/camera_navigation.py
```

macOS needs `mjpython` for the passive viewer. Other platforms can use Python.
The MuJoCo window shows the robot, a live camera inset with detected corners,
and the controller status. Press Q or close the window to end the demo.
The robot stops on arrival; relaunch to repeat. Do not use the viewer's control
sliders during this demo: the controller sets wheel commands at 10 Hz.

```sh
.venv/bin/python robotis_tb3/camera_navigation.py --headless --output camera-navigation-output
.venv/bin/python -m unittest discover -s robotis_tb3 -p test_camera_navigation.py -v
```

Headless means no viewer, but it still needs graphics access to render camera
images. Headless runs exit unsuccessfully if arrival is not reached within the
time limit. `--marker-y -0.4` tests the opposite turn, `--marker-y 3` puts the
marker out of view, and `--seconds` sets the simulation time limit.

## Data flow and limits

Rendered RGB -> marker corners -> solvePnP -> distance/bearing -> wheel commands.
No ground-truth base position feeds the controller. A separate end-of-run
report uses MuJoCo geometry solely to evaluate camera distance accuracy.
Results, measurements, wheel commands, and start/finish frames are saved under
the selected output directory.

Missing markers, wrong marker IDs, or invalid pose fits cause zero wheel
commands. Large bearing errors cause turning without forward motion. Arrival
requires five consecutive in-tolerance frames and latches the stopped state.
A minimum approach command compensates for friction in the stock wheel model.

This is single-marker docking in an open simulated scene. It does not implement
obstacle avoidance, route planning, an EKF, real camera calibration, or hardware
control. The marker is a visual target with no collision geometry. Camera pose,
marker size, and friction assumptions need validation before hardware use.

References:
- https://docs.opencv.org/5.0/tutorials/objdetect/aruco_detection/aruco_detection.html
- https://mujoco.readthedocs.io/en/stable/python.html
