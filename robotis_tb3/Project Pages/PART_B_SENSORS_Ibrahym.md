# Part B TurtleBot3 Sensors ``
## Lidar
### Values justification
`sigma_rho=0.02` chosen to simulate small noise variation.

from https://docs.robotis.com/docs/systems/turtlebot3/more_info/lds_01/ :

`MAX_RANGE = 3.5` , `MIN_RANGE = 0.012` determined by Distance Range. `angles` determined by Angular Range (360 degrees) and Angular Resolution (1 degree), 

### Assumptions
- Measurements of lidar pose is accurate and is rigidly mounted to the robot.
- No reflective surfaces are present in the scene.

### Equations
`direction` uses rotation matrix multiplication $(d1 = R12 d1)$ to transform the lidar ray direction from the sensor's coordinate frame into the world coordinate frame.

### Canvas material
 **Week 2 Part C** - `h_lidar` was used as a reference for setting up `observe`. Return type and operations needed to be modified to suit the mujoco environment.

 **Week 2 Part C.1** - `animate_lidar` was used as a template for simulating lidar readings for all the angles. code had to be modified to use `mujoco.mj_ray` for the simulation.

## Camera
### Values justification
`MARKER_SIZE = 0.22` chosen for the simulation to be readable and realistic. 

`CAMERA_TILT_DEGREES = 10` chosen for the simulation to better capture the marker position.

From https://www.raspberrypi.com/documentation/accessories/camera.html :

`WIDTH=640`, `HEIGHT=480`, `FOV=60` determined by  Camera module 2 640 × 480p206.

### Assumptions
- We assume only one ArUco marker is present at the scene, CalibratedMarkerCamera takes the first ArUco marker detected.
- Measurements of Camera pose and Marker size is accurate.
- No reflective surfaces are present in the scene.
- Camera is rigidly mounted to the robot.
- Scene lighting is sufficient for the camera to detect the marker.

### Equations
Focal length `focal` was calculated using $f = HEIGHT / (2 tan(FOV/2))$ to determine how 3D points are projected onto the 2D image plane.

Principal point `pp` was found by using the equation $(WIDTH - 1) / 2$ for horizontal pixel coordinate (u) and $(HEIGHT - 1) / 2$ for vertical pixel coordinate (v).

`distance` uses Euclidean distance formula $d= \sqrt{x^2+h^2}$ to estimate the distance from the marker to the camera.

`heading` uses the inverse tangent function $tan{^-1}(x / h)$ to estimate the angle of the marker relative to the camera's forward direction.

`robot_yaw` uses the inverse tangent function $tan{^-1}(x / h)$ to estimate the ange of the  robot relative to the world coordinate frame derived from the rotation matrix `Rwr`.

### Canvas material
**Week 3 Setup B2** - Refrenced for principal point calculations.

 **Week 3 Part E** - `render_view` used as refrence for camera calibration and arUco marker setup.

 **Week 3 Part E1.3** - `track_marker` refrenced for arUco detection and camera pose setup.