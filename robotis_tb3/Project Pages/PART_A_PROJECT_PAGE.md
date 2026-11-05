# Part A TurtleBot3 model

## State and wheel inputs

We use a flat floor pose s = [x, y, theta]. x and y are positions in metres in the MuJoCo world frame. theta is the heading in radians, measured anticlockwise from world +x.

We choose robot +x as forward and robot +y as left. The input u = [w_left, w_right] is the actual wheel rates in rad/s. For odometry we use changes in the wheel joint angles, not the command values. This matters if the actuators lag or the wheels slip.

## Values from the scene

The loaded Waffle Pi model gives a wheel radius r = 0.0330 m from the tire mesh and wheel centre spacing b = 0.288 m. Its physics step is 0.002 s. The wheel rate command limit is 7.88 rad/s in either direction.

We choose ten physics steps per model update. That gives dt = 0.020 s, or 50 Hz. This update rate is our choice, not a scene value.

## Motion equations

Wheel radius r and wheel spacing b turn the two wheel rates into forward speed and yaw rate.

v = r (w_left + w_right) / 2, and yaw_rate = r (w_right - w_left) / b.

The state rates are x_dot = v cos(theta), y_dot = v sin(theta), and theta_dot = yaw_rate.

From encoder angle changes d_left and d_right, distance = r (d_left + d_right) / 2 and dtheta = r (d_right - d_left) / b. We integrate that short arc each update.

## Assumptions

The model assumes a flat floor, equal wheel radii and negligible slip. If wheel commands are used as inputs, we assume the actuators track them quickly. The encoder update uses measured angles instead. Slip error still needs validation.

## Controllability and observability

Linearising at heading theta and forward speed v gives A[0,2] = -v sin(theta) and A[1,2] = v cos(theta). All other entries of A are zero.

The two columns of B share the x and y terms r cos(theta)/2 and r sin(theta)/2. Their heading terms are -r/b for the left wheel and +r/b for the right wheel.

With both wheels at 5 rad/s, the controllability matrix [B, AB, A^2B] has rank 3. At zero forward speed the local linearised rank is 2. The rank drop is only about this small-signal model at rest. The robot can still turn, then drive to a different position.

We choose a full base pose [x, y, theta] from a known ArUco marker. When it is visible and the camera reading has been converted to world coordinates, C = I and observability rank is 3. Out of view there is no camera pose update. Wheel odometry keeps predicting motion, but absolute pose can drift. If sensing provides only range and bearing, C and its rank check must be redone.

## Sources used

The wheel equations build on the Week 2 wheel odometry tutorial. The pose update and state Jacobian follow the Week 8 motion example.  Robot dimentions and actuator limits are read from the ROBOTIS Waffle Pi MuJoCo scene.