# Part A modelling notes

## Week 2 wheel odometry

I copied the diff_drive_step function straight from the Week 2 tutorial solutions notebook, code cell 6. It uses the left and right wheel turns to estmate x, y and heading. I haven't changed it for our TurtleBot yet.

## Week 5 controllability check

This is code cell 19 from 41014_Week5_State_Space_Tutorial_SOLUTIONS.ipynb. It makes two small matrices, A and B. The code checks the controllability rank at different forward speeds. At zero speed the rank drops. I haven't adtpted it to our TurtleBot yet.

## Week 5 unicycle example

This came from code cell 18 in the Week 5 state space tutorial solutions. It makes a toolbox Unicycle with W set to 1 and a 0.05 second step, then prints one motion derivative. Those are still tu orial values, not ours.

## Week 7 observability example

This is code cell 16 from 41014_Week7_Observers_and_KF_Tutorial_SOLUTIONS.ipynb. It sets up a small x, y and heading model, then checks observability as forward speed changes. The tutorial measures x and y only. We need to replace that sensor model with our camera measurements before claiming this result for the TurtleBot.

## Week 7 angle wrapping

This is code cell 3 from the Week 7 observers and KF solutions. It keeps heading angles between minus pi and pi. 

## Week 8 motion model

This is code cell 4 from the Week 8 LQG solutions. It predicts the next x, y and heading from speed and turn rate. The 0.1 second step is copied from the tutorial.

## Week 8 state Jacobian

This is also from code cell 4 in the Week 8 LQG solutions. It describes how the predicted pose changes when the starting pose changes. An EKF can use it for uncertainty.

## Week 2 tutorial parameters

Code cell 2 uses a 0.05 m wheel radius, 0.30 m wheel spacing and 0.01 s step. These are example values. We need to read the TurtleBot scene before using them.

## Week 2 wheel encoder

Code cell 3 counts wheel turns in steps called ticks, then converts the count back to an angle. It also adds optional read noise. This is still the tutorial sensor model.

## Week 2 odometry and slip

Code cell 6 drives a toolbox robot and builds a second pose from encoder readings. Wheel slip makes the two paths differ. The first path is just for comparison. This is not our MuJoCo run.

## Week 2 test paths

Code cell 6 also makes straight and square speed commands. We can use simple paths like thesh to check whether our model follows the expected movement. The numbers are still tutorial examples.

## Week 7 marker measurement model

This one is from code cell 32 in the Week 7 observer tutorial. It works out a marker's range and bearing from a pose.

The other function is the Jacobian. Basically, it shows how the reading shifts if the pose shifts a bit. Both are copied exactly.

We can decide later whether our camera gives these measurements or a full pose.

## Before we adapt this

The raw pieces are here now. Week 2 gives us wheel inputs and encoder odometry. Weeks 5 and 7 have the rank checks, and Week 8 has the pose update and Jacobian.


