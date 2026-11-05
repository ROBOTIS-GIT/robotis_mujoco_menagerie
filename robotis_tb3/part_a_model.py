from turtle import stamp

import numpy as np
from roboticstoolbox import DiffSteer, Unicycle


def diff_drive_step(x, dphi_L, dphi_R, r, W):
    """Propagate pose x = [x, y, theta] by one odometry increment."""
    d    = 0.5 * r * (dphi_L + dphi_R)          # body displacement
    dpsi = r * (dphi_R - dphi_L) / W            # heading change
    return x + np.array([d * np.cos(x[2] + 0.5 * dpsi),
                         d * np.sin(x[2] + 0.5 * dpsi),
                         dpsi])
# From Week 2 tutorial solutions, code cell 6.
# This uses wheel turns to update x, y and heading.
# Copied exactly, no TurtleBot changes yet.

B_e = np.array([[0.0], [1.0]])
print("  v0      det(C)     rank(C)")
for v0 in [1.0, 0.5, 0.1, 0.01, 0.0]:
    A_e = np.array([[0.0, v0], [0.0, 0.0]])
    Co_e = ct.ctrb(A_e, B_e)
    print("%5.2f   %8.3f   %d" % (v0, np.linalg.det(Co_e), np.linalg.matrix_rank(Co_e)))

# From Week 5 state space tutorial solutions, code cell 19.
# This prints the controllability rank for a few forward speeds.
# Copied exactly. Not ad pted to tur TurtleBot yet.    

veh = Unicycle(W=1, dt=0.05)
print(veh)
print("deriv at x = (0, 0, pi/2), u = (1, 0):", veh.deriv([0, 0, np.pi/2], [1.0, 0.0]))

# From Week 5 state space tutorial solutions, code cell 18.
# This prints the motion derivative for one pose and input.
# Copied exactly. These are still tutorial values, not ours.

def unicycle_lin(v0):
    Au = np.array([[0.0, 0, 0], [0, 0, v0], [0, 0, 0]])
    Bu = np.array([[1.0, 0], [0, 0], [0, 1]])
    Cu = np.array([[1.0, 0, 0], [0, 1, 0]])
    return Au, Bu, Cu

for v0 in [1.0, 0.3, 0.0]:
    Au, Bu, Cu = unicycle_lin(v0)
    Ou = ct.obsv(Au, Cu)
    print(f"v0 = {v0:4.1f} m/s   rank O = {np.linalg.matrix_rank(Ou)}",
          "  <- observable" if np.linalg.matrix_rank(Ou) == 3 else "  <- heading INVISIBLE")

Au, Bu, Cu = unicycle_lin(1.0)
stamp("O at v0 = 1", ct.obsv(Au, Cu))

# From Week 7 observers and KF tutorial solutions, code cel 16.
# This checks if x and y readings reveal heading while the robot moves.
# Copied exactly. Its x and y sensor setup is not our camera model yet.

def wrap(a):
    '''Wrap angle(s) to (-pi, pi].'''
    return (a + np.pi) % (2 * np.pi) - np.pi

# From Week 7 observers and KF tutorial solutions, code cell 3.
# This keeps a heading angle between minus pi and pi.
# Copied exactly with no TurtleBot changes yet.

dt = 0.1

def f_uni(xh, u):
    '''Nonlinear motion model f(x, u). u = (v, omega).'''
    v, w = u
    x, y, th = xh
    if abs(w) < 1e-6:
        return np.array([x + v * dt * np.cos(th),
                         y + v * dt * np.sin(th),
                         wrap(th)])
    th_n = th + w * dt
    return np.array([x + v / w * (np.sin(th_n) - np.sin(th)),
                     y + v / w * (-np.cos(th_n) + np.cos(th)),
                     wrap(th_n)])

# From Week 8 LQG separation tutorial solutions, code cell 4.
# It moves x, y and heading using forward and turnig speed.
# Copied exactly. The step time is still the tutorial value.

def F_uni(xh, u):
    '''Jacobian F_t = df/dx, evaluated at the current estimate.'''
    v, w = u
    th = xh[2]
    F = np.eye(3)
    if abs(w) < 1e-6:
        F[0, 2] = -v * dt * np.sin(th)
        F[1, 2] =  v * dt * np.cos(th)
    else:
        th_n = th + w * dt
        F[0, 2] = v / w * (np.cos(th_n) - np.cos(th))
        F[1, 2] = v / w * (np.sin(th_n) - np.sin(th))
    return F

# From Week 8 LQG separation tutorial solutions, code cell 4.
# This is the state Jacobian for the motion model above.
# An EKF can use it to update uncertainty. Copied exactly.

# The robot we will use all tutorial: a small differential drive platform.
R_TRUE = 0.050      # wheel radius [m]
W_TRUE = 0.300      # track width [m]
DT     = 0.01       # sampling interval [s], so 100 Hz

# From Week 2 sensing tutorial solutions, code cell 2.
# These are the tutorial robot values, not TurtleBot values.
# Copied exactly. We will check the scene later.

class WheelEncoder:
    """Incremental quadrature encoder.

    Holds an accumulated tick count, like the real counter register.
    read() takes the true wheel rotation over one interval and returns the
    rotation implied by the change in tick count.
    """

    def __init__(self, cpr=1024, noise_ticks=0.5, seed=0):
        self.cpr = cpr                      # counts per revolution (after quadrature)
        self.noise_ticks = noise_ticks      # sd of read noise, in ticks
        self.rng = np.random.default_rng(seed)
        self.phi = 0.0                      # true accumulated wheel angle [rad]
        self.count = 0                      # accumulated tick count

    @property
    def resolution(self):
        """Smallest resolvable change in wheel angle, Delta [rad]."""
        return 2 * np.pi / self.cpr

    def read(self, dphi_true):
        self.phi += dphi_true
        total = int(round(self.phi / self.resolution
                          + self.noise_ticks * self.rng.standard_normal()))
        dticks = total - self.count
        self.count = total
        return dticks * self.resolution


enc = WheelEncoder(cpr=1024, noise_ticks=0.0)
print(f"Delta = {np.degrees(enc.resolution):.4f} deg per count")
print(f"      = {R_TRUE*enc.resolution*1000:.3f} mm of wheel travel per count")

# From Week 2 sensing tutorial solutions, code cell 3.
# The class turns wheel rotation into noisy encoder counts and back.
# Copied exactly. The printout still uses the tutorial wheel radius.

def run(u, r_odo=R_TRUE, W_odo=W_TRUE, cpr=1024, noise_ticks=0.5,
        slip_sd=0.0, diff_slip=True, seed=0):
    """Drive the robot along the command sequence u = [(v, omega), ...].

    Returns (ground_truth, odometry), each (n+1, 3).
    slip_sd is the sd of the wheel slip ratio, the fraction of wheel rotation
    that does not turn into ground motion. The encoders never see it.
    """
    rng = np.random.default_rng(seed)
    encL = WheelEncoder(cpr, noise_ticks, seed + 1000)
    encR = WheelEncoder(cpr, noise_ticks, seed + 2000)
    veh  = DiffSteer(W=W_TRUE, dt=DT, x0=[0, 0, 0], animation=None)

    x = np.zeros(3)
    odo = [x.copy()]
    for v, w in u:
        # what the wheels really turn
        dphi_L = (v - w * W_TRUE / 2) * DT / R_TRUE
        dphi_R = (v + w * W_TRUE / 2) * DT / R_TRUE

        # what reaches the ground, after slip
        if diff_slip:
            sL, sR = rng.normal(0.0, slip_sd, 2)
        else:
            sL = sR = rng.normal(0.0, slip_sd)
        aL, aR = dphi_L * (1 - sL), dphi_R * (1 - sR)

        # DiffSteer takes the left and right wheel speeds in m/s
        veh.step((R_TRUE * aL / DT, R_TRUE * aR / DT), animate=False)

        # what the encoders report, and what odometry makes of it
        x = diff_drive_step(x, encL.read(dphi_L), encR.read(dphi_R), r_odo, W_odo)
        odo.append(x.copy())

    gt = np.vstack([np.zeros(3), np.array(veh.x_hist)])
    return gt, np.array(odo)

# From Week 2 sensing tutorial solutions, code cell 6.
# It compares simulated motion with pose built from encoder readings.
# Slip makes them disagree. Copied exactly for modelling tests.

def square(side=3.0, v=0.5, w=0.5, sense=+1):
    """Command sequence for a square of the given side. sense = +1 CCW, -1 CW."""
    ns, nt = int(round(side / v / DT)), int(round((np.pi / 2) / w / DT))
    return np.array([c for _ in range(4)
                     for c in [(v, 0.0)] * ns + [(0.0, sense * w)] * nt])


def straight(dist, v=0.5):
    return np.array([(v, 0.0)] * int(round(dist / v / DT)))

# From Week 2 sensing tutorial solutions, code cell 6.
# These make straight and square input sequences to test the model.
# Copied exactly. The speeds and nistances are tutorial values.

def h_rb(xh, mk):
    '''Range and bearing from pose xh to marker mk.'''
    dx, dy = mk[0] - xh[0], mk[1] - xh[1]
    return np.array([np.hypot(dx, dy), wrap(np.arctan2(dy, dx) - xh[2])])


def H_rb(xh, mk):
    '''Jacobian H_t = dh/dx, evaluated at the predicted estimate.'''
    dx, dy = mk[0] - xh[0], mk[1] - xh[1]
    q = dx**2 + dy**2
    r = np.sqrt(q)
    return np.array([[-dx / r, -dy / r,  0.0],
                     [ dy / q, -dx / q, -1.0]])

# From Week 7 observers and KF tutorial solutions, code cell 32.
# Thesh give marker range and bearing, then their stase Jacobian.
# Copied exactly. Our camera may use a different measurement.