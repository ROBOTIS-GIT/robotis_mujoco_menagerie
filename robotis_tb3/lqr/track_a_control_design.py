"""Part D, Step 1: path-tracking error model, controllability and LQR design.

No simulator imports. Pure numpy/scipy so every number can be reproduced and
defended in the viva.

Frames and signs (match the rest of the project)
------------------------------------------------
World x forward along the path, y left, theta anticlockwise from world +x.
For a locally straight path segment with tangent heading psi_p:

    e_y     = signed lateral offset of the robot from the path, left positive [m]
    e_theta = wrap(theta - psi_p)                                       [rad]

Unicycle kinematics in these error coordinates (speed v, yaw rate w):

    e_y_dot     = v sin(e_theta)
    e_theta_dot = w                    (path curvature kappa = 0 on a segment)

Linearising about e = 0 (sin e ~ e):

    x = [e_y, e_theta]^T,   x_dot = A x + B w,
    A = [[0, v],            B = [[0],
         [0, 0]]                 [1]]

Controllability matrix [B, AB] = [[0, v], [1, 0]] has rank 2 iff v != 0.
This is the same rank drop at zero speed found in Part A, now for the
error model: while stationary, the robot cannot correct lateral error by
turning alone. The planner/docking logic handles this by rotating in place
first, then driving.
"""
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are

DT = 0.1            # control period [s] (10 Hz loop in track_a_project.py)
W_MAX = 0.8         # yaw-rate saturation used by the tracker [rad/s]
V_NOM = 0.18        # nominal path speed used by the tracker [m/s]
LOOKAHEAD = 0.20    # lookahead distance of the current tracker [m]
K_HEADING = 2.8     # current heading gain [1/s]


def error_model(v):
    """Continuous linearised path-error model (A, B) at forward speed v."""
    A = np.array([[0.0, v], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    return A, B


def discretise(v, dt=DT):
    """Exact zero-order-hold discretisation (A is nilpotent, so it is closed form)."""
    Ad = np.array([[1.0, v * dt], [0.0, 1.0]])
    Bd = np.array([[0.5 * v * dt**2], [dt]])
    return Ad, Bd


def controllability_rank(v):
    A, B = error_model(v)
    C = np.hstack([B, A @ B])
    return int(np.linalg.matrix_rank(C, tol=1e-12))


def bryson_weights(e_y_max=0.10, e_theta_max=0.35, w_max=W_MAX):
    """Bryson's rule: weight = 1/(max acceptable value)^2.

    e_y_max      0.10 m  -> half the 0.20 m grid-path inflation margin
    e_theta_max  0.35 rad (~20 deg) -> just under the tracker's 0.4 rad
                  'rotate without advancing' threshold
    w_max        0.80 rad/s -> the existing yaw saturation
    """
    Q = np.diag([1.0 / e_y_max**2, 1.0 / e_theta_max**2])
    R = np.array([[1.0 / w_max**2]])
    return Q, R


def lqr_discrete(v, Q, R, dt=DT):
    """Discrete LQR gain K (1x2) for u = -K x at speed v."""
    Ad, Bd = discretise(v, dt)
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K


def lqr_continuous(v, Q, R):
    A, B = error_model(v)
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def heuristic_equivalent_gain(k_heading=K_HEADING, lookahead=LOOKAHEAD):
    """Small-error gain of the CURRENT tracker, in the same coordinates.

    Current law: w = k_h * bearing, where bearing is the angle from the robot's
    heading to a point `lookahead` metres along the path. For small errors on
    a straight path, target = (L, -e_y) relative to the robot, so

        bearing ~ atan2(-e_y, L) - e_theta ~ -e_y / L - e_theta
        w = -(k_h / L) e_y - k_h e_theta        -> K = [k_h/L, k_h]
    """
    return np.array([[k_heading / lookahead, k_heading]])


def closed_loop_poles(v, K, dt=None):
    """Closed-loop eigenvalues. Continuous if dt is None, else discrete (z-plane)."""
    if dt is None:
        A, B = error_model(v)
        return np.linalg.eigvals(A - B @ K)
    Ad, Bd = discretise(v, dt)
    return np.linalg.eigvals(Ad - Bd @ K)


def damping_and_wn(poles):
    """Natural frequency and damping ratio of the dominant complex pair."""
    p = poles[np.argmax(np.abs(poles.imag))]
    wn = abs(p)
    zeta = -p.real / wn if wn > 0 else np.nan
    return wn, zeta


def simulate_nonlinear(K, v=V_NOM, e_y0=0.20, e_theta0=0.0, t_end=15.0,
                       dt=DT, w_max=W_MAX, substeps=10):
    """Closed loop with the NONLINEAR error kinematics and yaw saturation.

    The controller runs at `dt` (zero-order hold); the plant is integrated at
    dt/substeps. Returns time, e_y, e_theta, w arrays.
    """
    n = int(round(t_end / dt))
    x = np.array([e_y0, e_theta0])
    T, EY, ET, W = [], [], [], []
    h = dt / substeps
    for k in range(n):
        w = float(np.clip(-(K @ x).item(), -w_max, w_max))
        T.append(k * dt); EY.append(x[0]); ET.append(x[1]); W.append(w)
        for _ in range(substeps):
            x = x + h * np.array([v * np.sin(x[1]), w])
    return np.array(T), np.array(EY), np.array(ET), np.array(W)


def step_metrics(T, EY, W, band=0.02):
    """Overshoot (fraction of the initial offset), settling time, control effort."""
    e0 = EY[0]
    overshoot = max(0.0, -np.min(np.sign(e0) * EY)) / abs(e0)
    outside = np.where(np.abs(EY) > band)[0]
    settle = T[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(T) else (
        0.0 if not len(outside) else np.inf)
    effort = float(np.sum(W**2) * (T[1] - T[0]))      # integral of w^2 dt
    return dict(overshoot=overshoot, settle_s=settle, effort_w2=effort,
                peak_w=float(np.max(np.abs(W))))


def augmented_discrete(v, tau, g, dt=DT):
    """Error model plus a first-order yaw actuator, exact ZOH discretisation.

    x = [e_y, e_theta, w_act],  w_act_dot = (g*w_cmd - w_act)/tau
    g   = steady-state yaw gain of the real plant (1.0 = ideal)
    tau = actuator lag [s]
    """
    from scipy.linalg import expm
    A = np.array([[0, v, 0], [0, 0, 1], [0, 0, -1.0 / tau]])
    B = np.array([[0.0], [0.0], [g / tau]])
    M = np.zeros((4, 4)); M[:3, :3] = A * dt; M[:3, 3:] = B * dt
    E = expm(M)
    return E[:3, :3], E[:3, 3:]


def closed_loop_robustness(K2, v, tau, g, dt=DT):
    """Spectral radius and slowest-oscillation damping of the REAL loop when the
    2-state gain K2 = [K_ey, K_etheta] is applied to a lagged, mis-scaled plant."""
    Ad, Bd = augmented_discrete(v, tau, g, dt)
    K3 = np.hstack([np.atleast_2d(K2), [[0.0]]])
    z = np.linalg.eigvals(Ad - Bd @ K3)
    s = np.log(z.astype(complex)) / dt
    wn = np.abs(s)
    zeta = np.where(wn > 1e-9, -s.real / wn, 1.0)
    return float(np.max(np.abs(z))), float(np.min(zeta))


def max_stable_yaw_gain(K2, v, tau, dt=DT, g_hi=20.0):
    """Largest plant yaw gain g (>=1) that keeps the loop stable (gain margin)."""
    lo, hi = 1.0, g_hi
    if closed_loop_robustness(K2, v, tau, hi, dt)[0] < 1:
        return np.inf
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if closed_loop_robustness(K2, v, tau, mid, dt)[0] < 1:
            lo = mid
        else:
            hi = mid
    return lo


def robustness_report(tau=0.11):
    Q, R = bryson_weights()
    K = lqr_discrete(V_NOM, Q, R)[0]
    print("\n=== Robustness of the approved LQR gain K=%s ===" % np.round(K, 3))
    print("Plant = lagged yaw actuator (tau=%.2f s) with yaw gain g and speed v" % tau)
    gs = (0.3, 0.5, 0.75, 1.0, 1.5, 2.0)
    print("rho = spectral radius (<1 stable); zeta = worst damping ratio")
    print("  v[m/s] |" + "".join(f"   g={g:<4}        " for g in gs))
    for v in (0.03, 0.06, 0.096, 0.18):
        row = f"  {v:5.3f}  |"
        for g in gs:
            rho, z = closed_loop_robustness(K, v, tau, g)
            row += f" rho={rho:.3f} z={z:4.2f} "
        print(row)
    print("Gain margin on yaw gain (max g before instability), K fixed:")
    for v in (0.06, 0.096, 0.18):
        print(f"  v={v:5.3f}: g_max = {max_stable_yaw_gain(K, v, tau):.2f}")
    print("Same, with an ideal actuator (tau -> 0.01 s):")
    print(f"  v=0.180: g_max = {max_stable_yaw_gain(K, 0.18, 0.01):.2f}")


def main(make_plot=True):
    print("=== Controllability of the path-error model ===")
    for v in (0.0, 0.05, 0.18):
        print(f"v = {v:4.2f} m/s -> rank [B, AB] = {controllability_rank(v)} (need 2)")

    Q, R = bryson_weights()
    print("\nBryson weights: Q = diag(%.1f, %.1f), R = %.2f" % (Q[0, 0], Q[1, 1], R[0, 0]))

    K_h = heuristic_equivalent_gain()
    K_lqr = lqr_discrete(V_NOM, Q, R)
    K_lqr_c = lqr_continuous(V_NOM, Q, R)

    print("\n=== Gains at v = %.2f m/s, u = -K [e_y, e_theta] ===" % V_NOM)
    print("current heuristic (equiv.) K =", np.round(K_h, 3))
    print("LQR discrete (dt=0.1 s)    K =", np.round(K_lqr, 3))
    print("LQR continuous             K =", np.round(K_lqr_c, 3))

    print("\n=== Closed-loop poles (continuous) ===")
    for name, K in (("heuristic", K_h), ("LQR", K_lqr_c)):
        p = closed_loop_poles(V_NOM, K)
        wn, z = damping_and_wn(p)
        print(f"{name:10s} poles {np.round(p, 3)}  wn={wn:.2f} rad/s  zeta={z:.2f}")
    print("\n=== Closed-loop poles (discrete, |z|<1 is stable) ===")
    for name, K in (("heuristic", K_h), ("LQR", K_lqr)):
        z = closed_loop_poles(V_NOM, K, DT)
        print(f"{name:10s} z {np.round(z, 3)}  |z|max={np.max(np.abs(z)):.3f}")

    print("\n=== Nonlinear step: 0.20 m lateral offset, saturated w ===")
    results = {}
    for name, K in (("heuristic", K_h), ("LQR", K_lqr)):
        T, EY, ET, W = simulate_nonlinear(K)
        results[name] = (T, EY, ET, W)
        m = step_metrics(T, EY, W)
        print(f"{name:10s} overshoot={100*m['overshoot']:.0f}%  settle(2cm)={m['settle_s']:.1f}s  "
              f"int w^2={m['effort_w2']:.3f}  peak|w|={m['peak_w']:.2f}")

    print("\n=== Gain vs speed (LQR is speed-dependent) ===")
    for v in (0.06, 0.10, 0.14, 0.18):
        print(f"v={v:4.2f}  K={np.round(lqr_discrete(v, Q, R), 3)}")

    robustness_report()

    if make_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
        for name, (T, EY, ET, W) in results.items():
            ax[0].plot(T, 100 * EY, label=name)
            ax[1].plot(T, np.degrees(ET), label=name)
            ax[2].plot(T, W, label=name)
        ax[0].set(xlabel="t [s]", ylabel="lateral error e_y [cm]", title="Lateral error")
        ax[1].set(xlabel="t [s]", ylabel="heading error [deg]", title="Heading error")
        ax[2].set(xlabel="t [s]", ylabel="yaw rate cmd [rad/s]", title="Control effort")
        for a in ax:
            a.grid(alpha=.3); a.legend()
        fig.tight_layout()
        fig.savefig("step1_lqr_vs_heuristic.png", dpi=150)
        print("\nsaved step1_lqr_vs_heuristic.png")


if __name__ == "__main__":
    main()
