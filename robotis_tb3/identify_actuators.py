"""Part D, Step 2a: identify the wheel actuator dynamics the controller sits on.

Bench test only (open loop, no estimator, no controller). Ground-truth base
velocity is read here purely to characterise the plant, like a lab step test.

Measures, on the same ROBOTIS Waffle Pi model and floor used by the project:
  1. first-order lag time constant tau and dead time of each wheel,
  2. steady-state wheel-rate shortfall (friction offset) vs command level,
  3. forward-speed shortfall at the tracker's cruise command,
  4. smallest in-place yaw command that actually turns the robot.

Run:  python3 identify_actuators.py [--friction 1.0] [--output actuator_id]
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import curve_fit

from turtlebot_model import load_scene

HERE = Path(__file__).parent


def build_bench_model(friction=1.0):
    """Waffle Pi on the same floor definition used by build_room (no obstacles)."""
    root = ET.parse(HERE / 'turtlebot3_waffle_pi.xml').getroot()
    root.find('compiler').set('meshdir', str(HERE / 'assets'))
    world = root.find('worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .05',
                  friction=f'{friction} .005 .0001')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


class Bench:
    def __init__(self, friction=1.0):
        self.model = build_bench_model(friction)
        self.spec = load_scene()[1]
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                    for n in ('wheel_left', 'wheel_right')]
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
               for n in ('wheel_left', 'wheel_right')]
        self.dof = [int(m.jnt_dofadr[j]) for j in jid]
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0
        for _ in range(500):             # same 500-step settle as track_a_project
            mujoco.mj_step(self.model, self.data)

    def run(self, left_cmd, right_cmd, seconds):
        """Hold a wheel-rate command; return t, wheel rates (2xN), v, yaw rate."""
        self.reset()
        self.data.ctrl[self.act[0]] = left_cmd
        self.data.ctrl[self.act[1]] = right_cmd
        n = int(seconds / self.model.opt.timestep)
        t = np.arange(1, n + 1) * self.model.opt.timestep
        wr = np.zeros((2, n)); v = np.zeros(n); yaw = np.zeros(n)
        for k in range(n):
            mujoco.mj_step(self.model, self.data)
            wr[:, k] = [self.data.qvel[self.dof[0]], self.data.qvel[self.dof[1]]]
            R = self.data.xmat[self.base].reshape(3, 3)
            v[k] = float(np.dot(R[:2, 0], self.data.qvel[0:2]))   # forward speed, world-frame lin. vel.
            yaw[k] = self.data.qvel[5]
        return t, wr, v, yaw


def first_order(t, gain, tau, td):
    x = np.clip(t - td, 0.0, None)
    return gain * (1.0 - np.exp(-x / tau))


def fit_lag(t, y):
    """Fit y = gain*(1-exp(-(t-td)/tau)); returns gain, tau, td."""
    p0 = (y[-1] if abs(y[-1]) > 1e-9 else 1.0, 0.1, 0.0)
    try:
        popt, _ = curve_fit(first_order, t, y, p0=p0,
                            bounds=([-50, 1e-3, 0.0], [50, 5.0, 0.5]))
        return popt
    except Exception:
        return np.array([np.nan, np.nan, np.nan])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--friction', type=float, default=1.0)
    ap.add_argument('--output', default='actuator_id')
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    b = Bench(args.friction)
    r, track = b.spec.wheel_radius_m, b.spec.wheel_track_m
    results = {'friction': args.friction, 'wheel_radius_m': r, 'track_m': track}

    # 1+2. straight-line wheel step responses at several command levels
    print('--- Both wheels stepped from rest: wheel-rate response ---')
    print(' cmd[rad/s]  ss[rad/s]  shortfall  tau[s]  dead[s]  v_ss[m/s]')
    rows = []
    for cmd in (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.45, 7.0):
        t, wr, v, yaw = b.run(cmd, cmd, 3.0)
        ss = wr[:, -500:].mean()                    # last 1 s
        gain, tau, td = fit_lag(t, wr.mean(axis=0))
        vss = v[-500:].mean()
        rows.append(dict(cmd=cmd, ss=float(ss), shortfall=float(cmd - ss),
                         tau=float(tau), dead=float(td), v_ss=float(vss)))
        print(f'{cmd:9.2f}  {ss:9.3f}  {cmd-ss:9.3f}  {tau:6.3f}  {td:7.3f}  {vss:9.4f}')
    results['straight_steps'] = rows
    moving = [x for x in rows if x['ss'] > 0.2]
    results['straight_tau_mean_s'] = float(np.mean([x['tau'] for x in moving]))
    results['straight_shortfall_mean_rad_s'] = float(np.mean([x['shortfall'] for x in moving]))

    # 3. cruise command used by the tracker: 0.18 m/s
    cruise = 0.18 / r
    t, wr, v, _ = b.run(cruise, cruise, 4.0)
    results['cruise'] = dict(cmd_v=0.18, v_ss=float(v[-500:].mean()),
                             v_at_0p1s=float(v[int(0.1 / b.model.opt.timestep) - 1]))
    print(f"\nCruise: command 0.180 m/s -> steady {results['cruise']['v_ss']:.4f} m/s; "
          f"after one 0.1 s control period {results['cruise']['v_at_0p1s']:.4f} m/s")

    # minimum forward command that moves the robot
    print('\n--- Forward-speed command sweep (does it move?) ---')
    fwd = []
    for vc in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15):
        c = vc / r
        _, _, v, _ = b.run(c, c, 3.0)
        fwd.append(dict(v_cmd=vc, v_ss=float(v[-500:].mean())))
        print(f' v_cmd={vc:5.2f} m/s -> v_ss={v[-500:].mean():7.4f} m/s')
    results['forward_sweep'] = fwd

    # 4. in-place rotation deadband (wheels +/- equal and opposite)
    print('\n--- In-place yaw command sweep ---')
    yawrows = []
    for wc in (0.10, 0.20, 0.30, 0.40, 0.50, 0.65, 0.80):
        c = wc * track / (2 * r)                  # wheel rate for yaw command wc
        t, wr, v, yaw = b.run(-c, c, 3.0)
        w_ss = yaw[-500:].mean()
        gain, tau, td = fit_lag(t, yaw)
        yawrows.append(dict(w_cmd=wc, w_ss=float(w_ss), tau=float(tau)))
        print(f' w_cmd={wc:4.2f} rad/s -> w_ss={w_ss:7.4f} rad/s  tau={tau:5.3f}s')
    results['yaw_sweep'] = yawrows

    # 5. yaw response while driving (the case that matters for path tracking)
    print('\n--- Yaw while cruising: v=0.18 m/s, small yaw commands ---')
    drow = []
    for wc in (0.05, 0.10, 0.20, 0.40):
        vl = 0.18 - 0.5 * track * wc; vr = 0.18 + 0.5 * track * wc
        _, _, v, yaw = b.run(vl / r, vr / r, 4.0)
        drow.append(dict(w_cmd=wc, w_ss=float(yaw[-500:].mean())))
        print(f' w_cmd={wc:4.2f} rad/s -> w_ss={yaw[-500:].mean():7.4f} rad/s')
    results['yaw_while_driving'] = drow

    (out / f'actuator_id_friction{args.friction}.json').write_text(json.dumps(results, indent=2))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    t, wr, v, yaw = b.run(cruise, cruise, 1.5)
    ax[0].plot(t, wr.mean(axis=0), label='measured wheel rate')
    g, tau, td = fit_lag(t, wr.mean(axis=0))
    ax[0].plot(t, first_order(t, g, tau, td), '--', label=f'fit: tau={tau:.3f}s')
    ax[0].axhline(cruise, color='gray', lw=.8, label='command')
    ax[0].set(xlabel='t [s]', ylabel='wheel rate [rad/s]', title='Wheel step response')
    ax[1].plot([x['cmd'] for x in rows], [x['ss'] for x in rows], 'o-')
    ax[1].plot([0, 7.5], [0, 7.5], ':', color='gray')
    ax[1].set(xlabel='command [rad/s]', ylabel='steady wheel rate [rad/s]',
              title='Friction offset (stiction band)')
    ax[2].plot([x['w_cmd'] for x in yawrows], [x['w_ss'] for x in yawrows], 'o-',
               label='in place')
    ax[2].plot([x['w_cmd'] for x in drow], [x['w_ss'] for x in drow], 's-',
               label='while driving 0.18 m/s')
    ax[2].plot([0, .8], [0, .8], ':', color='gray')
    ax[2].set(xlabel='yaw command [rad/s]', ylabel='achieved yaw rate [rad/s]',
              title='Yaw deadband')
    for a in ax:
        a.grid(alpha=.3); a.legend() if a.get_legend_handles_labels()[0] else None
    fig.tight_layout()
    fig.savefig(out / f'actuator_id_friction{args.friction}.png', dpi=150)
    print(f'\nsaved to {out}/')


if __name__ == '__main__':
    main()
