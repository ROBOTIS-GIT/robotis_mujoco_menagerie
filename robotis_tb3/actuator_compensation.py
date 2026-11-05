"""Part D, Step 2b: actuator model and wheel-command compensator. No simulator imports.

Why this exists
---------------
Bench tests (identify_actuators.py) on the project's MuJoCo Waffle Pi show the
wheel velocity actuators are far from ideal:

  * a friction offset of about 2.0 rad/s: wheel commands below it do nothing and
    above it the wheel runs ~2 rad/s slower than commanded (larger when turning),
  * ground speed is ~0.85 of r*wheel_rate (effective rolling radius ~0.0281 m),
  * yaw rate is only ~0.70-0.75 of the nominal r*(wR-wL)/track in place, and lower
    again (about 0.5 of nominal) while driving,
  * a first-order lag of about 0.11 s (about one control period).

The controller therefore works in (v, w) but the plant delivers something
different. This module maps a desired ground-truth-meaning twist (v, w) to wheel
commands so the plant delivers it, using only the encoder wheel rates.

Recommended: feedforward only (use_trim=False, kw_drive == kw)
-----------------------------------------------------------
Desired wheel rates from the measured kinematic gains plus a signed friction offset
c_ff. Two refinements were built and tested on held-out twists and did NOT help:
  * a PI trim per wheel on the encoder rate error (made speed worse when turning),
  * a speed-blended yaw gain (overshot yaw at low speed; worse on held-out cases).
They stay in the code (use_trim=True, kw_drive=0.054) so the result is reproducible.

Speed/yaw priority: the actuator saturates at +-7.88 rad/s, of which c_ff is spent
on friction, so wheel rates above ~5.9 rad/s cannot be reached. When the sum of
speed and yaw demand exceeds that, speed is reduced first so steering is kept.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ActuatorModel:
    kv: float = 0.0281        # ground speed per mean wheel rate [m/rad] (nominal 0.0330)
    kw: float = 0.075         # yaw per wheel-rate difference, turning IN PLACE (nominal 0.1146)
    kw_drive: float = 0.075   # yaw gain while driving. Equal to kw = single gain (recommended).
                              # 0.054 was tried (mode-dependent): did NOT generalise, see README
    c_ff: float = 2.0         # friction offset [rad/s]
    tau: float = 0.11         # first-order lag [s]
    cmd_limit: float = 7.88   # actuator command limit [rad/s]


def effective_kw(v, model):
    """Yaw gain blended from in-place (|v| <= 0.02 m/s) to driving (|v| >= 0.06 m/s)."""
    s = float(np.clip((abs(v) - 0.02) / 0.04, 0.0, 1.0))
    return model.kw * (1.0 - s) + model.kw_drive * s


def wheel_rates_for_twist(v, w, model=ActuatorModel()):
    """Desired wheel rates (left, right) that deliver ground twist (v, w), speed-limited.

    Returns (rates, v_applied, w_applied)."""
    avail = model.cmd_limit - model.c_ff            # usable wheel rate after friction
    diff = w / effective_kw(v, model)                # wR - wL
    if abs(diff) / 2.0 > avail:                      # yaw alone exceeds the actuator
        diff = np.sign(diff) * 2.0 * avail
    mean = v / model.kv
    room = avail - abs(diff) / 2.0                   # yaw has priority, trim speed
    mean = float(np.clip(mean, -room, room))
    rates = np.array([mean - diff / 2.0, mean + diff / 2.0])
    return rates, mean * model.kv, diff * effective_kw(v, model)


def max_speed_for_yaw(w, model=ActuatorModel()):
    """Largest forward speed the actuators can deliver while also turning at w [m/s].

    The 7.88 rad/s command limit minus the friction offset is shared between speed and
    yaw, so fast turning leaves almost no speed. Use this to schedule v from |w|."""
    avail = model.cmd_limit - model.c_ff
    room = avail - abs(w) / effective_kw(0.0, model) / 2.0
    return float(max(0.0, room) * model.kv)


class WheelCompensator:
    def __init__(self, model=ActuatorModel(), dt=0.1, ki=1.0, trim_limit=1.0,
                 deadzone=0.05, use_trim=True):
        self.model, self.dt, self.ki = model, dt, ki
        self.trim_limit, self.deadzone, self.use_trim = trim_limit, deadzone, use_trim
        self.trim = np.zeros(2)

    def reset(self):
        self.trim[:] = 0.0

    def command(self, v, w, measured_rates=None):
        """Wheel commands [rad/s] for desired twist (v, w).

        measured_rates: encoder wheel rates [rad/s] from the last period, or None."""
        want, _, _ = wheel_rates_for_twist(v, w, self.model)
        cmd = np.zeros(2)
        for i in range(2):
            if abs(want[i]) < self.deadzone:
                self.trim[i] = 0.0                    # nothing to push; do not wind up
                continue
            if self.use_trim and measured_rates is not None:
                err = want[i] - measured_rates[i]
                self.trim[i] = np.clip(self.trim[i] + np.sign(want[i]) * self.ki * self.dt * err,
                                       -self.trim_limit, self.trim_limit)
            cmd[i] = want[i] + np.sign(want[i]) * (self.model.c_ff + self.trim[i])
        return np.clip(cmd, -self.model.cmd_limit, self.model.cmd_limit)


# ---- integration helpers (used by track_a_project.py) ---------------------------------
COMPENSATE_MODES = ('off', 'path', 'all')


def use_compensation(mode, docking):
    """Whether this frame's command goes through the compensator.

    off  : never (the original twist_to_wheel_rates mapping; all earlier results)
    path : path following and no-path search only; the docking relay keeps the original
           mapping, because its speeds and relay turn were tuned on the weak plant
    all  : every frame, docking included"""
    if mode not in COMPENSATE_MODES:
        raise ValueError(f'compensate mode must be one of {COMPENSATE_MODES}, got {mode!r}')
    return mode == 'all' or (mode == 'path' and not docking)


def build_compensator(spec, dt=0.1):
    """Recommended compensator (feedforward only), limited to the scene's wheel-rate limit."""
    return WheelCompensator(ActuatorModel(cmd_limit=float(spec.max_wheel_rate_rad_s)), dt=dt, use_trim=False)

