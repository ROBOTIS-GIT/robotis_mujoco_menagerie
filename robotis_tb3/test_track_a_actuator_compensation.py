"""Sim-free tests for the actuator compensator."""
import unittest
import numpy as np
from actuator_compensation import (ActuatorModel, WheelCompensator,
                                   wheel_rates_for_twist, max_speed_for_yaw, effective_kw)


class CompensatorTests(unittest.TestCase):
    def setUp(self):
        self.m = ActuatorModel()

    def test_kinematic_inverse(self):
        rates, v, w = wheel_rates_for_twist(0.08, 0.2, self.m)
        self.assertAlmostEqual(self.m.kv * rates.mean(), 0.08)
        self.assertAlmostEqual(self.m.kw * (rates[1] - rates[0]), 0.2)
        self.assertAlmostEqual(v, 0.08); self.assertAlmostEqual(w, 0.2)

    def test_command_adds_signed_friction_offset(self):
        c = WheelCompensator(self.m, use_trim=False)
        cmd = c.command(0.1, 0.0)
        np.testing.assert_allclose(cmd, 0.1 / self.m.kv + self.m.c_ff)
        np.testing.assert_allclose(c.command(-0.1, 0.0), -cmd)

    def test_zero_request_gives_zero_command(self):
        np.testing.assert_array_equal(WheelCompensator(self.m).command(0.0, 0.0), [0, 0])

    def test_command_never_exceeds_actuator_limit(self):
        c = WheelCompensator(self.m, use_trim=False)
        for v in (-0.3, 0.05, 0.18, 0.4):
            for w in (-1.2, -0.8, 0.0, 0.4, 0.8, 1.5):
                self.assertLessEqual(np.max(np.abs(c.command(v, w))), self.m.cmd_limit + 1e-9)

    def test_yaw_has_priority_over_speed(self):
        _, v, w = wheel_rates_for_twist(0.18, 0.6, self.m)
        self.assertAlmostEqual(w, 0.6)          # yaw kept
        self.assertLess(v, 0.18)                # speed trimmed
        self.assertAlmostEqual(v, max_speed_for_yaw(0.6, self.m), places=6)

    def test_max_speed_decreases_with_yaw(self):
        s = [max_speed_for_yaw(w, self.m) for w in (0.0, 0.2, 0.4, 0.6, 0.8)]
        self.assertTrue(all(a >= b for a, b in zip(s, s[1:])))
        self.assertAlmostEqual(s[0], self.m.kv * (self.m.cmd_limit - self.m.c_ff))

    def test_trim_is_bounded_and_resets(self):
        c = WheelCompensator(self.m, ki=5.0, trim_limit=1.0, use_trim=True)
        for _ in range(100):
            c.command(0.1, 0.0, measured_rates=np.zeros(2))   # wheels never move
        self.assertLessEqual(np.max(np.abs(c.trim)), 1.0 + 1e-12)
        c.command(0.0, 0.0, measured_rates=np.zeros(2))
        np.testing.assert_array_equal(c.trim, [0, 0])

    def test_effective_kw_blend(self):
        m = ActuatorModel(kw=0.075, kw_drive=0.054)
        self.assertAlmostEqual(effective_kw(0.0, m), 0.075)
        self.assertAlmostEqual(effective_kw(0.1, m), 0.054)
        self.assertAlmostEqual(effective_kw(0.04, m), 0.0645)


if __name__ == "__main__":
    unittest.main()
