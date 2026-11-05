"""Sim-free checks for the Part D error model, LQR design and robustness analysis.
Run from the repo root: python -m unittest discover -s robotis_tb3 -p "test_track_a*.py" -v
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # robotis_tb3/ (`lqr` package)
import unittest
import numpy as np
from lqr import track_a_control_design as d


class DesignTests(unittest.TestCase):
    def setUp(self):
        self.Q, self.R = d.bryson_weights()

    def test_bryson_weights(self):
        np.testing.assert_allclose(np.diag(self.Q), [100.0, 1 / 0.35**2])
        self.assertAlmostEqual(self.R[0, 0], 1 / 0.8**2)

    def test_controllability_rank_drops_at_zero_speed(self):
        self.assertEqual(d.controllability_rank(0.0), 1)
        self.assertEqual(d.controllability_rank(0.05), 2)

    def test_discretisation_matches_matrix_exponential(self):
        from scipy.linalg import expm
        v, dt = 0.18, 0.1
        A, B = d.error_model(v)
        M = np.zeros((3, 3)); M[:2, :2] = A * dt; M[:2, 2:] = B * dt
        E = expm(M)
        Ad, Bd = d.discretise(v, dt)
        np.testing.assert_allclose(Ad, E[:2, :2], atol=1e-12)
        np.testing.assert_allclose(Bd, E[:2, 2:], atol=1e-12)

    def test_lqr_gain_stabilises_and_is_positive(self):
        K = d.lqr_discrete(d.V_NOM, self.Q, self.R)
        self.assertTrue(np.all(K > 0))
        self.assertLess(max(abs(d.closed_loop_poles(d.V_NOM, K, d.DT))), 1.0)

    def test_lqr_satisfies_discrete_riccati_optimality(self):
        # Perturbing the optimal gain must not reduce the quadratic cost.
        Ad, Bd = d.discretise(d.V_NOM)
        def cost(K):
            Acl = Ad - Bd @ K
            Qc = self.Q + K.T @ self.R @ K
            from scipy.linalg import solve_discrete_lyapunov
            P = solve_discrete_lyapunov(Acl.T, Qc)
            return np.trace(P)
        K = d.lqr_discrete(d.V_NOM, self.Q, self.R)
        for dk in ([0.5, 0], [-0.5, 0], [0, 0.3], [0, -0.3]):
            self.assertGreater(cost(K + np.array([dk])), cost(K) - 1e-9)

    def test_heuristic_equivalent_gain(self):
        np.testing.assert_allclose(d.heuristic_equivalent_gain(), [[14.0, 2.8]])

    def test_heuristic_damping_matches_hand_derivation(self):
        # s^2 + k_theta s + v k_y = 0  ->  zeta = k_theta / (2 sqrt(v k_y))
        Kh = d.heuristic_equivalent_gain()
        wn, z = d.damping_and_wn(d.closed_loop_poles(d.V_NOM, Kh))
        self.assertAlmostEqual(z, 2.8 / (2 * np.sqrt(0.18 * 14.0)), places=6)

    def test_augmented_model_reduces_to_ideal_when_lag_small(self):
        K = d.lqr_discrete(d.V_NOM, self.Q, self.R)[0]
        rho, _ = d.closed_loop_robustness(K, d.V_NOM, tau=1e-3, g=1.0)
        ideal = max(abs(d.closed_loop_poles(d.V_NOM, K[None, :], d.DT)))
        self.assertAlmostEqual(rho, ideal, delta=0.01)

    def test_stable_over_measured_plant_range(self):
        K = d.lqr_discrete(d.V_NOM, self.Q, self.R)[0]
        for v in (0.03, 0.096, 0.18):
            for g in (0.3, 1.0, 2.0):
                rho, zeta = d.closed_loop_robustness(K, v, 0.11, g)
                self.assertLess(rho, 1.0)
                self.assertGreater(zeta, 0.3)

    def test_gain_margin_is_large(self):
        K = d.lqr_discrete(d.V_NOM, self.Q, self.R)[0]
        self.assertGreater(d.max_stable_yaw_gain(K, 0.096, 0.11), 5.0)


if __name__ == "__main__":
    unittest.main()
