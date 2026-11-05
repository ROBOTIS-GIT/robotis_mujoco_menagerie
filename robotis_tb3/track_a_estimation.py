"""Estimated-state API: no MuJoCo data is accepted by the EKF or controller."""
import numpy as np
from turtlebot_model import (step_pose_from_encoder, state_jacobian_from_encoder,
                             wheel_increment_jacobian, wrap_angle)

LANDMARK = np.array([3.0, 0.0])
CAMERA_FORWARD = .09
R_CAMERA = np.diag([.03**2, np.deg2rad(1.5)**2, np.deg2rad(8)**2])


def marker_measurement(pose):
    x, y, heading = pose
    camera_xy = np.array([x, y]) + CAMERA_FORWARD * np.array([np.cos(heading), np.sin(heading)])
    delta = LANDMARK - camera_xy
    return np.array([np.linalg.norm(delta), wrap_angle(np.arctan2(delta[1], delta[0]) - heading)])


def measurement_jacobian(pose):
    # Central differences also account for the camera's offset from the base.
    H = np.zeros((2, 3))
    for i in range(3):
        step = np.eye(3)[i] * 1e-5
        difference = marker_measurement(pose + step) - marker_measurement(pose - step)
        difference[1] = wrap_angle(difference[1])
        H[:, i] = difference / 2e-5
    return H


class PoseEKF:
    def __init__(self, initial_pose, spec, r_scale=1.0, q_scale=1.0):
        if not all(np.isfinite(s) and s > 0 for s in (r_scale, q_scale)):
            raise ValueError("Q and R covariance scales must be finite and positive")
        self.pose = np.array(initial_pose, dtype=float)
        self.P = np.diag([.06**2, .06**2, np.deg2rad(5)**2])
        self.spec = spec
        self.q_scale = q_scale
        self.R = R_CAMERA * r_scale
        self.accepted = self.rejected = 0
        self.last_nis = None

    def predict(self, increments):
        dl, dr = increments
        F = state_jacobian_from_encoder(self.pose, dl, dr, self.spec)
        G = wheel_increment_jacobian(self.pose, dl, dr, self.spec)
        wheel_std = .002 + .04 * np.abs(increments)
        Q = G @ np.diag(wheel_std**2) @ G.T + np.diag([.003**2, .003**2, .02**2])
        self.pose = step_pose_from_encoder(self.pose, dl, dr, self.spec)
        self.P = F @ self.P @ F.T + self.q_scale * Q

    def correct(self, observation):
        if observation is None:
            return False
        z = np.asarray(observation, dtype=float)
        if z.shape != (3,) or not np.isfinite(z).all():
            self.rejected += 1
            return False
        H = np.vstack((measurement_jacobian(self.pose), [0,0,1]))
        predicted = np.r_[marker_measurement(self.pose), self.pose[2]]
        residual = z - predicted
        residual[1] = wrap_angle(residual[1])
        residual[2] = wrap_angle(residual[2])
        S = H @ self.P @ H.T + self.R
        self.last_nis = float(residual @ np.linalg.solve(S, residual))
        # 99% chi-square threshold for a three-component observation.
        if self.last_nis > 11.345:
            self.rejected += 1
            return False
        K = np.linalg.solve(S, H @ self.P).T
        self.pose += K @ residual
        self.pose[2] = wrap_angle(self.pose[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = (self.P + self.P.T) / 2
        self.accepted += 1
        return True
