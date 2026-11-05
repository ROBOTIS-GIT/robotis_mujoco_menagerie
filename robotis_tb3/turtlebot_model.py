from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


SCENE_PATH = Path(__file__).with_name("scene_turtlebot3_waffle_pi.xml")
CONTROL_STEPS = 10


@dataclass(frozen=True)
class TurtleBotSpec:
    wheel_radius_m: float
    wheel_track_m: float
    physics_dt_s: float
    update_dt_s: float
    max_wheel_rate_rad_s: float
    steps_per_update: int


def _id(model, obj_type, name):
    identifier = mujoco.mj_name2id(model, obj_type, name)
    if identifier < 0:
        raise ValueError(f"Missing {name} in the MuJoCo model")
    return identifier


def _tire_radius(model, mesh_name):
    mesh_id = _id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
    start = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    vertices = model.mesh_vert[start:start + count]
    spans = np.ptp(vertices, axis=0)
    radial_diameters = np.sort(spans)[-2:]
    if not np.isclose(radial_diameters[0], radial_diameters[1], rtol=0.05):
        raise ValueError(f"The {mesh_name} tire is not roughly round")
    return float(np.mean(radial_diameters) / 2.0)


def load_scene(scene_path=SCENE_PATH, steps_per_update=CONTROL_STEPS):
    if not isinstance(steps_per_update, int) or steps_per_update < 1:
        raise ValueError("steps_per_update must be a positive integer")
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    left_body = _id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_left")
    right_body = _id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_right")
    left_position = model.body_pos[left_body]
    right_position = model.body_pos[right_body]
    if not np.isclose(left_position[0], right_position[0], atol=1e-4):
        raise ValueError("Wheel centres do not share the same forward position")
    wheel_track = float(abs(left_position[1] - right_position[1]))
    radii = [_tire_radius(model, name) for name in ("left_tire", "right_tire")]
    if not np.isclose(radii[0], radii[1], rtol=0.02):
        raise ValueError("Left and right tire radii disagree")
    actuator_ids = [_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                    for name in ("wheel_left", "wheel_right")]
    speed_limits = [min(-model.actuator_ctrlrange[i, 0],
                        model.actuator_ctrlrange[i, 1]) for i in actuator_ids]
    spec = TurtleBotSpec(
        wheel_radius_m=float(np.mean(radii)),
        wheel_track_m=wheel_track,
        physics_dt_s=float(model.opt.timestep),
        update_dt_s=float(model.opt.timestep * steps_per_update),
        max_wheel_rate_rad_s=float(min(speed_limits)),
        steps_per_update=steps_per_update,
    )
    return model, spec

# Adapted from the Week 2 wheel geometry example.
# Wheel spacing and step time come from the ROBOTIS MuJoCo scene.
# Tire radius comes from the mesh. Ten steps is our choice.

def wrap_angle(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def wheel_rates_to_twist(left_rate, right_rate, spec):
    forward_speed = 0.5 * spec.wheel_radius_m * (left_rate + right_rate)
    turn_rate = spec.wheel_radius_m * (right_rate - left_rate) / spec.wheel_track_m
    return np.array([forward_speed, turn_rate], dtype=float)


def twist_to_wheel_rates(forward_speed, turn_rate, spec):
    half_turn = 0.5 * spec.wheel_track_m * turn_rate
    return np.array([(forward_speed - half_turn) / spec.wheel_radius_m,
                     (forward_speed + half_turn) / spec.wheel_radius_m])


def limit_wheel_rates(rates, spec):
    rates = np.asarray(rates, dtype=float)
    scale = max(1.0, float(np.max(np.abs(rates))) / spec.max_wheel_rate_rad_s)
    return rates / scale


def step_pose_from_encoder(pose, delta_left, delta_right, spec):
    pose = np.asarray(pose, dtype=float)
    travel = 0.5 * spec.wheel_radius_m * (delta_left + delta_right)
    turn = spec.wheel_radius_m * (delta_right - delta_left) / spec.wheel_track_m
    heading = pose[2]
    if abs(turn) < 1e-8:
        mid_heading = heading + 0.5 * turn
        dx = travel * np.cos(mid_heading)
        dy = travel * np.sin(mid_heading)
    else:
        arc_radius = travel / turn
        dx = arc_radius * (np.sin(heading + turn) - np.sin(heading))
        dy = arc_radius * (np.cos(heading) - np.cos(heading + turn))
    return np.array([pose[0] + dx, pose[1] + dy,
                     wrap_angle(heading + turn)])


def state_jacobian_from_encoder(pose, delta_left, delta_right, spec):
    next_pose = step_pose_from_encoder(pose, delta_left, delta_right, spec)
    dx = next_pose[0] - pose[0]
    dy = next_pose[1] - pose[1]
    jacobian = np.eye(3)
    jacobian[0, 2] = -dy
    jacobian[1, 2] = dx
    return jacobian


def wheel_increment_jacobian(pose, delta_left, delta_right, spec, eps=1e-6):
    increments = np.array([delta_left, delta_right], dtype=float)
    jacobian = np.zeros((3, 2))
    for index in range(2):
        higher = increments.copy()
        lower = increments.copy()
        higher[index] += eps
        lower[index] -= eps
        plus = step_pose_from_encoder(pose, higher[0], higher[1], spec)
        minus = step_pose_from_encoder(pose, lower[0], lower[1], spec)
        difference = plus - minus
        difference[2] = wrap_angle(difference[2])
        jacobian[:, index] = difference / (2.0 * eps)
    return jacobian

# Adapted from Week 2 diff_drive_step and Week 8 f_uni.
# It uses the Waffle Pi dimensions above and encoder angle changes.
# The wheel Jacobian passes encoder noise into EKF covariance.
# No true robot pose is used here.

CAMERA_XY_STD_M = 0.03
CAMERA_YAW_STD_RAD = np.deg2rad(5.0)


def compose_pose(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    c, s = np.cos(first[2]), np.sin(first[2])
    return np.array([first[0] + c * second[0] - s * second[1],
                     first[1] + s * second[0] + c * second[1],
                     wrap_angle(first[2] + second[2])])


def inverse_pose(pose):
    x, y, heading = np.asarray(pose, dtype=float)
    c, s = np.cos(heading), np.sin(heading)
    return np.array([-c * x - s * y, s * x - c * y,
                     wrap_angle(-heading)])


def world_base_from_marker(marker_world, marker_in_camera, camera_in_base):
    if marker_in_camera is None:
        return None
    return compose_pose(
        compose_pose(marker_world, inverse_pose(marker_in_camera)),
        inverse_pose(camera_in_base),
    )


def camera_pose_prediction(state):
    return np.asarray(state, dtype=float).copy()


def camera_pose_jacobian():
    return np.eye(3)


def camera_pose_covariance():
    return np.diag([CAMERA_XY_STD_M ** 2, CAMERA_XY_STD_M ** 2,
                    CAMERA_YAW_STD_RAD ** 2])


def camera_pose_residual(observed, predicted):
    residual = np.asarray(observed, dtype=float) - np.asarray(predicted, dtype=float)
    residual[2] = wrap_angle(residual[2])
    return residual

# These pose transforms are for the chosen marker measurement.
# Camera axes need converting to robot x forward and y left.
# If the marker is not seen, the EKF skips that camera update.
# Three cm and five degrees are starting noise guesses.

def linearize_wheel_model(pose, nominal_rates, spec):
    heading = float(np.asarray(pose, dtype=float)[2])
    left_rate, right_rate = np.asarray(nominal_rates, dtype=float)
    forward_speed, _ = wheel_rates_to_twist(left_rate, right_rate, spec)
    c, s = np.cos(heading), np.sin(heading)
    radius = spec.wheel_radius_m
    track = spec.wheel_track_m
    A = np.zeros((3, 3))
    A[0, 2] = -forward_speed * s
    A[1, 2] = forward_speed * c
    B = np.array([[0.5 * radius * c, 0.5 * radius * c],
                  [0.5 * radius * s, 0.5 * radius * s],
                  [-radius / track, radius / track]])
    C = camera_pose_jacobian()
    return A, B, C


def controllability_matrix(A, B):
    return np.hstack((B, A @ B, A @ A @ B))


def observability_matrix(A, C):
    return np.vstack((C, C @ A, C @ A @ A))
# A and B come from the Week 2 drive model near a chosen motion.
# C uses base x, y and heading when the marker is visible.

class WheelIO:
    def __init__(self, model, data, spec):
        self.model = model
        self.data = data
        self.spec = spec
        joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                     for name in ("wheel_left", "wheel_right")]
        self.angle_addresses = [int(model.jnt_qposadr[i]) for i in joint_ids]
        self.actuator_ids = [_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                             for name in ("wheel_left", "wheel_right")]

    def read_angles(self):
        return np.array([self.data.qpos[i] for i in self.angle_addresses],
                        dtype=float)

    def set_rates(self, left_rate, right_rate):
        rates = limit_wheel_rates([left_rate, right_rate], self.spec)
        for actuator_id, rate in zip(self.actuator_ids, rates):
            self.data.ctrl[actuator_id] = rate
        return rates

    def step_update(self):
        for _ in range(self.spec.steps_per_update):
            mujoco.mj_step(self.model, self.data)
# Joint and actuator names come from the ROBOTIS scene.
# Wheel angles and commands do not use the true base pose.

def state_derivative(pose, left_rate, right_rate, spec):
    heading = float(np.asarray(pose, dtype=float)[2])
    forward_speed, turn_rate = wheel_rates_to_twist(left_rate, right_rate, spec)
    return np.array([forward_speed * np.cos(heading),
                     forward_speed * np.sin(heading), turn_rate])


def main():
    model, spec = load_scene()
    print("Wheel radius m", spec.wheel_radius_m)
    print("Wheel spacing m", spec.wheel_track_m)
    print("Physics step s", spec.physics_dt_s)
    print("Model update s", spec.update_dt_s)
    print("Wheel rate limit rad/s", spec.max_wheel_rate_rad_s)
    A, B, C = linearize_wheel_model([0.0, 0.0, 0.0], [5.0, 5.0], spec)
    print("Moving controllability rank", np.linalg.matrix_rank(
        controllability_matrix(A, B)))
    print("Visible marker observability rank", np.linalg.matrix_rank(
        observability_matrix(A, C)))

    data = mujoco.MjData(model)
    wheels = WheelIO(model, data, spec)
    for _ in range(round(1.0 / spec.physics_dt_s)):
        mujoco.mj_step(model, data)
    previous_angles = wheels.read_angles()
    encoder_pose = np.zeros(3)
    wheels.set_rates(5.0, 5.0)
    for _ in range(round(2.0 / spec.update_dt_s)):
        wheels.step_update()
        angles = wheels.read_angles()
        encoder_pose = step_pose_from_encoder(
            encoder_pose, *(angles - previous_angles), spec)
        previous_angles = angles
    print("Encoder pose change after 2 s", encoder_pose)


if __name__ == "__main__":
    main()
# The state rates follow the Week 2 differential drive equations.
# Two seconds is just a short check of the scene and encoder model.