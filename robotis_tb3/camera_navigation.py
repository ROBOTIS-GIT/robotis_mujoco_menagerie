"""ArUco visual docking demo. Commands depend only on RGB images, never base pose."""
import argparse
import json
import time
from threading import Event
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import mujoco
import numpy as np

from turtlebot_model import load_scene, WheelIO, twist_to_wheel_rates

HERE = Path(__file__).resolve().parent
WIDTH, HEIGHT, FOV = 640, 480, 60.0
MARKER_SIZE = 0.22
STOP_DISTANCE = 0.55  # horizontal camera-to-marker distance, metres
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def build_scene(marker_y=0.4):
    """Add a body-fixed camera and a textured target to the stock robot."""
    root = ET.parse(HERE / 'turtlebot3_waffle_pi.xml').getroot()
    root.find('compiler').set('meshdir', str(HERE / 'assets'))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth=str(WIDTH), offheight=str(HEIGHT))
    ET.SubElement(visual, 'headlight', ambient='0.5 0.5 0.5', diffuse='0.6 0.6 0.6')
    world = root.find('worldbody')
    base = world.find("body[@name='base']")
    ET.SubElement(base, 'camera', name='navigation', pos='0.09 0 0.18',
                  xyaxes='0 -1 0 0 0 1', fovy=str(FOV))
    ET.SubElement(world, 'light', pos='0 0 3', directional='true', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .05',
                  rgba='.25 .35 .4 1')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', name='marker_texture', type='2d',
                  file=str(HERE / 'assets' / 'navigation_marker.png'))
    ET.SubElement(asset, 'material', name='marker_material', texture='marker_texture',
                  texrepeat='1 1', texuniform='false', emission='1', specular='0')
    # Surface faces -world x; local right is -world y and local up is +world z.
    ET.SubElement(world, 'geom', name='marker', type='plane',
                  pos=f'2 {marker_y} 0.18', xyaxes='0 -1 0 0 0 1',
                  size='.15 .15 .001', material='marker_material',
                  contype='0', conaffinity='0')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def camera_matrix():
    focal = HEIGHT / (2 * np.tan(np.deg2rad(FOV) / 2))
    return np.array([[focal, 0, (WIDTH - 1)/2],
                     [0, focal, (HEIGHT - 1)/2], [0, 0, 1]], dtype=float)


class MarkerCamera:
    def __init__(self):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(DICTIONARY, params)
        s = MARKER_SIZE / 2
        self.points = np.array([[-s, s, 0], [s, s, 0],
                                [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def observe(self, rgb):
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None or 0 not in ids.flatten():
            return None, corners, ids
        index = list(ids.flatten()).index(0)
        ok, rvec, tvec = cv2.solvePnP(self.points, corners[index].reshape(4, 2),
                                      camera_matrix(), np.zeros(5),
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
            return None, corners, ids
        projected, _ = cv2.projectPoints(self.points, rvec, tvec, camera_matrix(), np.zeros(5))
        error = np.mean(np.linalg.norm(projected.reshape(4, 2) - corners[index].reshape(4, 2), axis=1))
        if error > 3:
            return None, corners, ids
        right, _, forward = tvec.flatten()
        return (float(np.hypot(right, forward)), float(-np.arctan2(right, forward))), corners, ids


class DockingController:
    def __init__(self):
        self.arrival_frames = 0
        self.arrived = False

    def command(self, observation):
        if self.arrived:
            return 0.0, 0.0, 'ARRIVED'
        if observation is None:
            self.arrival_frames = 0
            return 0.0, 0.0, 'MARKER LOST - STOPPED'
        distance, bearing = observation
        error = distance - STOP_DISTANCE
        if abs(error) < 0.025 and abs(bearing) < np.deg2rad(3):
            self.arrival_frames += 1
        else:
            self.arrival_frames = 0
        if self.arrival_frames >= 5:
            self.arrived = True
            return 0.0, 0.0, 'ARRIVED'
        turn = float(np.clip(1.8 * bearing, -.6, .6))
        # The stock velocity actuators cannot overcome wheel friction at tiny
        # commands. Keep a modest approach speed until inside the stop band.
        speed = max(.09, min(.16, .7 * error)) if error > .02 else 0.0
        if abs(bearing) > .35:
            speed = 0.0
        return speed, turn, 'APPROACHING'


def run(args):
    model = build_scene(args.marker_y)
    _, spec = load_scene()
    data = mujoco.MjData(model)
    wheels = WheelIO(model, data, spec)
    # Settle under gravity before collecting images.
    for _ in range(500):
        mujoco.mj_step(model, data)
    detector, controller = MarkerCamera(), DockingController()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    viewer = None
    quit_requested = Event()
    if not args.headless:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(
            model, data, key_callback=lambda key: quit_requested.set() if key in (81, 256) else None)
        viewer.cam.lookat[:] = [1.0, 0.2, 0.12]
        viewer.cam.distance = 3.3
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -35
    records = []
    seen = 0
    status = 'STOPPED'
    start = data.time
    try:
        with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
            for frame in range(round(args.seconds / .1)):
                tick = time.monotonic()
                if viewer is not None and (not viewer.is_running() or quit_requested.is_set()):
                    break
                renderer.update_scene(data, camera='navigation')
                rgb = renderer.render()
                observation, corners, ids = detector.observe(rgb)
                speed, turn, status = controller.command(observation)
                rates = wheels.set_rates(*twist_to_wheel_rates(speed, turn, spec))
                seen += observation is not None
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                label = status
                if observation:
                    label += f'  {observation[0]:.2f}m  {np.rad2deg(observation[1]):+.1f}deg'
                cv2.putText(bgr, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 180, 0), 2)
                if frame == 0:
                    cv2.imwrite(str(output / 'camera-start.png'), bgr)
                if frame % 10 == 0 or controller.arrived:
                    print(f'{data.time-start:5.1f}s {label}', flush=True)
                records.append({'time': float(data.time-start), 'measurement': observation,
                                'wheel_commands': rates.tolist(), 'status': status})
                for _ in range(5):
                    wheels.step_update()
                if viewer is not None:
                    viewer.sync()
                    viewer.set_images((mujoco.MjrRect(10, 10, 320, 240),
                                       cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), (320, 240))))
                    viewer.set_texts((None, None, label + '\nQ: stop and close', None))
                    time.sleep(max(0, .1 - (time.monotonic()-tick)))
                if controller.arrived:
                    break
            wheels.set_rates(0, 0)
            for _ in range(50):
                wheels.step_update()
            renderer.update_scene(data, camera='navigation')
            final_rgb = renderer.render()
            final_observation, _, _ = detector.observe(final_rgb)
            cv2.imwrite(str(output / 'camera-finish.png'), cv2.cvtColor(final_rgb, cv2.COLOR_RGB2BGR))
            # Ground truth is read only here for independent test reporting;
            # the detector and controller above never receive it.
            mujoco.mj_forward(model, data)
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'navigation')
            marker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'marker')
            true_distance = float(np.linalg.norm(
                data.geom_xpos[marker_id, :2] - data.cam_xpos[camera_id, :2]))
            result = {'evaluation_true_distance_m': true_distance,
                      'evaluation_stopped_speed_m_s': float(np.linalg.norm(data.qvel[:3])),
                      'arrived': controller.arrived, 'frames': len(records), 'detections': seen,
                      'final_measurement': final_observation, 'stop_distance_m': STOP_DISTANCE}
            (output / 'navigation-result.json').write_text(json.dumps(result, indent=2) + '\n')
            (output / 'navigation-trace.json').write_text(json.dumps(records, indent=2) + '\n')
            print(json.dumps(result), flush=True)
            if viewer is not None:
                viewer.sync()
                viewer.set_images((mujoco.MjrRect(10, 10, 320, 240), cv2.resize(final_rgb, (320, 240))))
                final_status = 'ARRIVED' if controller.arrived else 'STOPPED BEFORE ARRIVAL'
                viewer.set_texts((None, None, final_status + '\nRobot stopped. Q: close', None))
                print('Robot stopped. Close MuJoCo or press Q.', flush=True)
                while viewer.is_running() and not quit_requested.is_set():
                    viewer.sync()
                    time.sleep(.05)
    finally:
        wheels.set_rates(0, 0)
        if viewer is not None:
            viewer.close()
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='Run rendered-camera verification without windows')
    parser.add_argument('--seconds', type=float, default=35)
    parser.add_argument('--marker-y', type=float, default=.4)
    parser.add_argument('--output', default='camera-navigation-output')
    args = parser.parse_args()
    result = run(args)
    if args.headless and not result['arrived']:
        raise SystemExit('Navigation did not reach the target within the time limit')
