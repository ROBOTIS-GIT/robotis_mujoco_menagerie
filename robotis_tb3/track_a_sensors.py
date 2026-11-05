from machinevisiontoolbox import CentralCamera, Image
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent

WIDTH, HEIGHT, FOV = 640, 480, 60.0
MARKER_SIZE = 0.2
CAMERA_TILT_DEGREES = 10

def build_room(obstacle_shift=0.0, friction=1.0):
    root = ET.parse(HERE / 'turtlebot3_waffle_pi.xml').getroot()
    root.find('compiler').set('meshdir', str(HERE / 'assets'))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth=str(WIDTH), offheight=str(HEIGHT))
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.6 .6 .6')
    ET.SubElement(root, 'statistic', center='1.4 0 .2', extent='3')
    world = root.find('worldbody')
    base = world.find("body[@name='base']")
    ET.SubElement(base, 'camera', name='navigation', pos='.09 0 .30', xyaxes='0 -1 0 -0.173648 0 0.984808', fovy=str(FOV))
    ET.SubElement(base, 'site', name='lidar', pos='-.049 0 .15', size='.005', rgba='0 1 0 1')
    ET.SubElement(world, 'light', pos='0 0 3', directional='true', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 .05',
                  rgba='.25 .35 .4 1', friction=f'{friction} .005 .0001')
    assets = root.find('asset')
    ET.SubElement(assets, 'texture', name='marker_texture', type='2d', file=str(HERE/'assets/navigation_marker.png'))
    ET.SubElement(assets, 'material', name='marker_material', texture='marker_texture', texrepeat='1 1',
                  texuniform='false', emission='1', specular='0')
    ET.SubElement(world, 'geom', name='marker', type='plane', pos='3 0 .55', xyaxes='0 -1 0 0 0 1',
                  size='.15 .15 .001', material='marker_material', contype='0', conaffinity='0', group='4')
    boxes = [ ('obstacle_main', (1.25, obstacle_shift, .15), (.22, .25, .15)),
              ('obstacle_side', (2.1, -.95, .15), (.2, .2, .15)),
              ('wall_back', (-.7, 0, .35), (.05, 1.6, .35)),
              ('wall_station', (3.2, 0, .35), (.05, 1.6, .35)),
              ('wall_left', (1.25, 1.6, .35), (2, .05, .35)),
              ('wall_right', (1.25, -1.6, .35), (2, .05, .35)),
               ]
    for name, pos, size in boxes:
        ET.SubElement(world, 'geom', name=name, type='box', pos=' '.join(map(str,pos)),
                      size=' '.join(map(str,size)), rgba='.65 .35 .15 1', group='5')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


class Lidar:
    angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    MAX_RANGE = 3.5
    MIN_RANGE = 0.012
    def __init__(self, model):
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'lidar') #
        self.groups = np.ones(6, dtype=np.uint8)

    def read(self, model, data, rng, sigma_rho=0.02):
        T = data.site_xpos[self.site]                  # LiDAR translation vector in world coordinates
        R = data.site_xmat[self.site].reshape(3, 3)    # Lidar rotation matrix in world coordinates

        distances = []
        for angle in self.angles:
            direction = R @ np.array([np.cos(angle),np.sin(angle),0.0])                  # Direction to cast ray
            hit = mujoco.mj_ray(model, data, T, direction,self.groups, True, -1, None)   # Simulate ray cast in that direction
                                                                                         # -1 if nothing is detected
            if hit < 0 or hit > self.MAX_RANGE:                                                                        
                distance = self.MAX_RANGE
            else:
                distance = hit
            distance += rng.normal(0, sigma_rho)                                         # Adds noise
            distance = np.clip(distance, self.MIN_RANGE, self.MAX_RANGE)                 # Constrains distance to limits
            distances.append(distance)
        return np.asarray(distances)

def evaluation_pose(model, data):
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
    rotation = data.xmat[base].reshape(3,3)
    return np.array([*data.xpos[base,:2], np.arctan2(rotation[1,0], rotation[0,0])])


def evaluation_collision(model, data):
    for contact in data.contact:
        if model.geom_group[contact.geom1] == 5 or model.geom_group[contact.geom2] == 5:
            return True
    return False

class CalibratedMarkerCamera:

    focal = HEIGHT / (2 * np.tan(np.deg2rad(FOV) / 2))

    def __init__(self):
        self.cam = CentralCamera( f=self.focal, imagesize=[WIDTH, HEIGHT], # Setup camera
                                  pp=[(WIDTH - 1) / 2, (HEIGHT - 1) / 2],
                                  name='camera')
        
    def observe(self, rgb):
        im = Image(rgb)
        fids = im.fiducial(dict='4x4_50', K=self.cam.K, side=MARKER_SIZE)  # Detect and estimate arUco pose
        if not fids:
            return None, None, None
        pose = fids[0].pose
        if not np.isfinite(pose.t).all() or not np.isfinite(pose.R).all(): # Checks if pose values are valid
          return None, None, None
        x, y, z = pose.t

        pitch = np.deg2rad(CAMERA_TILT_DEGREES)
        horizontal_distance = (np.cos(pitch) * z + np.sin(pitch) * y) # Only the horizontal distance from the camera to the marker
        distance = np.hypot(x, horizontal_distance)
        heading = -np.arctan2(x, horizontal_distance)

        R_cm = pose.R
        R_wm = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0] ])
        R_cb = np.array([
            [0, np.sin(pitch), np.cos(pitch)],
            [-1, 0, 0],
            [0, -np.cos(pitch), np.sin(pitch)]])
        R_wc = R_wm @ R_cm.T
        R_wb = R_wc @ R_cb.T
        robot_yaw = np.arctan2(R_wb[1, 0], R_wb[0, 0])

        ids = []
        corners = []
        for fid in fids:
            ids.append(fid.id)
            corners.append(fid.corners)
        return np.array([distance, heading, robot_yaw]), corners, ids