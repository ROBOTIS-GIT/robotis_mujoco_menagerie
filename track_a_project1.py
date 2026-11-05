import argparse

import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path

from robotis_tb3.track_a_sensors import CalibratedMarkerCamera, Lidar, evaluation_pose
from robotis_tb3.turtlebot_model import WheelIO, load_scene

SCENE_PATH = Path(__file__).resolve().parent / "robotis_tb3" / "project_scene.xml"


def run(args):

    #setup mujoco environment
    rng=np.random.default_rng(args.seed)
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data=mujoco.MjData(model)
    camera = CalibratedMarkerCamera()
    lidar = Lidar(model)

    frames = []
    with mujoco.viewer.launch_passive(model, data) as viewer: #laod the veiw for project_scene.xml
        with mujoco.Renderer(model) as renderer:              #used to capture images from the camera
                        while viewer.is_running():
                            mujoco.mj_step(model, data)
                            renderer.update_scene(data, camera="navigation")
                            image = renderer.render()
                            camera_pose_estimates, corners, ids = camera.observe(image) #needs to be checked
                            lidar_readings = lidar.read(model, data, rng)

                                #viewer.sync() #can be used to update viewer after mujoco.mj_step(model, data)

                                # currently has lidar and camera readings working in the loop. control and model
                                # need to be implemented next
                            

                                

    
def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--obstacle-shift',type=float,default=0)
    p.add_argument('--friction',type=float,default=1)
    p.add_argument( '--scene', type=str, default='scene_turtlebot3_waffle_pi.xml')
    return p.parse_args()
if __name__=='__main__':
    args = parser()
    run(args)
