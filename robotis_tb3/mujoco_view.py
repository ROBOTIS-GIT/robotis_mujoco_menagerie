import mujoco
from mujoco.viewer import launch

# XML 모델 경로
model_path = "scene_turtlebot3_waffle_pi.xml"

# MuJoCo 모델 로드
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Viewer 실행
launch(model)