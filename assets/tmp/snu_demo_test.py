import pybullet as p
import time
import pybullet_data
import sim

# 기본 세팅
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])


boxId  = p.loadURDF("assets/dummy/dummy_tip.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)
tool_tip_pose = p.getLinkState(boxId,1)[0:2]
_base         = p.getLinkState(boxId,2)[0:2]

# hole pose  더미 구멍 위치
a = sim.SphereMarker(position=tool_tip_pose[0], radius=0.01, orientation=tool_tip_pose[1])

# base pose  더미 load시 기본 위치
c = sim.SphereMarker(position=_base[0], radius=0.03, rgba_color = (0, 0, 1, 0.8), orientation=tool_tip_pose[1])
# b = sim.SphereMarker(position=cubeStartPos, radius=0.01,rgba_color = (0, 1, 0, 0.8), orientation=cubeStartOrientation)
while True:
    p.stepSimulation()
    time.sleep(1./240.)