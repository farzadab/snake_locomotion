import pybullet as p
import pybullet_data
from snake import Snake
import time
from math import *

physicsClient = p.connect(p.GUI) #or​ ​p.DIRECT​ ​for​ ​non-graphical​ ​version
s = Snake(10)
s.load(p)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use​ ​by​ ​loadURDF
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
stepsize = 0.0015
w = pi/100#2*stepsize#0.10
prog = -pi/3#0.18
p.setTimeStep(stepsize)
for ts in range(floor(10/stepsize)):
    # p.resetDebugVisualizerCamera(15, 100, -50, s.calcCOM())
    # prog = pi/5 + pi/4*sin(ts / 200)
    # w = pi/50 + pi/100*sin(ts / 300)
    for i, joint in enumerate(s.joints):
        # joint.set_dest_vertical((0.02) * abs(sin(w * ts + i*prog + pi/2)))
        joint.set_dest_horizontal(sin(w * ts + i*prog))
        # p.setJointMotorControl2(snakeId, i, p.POSITION_CONTROL, (hi-lo) * sin(w * ts + i*prog))
    s.fix_torques()
    p.stepSimulation()
    time.sleep(stepsize*3/4)

# import pybullet as p
# import pybullet_data
# import numpy as np
# import time
# from math import *
# from model_generator import createNLinkSnake

# n = 20
# w = pi/10#0.10
# prog = pi/20#0.18
# createNLinkSnake(n)
# physicsClient = p.connect(p.GUI) #or​ ​p.DIRECT​ ​for​ ​non-graphical​ ​version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use​ ​by​ ​loadURDF
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# cubeStartPos = [0,0,.25]
# cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# snakeId = p.loadURDF('snake.urdf', cubeStartPos, cubeStartOrientation)
# p.getJointInfo(snakeId, 0)
# nJoints = p.getNumJoints(snakeId)
# lo, hi = p.getJointInfo(snakeId, 0)[8:10]
# pos = [0] * nJoints
# # snakeId = p.loadURDF('sphere2.urdf', cubeStartPos, cubeStartOrientation)

# for ts in range(1000):
#     for i in range(nJoints):
#         p.setJointMotorControl2(snakeId, i, p.POSITION_CONTROL, (hi-lo) * sin(w * ts + i*prog))
#     p.stepSimulation()
#     time.sleep(0.05)
