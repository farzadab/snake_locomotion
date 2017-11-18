import pybullet as p
import pybullet_data
import numpy as np
import time
from math import *
from model_generator import createNLinkSnake

n = 20
w = pi/9
prog = pi/2
createNLinkSnake(n)
physicsClient = p.connect(p.GUI) #or​ ​p.DIRECT​ ​for​ ​non-graphical​ ​version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use​ ​by​ ​loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,.25]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
snakeId = p.loadURDF('snake.urdf', cubeStartPos, cubeStartOrientation)
p.getJointInfo(snakeId, 0)
nJoints = p.getNumJoints(snakeId)
lo, hi = p.getJointInfo(snakeId, 0)[8:10]
pos = [0] * nJoints
# snakeId = p.loadURDF('sphere2.urdf', cubeStartPos, cubeStartOrientation)

for ts in range(400):
    for i in range(nJoints):
        p.setJointMotorControl2(snakeId, i, p.POSITION_CONTROL, (hi-lo) * sin(w * ts + i*prog))
    p.stepSimulation()
    time.sleep(0.05)
