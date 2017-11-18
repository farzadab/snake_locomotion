import pybullet as p
import pybullet_data
import numpy as np
import time
from math import *
from model_generator import createNLinkSnake

n = 20
w = pi/9
prog = pi/2

def calcCOM(pc, bodyId, numLinks):
    return np.average(
        [pc.getLinkState(bodyId, i)[0] for i in range(numLinks-1)]
        + [ pc.getBasePositionAndOrientation(bodyId)[0] ]
    )

def simulateDistanceTraveled(w, prog):
    createNLinkSnake(n)
    physicsClient = p.connect(p.DIRECT) #or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use by loadURDF
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
    com_start = calcCOM(p, snakeId, n)
    for ts in range(400):
        for i in range(nJoints):
            p.setJointMotorControl2(snakeId, i, p.POSITION_CONTROL, (hi-lo) * sin(w * ts + i*prog))
        p.stepSimulation()
    
    distance_traveled = np.linalg.norm(np.subtract(calcCOM(p, snakeId, n), com_start))
    print('%4.2f %4.2f %7.3f' %(w, prog, distance_traveled))
    return distance_traveled

all_distances = [(simulateDistanceTraveled(w, prog), w, prog) for w in np.linspace(0, pi/4, 20) for prog in np.linspace(0, pi/2, 20)]
print(sorted(all_distances)[0])