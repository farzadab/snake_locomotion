import sys
import numpy as np
from math import *
from rllab_env import SimpleSnakeEnv
from controllers import SimpleSinusoidController


def simulateDistanceTraveled(ienv, w, phase):
    ienv.reset()
    com_start = ienv.snake.calc_COM()
    for _ in range(4000):
        ienv.step([w, phase])
    diff = np.subtract(ienv.snake.calc_COM(), com_start)
    distance_traveled = np.linalg.norm(diff)
    print('%4.2f %4.2f: %7.4f, [%7.3f, %7.3f]' %(w, phase, distance_traveled, diff[0], diff[1]))
    sys.stdout.flush()
    return distance_traveled


if __name__ == '__main__':
    NUM_LINKS = 10
    env = SimpleSnakeEnv(
        num_links=NUM_LINKS, logging=False, graphical=False,
        controller=SimpleSinusoidController(NUM_LINKS))
    all_distances = [
        (simulateDistanceTraveled(env, w, phase), w, phase)
        for w in np.linspace(0, pi/4, 20)[1:] for phase in np.linspace(0, pi/4, 20)
    ]
    print(sorted(all_distances, reverse=True)[0])
