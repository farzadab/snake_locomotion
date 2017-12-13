import numpy as np
import pybullet as p
import pybullet_data
from rllab.envs.base import Env
from rllab.spaces import Box
from snake import Snake
from random import uniform
from math import pi, sin, cos
# import time
# from numpy.fft import ifft

class LLC(object):
    '''
    Base class for a Low Level Controller
    '''
    def num_inputs(self, num_links):
        return num_links-1

    def transform(self, hl_controls, step_num=0):
        '''
        Gets as input an array of high level control inputs
        and outputs the low level controls
        '''
        return hl_controls


class FFTController(LLC):
    '''
    This is not really a controller, it's just a transformation on the input space for the robot
    '''
    # def __init__()
    def num_inputs(self, num_links):
        return num_links-3

    def transform(self, hl_controls, step_num=0):
        '''
        Uses inverse DFT to create the inputs for the robot model
        The outputs are always real values
        '''
        first_half = np.array(hl_controls[::2]) + np.array(hl_controls[1::2]) * 1j
        whole_array = np.concatenate([
            [0],
            first_half,
            [0],
            np.flip(first_half.conj(), axis=0),
            # first_half.conj(),
        ])
        shift = np.exp(-2j * pi * step_num / SimpleSnakeEnv.STEP_DURATION * np.array(range(len(whole_array))))
        # The final `.real()` shouldn't be needed here, just taking care of numerical errors
        return np.fft.ifft(whole_array * shift).real * len(whole_array)


class SimpleSnakeEnv(Env):
    '''
    An RL-Lab environment for our snake model
    By default it uses FFTController
    '''
    GOAL_ACHIEVEMENT_THRESHOLD = 1
    STEP_DURATION = 400
    def __init__(self, num_links=11, controller=None, graphical=False, stepsize=0.0015):
        # Assuming num_links is an odd value
        self.num_links = num_links
        self.graphical = graphical
        self.stepsize = stepsize
        self.snake = Snake(self.num_links)
        self.__init_world()
        self.controller = controller
        self.step_num = 0
        if controller is None:
            self.controller = FFTController()

    def __init_world(self):
        if self.graphical:
            self.p_client_id = p.connect(p.GUI)
        else:
            self.p_client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used​ ​for loading plane.urdf
        self.reset()

    def __load_model(self):
        self.snake.load(p)

    def __apply_action(self, action):
        dests = self.controller.transform(action, self.step_num)
        for i in range(self.num_links-1):
            self.snake.set_joint_pos_horizontal(i, dests[i])
        self.snake.fix_torques()
    
    def __calc_reward(self, action, last_com):
        '''
        We don't use action in this reward (at least not for now)
        '''
        if self.__is_done():
            return 100
        cur_com = self.snake.calc_COM()
        goal = self.__calc_pos_from_objective(cur_com)
        d_v = np.subtract(cur_com, last_com)[:2]
        distance_penalty = (goal ** 2).mean() / np.linalg.norm(self.objective) / 100
        velocity_reward = np.dot(goal, d_v) / np.linalg.norm(goal)
        return -1 + velocity_reward - distance_penalty

    def __is_done(self):
        diff = self.__calc_pos_from_objective(self.snake.calc_COM()) ** 2
        return diff.mean() < SimpleSnakeEnv.GOAL_ACHIEVEMENT_THRESHOLD

    def step(self, action):
        """
        Run one timestep of the environment's dynamics. When end of episode
        is reached, reset() should be called to reset the environment's internal state.
        Input
        -----
        action : an action provided by the environment
        Outputs
        -------
        (observation, reward, done, info)
        observation : agent's observation of the current environment
        reward [Float] : amount of reward due to the previous action
        done : a boolean, indicating whether the episode has ended
        info : a dictionary containing other diagnostic information from the previous action
        """
        self.step_num += 1
        last_com = self.snake.calc_COM()
        self.__apply_action(action)
        p.stepSimulation()
        rew = self.__calc_reward(action, last_com)
        if self.step_num % 300 == 0:
            print(self.objective, self.snake.calc_COM()[:2], rew)
        return self.__observe(), rew, self.__is_done(), {}

    def __create_random_objective(self):
        radius = uniform(3, 7)  # TODO: get these as parameters
        theta = uniform(-pi/2, pi/2)
        return [radius * sin(theta), radius * cos(theta)]

    def __calc_pos_from_objective(self, pos):
        return np.subtract(pos[:2], self.objective)

    def reset(self):
        """
        Resets the state of the environment, returning an initial observation.
        Outputs
        -------
        observation : the initial observation of the space. (Initial reward is assumed to be 0.)
        """
        print('env was reset')
        self.step_num = 0
        p.resetSimulation()
        p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -10)
        p.setTimeStep(self.stepsize)
        self.__load_model()
        self.objective = self.__create_random_objective()
        return self.__observe()

    @property
    def action_space(self):
        """
        Returns a Space object
        """
        # Assuming num_links is an odd value
        return Box(low=-1, high=1, shape=(self.controller.num_inputs(self.num_links),))

    def __observe(self):
        def clean_and_reorient(state):
            p, v = state
            return self.__calc_pos_from_objective(p), v[:2]
        link_states = [
            clean_and_reorient(x)
            for x in self.snake.get_link_states()
        ]
        return np.concatenate([
            np.array(link_states).flatten(),
            [self.step_num % SimpleSnakeEnv.STEP_DURATION],
        ])

    @property
    def observation_space(self):
        """
        Returns a Space object
        """
        return Box(low=-np.inf, high=np.inf, shape=(2*2*self.num_links+1,))
