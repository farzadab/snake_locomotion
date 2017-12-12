'''
    A Snake model that can be loaded and controlled in a PyBullet environment
'''
from model_generator import createNLinkSnake, DEFAULT_OUTPUT_FILE as SnakeUrdfFile
from math import sqrt
import numpy as np


class ObjectWorld(object):
    '''
    Just a proxy for interacting with PyBullet
    that always invokes methods with `object_id` as the first argument
    '''
    def __init__(self, world, object_id):
        self.world = world
        self.object_id = object_id

    def __wrap(self, func):
        def call_with_object_id(*args, **kwargs):
            return func(self.object_id, *args, **kwargs)
        return call_with_object_id

    def __getattr__(self, attr_name):
        if hasattr(self.world, attr_name):
            attr = getattr(self.world, attr_name)
            if callable(attr):
                return self.__wrap(attr)
            return attr


class Snake(object):
    '''
    A Snake model that can be loaded and controlled in a PyBullet environment
    '''
    def __init__(self, n):
        createNLinkSnake(n)
        self.num_links = n
        self.joints = []
        self.my_id = -1
        self.world = None

    def load(self, world, p_x=0, p_y=0, r_x=0, r_y=0, r_z=0):
        '''Load this snake in the simulated world'''
        # start_pos = [p_x, p_y, .4]
        start_pos = [p_x, p_y, .25]
        start_quaternion = world.getQuaternionFromEuler([r_x, r_y, r_z])
        self.my_id = world.loadURDF(SnakeUrdfFile, start_pos, start_quaternion)
        self.world = ObjectWorld(world, self.my_id)
        self.joints = [
            Joint(self.world, i, -1) for i in range(0, self.world.getNumJoints())
        ]

    def calcCOM(self):
        return np.average(
            [self.world.getLinkState(i)[0] for i in range(self.num_links-1)]
            + [self.world.getBasePositionAndOrientation()[0]],
            axis=0)

    def set_joint_pos_vertical(self, joint_ind, dest):
        '''
        Set the joint position to be used in the PD-controller.
        This does **not** set torque values.
        '''
        if joint_ind >= len(self.joints):
            # TODO: logger
            return
        self.joints[joint_ind].set_dest_vertical(dest)

    def set_joint_pos_horizontal(self, joint_ind, dest):
        '''
        Set the joint position to be used in the PD-controller.
        This does **not** set torque values.
        '''
        if joint_ind >= len(self.joints):
            # TODO: logger
            return
        self.joints[joint_ind].set_dest_horizontal(dest)

    def fix_torques(self):
        for joint in self.joints:
            joint.fix_torques()


class Joint(object):
    def __init__(self, world, joint_h, joint_v):
        self.world = world
        self.joints = {}
        self._set_joint('vertical', joint_v)
        self._set_joint('horizontal', joint_h)

    def _set_joint(self, orientation, joint_num):
        if joint_num != -1:
            self.joints[orientation] = {
                'index': joint_num,
                'controller': PDController(*self._get_joint_limits(joint_num))
            }

    def _get_joint_limits(self, joint_num):
        if joint_num == -1:
            return (0, 0)
        return self.world.getJointInfo(joint_num)[8:10]

    def _set_dest(self, orientation, dest):
        if orientation in self.joints:
            self.joints[orientation]['controller'].set_dest(dest)

    def set_dest_vertical(self, dest):
        self._set_dest('vertical', dest)

    def set_dest_horizontal(self, dest):
        self._set_dest('horizontal', dest)

    def fix_torques(self):
        for joint in self.joints.values():
            c_pos, c_vel, _, b = self.world.getJointState(joint['index'])
            # print(i, c_pos, c_vel, b, joint.calculate_torque(c_pos, c_vel))
            self.world.setJointMotorControl2(
                joint['index'],
                self.world.TORQUE_CONTROL,
                force=joint['controller'].calculate_torque(c_pos, c_vel)
            )


class PDController(object):
    def __init__(self, low, high):
        self.kp = 10000
        self.lower_limit = low
        self.higher_limit = high
        self.dest = 0

    def set_dest(self, dest):
        # self.dest = self.lower_limit + (self.higher_limit - self.lower_limit) * (max(-1, min(1, dest)) + 1) / 2
        # Assuming higher and lower limits are the same
        self.dest = self.higher_limit * max(-1, min(1, dest))

    def calculate_torque(self, cur_pos, cur_vel):
        # return 0
        # print(cur_pos, self.dest - cur_pos, cur_vel, sqrt(self.kp * 4 * 1), (self.dest - cur_pos) * self.kp - cur_vel * sqrt(self.kp * 4 * 1))
        return fix_max(800, (self.dest - cur_pos) * self.kp - cur_vel * sqrt(self.kp * 4 * 1))

def fix_max(a, b):
    if abs(b) > a:
        return a * b / abs(b)
    return b# + 0 * b/abs(b)
