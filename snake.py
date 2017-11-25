'''
    A Snake model that can be loaded and controlled in a PyBullet environment
'''
from model_generator import createNLinkSnake
from math import sqrt


class Snake(object):
    '''A Snake model that can be loaded and controlled in a PyBullet environment'''
    def __init__(self, n):
        createNLinkSnake(n)
        self.num_links = n
        self.joints = []
        self.my_id = -1
        self.world = None

    def load(self, world, p_x=0, p_y=0, r_x=0, r_y=0, r_z=0):
        '''Load this snake in the simulated world'''
        self.world = world
        start_pos = [p_x, p_y, .25]
        start_quaternion = world.getQuaternionFromEuler([r_x, r_y, r_z])
        self.my_id = world.loadURDF('snake.urdf', start_pos, start_quaternion)
        self.joints = [
            PDJoint(*self.world.getJointInfo(self.my_id, i)[8:10])
            for i in range(self.world.getNumJoints(self.my_id))
        ]

    def set_joint_pos_vertical(self, joint_ind, dest):
        '''
        Set the joint position to be used in the PD-controller.
        This does **not** set torque values.
        '''
        if joint_ind >= len(self.joints):
            # TODO: logger
            return
        self.joints[joint_ind].set_dest_vertical(dest)

    def fix_torques(self):
        for i, joint in enumerate(self.joints):
            c_pos, c_vel, _, b = self.world.getJointState(self.my_id, i)
            # if i == 0:
            #     t = joint.calculate_torque(c_pos, c_vel)
            print(i, c_pos, c_vel, b, joint.calculate_torque(c_pos, c_vel))
            # self.world.setJointMotorControl2(
            #     self.my_id,
            #     i,
            #     self.world.POSITION_CONTROL,
            #     0.4
            #     # force=joint.calculate_torque(c_pos, c_vel)
            # )
            self.world.setJointMotorControl2(
                self.my_id,
                i,
                self.world.TORQUE_CONTROL,
                force=joint.calculate_torque(c_pos, c_vel)
            )


class PDJoint(object):
    def __init__(self, low, high):
        self.kp = 100
        self.lower_limit = low
        self.higher_limit = high
        self.dest_vertical = 0

    def set_dest_vertical(self, dest_vertical):
        if dest_vertical < self.lower_limit:
            dest_vertical = self.lower_limit
        if dest_vertical > self.higher_limit:
            dest_vertical = self.higher_limit
        self.dest_vertical = dest_vertical
        print(self.dest_vertical)

    def calculate_torque(self, cur_pos, cur_vel):
        # return 0
        return fix_max(400, (self.dest_vertical - cur_pos) * self.kp - cur_vel * self.kp/10)

def fix_max(a, b):
    if abs(b) > a:
        return a * b / abs(b)
    return b + 240 * b/abs(b)
