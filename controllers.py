import numpy as np
import pybullet as p
import pybullet_data
from rllab.envs.base import Env
from rllab.spaces import Box
from snake import Snake
from random import uniform
from math import pi, sin, cos
import rllab_env
# import time
# from numpy.fft import ifft

class LLC(object):
    '''
    Base class for a Low Level Controller

      This is not really be a controller in the sense that you know,
    it's just a transformation on the input space for the robot.
    '''
    def __init__(self, num_outputs):
        self.num_outputs = num_outputs

    def num_inputs(self):
        return self.num_outputs

    def transform(self, hl_controls, step_num=0):
        '''
        Gets as input an array of high level control inputs
        and outputs the low level controls
        '''
        return hl_controls


class RotatingLLC(LLC):
    '''
    Same as the Base LLC, but tries to add a sinusoid to the data
    '''
    def transform(self, hl_controls, step_num=0):
        '''
        Gets as input an array of high level control inputs
        and outputs the low level controls but shifts them using a sinusoid based on `step_num`
        '''
        shift = np.exp(
            -2j * pi * step_num / rllab_env.SimpleSnakeEnv.STEP_DURATION
            * np.array(range(len(hl_controls)))
        )
        return (hl_controls * shift).real


class FFTController(LLC):
    '''
    A controller that uses inverse DFT to create the inputs for the model
    '''
    def num_inputs(self):
        return self.num_outputs-2

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
        # The final `.real()` shouldn't be needed here, just taking care of numerical errors
        return np.fft.ifft(whole_array).real * len(whole_array)


class RotatingFFTController(FFTController):
    '''
    Same as an FFTController, but it adds a shift to the output using the `step_num`
    It extends (rather simple-mindedly) the following simple rule in DSP:
            x[n-D]  <->  exp(-2pi*j*k/N) X[k]
    '''
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
        shift = np.exp(
            -2j * pi * step_num / rllab_env.SimpleSnakeEnv.STEP_DURATION
            * np.array(range(len(whole_array)))
        )
        # The final `.real()` shouldn't be needed here, just taking care of numerical errors
        return np.fft.ifft(whole_array * shift).real * len(whole_array)


class SimpleSinusoidController(LLC):
    '''
    Simply drives a sinusoid to the output
    '''
    def num_inputs(self):
        return 2

    def transform(self, hl_controls, step_num=0):
        '''
        Creates a sinusoid using `step_num` and the input variables:
                sin(w * step_num + i*phase)
        '''
        w, phase = hl_controls
        return np.sin(w * step_num + phase * np.array(list(range(self.num_outputs))))
