
import numpy as np
import os
import gym
from gym import error, spaces
from gym import utils
from gym.utils import seeding
import scipy.integrate as spi
import matplotlib.pyplot as plt
import time
import random
import helper as helper


class bicycleModel():
    def __init__(self, x1 = 0, y1= 0, theta1 = 0, phi = 0, L1= 3, L2= 1.5, L3= 1.5, dt= 0.1):

        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.dt = dt

        # init input
        self.velo = 0
        self.delta = 0

        # host states
        self.x1 = x1
        self.y1 = y1
        self.theta1 = theta1

        # host-trailer angle
        self.phi = phi

        # trailer states
        self.theta2 = phi + self.theta1
        self.x2 = self.x1 - L2 * np.cos(self.theta1) - L3 * np.cos(self.theta2)
        self.y2 = self.y1 - L2 * np.sin(self.theta1) - L3 * np.sin(self.theta2)

        self.state = np.array([self.x1, self.y1, self.theta1, self.x2, self.y2, self.theta2, self.phi])

    def move(self, velo, delta):
        self.velo = velo
        self.delta = delta

        # host vehicle state update 
        x1_dot = self.velo * np.cos( self.theta1 )
        y1_dot = self.velo * np.sin( self.theta1 )
        theta1_dot = self.velo * np.tan( delta ) / self.L1
        # trailer vehicle state update
        x2_dot = self.velo * np.cos(self.phi) * \
            (1 - self.L2 / self.L1 * np.tan(self.phi) * np.tan(delta)) * np.cos(self.theta2)
        y2_dot = self.velo * np.cos(self.phi) * \
            (1 - self.L2 / self.L1 * np.tan(self.phi) * np.tan(delta)) * np.sin(self.theta2)
        theta2_dot = -self.velo * \
            (np.sin(self.phi)/self.L3 + self.L2/(self.L1*self.L3) * np.cos(self.phi) * np.tan(delta))

        # host states
        self.x1 += x1_dot * self.dt
        self.y1 += y1_dot * self.dt
        self.theta1 = helper.wrapToPi(theta1_dot * self.dt + self.theta1)
        self.x2 += x2_dot * self.dt
        self.y2 += y2_dot * self.dt
        self.theta2 = helper.wrapToPi(theta2_dot * self.dt + self.theta2)
        self.phi = helper.wrapToPi(self.theta2 - self.theta1)

        self.state = np.array([self.x1, self.y1, self.theta1, self.x2, self.y2, self.theta2, self.phi])

    def reset(self):
        pass