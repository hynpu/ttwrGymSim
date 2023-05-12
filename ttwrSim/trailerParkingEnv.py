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
from helper import *
from vehicleParams import *
from bicycleModel import *

class TrailerParkingEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}
    
    def __init__(self) -> None:

        self.xAxis = [0, 30]
        self.yAxis = [0, 30]
        self.maxSteerAngle = maxSteerAngle
         
        self.action_space = spaces.Box(low=-self.maxSteerAngle,
                                       high=self.maxSteerAngle, shape=(1,))
        self.observation_space = spaces.Box(low=self.low_state,
                                            high=self.high_state)
        
        self.vehicle = bicycleModel()

    def step(self, action):
        done = False
        velo, delta = action
        # saturate the steering angle
        if delta <= -self.maxSteerAngle:
            delta = -self.maxSteerAngle
        elif delta >= self.maxSteerAngle:
            delta = self.maxSteerAngle
        
        # update the state
        self.vehicle.move(velo, delta)

    def isDone():
        pass