'''
File: motion_control.py
Project: Mobile_Soccer
File Created: Tuesday, 30th November 2021 5:56:18 pm
Author: JiangfanLi (rdemezerl@gmail.com)
Description: Interface of Thymio Control with motion
-----
Last Modified: Thursday, 2nd December 2021 7:05:27 pm
Modified By: JiangfanLi (rdemezerl@gmail.com>)
'''

from global_navigation import *
# import filtering

import time

class ThymioController:
    def __init__(self):
        """Thymio Controller

        Interface to Communicate with Thymio
        """
        pass

class MotionController:
    def __init__(self, time_interval = 10):
        """Control the Thymio
        
        Interface between high-level command and Thymio motion
        """
        self.thymio = ThymioController()
        self.interval = time_interval # ms
        self.timer = time.time()
        
        pass

    # -- Movement --
    def approach(self, delta_r, delta_theta = 0):
        """approach to the goal point"""
        # assume u only move <interval> ms. 
        # max speeds
        
        pass

    def rotate(self, delta_theta):
        """rotate in place
        """
        
        pass

    def move(self, vel, omega):
        """
        move with transitional velocity and rotational velocity
        """
        
        self.timer = time.time()
        pass

    def stop(self):
        pass

    # -- Sensor --
    def obs_front(self):
        """if there's obstacles on the way forward"""
        return True # TODO

    def acc(self):
        pass
