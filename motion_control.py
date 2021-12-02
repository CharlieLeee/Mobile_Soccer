'''
File: motion_control.py
File Created: Wednesday, 24th November 2021 11:12:29 am
Author: LI Jiangfan 
-----
Last Modified: Wednesday, 24th November 2021 11:57:41 am
Modified By: LI Jiangfan 
'''

from global_navigation import *
# import filtering

global G_mc_timer

class MotionController:
    def __init__(self, time_interval = 10):
        """Control the Thymio
        
        Interface between high-level command and Thymio motion
        """
        self.interval = time_interval # ms
        
        pass

    def approach(self, delta_r, delta_theta = 0):
        """approach to the goal point"""
        # u only move <interval> ms.
        
        pass

    def rotate(self, delta_theta):
        pass

    def move(self, vel, omega):
        """
        move with transitional velocity and rotational velocity
        """
        pass

    def stop(self):
        pass

    def go_pos(self, wps):
        """Go to the position directly"""
        target = wps.get()
        # get current pos
        # if current pos is close enough to the target:
        #   Pop out this pos
        # else:
        #  turn to the desired direction
        #  Go!
        return wps

    def obs_forward(self):
        """if there's obstacles on the way forward"""
        return True # TODO
