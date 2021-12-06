'''
File: motion_control.py
File Created: Wednesday, 24th November 2021 11:12:29 am
Author: LI Jiangfan 
-----
Last Modified: Wednesday, 24th November 2021 11:57:41 am
Modified By: LI Jiangfan 
'''

from global_navigation import *
from Thymio import Thymio
# load configuration
load_dotenv()

# import filtering
import filtering
global G_mc_timer

class MotionController:
    def __init__(self, thymio: Thymio, time_interval: float=10):
        """Control the Thymio
        
        Interface between high-level command and Thymio motion
        """
        self.thymio = thymio
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
        """Stop both motors
        """
        self.thymio.set_var("motor.left.target", 0)
        self.thymio.set_var("motor.right.target", 0)
        
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
