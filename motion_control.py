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

class MotionController:
    def __init__(self):
        """Control the Thymio
        
        Interface between high-level command and Thymio motion
        """
        
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
