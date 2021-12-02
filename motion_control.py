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

from thymio_interface import ThymioInterface

import time

class MotionController:
    def __init__(self, thymio, time_interval = 10, eps_delta_r = 1, eps_delta_theta = 0.1):
        """Motion Controller

        Connected with thymio interface
        Interface between high-level command and Thymio motion
        """
        self.thymio = thymio            # thymio interface
        self.interval = time_interval   # ms, control frequency
        self.timer = time.time()
        
        self.eps_delta_r = eps_delta_r
        self.eps_delta_theta = eps_delta_theta
        pass

    # -- Local Navigation --
    # !!! Please Move Local Navigation Module Here !!!
    def avoid(self):
        pass

    # -- Path Tracking --
        
    def path_tracking(self, waypoint, Thymio_state, verbose = False):
        """Follow the path

        @return: waypoint reached
        """
        # 4 Track the waypoints
        # 4.1 Are we close enough to the next waypoint?  
        delta_r = Thymio_state.dis(waypoint)
        if delta_r < self.eps_delta_r:
            # check the rotation
            delta_theta = Thymio_state.delta_theta(waypoint)
            if delta_theta < self.eps_delta_theta:
                if verbose:
                    print("Path Finished")
                return True
            else:
                self.rotate(delta_theta) #PULSE
        else:
            # 4.2 Go to the next waypoint
            headto_theta = Thymio_state.headto(waypoint)
            if headto_theta > self.eps_delta_theta:
                self.approach(delta_r, headto_theta)
            else:
                self.approach(delta_r, 0)
        return False

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
