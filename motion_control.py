'''
File: motion_control.py
Author: JiangfanLi 
-------
Description: 
Motion Control with Thymio Interface
'''

import filtering
from Thymio import Thymio
import time

class MotionController:
    def __init__(self, thymio, time_interval = 0.1, # s 
                 eps_delta_r = 1, eps_delta_theta = 0.1,
                 max_speed = 100, 
                 speed_scale = 0.000315, # (m/s) / speed_in_motor_command; 0.000315 for speed<200; 0.0003 for speed \in (200,400)
                 rotate_scale = 0.01, # TODO (rad/s) / speed_in_motor_command
                 ):
        """Motion Controller

        Connected with thymio interface
        Interface between high-level command and Thymio motion
        """
        self.thymio = thymio            # thymio interface
        self.interval = time_interval   # ms, control frequency
        self.timer = time.time()
        self.displacement = [0, 0]
        self.speed = [0, 0]
        
        self.eps_delta_r = eps_delta_r
        self.eps_delta_theta = eps_delta_theta

        self.max_speed = max_speed
        self.speed_scale = speed_scale
        self.rotate_scale = rotate_scale
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
        """approach to the goal point
        
            move with modification of direction
        """
        # assume u only move <interval> s. 
        advance_speed = delta_r/self.interval/self.speed_scale, self.max_speed
        delta_speed = delta_theta/self.interval/self.rotate_scale
        self.move(advance_speed, min(delta_speed, self.max_speed/2))

    def rotate(self, delta_theta):
        """rotate in place
        """
        delta_speed = delta_theta/self.interval/self.rotate_scale
        self.move(0, min(delta_speed, self.max_speed))

    def move(self, vel, omega = 0):
        """
        move with transitional velocity and rotational velocity
        """
        vel = min(vel, self.max_speed - 2*abs(omega))
        self._set_motor(vel - omega, vel + omega)

    def stop(self):
        """Stop both motors
        """
        self._set_motor(0, 0)

    def _set_motor(self, ls, rs):
        self.thymio.set_var("motor.left.target", ls)
        self.thymio.set_var("motor.right.target", rs)
        starter = time.time()
        interval = starter - self.timer
        self.timer = starter
        for i in range(2):
            self.displacement[i] += self.speed[i]*interval
        self.speed = [ls, rs]

    def get_displacement(self):
        starter = time.time()
        interval = starter - self.timer
        self.timer = starter
        for i in range(2):
            self.displacement[i] += self.speed[i]*interval
        ret = self.displacement
        self.displacement = [0, 0]
        return ret

    # -- Sensor --
    def obs_front(self):
        """if there's obstacle on the way forward"""
        return True # TODO

    def acc(self):
        pass
