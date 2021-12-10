'''
File: motion_control.py
Author: JiangfanLi 
-------
Description: 
Motion Control with Thymio Interface
'''
from geo import *
from loguru import logger
from Thymio import Thymio
import time
import os
from dotenv import load_dotenv
load_dotenv()

class MotionController:
    def __init__(self, thymio, time_interval = 10, # ms 
                 eps_delta_r = 0.005, eps_delta_theta = 0.01,
                 max_speed = 100, 
                 speed_scale = 0.000315, # (m/s) / speed_in_motor_command; 0.000315 for speed<200; 0.0003 for speed \in (200,400)
                 rotate_scale = 0.0005, # TODO (rad/s) / speed_in_motor_command
                 obstSpeedGain = 5,  # /100 (actual gain: 5/100=0.05)
                 verbose = False
                 ):
        """Motion Controller

        Connected with thymio interface
        Interface between high-level command and Thymio motion
        """
        self.thymio = thymio            # thymio interface
        self.interval = time_interval   # s, control frequency
        self.timer = time.time()
        self.displacement = [0, 0]
        self.speed = [0, 0]
        
        self.eps_delta_r = eps_delta_r
        self.eps_delta_theta = eps_delta_theta

        self.max_speed = max_speed
        self.speed_scale = speed_scale
        self.rotate_scale = rotate_scale
        self.obstSpeedGain = obstSpeedGain

        self.verbose = verbose

    def __del__(self):
        self.thymio.terminating = True
        self.thymio.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.thymio.close()

    def close(self):
        self.thymio.close()

    # -- Local Navigation --
    def avoid(self):
        try:
            prox_horizontal = self.thymio["prox.horizontal"]
            obst = [prox_horizontal[0], prox_horizontal[4]]
            speed_left = self.max_speed + self.obstSpeedGain * (obst[0] / 100)
            speed_right = self.max_speed + self.obstSpeedGain * (obst[1] / 100) 
            self._set_motor(speed_left, speed_right)
        #if proximity sensors not found
        except KeyError:
            pass  # prox.horizontal not found

    # -- Path Tracking --        
    def path_tracking(self, waypoint, Thymio_state, theta_track = False):
        """Follow the path

        @return: waypoint reached
        """
        # 4 Track the waypoints
        # 4.1 Are we close enough to the next waypoint?  
        delta_r = Thymio_state.dis(waypoint)
        if delta_r < self.eps_delta_r:
            if self.verbose:
                print("Close to the point")
            # check the rotation
            delta_theta = Thymio_state.delta_theta(waypoint)
            if not theta_track or abs(delta_theta) < self.eps_delta_theta:
                if self.verbose:
                    print(Thymio_state,"Point Finished")
                return True
            else:
                self.rotate(delta_theta) #PULSE
        else:
            # 4.2 Go to the next waypoint
            headto_theta = Thymio_state.headto(waypoint)
            delta_theta = headto_theta - Thymio_state.ori
            delta_theta = Pos.projectin2pi(delta_theta)
            if self.verbose:
                print(F"headto_theta: {headto_theta}")
            if abs(delta_theta) > self.eps_delta_theta:#1.0:
                self.rotate(delta_theta)
            # elif abs(delta_theta) > self.eps_delta_theta:
            #     self.approach(delta_r, delta_theta)
            else:
                self.approach(delta_r, 0)
            return False

    # -- Movement --
    def approach(self, delta_r, delta_theta = 0):
        """approach to the goal point
        
            move with modification of direction
        """
        if self.verbose:
            print(F"approach to dr:{delta_r}, dt:{delta_theta}")
        # assume u only move <interval> s. 
        advance_speed = min(delta_r/self.interval/self.speed_scale * 50 + 20, self.max_speed)
        delta_speed = delta_theta/self.interval/self.rotate_scale
        if delta_speed > 0:
            delta_speed = min(delta_speed, self.max_speed/2)
            self.move(min(advance_speed, self.max_speed - 2*abs(delta_speed)), delta_speed)
        else:
            delta_speed = max(delta_speed, -self.max_speed/2)
            self.move(min(advance_speed, self.max_speed - 2*abs(delta_speed)), delta_speed)

    def rotate(self, delta_theta):
        """rotate in place
        """
        if self.verbose:
            print(F"rotate to dt:{delta_theta}")
        delta_speed = delta_theta/(self.interval)/self.rotate_scale
        if delta_speed > 0:
            self.move(0, min(delta_speed + 10, self.max_speed))
        else:
            self.move(0, max(delta_speed - 10, -self.max_speed))

    def move(self, vel, omega = 0):
        """
        move with transitional velocity and rotational velocity
        """
        if self.verbose:
            print(F"move with {vel}, {omega}")
        self._set_motor(vel - omega, vel + omega)

    def stop(self):
        """Stop both motors
        """
        self._set_motor(0, 0)

    def update_displacement(self):
        starter = time.time()
        interval = starter - self.timer
        self.timer = starter
        rls = self.thymio.get_var("motor.left.speed")
        rrs = self.thymio.get_var("motor.right.speed")
        rls = int(rls * float(os.getenv("OFFSET_WHEELS")))
        rls = rls if rls < 2 ** 15 else rls - 2 ** 16
        rrs = rrs if rrs < 2 ** 15 else rrs - 2 ** 16
        rls = 0 if abs(rls) > self.max_speed * 1.1 else rls
        rrs = 0 if abs(rrs) > self.max_speed * 1.1 else rrs 
        self.displacement[0] += rls*interval*self.speed_scale
        self.displacement[1] += rrs*interval*self.speed_scale

    def _set_motor(self, ls, rs):
        ls = (int)(ls)
        rs = (int)(rs)
        l_speed = int(ls / float(os.getenv("OFFSET_WHEELS")))
        l_speed = ls if ls >= 0 else 2 ** 16 + ls
        r_speed = rs if rs >= 0 else 2 ** 16 + rs           
        self.update_displacement()
        # logger.info(l_speed)
        self.thymio.set_var("motor.left.target", l_speed)
        self.thymio.set_var("motor.right.target", r_speed)

    def get_displacement(self):
        self.update_displacement()
        ret = self.displacement
        self.displacement = [0, 0]
        # if self.verbose:
        #     print(F"Displacement:{ret}")
        return ret

    # -- Sensor --
    def obs_front(self):
        """if there's obstacle on the way forward"""
        return True # TODO

    def acc(self):
        pass

if __name__ == "__main__":
    
    import time

    th = Thymio.serial(port="COM6",refreshing_rate=0.1)
    try:
        mc = MotionController(th)
        while True:
            try:
                if th["button.forward"]:
                    mc.move(100)
                elif th["button.left"]:
                    mc.move(100, 30)
                elif th["button.right"]:
                    mc.move(100, -30)
                elif th["button.center"]:
                    mc.stop()
                    break
                print(th["prox.horizontal"])
            except KeyError:
                pass
            time.sleep(0.2)
    finally:
        mc.close()