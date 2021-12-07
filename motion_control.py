'''
File: motion_control.py
Author: JiangfanLi 
-------
Description: 
Motion Control with Thymio Interface
'''
from loguru import logger
from numpy.core.numeric import load
import filtering
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
                 rotate_scale = 0.006, # TODO (rad/s) / speed_in_motor_command
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
    # !!! Please Move Local Navigation Module Here !!!
    def avoid(self):
        pass

    # -- Path Tracking --        
    def path_tracking(self, waypoint, Thymio_state, theta_track = False):
        """Follow the path

        @return: waypoint reached
        """
        # 4 Track the waypoints
        # 4.1 Are we close enough to the next waypoint?  
        delta_r = Thymio_state.dis(waypoint)
        if delta_r < self.eps_delta_r:
            # check the rotation
            delta_theta = Thymio_state.delta_theta(waypoint)
            if not theta_track or abs(delta_theta) < self.eps_delta_theta:
                if self.verbose:
                    print("Path Finished")
                return True
            else:
                self.rotate(delta_theta) #PULSE
        else:
            # 4.2 Go to the next waypoint
            headto_theta = Thymio_state.headto(waypoint)
            delta_theta = headto_theta - Thymio_state.ori
            if self.verbose:
                print(F"headto_theta: {headto_theta}")
            if abs(delta_theta) > 1.0:
                self.rotate(delta_theta)
            elif abs(delta_theta) > self.eps_delta_theta:
                self.approach(delta_r, delta_theta)
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
        advance_speed = min(1000.0*delta_r/self.interval/self.speed_scale, self.max_speed)
        delta_speed = 1000.0*delta_theta/self.interval/self.rotate_scale
        if delta_speed > 0:
            self.move(advance_speed, min(delta_speed, self.max_speed/2))
        else:
            self.move(advance_speed, max(delta_speed, -self.max_speed/2))

    def rotate(self, delta_theta):
        """rotate in place
        """
        delta_speed = delta_theta/(self.interval/1000.0)/self.rotate_scale
        if delta_speed > 0:
            self.move(0, min(delta_speed, self.max_speed))
        else:
            self.move(0, max(delta_speed, -self.max_speed))

    def move(self, vel, omega = 0):
        """
        move with transitional velocity and rotational velocity
        """
        if self.verbose:
            print(F"move with {vel}, {omega}")
        vel = min(vel, self.max_speed - 2*abs(omega))
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
        self.displacement[0] += rls*interval*self.speed_scale
        self.displacement[1] += rrs*interval*self.speed_scale

    def _set_motor(self, ls, rs):
        ls = (int)(ls)
        rs = (int)(rs)
        l_speed = int(ls / float(os.getenv("OFFSET_WHEELS")))
        l_speed = ls if ls >= 0 else 2 ** 16 + ls
        r_speed = rs if rs >= 0 else 2 ** 16 + rs
        self.update_displacement()
        logger.info(l_speed)
        self.thymio.set_var("motor.left.target", l_speed)
        self.thymio.set_var("motor.right.target", r_speed)

    def get_displacement(self):
        self.update_displacement()
        ret = self.displacement
        self.displacement = [0, 0]
        if self.verbose:
            print(F"Displacement:{ret}")
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