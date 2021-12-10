'''
File: soccer_player.py
Project: Mobile_Soccer
File Created: Thursday, 2nd December 2021 4:03:07 pm
Author: JiangfanLi (rdemezerl@gmail.com)
Description: Main Program for mobile final project
-----
Last Modified: Thursday, 2nd December 2021 7:04:42 pm
Modified By: JiangfanLi (rdemezerl@gmail.com>)
'''


import time
import numpy as np

from geo import *
from Thymio import Thymio
import motion_control
import vision
import filtering
import global_navigation

# -- Global Settings --
THYMIO_PORT = "COM5"
THYMIO_REFRESH_RATE = 1.0
G_verbose = True
S_camera_interval = 1000 #ms
S_motion_interval = 10 #ms
S_track_interval = 1.0 #s

S_epsilon_dis = 1
S_epsilon_theta = 0.1

S_stablize_filter_steps = 10
# -- Controllers --
G_mc = motion_control.MotionController(
    Thymio.serial(port=THYMIO_PORT, refreshing_rate=THYMIO_REFRESH_RATE), 
    eps_delta_r=S_epsilon_dis, eps_delta_theta=S_epsilon_theta, 
    time_interval = S_motion_interval, verbose=G_verbose)
G_mc.timer = time.time()
G_vision = vision.VisionProcessor()
G_vision.open()
pre_state = np.array([1, 1, 0]).reshape(-1, 1) # initial state
pre_cov = np.ones([3, 3]) * 0.03 # initial covariance
G_filter = filtering.KF(pre_state, pre_cov, qx=0.1, qy=0.1, qtheta=0.3, rl=0.1, rr=0.1, b=0.093)
G_filter.timer = time.time()

# -- States --
# timers
G_camera_timer = time.time()
G_track_timer = time.time()

def localizate():
    """Track Where Thymio is"""
    global G_camera_timer
    starter = G_filter.timer
    # 3. Localization 
    # 3.1 odometer
    dsl, dsr = G_mc.get_displacement()
    # 3.2 With Vision
    if starter - G_camera_timer > S_camera_interval:
        vision_thymio_state = G_vision._getThymio()
        # Vision Failed
        if vision_thymio_state is None:
            G_filter.kalman_filter(dsr, dsl)
        else:
            G_camera_timer = starter
            G_filter.kalman_filter(dsr, dsl, vision_thymio_state)
    else:        
        G_filter.kalman_filter(dsr, dsl)
    
    thymio_state = G_filter.get_state()
    return thymio_state

def main():
    """Main Program"""
    # 1. initialize everythings
    # 1.1 The Map
    G_map = G_vision.getMap()
    Ball_pos = G_vision.getBall()
    Gate_pos = G_vision.getGate()
    vision_thymio_state = G_vision._getThymio()
    while vision_thymio_state is None:
        vision_thymio_state = G_vision._getThymio()
    G_pp = global_navigation.PathPlanner(G_map, method="A*", neighbor=8, simplify = True)
    # 1.2 Where I am
    for _ in range(S_stablize_filter_steps):
        Thymio_state = G_filter.kalman_filter(0, 0, vision_thymio_state)

    # 2. main loop of 2 tasks
    for goal in ["ball", "gate"]:
        # 2.1 Task set up
        if goal == "ball":
            Goal_state = G_pp.approach(Ball_pos)
        else:
            Goal_state = G_pp.approach(Gate_pos)
        G_pp.set_goal(Goal_state.pos)
        G_pp.set_start(Thymio_state)
        path = G_pp.plan()
        Goal_state = Goal_state.multiply(G_map.scale)
        Global_path = G_pp.assign_ori(path, Goal_state.ori)
        # 2.2 Tackle the task
        while True:
            starter = time.time()
            # 3. Localization
            Thymio_state = localizate()
            # 2.2.1 Finished?
            if Thymio_state.dis(Goal_state) < S_epsilon_dis \
                and abs(Thymio_state.ori - Goal_state.ori) < S_epsilon_theta:
                if G_verbose:
                    print("Terminate Reached!")
                break
            # 2.2.2 Is there obstacles on the front?
            if G_mc.obs_front():
                G_mc.avoid() # do local navigation for, like, 10ms
                # TODO: replan
            else:
                if starter - G_track_timer > S_track_interval:
                    # 4. Follow the path    # <-- The only task can run under low frequency
                    reached = G_mc.path_tracking(Global_path[0], Thymio_state, len(Global_path) == 1)
                    if reached:
                        Global_path = Global_path[1:]
                        # assume Global_path is not empty because of 2.2.1
                    G_track_timer = starter
            loop_time = time.time() - starter
            # TODO: how long does it takes? -- about 1ms
            #       then, which motor speed fit the period time
            #             should we stop to calibrate pose?
            if G_verbose:
                print(F"looper time: {loop_time*1000 :.0f}ms")


if __name__ == "__main__":
    try:
        main()
    finally:
        G_mc.close()