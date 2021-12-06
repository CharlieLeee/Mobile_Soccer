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

from geo import *
import thymio_interface
import motion_control
import vision
import filtering
import global_navigation

# -- Global Settings --
G_verbose = True
S_camera_interval = 1000 #ms
S_motion_interval = 10

S_epsilon_dis = 1
S_epsilon_theta = 0.1

S_stablize_filter_steps = 10
# -- Controllers --
G_th = thymio_interface.ThymioInterface()
G_mc = motion_control.MotionController(G_th, S_motion_interval)
G_mc.timer = time.time()
G_vision = vision.Processor()
G_filter = filtering.KF()
G_filter.timer = time.time()

# -- States --
# timers
G_camera_timer = time.time()

def localizate():
    """Track Where Thymio is"""
    global G_camera_timer
    starter = G_filter.timer
    # 3. Localization 
    # 3.1 With Vision
    if starter - G_camera_timer > S_camera_interval:
        vision_thymio_state = G_vision.getThymio()
        G_camera_timer = starter
        thymio_state = G_filter.getState(vision_thymio_state)
    # 3.2 Without Vision
    else:
        thymio_state = G_filter.getState()
    return thymio_state

def main():
    """Main Program"""
    # 1. initialize everythings
    # 1.1 The Map
    G_map = G_vision.getMap()
    Ball_pos = G_vision.getBall()
    Gate_pos = G_vision.getGate()
    vision_thymio_state = G_vision.getThymio()
    G_pp = global_navigation.PathPlanner(G_map, method="A*", neighbor=8, simplify = True)
    # 1.2 Where I am
    for _ in range(S_stablize_filter_steps):
        Thymio_state = G_filter.getState(vision_thymio_state)

    # 2. main loop of 2 tasks
    for goal in ["ball", "gate"]:
        # 2.1 Task set up
        if goal == "ball":
            Goal_state = global_navigation.approach(Ball_pos)
        else:
            Goal_state = global_navigation.approach(Gate_pos)
        G_pp.set_goal(Goal_state.pos)
        G_pp.set_start(Thymio_state)
        path = G_pp.plan()
        Global_path = global_navigation.assign_ori(path, Goal_state.ori)
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
                G_mc.avoid() # do loval navigation for, like, 10ms
                # TODO: replan
            else:
                # 4. Follow the path
                reached = G_mc.path_tracking(Global_path[0], Thymio_state)
                if reached:
                    Global_path = Global_path[1:]
                    # assume Global_path is not empty because of 2.2.1
            loop_time = time.time() - starter
            # TODO: how long does it?
            #       then, which motor speed fit the period time
            #             should we stop to calibrate pose?
            if G_verbose:
                print(F"looper time: {loop_time*1000 :.0f}ms")


if __name__ == "__main__":
    main()