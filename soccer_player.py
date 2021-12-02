'''
File: soccer_player.py
Project: Mobile_Soccer
File Created: Thursday, 2nd December 2021 4:03:07 pm
Author: JiangfanLi (rdemezerl@gmail.com)
Description: Main Program for mobile final project
-----
Last Modified: Thursday, 2nd December 2021 4:09:07 pm
Modified By: JiangfanLi (rdemezerl@gmail.com>)
'''

import time

from geo import *
import motion_control
import vision
import filtering
import local_navigation
import global_navigation

# -- Global Settings --
G_verbose = True
S_camera_interval = 1000 #ms
S_motion_interval = 10

S_epsilon_dis = 1
S_epsilon_theta = 0.1

# -- Controllers --
G_mc = motion_control.MotionController(S_motion_interval)
G_vision = vision.Processor()
G_filter = filtering.Filter()

# -- States --
# Positions
G_thymio_state = None
G_path = None
# timers
G_camera_timer = time.time()
G_mc_timer = time.time()

def path_tracking():
    starter = time.time()
    # 3. Routine 
    # 3.1 Where I am
    # 3.1.1 With Vision
    if starter - G_camera_timer > S_camera_interval:
        G_thymio_state = G_filter.getState(vision = True)
        G_camera_timer = starter
    # 3.1.2 Without Vision
    else:
        G_thymio_state = G_filter.getState(vision = False)

    # 3.2 Track the waypoints
    assert len(G_path) > 0, "Path is empty."
    wp = G_path[0]
    # 3.2.1 Are we close enough to the next waypoint?  
    delta_r = G_thymio_state.dis(wp)
    if delta_r < S_epsilon_dis:
        # check the rotation
        delta_theta = G_thymio_state.delta_theta(wp)
        if delta_theta < S_epsilon_theta:
            # finished
            G_path = G_path[1:]  
            if len(G_path) == 0:
                G_mc.stop()
                if G_verbose:
                    print("Path Finished")
                return
        else:
            G_mc.rotate(delta_theta) #PULSE
    else:
        # 3.2.2 Go to the next waypoint
        headto_theta = G_thymio_state.headto(wp)
        if headto_theta > S_epsilon_theta:
            G_mc.approach(delta_r, headto_theta)
        else:
            G_mc.approach(delta_r, 0)
    
    loop_time = time.time() - starter
    if G_verbose:
        print(F"looper time: {loop_time*1000 :.0f}ms")


def main():
    """Main Program"""
    # 1. initialize everythings
    # 1.1 The Map
    G_map = G_vision.getMap()
    G_ball_pos = G_vision.getBall()
    G_gate_pos = G_vision.getGate()
    G_pp = global_navigation.PathPlanner(G_map, method="A*", neighbor=8, simplify = True)
    # assert G_map is not None, "Visual Extraction Failed"
    # 1.2 Where I am
    G_thymio_state = G_filter.getState()
    # 1.3 Where will I go

    # 2. main loop of 2 tasks
    for goal in ["ball", "gate"]:
        # 2.1 Task set up
        if goal == "ball":
            G_goal_pos = global_navigation.approach(G_ball_pos)
        else:
            G_goal_pos = global_navigation.approach(G_gate_pos)
        G_pp.set_goal(G_goal_pos)
        G_pp.set_start(G_thymio_state)
        G_path = G_pp.plan()
        # 2.2 Tackle the task
        while True:
            # 2.2.1 Finished?
            if G_thymio_state.dis(G_goal_pos) < S_epsilon_dis:
                break
            # 2.2.2 Is there obstacles on the front?
            if local_navigation.obs_front():
                local_navigation.avoid()
            else:
                path_tracking()



if __name__ == "__main__":
    main()