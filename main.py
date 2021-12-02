#import vision
from global_navigation import *
from local_navigation import obstacle_avoidance
#import filter

local = obstacle_avoidance

state = 1 # 0=gradient, 1=obstacle avoidance

#global path

while True:
    #vision -> robot pose
    #kalman -> robot pose estimation

    # acquisition from the proximity sensors to detect obstacles
    #variables = proximity sensors
    
    # tdmclient does not support yet multiple and/or in if statements:
    if state == 0:
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[0] > local.obstThrH):
            state = 1
        elif (obst[1] > local.obstThrH):
            state = 1
    elif state == 1:
        if obst[0] < local.obstThrL:
            if obst[1] < local.obstThrL : 
                # switch from obst avoidance to goal tracking if obstacle got unseen
                state = 0
    if  state == 0 :
        # goal tracking: turn toward the goal
        leds_top = [0,0,0]
        #PID CONTROLLER OF SPEED CORRECTION TOWARDS GOAL (constant speed + rot correction)
    else:
        leds_top = [30,30,30]
        # obstacle avoidance: accelerate wheel near obstacle
        local.obstacle_update(node, variables);
    
