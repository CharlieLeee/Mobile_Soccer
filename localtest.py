from Thymio import Thymio
import motion_control

import numpy as np
import time
import random
from geo import *
from vision import *
import filtering
from loguru import logger
import matplotlib.pyplot as plt

pre_state = np.array([0.0, 0.0, 0]).reshape(-1, 1) # initial state
pre_cov = np.ones([3, 3]) * 0.03 # initial covariance
G_filter = filtering.KF(pre_state, pre_cov, qx=0.1, qy=0.1, qtheta=0.3, rl=0.1, rr=0.1, b=0.0927)
G_filter.timer = time.time()

pre_state = np.array([0.0, 0.0, 0]).reshape(-1, 1) # initial state
pre_cov = np.ones([3, 3]) * 0.03 # initial covariance
G_filter = filtering.KF(pre_state, pre_cov, qx=0.1, qy=0.1, qtheta=0.3, rl=0.1, rr=0.1, b=0.0927)
G_filter.timer = time.time()

logger.info(G_filter.states)

from Thymio import Thymio
import motion_control

THYMIO_PORT = "COM5"
THYMIO_REFRESH_RATE = 1.0
G_verbose = True
S_camera_interval = 1000 #ms
S_motion_interval = 10 #ms
S_track_interval = 0.1 #s

S_epsilon_dis = 0.005
S_epsilon_theta = 0.1

G_mc = motion_control.MotionController(
    Thymio.serial(port=THYMIO_PORT, refreshing_rate=THYMIO_REFRESH_RATE), 
    S_motion_interval, eps_delta_r=S_epsilon_dis, eps_delta_theta=S_epsilon_theta, 
    max_speed=100,
    verbose=G_verbose)
G_track_timer = time.time()

#G_camera_timer = time.time()
G_track_timer = time.time()
G_mc.get_displacement()

def localizate():
    """Track Where Thymio is"""
    global G_camera_timer
    starter = G_filter.timer
    # 3. Localization 
    # 3.1 odometer
    dsl, dsr = G_mc.get_displacement()
    # 3.2 With Vision
        # if starter - G_camera_timer > S_camera_interval:
        #     vision_thymio_state = G_vision._getThymio()
        #     # Vision Failed
        #     if vision_thymio_state is None:
        #         G_filter.kalman_filter(dsr, dsl)
        #     else:
        #         G_camera_timer = starter
        #         G_filter.kalman_filter(dsr, dsl, vision_thymio_state)
        # else:        
        #     G_filter.kalman_filter(dsr, dsl)
    G_filter.kalman_filter(dsr, dsl)
    G_filter.plot_gaussian()
    thymio_state = G_filter.get_state()
    return thymio_state

print(localizate())