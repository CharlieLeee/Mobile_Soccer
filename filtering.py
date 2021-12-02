'''
Author: Chengkun Li
LastEditors: Chengkun Li
Date: 2021-11-25 00:32:13
LastEditTime: 2021-12-02 18:37:43
Description: All functions pertain to localization of the thymio using Kalman Filter.
FilePath: /Mobile_Soccer/filtering.py
'''

import loguru
import numpy as np
import matplotlib.pyplot as plt
from loguru import logger

class KF:
    """Kalman Filter Class
    The state of Thymio is encoded as [Px, Py, Rotation]
    Reference paper: https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1021.234&rep=rep1&type=pdf
    """
    def __init__(self, qx, qy, qtheta, rl, rr, b) -> None:
        """Initialization

        Args:
            px (float): x position of thymio
            py (float): y position of Thymio
            theta (float): rotation angle of thymio (in rads)
            qx (float): variance on x localization
            qy (float): variance on y localization
            qtheta (float): variance of rotation
            rl (float): variance of left encoder
            rr (float): variance of right encoder
            b (float): distance between two wheels of robot
        """
        self.est_state = np.zeros([3, 1])
        self.est_covar = 0.01 * np.ones([3, 3])
        self.b = b # constant: distance between wheels 
        
        # self.H is omitted since H here is an eye(3)
        # Covariance matrix of state 
        self.Q = np.array([
            [qx, 0, 0],
            [0, qy, 0],
            [0, 0, qtheta]
        ])
        # Covariance matrix of measurement
        self.R = np.array([
            [rr, 0],
            [0, rl]
        ])

    def update_params(self, qx ,qy, qtheta, rl, rr):
        """Set the params of covariance matrix on the fly

        Args:
            qx (float): variance on x localization
            qy (float): variance on y localization
            qtheta (float): variance of rotation
            rl (float): variance of left encoder
            rr (float): variance of right encoder
        """
        self.Q = np.array([
            [qx, 0, 0],
            [0, qy, 0],
            [0, 0, qtheta]
        ])
        # Covariance matrix of measurement
        self.R = np.array([
            [rr, 0],
            [0, rl]
        ])
    
    def __A_mat(self, theta, D, T):
        """Matrix A of state equation

        Args:
            theta (float): current orientation of thymio
            D (float): distance displacement; calculated as the mean of left and right
            T (float): rotation displacement; calculated by the difference between two 
                encoders divided by wheel distance b

        Returns:
            [type]: [description]
        """
        A = np.array([
            [1, 0, -D * np.sin(theta + 0.5*T)],
            [0, 1, D * np.cos(theta + 0.5*T)],
            [0, 0, 1]
        ])
        return A
    
    def __B_mat(self, theta, D, T):
        """Matrix B of state equation

        Args:
            theta (float): current orientation of thymio
            D (float): distance displacement (calculated as the mean of left and right)
            T (float): rotation displacement
        Returns:
            [type]: [description]
        """
        B = np.array([
            [0.5*np.cos(theta+0.5*T)-D/(2*self.b)*np.sin(theta+0.5*T), \
                0.5*np.cos(theta+0.5*T)+D/(2*self.b)*np.sin(theta+0.5*T)],
            [0.5*np.sin(theta+0.5*T)+D/(2*self.b)*np.cos(theta+0.5*T), \
                0.5*np.sin(theta+0.5*T)-D/(2*self.b)*np.cos(theta+0.5*T)],
            [1/self.b, -1/self.b],
        ])
        return B
    @logger.catch
    def kalman_filter(self, pre_state, pre_cov, dsr, dsl, measurement=None):
        
        # Get previous state
        theta = pre_state[-1][0] # [x, y, theta]
        D = (dsl + dsr) / 2
        T = (dsr - dsl) / self.b
        
        # x_{t+1} = Ax_t + Bu
        A = self.__A_mat(theta, D, T)
        B = self.__B_mat(theta, D, T)


        pre_state = np.array(pre_state).reshape(-1, 1) # reshape to [3, 1]
        
        # next state calculation
        # State transition according to EKF state function
        Fxu = np.array([D*np.cos(theta+T/2), D*np.sin(theta+T/2), T]).reshape(-1, 1)
        est_state = pre_state + Fxu
        est_cov = np.matmul(A, np.matmul(pre_cov, A.T)) + np.matmul(B, np.matmul(self.R, B.T))

        # if measurements exist, apply filter
        if measurement:
            # gain
            K = np.matmul(est_cov, np.linalg.inv(est_cov + self.Q))
            I = np.array(measurement).reshape(-1, 1) - est_state
            est_state += np.matmul(K, I)
            est_cov -= np.matmul(K, est_cov)

        return est_state, est_cov



if __name__ == '__main__':
    # initial state
    pre_state = np.array([1, 1, 0]).reshape(-1, 1)
    # initial covariance
    pre_cov = np.ones([3, 3]) * 0.03
    # displacement in left and right wheels
    dsl = [0.5, 0.5, 0.4, 0.7]
    dsr = [0.3, 0.9, 0.9, 0.9]
    # states and covariances
    states = []
    covs = []

    kf = KF(qx=0.1, qy=0.1, qtheta=0.3, rl=0.1, rr=0.1, b=0.08)
    for i in range(len(dsl)):
        states.append(pre_state)
        covs.append(pre_cov)
        pre_state, pre_cov = kf.kalman_filter(states[-1], covs[-1], dsl[i], dsr[i])
    
    plt.plot([state[0][0] for state in states], [state[1][0] for state in states])
    
    plt.show()
    logger.info(states)
