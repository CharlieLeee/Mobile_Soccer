'''
Author: Chengkun Li
LastEditors: Chengkun Li
Date: 2021-11-25 00:32:13
LastEditTime: 2021-12-02 19:09:02
Description: All functions pertain to localization of the thymio using Kalman Filter.
FilePath: /Mobile_Soccer/filtering.py
'''

import numpy as np
import matplotlib.pyplot as plt
from loguru import logger
from geo import *


class KF:
    """Kalman Filter Class
    The state of Thymio is encoded as [Px, Py, Rotation]
    Reference paper: https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1021.234&rep=rep1&type=pdf
    """
    def __init__(self, init_state, init_cov, qx, qy, qtheta, rl, rr, b) -> None:
        """Initialization

        Args:
            init_state (State): initial state of thymio
            init_cov (np.array): inital covariance of thymio
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

        # States and Covariance history
        self.states = [init_state]
        self.covs = [init_cov]

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
    def plot_gaussian(self, factor=100, dt=1e-2):
        
        def cov_ellipse(state, cov):
            # Covariance matrix correspond to x and y position
            Pxy = cov[0:2, 0:2]
            eigval, eigvec = np.linalg.eig(Pxy)
            
            x_idx = np.argmax(eigval)
            y_idx = np.argmin(eigval)
            
            T = np.arange(0, 2*np.pi+0.1, 0.1)
            a = math.sqrt(eigval[x_idx])
            b = math.sqrt(eigval[y_idx])
            x = [a * math.cos(t) for t in T]
            y = [b * math.sin(t) for t in T]

            angle = math.atan2(eigvec[y_idx][1], eigvec[y_idx][0])
            Rot = np.array([
                [math.cos(angle), math.sin(angle)],
                [-math.sin(angle), math.cos(angle)]
            ])
            fx = np.matmul(Rot, np.array([x, y]))
            px = np.array(fx[0, :] + state[0][0])
            py = np.array(fx[1, :] + state[1][0])
            
            return px, py

        # Plot
        fig, ax = plt.subplots()
        # ax.set_xlim([0, ])
        ax.invert_yaxis()
        # ax.set_ylim([0, ])
        ax.set_facecolor('green')
        x = np.array(self.states)[:, 0, :]
        y = np.array(self.states)[:, 1, :]
        ax.plot(y, x, '-w', \
                label='states')
        # Plot 
        
        for i in range(0, len(self.states), int(1/dt)):
            px, py = cov_ellipse(self.states[i], self.covs[i]/factor)
            
            if i == 0:
                ax.fill(py, px, alpha=0.4, facecolor='yellow', edgecolor='yellow', \
                    linewidth=1, zorder=1, label='covariances')
            ax.fill(py, px, alpha=0.4, facecolor='yellow', edgecolor='yellow', \
                    linewidth=1, zorder=1)
            # ax.plot(px, py, '--r', label='covariance at step {}'.format(i))
        ax.set_title('State of Thymio')
        ax.legend()
        fig.tight_layout()
        plt.show()
        

    @logger.catch
    def kalman_filter(self, dsr, dsl, measurement:State=None):
        
        # Get previous state
        pre_state = self.states[-1]
        pre_cov = self.covs[-1]

        theta = pre_state[-1][0] % (2*np.pi) # [x, y, theta]
        D = (dsl + dsr) / 2
        T = (dsr - dsl) / self.b
        
        # x_{t+1} = Ax_t + Bu
        A = self.__A_mat(theta, D, T)
        B = self.__B_mat(theta, D, T)


        pre_state = np.array(pre_state).reshape(-1, 1) # reshape to [3, 1]
        
        # next state calculation
        # State transition according to EKF state function
        
        # Constraints on theta+T/2 to prevent overflow
        theta_post = theta + T/2 
        Fxu = np.array([D*np.cos(theta_post), D*np.sin(theta_post), T]).reshape(-1, 1)
        #logger.info(Fxu)
        #logger.info(T)
        est_state = pre_state + Fxu
        # Constraints on T
        est_state[2][0] = est_state[2][0] % (2*np.pi)
        est_cov = np.matmul(A, np.matmul(pre_cov, A.T)) + np.matmul(B, np.matmul(self.R, B.T))

        # if measurements exist, apply filter
        if measurement:
            measurement = np.array([measurement.pos.x, measurement.pos.y, measurement.ori])
            # gain
            K = np.matmul(est_cov, np.linalg.inv(est_cov + self.Q))
            I = np.array(measurement).reshape(-1, 1) - est_state
            est_state += np.matmul(K, I)
            est_cov -= np.matmul(K, est_cov)
        
        self.states.append(est_state)
        self.covs.append(est_cov)
        
        return est_state, est_cov

    def get_state(self):
        state = self.states[-1]
        x, y, theta = state[0][0], state[1][0], state[2][0]
        return State(Pos(x, y), theta)


if __name__ == '__main__':
    # initial state
    pre_state = np.array([1, 1, 0]).reshape(-1, 1)
    # initial covariance
    pre_cov = np.ones([3, 3]) * 0.03
    # displacement in left and right wheels
    dsl = [0.3, .2, .1, .11]
    dsr = [.5, .1, .2, .3]
    measurement = [None, None, None, None]

    kf = KF(pre_state, pre_cov, qx=0.3, qy=0.3, qtheta=0.3, rl=0.1, rr=0.1, b=0.08)
    for i in range(len(dsl)):
        kf.kalman_filter(dsl[i], dsr[i], measurement[i])
        # print(kf.kalman_filter(dsl[i], dsr[i], measurement[i]))
    kf.plot_gaussian(dt=1)

    kfn = KF(pre_state, pre_cov, qx=0.3, qy=0.3, qtheta=0.3, rl=0.1, rr=0.1, b=0.08)
    for i in range(len(dsl)):
        kfn.kalman_filter(dsl[i], dsr[i])
        # print(kfn.kalman_filter(dsl[i], dsr[i]))
    kfn.plot_gaussian(dt=1)