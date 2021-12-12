'''
File: geo.py
Author: LI Jiangfan 
-----
Description:
Tools Classes for map, ...
'''

"""
 ------> y
 |
 |
 V x
"""
import math
import numpy as np

Thymio_Size = 0.08 # length from the center to the front, assuming it's the collision radius.
Ball_Size = 0.04 # radius

class Pos:
    def __init__(self, x, y):
        """2D Position"""
        self.x = x 
        self.y = y
        
    def dis(self, p):
        """The distance to another point(euclidean metric)
        """
        return math.sqrt((p.x - self.x) ** 2 + (p.y - self.y) ** 2)

    def dis_man(self, p):
        """The distance to another point(Manhattan)"""
        return abs(p.x - self.x) + abs(p.y - self.y)
    
    def delta_theta(self, p):
        """The direction to another point (-pi, pi]"""
        return math.atan2(p.y - self.y, p.x - self.x)

    def multiply(self, f):
        return Pos(self.x*f, self.y*f)

    @staticmethod
    def projectin2pi(t):
        """project to (-pi, pi]"""
        t = t % (2.0*math.pi)
        if t > math.pi:
            t -= 2*math.pi
        elif t <= -math.pi:
            t += 2*math.pi
        return t

    @staticmethod
    def portion(p1, p2, alpha):
        """return the middle point between p1 and p2

        @param alpha: the weight of p1. [0.0, 1.0]
        """
        if alpha < 0: alpha = 0.0
        elif alpha > 1: alpha = 1.0
        return Pos((int)(p1.x*alpha + p2.x*(1-alpha)), (int)(p1.y*alpha + p2.y*(1-alpha)))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __lt__(self, other):
        if self.x == other.x:
            return self.y <= other.y
        return self.x <= other.x

    def __str__(self):
        return F"Pos({self.x}, {self.y})"
        
class State:
    def __init__(self, p, t):
        """2D State/Pose = (Position, Orientation)"""
        self.pos = p
        self.ori = t
    
    def dis(self, s):
        """The distance to another state

        same as the `dis` of `Pos`
        """
        return self.pos.dis(s.pos)

    def headto(self, s):
        """The direction to another state
        
        same as the `delta_theta` of `Pos`
        """
        return self.pos.delta_theta(s.pos)
    
    def delta_theta(self, s):
        """The difference of orientation to another state
        """
        dt = s.ori - self.ori
        return Pos.projectin2pi(dt)

    def multiply(self, f):
        return State(self.pos.multiply(f), self.ori)

    def __str__(self) -> str:
        return F"State({self.pos.x}, {self.pos.y}, {self.ori})"


class GridMap: 
    # It's possible to use pixel map directly, 
    # It will be easier for Vision part, -- without merging operation
    # but more computational expensive.  -- if using A*
    # And it can be fluctuation of motion following continuous waypoints of high resolution.
    # which do you prefer?
    """Vision --set--> GridMap --> GN"""
    def __init__(self, h, w, s, obs_map = None):
        self.height = h # height - x
        self.width = w  # width - y
        self.scale = s # length of one pixel in meters
        self.obs_map = obs_map
        if obs_map is None:
            self.obs_map = np.zeros((h, w))

    def set_goal(self, s):
        self.goal = s

    def set_start(self, s):
        self.start = s

    def set_obs(self, obslist):
        self.obs = obslist
        for o in self.obs:
            self.obs_map[o.x, o.y] = True

    """functions for Map Checking"""
    def check(self, p):
        if p.x < 0 or p.x >= self.height:
            return False
        if p.y < 0 or p.y >= self.width:
            return False
        return not self.obs_map[p.x, p.y]

if __name__ == "__main__":
    s1 = State(Pos(0,0), 0.0)
    s2 = State(Pos(3,4), 0.2)
    print(F"s1:{s1}, s2:{s2}")
    print(F"s1.dis(s2):{s1.dis(s2)}")
    print(F"s1.delta_theta(s2):{s1.delta_theta(s2)}")
    print(F"s1.headto(s2):{s1.headto(s2)}")