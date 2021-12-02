'''
File: geo.py
File Created: Wednesday, 24th November 2021 9:49:35 am
Author: LI Jiangfan 
-----
Last Modified: Wednesday, 24th November 2021 11:57:04 am
Modified By: LI Jiangfan 
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

class Pos:
    def __init__(self, x, y):
        self.x = x # Int for GridMap Pos; Float for pos in meter? for Thymio position
        self.y = y
        
    def dis(self, p):
        return math.sqrt((p.x - self.x) ** 2 + (p.y - self.y) ** 2)

    def dis_man(self, p):
        return abs(p.x - self.x) + abs(p.y - self.y)
    
    def delta_theta(self, p):
        return math.atan2(p.y - self.y, p.x - self.x)

class State:
    def __init__(self, p, t):
        self.pos = p
        self.ori = t
    
    def dis(self, s):
        return self.pos.dis(s.pos)

    def headto(self, s):
        return self.pos.delta_theta(s.pos)
    
    def delta_theta(self, s):
        return s.ori - self.ori

class GridMap: 
    # It's possible to use pixel map directly, 
    # It will be easier for Vision part, -- without merging operation
    # but more computational expensive.  -- if using A*
    # And it can be fluctuation of motion following continuous waypoints of high resolution.
    # which do you prefer?
    """Vision --set--> GridMap --> GN"""
    def __init__(self, h, w, s):
        self.height = h # x
        self.width = w  # y
        self.scale = s

    def set_goal(self, p):
        self.goal = p

    def set_start(self, p):
        self.start = p

    def set_obs(self, obslist):
        self.obs = obslist

    """functions for Map Checking"""
    def check(self, p):
        # Will it be more efficient to use Boolean Map? 
        # -- depend on the number of obstacle cells.
        for o in self.obs:
            if p == o: return False
        return True