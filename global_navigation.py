'''
File: global_navigation.py
File Created: Wednesday, 24th November 2021 9:59:22 am
Author: LI Jiangfan 
-----
Last Modified: Wednesday, 24th November 2021 11:57:25 am
Modified By: LI Jiangfan 
-----
Description:
Global Navigatio Module: Return a feasible path from the start to the goal.
'''
from geo import *

from queue import Queue

class PathPlanner:
    def __init__(self, map = None, method = "A*", neighbor = 4, prune = False): 
        """tell me the map, I will give u a path

        u can use 'set_map' to tell me the map, 
        and modify it with 'set_goal' and 'set_start';
        or specify the map when calling 'plan'
        """
        self.map = map
        self.method = method
        self.neighbor = neighbor
        self.pruned = prune
        # self.settings = settings # TODO
    
    def set_map(self, map):
        self.map = map
    
    def set_goal(self, p):
        self.map.goal = p

    def set_start(self, p):
        self.map.start = p

    #def add_goal(self, p)
    #def add_obs(self, p, s)

    def plan(self, map = None):
        """return a path from the start to the goal
        
        @param map: GridMap with all the info(start, goal, obstacles)
        @return waypoints: Queue[Pos] 
            Note: If no feasible path found, the list will be empty
        """
        if map is None:
            map = self.map
        self.map = map
        if self.method == "A*":
            return self._a_star()
        elif self.method == "Greedy":
            pass
        elif self.method == "biRRT":
            pass
        else:
            print("Warning! unknown method of path planning, using default instead.")
            self.method = "A*"
            return self.plan(map)

    def _a_star(self):
        pass
        if self.pruned:
            pass

    def prune(self, waypoints):
        """merge waypoints to several nodes
        
        Provide staight lines for motion
        """
        pass

    """Map Search Operations"""

    def __get_neighbors(self, p):
        ava = [p.x > 0, p.x < self.map.height, p.y > 0, p.y < self.map.width]
        bias = [(-1, 0),(1, 0),(0, -1),(0, 1)]
        if self.neighbor == 4:
            ret = []
            for i in range(4):
                if ava[i]:
                    ret.append(Pos(p.x+bias[i][0], p.y+bias[i][1]))
            return ret
        elif self.neighbor == 8:
            ava += [ava[0] and ava[2], ava[0] and ava[3], ava[1] and ava[2], ava[1] and ava[3]]
            bias += [(-1, -1),(-1, 1),(1, -1),(1, 1)]
            ret = []
            for i in range(8):
                if ava[i]:
                    ret.append(Pos(p.x+bias[i][0], p.y+bias[i][1]))
            return ret            
        else:
            print("Warning! unsupported neighbor number, using default instead.")
            self.neighbor = 4
            return self.get_neighbors(p)

    def __check(self, p):
        return self.map.check(p)

    def __get_heuristic(self, p):
        return self.map.goal.dis(p)