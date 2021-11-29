'''
File: global_navigation.py
File Created: Wednesday, 24th November 2021 9:59:22 am
Author: LI Jiangfan 
-----
Last Modified: Monday, 29th November 2021 1:36:22 pm
Modified By: LI Jiangfan 
-----
Description:
Global Navigatio Module: Return a feasible path from the start to the goal.
'''
from geo import *

from queue import PriorityQueue
import numpy as np

class PathPlanner:
    def __init__(self, map = None, method = "A*", neighbor = 4, path_simplification = False): 
        """tell me the map, I will give u a path

        u can use 'set_map' to tell me the map, 
        and modify it with 'set_goal' and 'set_start';
        or specify the map when calling 'plan'
        """
        self.map = map
        self.method = method
        self.neighbor = neighbor
        self.path_simplification = path_simplification
        # self.settings = settings # TODO
    
    def set_map(self, map):
        self.map = map
    
    def set_goal(self, p):
        self.map.goal = p

    def set_start(self, p):
        self.map.start = p

    #def add_goal(self, p)
    #def add_obs(self, p, s)

    def enlarge_obs(self):
        """
        enlarge the obstacles 
        considering the size of Thymio
        """
        assert self.map is not None
        num = Thymio_Size / self.map.scale
        self.obs = [[False] * self.map.height] * self.map.width
        def ava(x, y):
            return x >=0 and x<self.map.height and y>=0 and y<self.map.width
        for i in range(self.map.height):
            for j in range(self.map.width):
                if not self.map.check(Pos(i,j)):
                    self.obs[i, j] = True
                    for m in range(-num, num):
                        for n in range(-num, num):
                            if ava(m, n):
                                self.obs[m,n] = True

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
        qps = PriorityQueue()
        gcost = [[math.inf]*self.map.height]*self.map.width
        gcost[self.map.start.x, self.map.start.y] = 0
        parent = [[math.inf]*self.map.height]*self.map.width
        parent[self.map.start.x, self.map.start.y] = self.map.start        
        def heuristic(p):
            return gcost[p.x, p.y] + p.dis(self.map.goal)
        def check(p):
            for pn in self.__get_neighbors(p):
                newcost =  gcost[p.x, p.y] + 1
                if self.__check(pn) and gcost[pn.x, pn.y] > newcost:
                    gcost[pn.x, pn.y] = newcost
                    parent[pn.x, pn.y] = p
                    qps.put((heuristic(pn, heuristic(pn))))

        qps.put((heuristic(self.map.start, heuristic(self.map.start))))
        while qps.not_empty:
            p = qps.get()
            if p == self.map.goal:
                w = self.map.goal
                waypoints = [w]
                while w != self.map.start:
                    w = parent[w.x, w.y]
                    waypoints.append(w)
                break
            else:
                check(p)
        waypoints = self.collect_wps(waypoints)
        if self.path_simplification:
            waypoints = self.path_simplification(waypoints)
        # Note: waypoints need to be reversed
        return waypoints

    def collect_wps(self, waypoints, eps = 1e-3):
        """merge waypoints in the same direction
        """
        p1 = waypoints[0]
        nwps = [p1]
        p2 = waypoints[1]
        i = 2
        while i < len(waypoints):
            p3 = waypoints[i]
            if abs((p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x)) > eps:
                nwps.append(p2)
            p1 = p2
            p2 = p3
            i += 1
        nwps.append(p2)
        return nwps

    def path_simplification(self, waypoints):
        """eliminate waypoints into several nodes
        
        Provide less lines for motion
        """
        def obsinbetween(p1, p2):
            if abs(p1.x - p2.x) > abs(p1.y - p2.y):
                s = abs(1.0*(p1.y - p2.y)/(p1.x - p2.x))
                if p1.x > p2.x:
                    ps = p1
                    pe = p2
                else:
                    ps = p2
                    pe = p1
                for i in range(1, pe.x - ps.x):
                    if not self.__check(Pos(ps.x + i, (int)(i*s + ps.y))):
                        return True
            else:
                s = abs(1.0*(p1.x - p2.x)/(p1.y - p2.y))
                if p1.y > p2.y:
                    ps = p1
                    pe = p2
                else:
                    ps = p2
                    pe = p1
                for i in range(1, pe.y - ps.y):
                    if not self.__check(Pos((int)(i*s + ps.x), ps.y + i)):
                        return True
            return False

        # grand-parent-child
        grand = waypoints[0]
        nwps = [grand]
        parent = waypoints[1]
        i = 2
        while i < len(waypoints):
            child = waypoints[i]
            if obsinbetween(grand, child):
                nwps.append(parent)
                grand = parent
            parent = child
        nwps.append(child)
        return nwps

        # TODO: rdp?

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
            return self.__get_neighbors(p)

    def __check(self, p):
        return not self.obs[p.x, p.y]
    
