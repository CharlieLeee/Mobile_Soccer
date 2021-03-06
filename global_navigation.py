'''
File: global_navigation.py
Author: LI Jiangfan
-----
Description:
Global Navigation Module:
* Return a feasible path from the start to the goal.
    * A*(optimal)
    * RRT(not optimal, but faster)
* Path Simplification
* Calculate Approach Pose
'''
from math import atan2, sin
from geo import *

from queue import PriorityQueue
import numpy as np
import random

import matplotlib.pyplot as plt

class PathPlanner:
    def __init__(self, map = None, method = "A*", neighbor = 4, path_simplification = False, plot = False):
        """tell me the map, I will give u a path

        u can use `set_map` to tell me the obstacle map,
        and modify it with `set_goal` and 'set_start';
        then call `plan` you will get a path
        @param map: the map. most important thing is the obstacles
        @param method: the method to plan. "A*" or "RRT"
        @param neighbor: the number of neighbor. 4 or 6
        @param path_simplification: do the path simplification or not
        @param plot: plot the map or not
        """
        self.map = map
        if self.map is not None:
            self.enlarge_obs()
        self.method = method
        self.neighbor = neighbor
        self.simplify = path_simplification

        self.plot = plot
        if self.plot:
            self._plot()


    def approach(self, pBall, bias_pos = None, theta = np.pi/2, factor = 1.5):
        """
        calculate a position for Thymio to approach the ball
        that the ball in front of the Thymio will be at the goal position.
        @param pBall: the position of the goal point for the basket to approach
        @param bias_pos: the position of the basket, in thymio's coordination
        @param theta: the orientation to approach to the ball
        """
        if bias_pos is None:
            dx = int((Thymio_Size + Ball_Size)/self.map.scale)
            dy = 0
        else:
            dx = bias_pos.x
            dy = bias_pos.y
        r = factor * Ball_Size*2 / self.map.scale
        pBasketx = pBall.x - r*math.cos(theta)
        pBaskety = pBall.y - r*math.sin(theta)
        p = Pos(int(pBasketx - dx*math.cos(theta) - dy*math.sin(theta)),
                            int(pBaskety - dx*math.sin(theta) + dy*math.cos(theta)))
        if not self._check(p):
            raise Exception("The approach position collide with obstacles!")
        if self._obsinbetween(p, pBall):
            raise Exception("There's obstacle between approach point to the ball")
        return State(p, theta)

    def set_map(self, map):
        self.map = map

    def set_goal(self, s):
        """set the desired goal state"""
        self.map.goal = s.pos
        self.goalori = s.ori

    def set_start(self, s):
        """set the start state

        note that we suppose thymio can rotate in place,
        we don't care about the orientatin of the start state.
        """
        self.map.start = s.pos

    def assign_ori(self, path, factor = 1.5):
        """
        assign orientation for waypoints

        this function aims to provide more information about the path.
        first it will tell the waypoints what the direction to the next.
        second it insert a waypoint before the goal, so that it's able to 
        not rotate anymore at the last waypoint.
        
        @param path: list of waypoints(Pos)
        @return: list of waypoints(State)
        """
        sPath = [State(path[i].multiply(self.map.scale), path[i].delta_theta(path[i+1]))\
            for i in range(1, len(path) - 1)
        ]
        q = PriorityQueue()
        goal = path[-1]
        dis_num = factor*(2* Ball_Size)/self.map.scale
        tanv = math.tan(self.goalori)
        if abs(tanv) < 1:
            dir = 1 if abs(self.goalori) > math.pi/2 else -1
            for i in range(1, self.map.height):
                x = goal.x + i*dir
                if x >= 0 and x < self.map.height:
                    y = goal.y + (int)(dir*i*tanv)
                    if y >= 0 and y < self.map.width:
                        p = Pos(x,y)
                        if p.dis(goal) > dis_num:
                            if self._check(p):
                                if not self._obsinbetween(p, path[-2]):
                                    q.put((p.dis(goal) - dis_num, p))
                                else:
                                    break
                            else:
                                break
                        else:
                            continue
                    else:
                        break
                else:
                    break
        else:
            dir = 1 if (self.goalori > math.pi) or (self.goalori < 0) else -1
            for j in range(self.map.width):
                y = goal.y + j*dir
                if y >= 0 and y < self.map.width:
                    x = goal.x + (int)(dir*j/tanv)
                    if x >= 0 and x < self.map.height:
                        p = Pos(x,y)
                        if p.dis(goal) > dis_num:
                            if self._check(p):
                                if not self._obsinbetween(p, path[-2]):
                                    q.put((abs(p.dis(goal) - dis_num), p))
                                else:
                                    break
                            else:
                                break
                        else:
                            continue
                    else:
                        break
                else:
                    break
        
        if q.empty():
            raise Exception("Error: assign orientation failed.")
        c, p = q.get()
        sPath.append(State(p.multiply(self.map.scale), self.goalori))
        if len(sPath) > 1:
            sPath[-2].ori = path[-2].delta_theta(p)
        sPath.append(State(goal.multiply(self.map.scale), self.goalori))
        # for s in sPath:
        #     print(s)
        return sPath

    def enlarge_obs(self, clearance_factor = 1.2):
        """
        enlarge the obstacles

        considering the size of Thymio and the ball
        @param clearance_factor: factor to increase the clearance
        """
        assert self.map is not None
        num = int(clearance_factor * 2 * (Thymio_Size + 2*Ball_Size) / self.map.scale)

        self.obs = self.map.obs_map
        # self.obs = np.zeros((self.map.height, self.map.width))
        # for i in range(self.map.height):
        #     for j in range(self.map.width):
        #         if not self.map.check(Pos(i,j)):
        #             self.obs[i,j] = True
        
        import cv2
        self.obs = cv2.dilate(self.obs, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (num,num)))

    def plan(self, map = None):
        """return a path from the start to the goal

        @param map: GridMap with all the info(start, goal, obstacles)
        @return waypoints: list[Pos]
            Note: If no feasible path found,
                for A*, the list will be empty
                for RRT, it will never ends
        """
        if map is None:
            if self.map is None:
                raise Exception("Error: Please assign a Map first.")
        else:
            self.map = map
            self.enlarge_obs()

        if self.method == "A*":
            ret = self._a_star()
        elif self.method == "Greedy":
            pass
        elif self.method == "RRT":
            ret = self._rrt()
        else:
            print("Warning! unknown method of path planning, using default instead.")
            self.method = "A*"
            ret = self._a_star()

        if self.simplify:
            ret = self.collect_wps(ret)
            ret = self.path_simplification(ret)
        if self.plot:
            self._plot(ret)
        return ret


    def _a_star(self):
        qps = PriorityQueue()
        gcost = [[math.inf for _ in range(self.map.width)] for _ in range(self.map.height)]
        gcost[self.map.start.x][self.map.start.y] = 0
        parent = [[None for _ in range(self.map.width)] for _ in range(self.map.height)]
        parent[self.map.start.x][self.map.start.y] = self.map.start
        def heuristic(p):
            return gcost[p.x][p.y] + p.dis(self.map.goal)

        qps.put((heuristic(self.map.start), self.map.start))
        waypoints = []
        while not qps.empty():
            c, p = qps.get()
            if p == self.map.goal:
                w = self.map.goal
                waypoints.append(w)
                while w != self.map.start:
                    w = parent[w.x][w.y]
                    waypoints.append(w)
                break
            else:
                # check(p)
                for pn,c in self._get_neighbors(p):
                    newcost =  gcost[p.x][p.y] + c
                    if self._check(pn) and gcost[pn.x][pn.y] > newcost:
                        gcost[pn.x][pn.y] = newcost
                        parent[pn.x][pn.y] = p
                        qps.put((heuristic(pn), pn))

        if len(waypoints) == 0:
            print("Warning: A* can not find a path!!")
            return []
        waypoints.reverse()
        return waypoints[1:]

    def _rrt(self, p_bias = 0.1):
        """
        biased rrt
        """
        step_len = min(self.map.height, self.map.width)/20.0
        qps = PriorityQueue()
        nodes = [self.map.start]
        gcost = [0]
        parent = [None]

        waypoints = []
        while True:      # for _ in range(max_step)
            # 1. sample
            if random.random() < p_bias:
                q = self.map.goal
            else:
                q = self._valid_sample()
            # 2. nearest node
            ni = 0
            mdis = nodes[ni].dis(q)
            onTree = False
            for n in range(len(nodes)):
                if nodes[n] == q:
                    onTree = True
                    break
                tdis = nodes[n].dis(q)
                if tdis < mdis:
                    mdis = tdis
                    ni = n
            if onTree:
                continue
            # 3. try one step
            nn = Pos.portion(q, nodes[ni], min(1, step_len/mdis))
            if self._check(nn) and not self._obsinbetween(nodes[ni], nn):
                # 4.1 check goal condition
                if nn.dis(self.map.goal) < 1:
                    waypoints.append(nn)
                    w = ni
                    while w != None:
                        waypoints.append(nodes[w])
                        w = parent[w]
                    break
                nodes.append(nn)
                gcost.append(nn.dis(nodes[ni]) + gcost[ni])
                parent.append(ni)

        if len(waypoints) == 0:
            return []
        waypoints.reverse()
        return waypoints[1:]


    def collect_wps(self, waypoints, eps = 1e-3):
        """merge waypoints in the same direction
        """
        if len(waypoints) <= 2:
            print("too short!")
            return waypoints
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

    def _obsinbetween(self, p1, p2):
        if abs(p1.x - p2.x) > abs(p1.y - p2.y):
            s = abs(1.0*(p1.y - p2.y)/(p1.x - p2.x))
            if p1.x > p2.x:
                ps = p2
                pe = p1
            else:
                ps = p1
                pe = p2
            dir = 1.0 if ps.y < pe.y else -1.0
            for i in range(1, pe.x - ps.x - 1):
                if not self._check(Pos(ps.x + i, (int)(dir*i*s + ps.y))):
                    return True
        elif p1.y == p2.y:
            if p1.x > p2.x:
                ps = p2
                pe = p1
            else:
                ps = p1
                pe = p2
            for i in range(1, pe.y - ps.y -1):
                if not self._check(Pos(ps.x + i, ps.y)):
                    return True
        else:
            s = abs(1.0*(p1.x - p2.x)/(p1.y - p2.y))
            if p1.y > p2.y:
                ps = p2
                pe = p1
            else:
                ps = p1
                pe = p2
            dir = 1.0 if ps.x < pe.x else -1.0
            for i in range(1, pe.y - ps.y - 1):
                if not self._check(Pos((int)(ps.x + dir*i*s), ps.y + i)):
                    return True
        return False

    def path_simplification(self, waypoints):
        """eliminate waypoints into several nodes

        Provide less lines for motion
        """
        if len(waypoints) <= 2:
            print("too short!")
            return waypoints
        # grand-parent-child
        grand = waypoints[0]
        nwps = [grand]
        parent = waypoints[1]
        i = 2
        while i < len(waypoints):
            child = waypoints[i]
            if self._obsinbetween(grand, child):
                nwps.append(parent)
                grand = parent
            parent = child
            i += 1
        nwps.append(child)
        return nwps

    """Map Search Operations"""

    def _get_neighbors(self, p):
        ava = [p.x > 0, p.x < self.map.height-1, p.y > 0, p.y < self.map.width-1]
        bias = [(-1, 0),(1, 0),(0, -1),(0, 1)]
        if self.neighbor == 4:
            ret = []
            for i in range(4):
                if ava[i]:
                    ret.append((Pos(p.x+bias[i][0], p.y+bias[i][1]), 1))
            return ret
        elif self.neighbor == 8:
            ava += [ava[0] and ava[2], ava[0] and ava[3], ava[1] and ava[2], ava[1] and ava[3]]
            bias += [(-1, -1),(-1, 1),(1, -1),(1, 1)]
            ret = []
            for i in range(8):
                if ava[i]:
                    ret.append((Pos(p.x+bias[i][0], p.y+bias[i][1]), 1 if i<4 else 1.5))
            return ret
        else:
            print("Warning! unsupported neighbor number, using default instead.")
            self.neighbor = 4
            return self._get_neighbors(p)

    def _check(self, p):
        return not self.obs[p.x][p.y]

    def _valid_sample(self):
        while True:
            x = random.randint(0, self.map.height-1)
            y = random.randint(0, self.map.width-1)
            sample = Pos(x,y)
            if self._check(sample):
                return sample

    """Visualization"""
    def _plot(self, path = None):
        # plot the map
        image = np.zeros(shape=(self.map.height, self.map.width))
        def plot_point(p, value):
            image[p.x, p.y] = value

        plot_point(self.map.start, 80)
        plot_point(self.map.goal, 120)
        for i in range(self.map.height):
            for j in range(self.map.width):
                if self.obs[i][j]:
                    plot_point(Pos(i,j),40)

        if path is not None:
            for p in path:
                plot_point(p, 160)
            if len(path) > 0:
                cp = self.collect_wps(path)
                for p in cp:
                    plot_point(p,200)
                sp = self.path_simplification(cp)
                for p in sp:
                    plot_point(p,250)

        plt.imshow(image)
        plt.show()

if __name__ == "__main__":
    # generate a random map
    # Note: 600*800 is too big for A star
    #       using RRT instead
    h, w = 50, 100
    goalpoint = Pos(int(h/2), w-1)
    rmap = GridMap(h, w, 0.01)
    rmap.set_start(Pos(0,0))
    rmap.set_goal(goalpoint)
    import random
    obslist = [Pos(random.randint(11,h-11),random.randint(11,w-40)) for _ in range(10)]
    rmap.set_obs(obslist)

    # planner
    ppr = PathPlanner(rmap,path_simplification=True, plot=True,neighbor=8, method="RRT")
    app = ppr.approach(goalpoint)
    print("Goal is", goalpoint, "Approach with", app)
    ppr.set_goal(app)
    path = ppr.plan()
    for p in path:
        print(p)
    spath = ppr.assign_ori(path)
    for s in spath:
        print(s)

