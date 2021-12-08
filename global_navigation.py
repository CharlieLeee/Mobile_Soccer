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
from geo import *

from queue import PriorityQueue
import numpy as np
import random

import matplotlib.pyplot as plt

class PathPlanner:
    def __init__(self, map = None, method = "A*", neighbor = 4, path_simplification = False, plot = False): 
        """tell me the map, I will give u a path

        u can use 'set_map' to tell me the map, 
        and modify it with 'set_goal' and 'set_start';
        or specify the map(without enlarging the obstacles) 
        when calling 'plan'
        """
        self.map = map
        if self.map is not None:
            self.enlarge_obs()
        self.method = method
        self.neighbor = neighbor
        self.simplify = path_simplification
        # self.settings = settings # TODO

        self.plot = plot
        if self.plot:
            self._plot()
        
    
    def approach(self, pBall):
        """
        calculate a position for Thymio to approach the ball
        that the ball in front of the Thymio will be at the goal position.
        """
        num = (int)((Thymio_Size + Ball_Size) / self.map.scale)
        q = PriorityQueue()
        for i in range(max(0, pBall.x - num), 
            min(self.map.height, pBall.x + num)):
            for j in range(max(0, pBall.y), 
                min(self.map.width, pBall.y + num)):
                p = Pos(i,j)
                if self.map.check(p):
                    q.put((abs(pBall.dis(p) - (Thymio_Size + Ball_Size)), p))
        dis, p = q.get()
        return p
    
    def set_map(self, map):
        self.map = map
    
    def set_goal(self, p):
        self.map.goal = p

    def set_start(self, p):
        self.map.start = p

    def assign_ori(self, path, endori):
        """
        assign orientation for waypoints
        return list of States
        """
        sPath = [State(path[i].multiply(self.map.scale), path[i].delta_theta(path[i+1]))\
            for i in range(1, len(path) - 1)
        ]
        q = PriorityQueue()
        goal = path[-1]
        tanv = math.tan(endori)
        if abs(tanv) < 1:
            dir = 1 if abs(endori) > math.pi/2 else -1
            for i in range(1, self.map.height):
                x = goal.x + i*dir
                if x >= 0 and x < self.map.height:
                    y = goal.y + (int)(i*tanv)
                    if y >= 0 and y < self.map.width:
                        p = Pos(x,y)
                        if self.__check(p):
                            if not self._obsinbetween(p, path[-2]):
                                q.put((abs(p.dis(goal) - Thymio_Size/self.map.scale), p))
                            else:
                                break
                        else:
                            break
                    else:
                        break
                else:
                    break
        else:
            dir = 1 if endori > math.pi else -1
            for j in range(self.map.width):
                y = goal.y + j*dir
                if y >= 0 and y < self.map.width:
                    x = goal.x + (int)(j/tanv)
                    if x >= 0 and x < self.map.height:
                        p = Pos(x,y)
                        if self.__check(p):
                            if not self._obsinbetween(p, path[-2]):
                                q.put((abs(p.dis(goal) - Thymio_Size/self.map.scale), p))
                            else:
                                break
                        else:
                            break
                    else:
                        break
                else:
                    break

        c, p = q.get()
        sPath.append(State(p.multiply(self.map.scale), endori))
        sPath[-2].ori = path[-2].delta_theta(p)
        sPath.append(State(goal.multiply(self.map.scale), endori))
        # for s in sPath:
        #     print(s)
        return sPath

    def enlarge_obs(self):
        """
        enlarge the obstacles 
        considering the size of Thymio
        """
        assert self.map is not None
        num = (int)((Thymio_Size + 2*Ball_Size) / self.map.scale)
        self.obs = [[False for _ in range(self.map.width)] for _ in range(self.map.height)]
        def ava(x, y):
            return x >=0 and x<self.map.height and y>=0 and y<self.map.width
        for i in range(self.map.height):
            for j in range(self.map.width):
                if not self.map.check(Pos(i,j)):
                    self.obs[i][j] = True
                    if num != 0:
                        for m in range(-num, num):
                            for n in range(-num, num):
                                if m**2 + n**2 <= num ** 2 and ava(i+m, j+n):
                                    self.obs[i+m][j+n] = True

    def plan(self, map = None):
        """return a path from the start to the goal
        
        @param map: GridMap with all the info(start, goal, obstacles)
        @return waypoints: Queue[Pos] 
            Note: If no feasible path found, the list will be empty
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
                for pn,c in self.__get_neighbors(p):
                    newcost =  gcost[p.x][p.y] + c
                    if self.__check(pn) and gcost[pn.x][pn.y] > newcost:
                        gcost[pn.x][pn.y] = newcost
                        parent[pn.x][pn.y] = p
                        qps.put((heuristic(pn), pn))
                    
        if len(waypoints) == 0:
            return []
        waypoints.reverse()
        waypoints = self.collect_wps(waypoints)
        if self.simplify:
            waypoints = self.path_simplification(waypoints)
        return waypoints

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
                q = self.__valid_sample()
            # 2. nearest node
            ni = 0
            mdis = nodes[ni].dis(q)
            onTree = False
            for n in range(len(nodes)):
                if nodes[ni] == q:
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
            if self.__check(nn) and not self._obsinbetween(nodes[ni], nn):
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
        if self.simplify:
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
                if not self.__check(Pos(ps.x + i, (int)(dir*i*s + ps.y))):
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
                if not self.__check(Pos((int)(ps.x + dir*i*s), ps.y + i)):
                    return True
        return False

    def path_simplification(self, waypoints):
        """eliminate waypoints into several nodes
        
        Provide less lines for motion
        """
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

        # TODO: rdp?

    """Map Search Operations"""

    def __get_neighbors(self, p):
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
            return self.__get_neighbors(p)

    def __check(self, p):
        return not self.obs[p.x][p.y]    

    def __valid_sample(self):
        while True:
            x = random.randint(0, self.map.height-1)
            y = random.randint(0, self.map.width-1)
            sample = Pos(x,y)
            if self.__check(sample):
                return sample

    """Visualization"""
    def _plot(self, path = None):
        # plot the map
        self.map_image = np.zeros(shape=(self.map.height, self.map.width))
        def plot_point(p, value):
            self.map_image[p.x, p.y] = value

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
                    # print(p)

        plt.imshow(self.map_image)
        plt.show()

if __name__ == "__main__":
    # generate a random map
    # Note: 600*800 is too big for A star
    #       using RRT instead
    h, w = 30, 40
    rmap = GridMap(h, w, 0.01)
    rmap.set_start(Pos(0,0))
    rmap.set_goal(Pos(h-1, w-1))
    import random
    obslist = [Pos(random.randint(11,h-11),random.randint(11,w-11)) for _ in range(3)]
    rmap.set_obs(obslist)

    # planner
    ppr = PathPlanner(rmap,path_simplification=True, plot=True,neighbor=8, method="A*")
    path = ppr.plan()
    spath = ppr.assign_ori(path, 0.0)
    ppr._plot([Pos((int)(s.pos.x/0.01), (int)(s.pos.y/0.01)) for s in spath])
    #for p in path:
    #    print(p)
    
