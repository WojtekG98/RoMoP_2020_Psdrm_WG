#!/usr/bin/env python

import math
import random
import matplotlib.pyplot as plt
import heapq

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og


class Node():
    """A node class for A* pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position.getX() == other.position.getX() and self.position.getY() == other.position.getY()
        #return self.position[0] == other.position[0] and self.position[1] == other.position[1]

    def __str__(self):
        return str(self.position[0]) + ", " + str(self.position[1]) + ", " + str(self.f)

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


class Astar(ob.Planner):
    def __init__(self, si):
        super(Astar, self).__init__(si, "Astar")
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        solution = None
        approxsol = 0
        approxdif = 1e6
        start_state = pdef.getStartState(0)
        goal_state = goal.getState()
        start_node = Node(None, start_state)
        start_node.g = start_state.h = start_node.f = 0
        end_node = Node(None, goal_state)
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []
        heapq.heapify(open_list)
        adjacent_squares = ((0, -1, 3 * math.pi / 2), (0, 1, math.pi / 2), (-1, 0, math.pi), (1, 0, 0),
                            (-1, -1, 5 * math.pi / 4), (-1, 1, 3 * math.pi / 2), (1, -1, 7 * math.pi / 4),
                            (1, 1, math.pi / 4))

        heapq.heappush(open_list, start_node)
        while len(open_list) > 0 and not ptc():
            current_node = heapq.heappop(open_list)

            if current_node == end_node:  # if we hit the goal
                current = current_node
                path = []
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                for i in range(1, len(path)):
                    self.states_.append(path[len(path) - i - 1])
                solution = len(self.states_)
                break
            closed_list.append(current_node)

            children = []
            for new_position in adjacent_squares:
                node_position = si.allocState()
                # node_position = ob.State()
                if isinstance(si.getStateSpace(), type(ob.ReedsSheppStateSpace(6))):
                    current_node_x = current_node.position.getX()
                    current_node_y = current_node.position.getY()
                    node_position.setXY(current_node_x + new_position[0], current_node_y + new_position[1])
                    node_position.setYaw(new_position[2])
                if isinstance(si.getStateSpace(), type(ob.RealVectorStateSpace())):
                    node_position[0], node_position[1] = current_node.position[0] + new_position[0],\
                                                         current_node.position[1] + new_position[1]
                # print(node_position.getX(), node_position.getY(), node_position.getYaw())
                # node_position[0], node_position[1] = current_node_x + new_position[0], current_node_y + new_position[1]

                if not si.checkMotion(current_node.position, node_position):
                    continue

                if not si.satisfiesBounds(node_position):
                    continue

                new_node = Node(current_node, node_position)
                children.append(new_node)

            for child in children:
                if child in closed_list:
                    continue
                child.g = current_node.g + 1  # si.distance(child.position, current_node.position)
                child.h = goal.distanceGoal(child.position)
                child.f = child.g + child.h
                if len([i for i in open_list if child == i and child.g >= i.g]) > 0:
                    continue
                heapq.heappush(open_list, child)
                # open_list.append(child)
        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = True
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(Astar, self).clear()
        self.states_ = []


def isStateValid(state):
    x = state[0]
    y = state[1]
    z = state[2]
    return (x - 250) * (x - 250) + (y - 250) * (y - 250) > 100 * 100


def dist_between_states(state1, state2):
    return math.sqrt(math.pow(state2[0] - state1[0], 2) + math.pow(state2[1] - state1[1], 2))


def plan():
    N = 10
    # create an R^2 state space
    space = ob.RealVectorStateSpace(2)
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    start[0] = 0  # random.randint(0, int(N / 2))
    start[1] = 0  # random.randint(0, int(N / 2))
    goal = ob.State(space)
    goal[0] = N  # random.randint(int(N / 2), N)
    goal[1] = N  # random.randint(int(N / 2), N)
    ss.setStartAndGoalStates(start, goal)
    planner = Astar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(.01)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        matrix = ss.getSolutionPath().printAsMatrix()
        print(matrix)
        verts = []
        for line in matrix.split("\n"):
            x = []
            for item in line.split():
                x.append(float(item))
            if len(x) is not 0:
                verts.append(list(x))
        # print(verts)
        plt.axis([0, N, 0, N])
        x = []
        y = []
        for i in range(0, len(verts)):
            x.append(verts[i][0])
            y.append(verts[i][1])
        # plt.plot(verts[i][0], verts[i][1], 'r*-')
        plt.plot(x, y, 'ro-')
        plt.show()


if __name__ == "__main__":
    plan()
