#!/usr/bin/env python

import math
import random
import matplotlib.pyplot as plt

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og


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
        # step = 1
        actual_dist_to_start = 0
        start_state = pdef.getStartState(0)
        n = 8
        #tab_of_states = []
        #for i in range(0, n):
        #    tab_of_states.append(si.allocState())
        while not ptc():
            # print("step = ", step)
            tab_of_states = []
            min_dist_to_goal = math.inf
            min_dist_to_start = math.inf
            rstate = si.allocState()
            for i in range(0, n):
                tab_of_states.append(si.allocState())
            x_before = self.states_[-1][0]
            y_before = self.states_[-1][1]
            tab_of_states[0][0], tab_of_states[0][1] = int(x_before) + 1, int(y_before)
            tab_of_states[1][0], tab_of_states[1][1] = int(x_before) - 1, int(y_before)
            tab_of_states[2][0], tab_of_states[2][1] = int(x_before), int(y_before) + 1
            tab_of_states[3][0], tab_of_states[3][1] = int(x_before), int(y_before) - 1
            tab_of_states[4][0], tab_of_states[4][1] = int(x_before) + 1, int(y_before) + 1
            tab_of_states[5][0], tab_of_states[5][1] = int(x_before) + 1, int(y_before) - 1
            tab_of_states[6][0], tab_of_states[6][1] = int(x_before) - 1, int(y_before) + 1
            tab_of_states[7][0], tab_of_states[7][1] = int(x_before) - 1, int(y_before) - 1
            for i in range(0, 8):
                actual_state = tab_of_states[i]
                g = dist_between_states(actual_state, start_state)
                h = goal.distanceGoal(actual_state)**2
                f = g + h
                if not isStateValid(actual_state):
                    f = math.inf
                # print("g=",g,"h=",h)
                if f < min_dist_to_goal + min_dist_to_start:
                    rstate = actual_state
                    min_dist_to_goal = h
                    min_dist_to_start = g
            if si.checkMotion(self.states_[-1], rstate):
                # print("x =".join([''.join(['{:4}'.format(rstate[0])])]),
                #      ", y =".join([''.join(['{:4}'.format(rstate[1])])]))
                self.states_.append(rstate)
                sat = goal.isSatisfied(rstate)
                dist = goal.distanceGoal(rstate)
                if sat:
                    approxdif = dist
                    solution = len(self.states_)
                    break
                if dist < approxdif:
                    approxdif = dist
                    approxsol = len(self.states_)
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
    return (x-250) * (x-250) + (y-250) * (y-250) > 100*100
    # return 1


def dist_between_states(state1, state2):
    return math.sqrt(math.pow(state2[0] - state1[0], 2) + math.pow(state2[1] - state1[1], 2))


def plan():
    N = 500
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
    start[0] = random.randint(0, int(N/2))
    start[1] = random.randint(0, int(N/2))
    goal = ob.State(space)
    goal[0] = random.randint(int(N/2), N)
    goal[1] = random.randint(int(N/2), N)
    print(goal[0],goal[1],start[0],start[1])
    ss.setStartAndGoalStates(start, goal)
    planner = Astar(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(188000.0)
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
        x=[]
        y=[]
        for i in range(0, len(verts)):
            x.append(verts[i][0])
            y.append(verts[i][1])
           # plt.plot(verts[i][0], verts[i][1], 'r*-')
        plt.plot(x, y, 'ro-')
        plt.show()


if __name__ == "__main__":
    plan()
