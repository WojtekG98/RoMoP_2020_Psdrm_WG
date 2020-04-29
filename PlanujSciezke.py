import Astar
import sys
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import matplotlib.pyplot as plt
import random

N = 500.0
radius = 150
center = [250, 250]


def isStateValid(state):
    x = state[0]
    y = state[1]
    return sqrt((x-center[0])**2 +(y-center[1])**2) > radius




# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "astar":
        return Astar.Astar(si)
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrt":
        return og.RRT(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


def plan(runTime, plannerType, fname, space, start, goal):

    ss = og.SimpleSetup(space)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal)

    # Set the problem instance for our planner to solve
    ss.setPlanner(allocatePlanner(ss.getSpaceInformation(), plannerType))

    # attempt to solve the planning problem in the given runtime
    solved = ss.solve(runTime)
    if solved:
        return ss.getSolutionPath().printAsMatrix()

        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(ss.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")

def plot(start, goal, path, style):
    plt.axis([0, 500, 0, 500])
    verts = []
    for line in path.split("\n"):
        x = []
        for item in line.split():
            x.append(float(item))
        if len(x) is not 0:
            verts.append(list(x))
    x = []
    y = []
    for i in range(0, len(verts)):
        x.append(verts[i][0])
        y.append(verts[i][1])
    plt.plot(x, y, style)
    plt.plot(start[0], start[1], 'g*')
    plt.plot(goal[0], goal[1], 'y*')

if __name__ == '__main__':
    # Construct the robot state space in which we're planning.
    # We're planning in [0,N]x[0,N], a subset of R^2.
    space = ob.RealVectorStateSpace(2)

    # Set the bound of space to be in [0,N].
    space.setBounds(0.0, N)
    # Set our robot's starting state to be random
    start = ob.State(space)
    start[0] = random.randint(0, N/2)
    start[1] = random.randint(0, N/2)
    while not isStateValid(start):
        start[0] = random.randint(0, N/2)
        start[1] = random.randint(0, N/2)

    # Set our robot's goal state to be random
    goal = ob.State(space)
    goal[0] = random.randint(N/2, N)
    goal[1] = random.randint(N/2, N)
    while not isStateValid(goal):
        goal[0] = random.randint(N/2, N)
        goal[1] = random.randint(N/2, N)
    path = plan(20, 'RRT', 'path.txt', space, start, goal)
    plot(start, goal, path, 'ro-')
    path = plan(20, 'Astar', 'path2.txt', space, start, goal)
    plot(start, goal, path, 'bo-')

    plt.show()
