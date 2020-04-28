from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import matplotlib.pyplot as plt
import random

radius = 100
center = [250, 250]


class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        x = state[0]
        y = state[1]
        # if (self.clearance(state) < 0.0):
        # print("x:",x,", y:",y,",clearance: ",self.clearance(state))
        return self.clearance(state) > 0.0

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        # Extract the robot's (x,y) position from its state
        x = state[0]
        y = state[1]
        # Distance formula between two points, offset by the circle's
        # radius
        return sqrt((x - center[0]) * (x - center[0]) + (y - center[0]) * (y - center[0])) - radius


def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)


def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
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


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


def plan(runTime, plannerType, objectiveType, fname):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,500]x[0,500], a subset of R^2.
    space = ob.RealVectorStateSpace(2)

    # Set the bounds of space to be in [0,500].
    space.setBounds(0.0, 500.0)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    si.setup()

    # Set our robot's starting state to be random
    start = ob.State(space)
    start[0] = random.randint(0, 500)
    start[1] = random.randint(0, 500)

    while (sqrt((start[0] - center[0]) * (start[0] - center[0]) + (start[1] - center[0]) * (
            start[1] - center[0])) - radius < 0):
        start[0] = random.randint(0, 500)
        start[1] = random.randint(0, 500)

    # Set our robot's goal state to be random 
    goal = ob.State(space)
    goal[0] = random.randint(0, 500)
    goal[1] = random.randint(0, 500)
    while (sqrt((goal[0] - center[0]) * (goal[0] - center[0]) + (goal[1] - center[0]) * (
            goal[1] - center[0])) - radius < 0):
        goal[0] = random.randint(0, 500)
        goal[1] = random.randint(0, 500)

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
              'objective value of {2:.4f}'.format( \
            optimizingPlanner.getName(), \
            pdef.getSolutionPath().length(), \
            pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
        matrix = pdef.getSolutionPath().printAsMatrix()
        print(matrix)
        verts = []
        for line in matrix.split("\n"):
            x = []
            for item in line.split():
                x.append(float(item))
            if len(x) is not 0:
                verts.append(list(x))
        # print(verts)
        plt.axis([0, 500, 0, 500])
        x = []
        y = []
        for i in range(0, len(verts)):
            x.append(verts[i][0])
            y.append(verts[i][1])
        # plt.plot(verts[i][0], verts[i][1], 'r*-')
        plt.plot(x, y, 'ro-')
        plt.show()
        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(pdef.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")


if __name__ == "__main__":
    plan(10, 'RRT', 'PathLength', 'path.txt')
