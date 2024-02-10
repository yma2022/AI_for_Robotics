######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

from Utilities import test

# SEARCH LESSON MODULES
print("SEARCH LESSON MODULES", end="")

# --------------------------------------------------------------------
# 9. FIRST SEARCH PROGRAM
print("\n9. FIRST SEARCH PROGRAM")
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
#
#  Comment out any print statements used for debugging.

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    path = None
    # TODO: ADD CODE HERE
    closed_list = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed_list[init[0]][init[1]] = 1

    x = init[0]
    y = init[1]
    g = 0

    open_list = [[g, x, y]]

    found = False
    resign = False

    while not found and not resign:
        if len(open_list) == 0:
            resign = True
            path = 'fail'
            print(path)
        else:
            open_list.sort()
            open_list.reverse()
            next_node = open_list.pop()
            x = next_node[1]
            y = next_node[2]
            g = next_node[0]

            if x == goal[0] and y == goal[1]:
                found = True
                path = next_node
                print(path)
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed_list[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_list.append([g2, x2, y2])
                            closed_list[x2][y2] = 1
    print(closed_list)
    return path


print(search(grid, init, goal, cost))

print("EXTRA TEST CASES (1):")
try:
    response = test.run_grader_1(search)
    print(response)
except Exception as err:
    print(str(err))

# --------------------------------------------------------------------
# 10. EXPANSION GRID
print("\n10. EXPANSION GRID")
# Modify the function search so that it returns
# a table of values called expand. This table
# will keep track of which step each node was
# expanded.
#
# Make sure that the initial cell in the grid
# you return has the value 0.
#
#  Comment out any print statements used for debugging.

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    path = None
    # TODO: ADD CODE HERE
    closed_list = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed_list[init[0]][init[1]] = 1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0

    open_list = [[g, x, y]]

    found = False
    resign = False
    count = 0

    while not found and not resign:
        if len(open_list) == 0:
            resign = True
            path = 'fail'
            print(path)
        else:
            open_list.sort()
            open_list.reverse()
            next_node = open_list.pop()
            x = next_node[1]
            y = next_node[2]
            g = next_node[0]
            expand[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
                path = next_node
                print(path)
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed_list[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_list.append([g2, x2, y2])
                            closed_list[x2][y2] = 1
    print(closed_list)
    return path, expand


path, expand = search(grid, init, goal, cost)
print('Path:', path)
print('Expand:')
for i in range(len(expand)):
    print(expand[i])


print("EXTRA TEST CASES (2):")
try:
    response = test.run_grader_2(search)
    print(response)
except Exception as err:
    print(str(err))

# --------------------------------------------------------------------
# 11. PRINT PATH
print("\n11. PRINT PATH")
# Modify the the search function so that it returns
# a shortest path as follows:
#
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left,
# up, and down motions. Note that the 'v' should be
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    # Copy and paste your solution code from the previous exercise (#10)
    # TODO: CHANGE/UPDATE CODE
    path = None
    
    closed_list = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed_list[init[0]][init[1]] = 1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0

    open_list = [[g, x, y]]

    found = False
    resign = False
    count = 0

    while not found and not resign:
        if len(open_list) == 0:
            resign = True
            path = 'fail'
            print(path)
        else:
            open_list.sort()
            open_list.reverse()
            next_node = open_list.pop()
            x = next_node[1]
            y = next_node[2]
            g = next_node[0]
            expand[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
                path = next_node
                print(path)
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed_list[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_list.append([g2, x2, y2])
                            closed_list[x2][y2] = 1
                            action[x2][y2] = i
    print(closed_list)

    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'

    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2

    return path, expand, policy


path, expand, policy = search(grid, init, goal, cost)
for i in range(len(policy)):
    print(policy[i])

# --------------------------------------------------------------------
# 13. IMPLEMENT A*
print("\n13. IMPLEMENT A*")
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
#
# If there is no path from init to goal,
# the function should return the string 'fail'

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost, heuristic):
    # Copy and paste your solution code from the previous exercise (#11)
    # TODO: CHANGE/UPDATE CODE
    path = None
    closed_list = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed_list[init[0]][init[1]] = 1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h
    open_list = [[f, g, h, x, y]]

    found = False
    resign = False
    count = 0

    while not found and not resign:
        if len(open_list) == 0:
            resign = True
            path = 'fail'
            print(path)
        else:
            open_list.sort()
            open_list.reverse()
            next_node = open_list.pop()
            x = next_node[3]
            y = next_node[4]
            g = next_node[1]
            expand[x][y] = count
            count += 1

            if x == goal[0] and y == goal[1]:
                found = True
                path = next_node
                print(path)
            else:
                heuristic_val = dict()
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        if closed_list[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            open_list.append([f2, g2, h2, x2, y2])
                            closed_list[x2][y2] = 1
                            action[x2][y2] = i
    # print(closed_list)

    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'

    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2

    print("path:", path, "expand:", expand, "policy:", policy, "action:", action)
    return path, expand, policy


path, expand, policy = search(grid, init, goal, cost, heuristic)
for i in range(len(expand)):
    print(expand[i])

# --------------------------------------------------------------------
# 18. VALUE PROGRAM
print("\n18. VALUE PROGRAM")
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]

    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        change = True
                elif grid[x][y] == 0:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                        if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                            v2 = value[x2][y2] + cost
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
    # TODO: ADD CODE HERE
    # make sure your function returns a grid of values as
    # demonstrated in the previous video.
    return value


value = compute_value(grid, goal, cost)
for i in range(len(value)):
    print(value[i])

# --------------------------------------------------------------------
# 19. OPTIMUM POLICY
print("\n19. OPTIMUM POLICY")
# Write a function optimum_policy that returns
# a grid which shows the optimum policy for robot
# motion. This means there should be an optimum
# direction associated with each navigable cell from
# which the goal can be reached.
#
# Unnavigable cells as well as cells from which
# the goal cannot be reached should have a string
# containing a single space (' '), as shown in the
# previous video. The goal cell should have '*'.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):
    # Copy and paste your solution code from the previous exercise (#18)
    # TODO: CHANGE/UPDATE CODE
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True
                elif grid[x][y] == 0:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                        if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                            v2 = value[x2][y2] + cost
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[i]
    return value, policy


value, policy = compute_value(grid, goal, cost)
for i in range(len(policy)):
    print(policy[i])

# --------------------------------------------------------------------
# 20. LEFT TURN POLICY
print("\n20. LEFT TURN POLICY")
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.

# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0]  # given in the form [row,col,direction]

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 20]  # cost has 3 values, corresponding to making a right turn, no turn, and a left turn

# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1,  0],  # go up
            [0, -1],  # go left
            [1,  0],  # go down
            [0,  1]]  # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']


def optimum_policy2D(grid,init,goal,cost):
    value = [[[999 for col in range(len(grid[0]))] for row in range(len(grid))],
             [[999 for col in range(len(grid[0]))] for row in range(len(grid))],
             [[999 for col in range(len(grid[0]))] for row in range(len(grid))],
             [[999 for col in range(len(grid[0]))] for row in range(len(grid))]]
    # TODO: ADD CODE HERE
    policy = [[[' ' for col in range(len(grid[0]))] for row in range(len(grid))],
              [[' ' for col in range(len(grid[0]))] for row in range(len(grid))],
              [[' ' for col in range(len(grid[0]))] for row in range(len(grid))],
              [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]]
    policy2D = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):
                    if goal[0] == x and goal[1] == y:
                        if value[orientation][x][y] > 0:
                            value[orientation][x][y] = 0
                            policy[orientation][x][y] = '*'
                            change = True
                    elif grid[x][y] == 0:
                        for i in range(len(action)):
                            o2 = (orientation + action[i]) % 4
                            x2 = x + forward[o2][0]
                            y2 = y + forward[o2][1]
                            if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]) and grid[x2][y2] == 0:
                                v2 = value[o2][x2][y2] + cost[i]
                                if v2 < value[orientation][x][y]:
                                    change = True
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[i]

    x = init[0]
    y = init[1]
    orientation = init[2]
    policy2D[x][y] = policy[orientation][x][y]
    while policy[orientation][x][y] != '*':
        if policy[orientation][x][y] == '#':
            o2 = orientation
        elif policy[orientation][x][y] == 'R':
            o2 = (orientation - 1) % 4
        elif policy[orientation][x][y] == 'L':
            o2 = (orientation + 1) % 4
        x = x + forward[o2][0]
        y = y + forward[o2][1]
        orientation = o2
        policy2D[x][y] = policy[orientation][x][y]
        
    return policy2D


policy2D = optimum_policy2D(grid, init, goal, cost)
for i in range(len(policy2D)):
    print(policy2D[i])
