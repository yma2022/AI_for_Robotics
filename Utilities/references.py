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

# Please note that the following methods listed below are incomplete:
# make_heuristic, astar, smooth,
# Use the lectures and videos for help on finishing them yourselves. Good luck!

from math import *
from copy import deepcopy
import random


class plan:
    # --------
    # init:
    #    creates an empty plan
    #

    def __init__(self, grid, init, goal, cost=1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []

    # --------
    #
    # make heuristic function for a grid

    def make_heuristic(self, grid, goal, cost):
        self.heuristic = [[0 for col in range(len(grid[0]))]
                          for row in range(len(grid))]
        
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + abs(j - self.goal[1])

        
        # raise NotImplementedError('Student code not implemented (comment out this line when done)')

        # TODO: Implement a manhattan dist for the heuristic

    # ------------------------------------------------
    #
    # A* for searching a path to the goal
    #
    #

    def astar(self):

        if self.heuristic == []:
            raise ValueError("Heuristic must be defined to run A*")

        # internal motion parameters
        delta = [[-1, 0],  # go up
                 [0, -1],  # go left
                 [1, 0],  # go down
                 [0, 1]]  # do right

        # open list elements are of the type: [f, g, h, x, y]

        closed = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        action = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]

        closed[self.init[0]][self.init[1]] = 1

        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g + h

        open = [[f, g, h, x, y]]

        found = False  # flag that is set when search complete
        resign = False  # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open) == 0:
                resign = True
                return "Fail"
            
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]

            if x == self.goal[0] and y == self.goal[1]:
                found = True

            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]

                    if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            g2 = g + self.cost
                            h2 = self.heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

            count += 1

        # extract the path
        invpath = []
        x = self.goal[0]
        y = self.goal[1]
        invpath.append([x, y])
        while x != self.init[0] or y != self.init[1]:
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            x = x2
            y = y2
            invpath.append([x, y])

        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])
                

        #raise NotImplementedError('Student code not implemented (comment out this line when done)')

        # TODO: Fill in this method using a modified solution code of
        #  exercise/lesson #13 in Search Module (do NOT return values)
        # (i.e. Use self.grid for grid, self.init for init, self.heuristic for
        # heuristic,self.goal for goal, and self.cost for cost)
        # There is no need to return values since 'self' is being updated

        # Finish code for this method here!

        # Note: Instead of path being a 2d grid as shown in lectures, please set self.path
        # as a list of [x,y] locations along the path starting with the init location and
        # ending with the goal

    # ------------------------------------------------
    #
    # this is the smoothing function
    #

    def smooth(self, weight_data=0.1, weight_smooth=0.1,
               tolerance=0.000001):

        if self.path == []:
            raise ValueError("Run A* first before smoothing path")

        # raise NotImplementedError('Student code not implemented (comment out this line when done)')

        # TODO: Fill in this method using a modified solution code of
        #  exercise/lesson #6 in PID Module (do NOT return values)
        # (i.e. Use self.path for path, and self.spath for newpath)
        # There is no need to return values since 'self' is being updated

        # The code is started for you below
        self.spath = deepcopy(self.path)  # spath is similar to newpath from lesson modules

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path) - 1):
                for j in range(len(self.path[0])):
                    aux = self.spath[i][j]
                    self.spath[i][j] += weight_data * \
                        (self.path[i][j] - self.spath[i][j])
                    self.spath[i][j] += weight_smooth * \
                        (self.spath[i - 1][j] + self.spath[i + 1][j] - (2.0 * self.spath[i][j]))
                    
                    if i >= 2:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i - 1][j] - self.spath[i - 2][j] - self.spath[i][j])
                    if i <= len(self.path) - 3:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i + 1][j] - self.spath[i + 2][j] - self.spath[i][j])
                        
            change += abs(aux - self.spath[i][j])


        # Finish code for this method here!



# ------------------------------------------------
#
# this is the robot class
#

class robot:

    # --------
    # init:
    #	creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions = 0
        self.num_steps = 0

    # --------
    # set:
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)

    # --------
    # set_noise:
    #	sets the noise parameters
    #

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    # --------
    # check:
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = sqrt((self.x - float(i)) ** 2 +
                                (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True

    def check_goal(self, goal, threshold=1.0):
        dist = sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold

    # --------
    # move:
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, grid, steering, distance,
             tolerance=0.001, max_steering_angle=pi / 4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        res = robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions = self.num_collisions
        res.num_steps = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:
                
            # approximate by straight line motion

            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:
                
            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        return res

        # raise NotImplementedError('Student code not implemented (comment out this line when done)')
        # TODO: Fill in this method using a modified solution code of
        #  Problem Set 3 Question 6 (move function)
        # Finish code for this method here! Apply noise, then execute motion.


        # check for collision
        # res.check_collision(grid)

        return  # return the robot

    # --------
    # sense:
    #

    def sense(self):

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #

    def measurement_prob(self, measurement):

        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0) \
                / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        error *= exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0) \
                    / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        
        # raise NotImplementedError('Student code not implemented (comment out this line when done)')
        # TODO: use the Gaussian method below to calculate the error
        # error = Gaussian()

        return error

    # TODO: Finish the Gaussian method below including the call, signature, and implementation
    #   Use exercise/lesson #8 in the Kalman Filters modules for help
    # def Gaussian(self, ):
    #     # Implement a Gaussian and return the value
    #     return


    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]' % (self.x, self.y)


# ------------------------------------------------
#
# this is the particle filter class
#

class particles:

    # --------
    # init:
    #	creates particle set with given initial position
    #

    def __init__(self, x, y, theta,
                 steering_noise, distance_noise, measurement_noise, N=100):

        self.N = N
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

        self.data = []

        for i in range(self.N):
            r = robot()
            r.set(x, y, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)

        # TODO: Create a list (self.data) of N particles
        #  Be sure to set the position and noise for each
        # raise NotImplementedError('Student code not implemented (comment out this line when done)')

    # --------
    #
    # extract position from a particle set
    #

    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.data[i].orientation
                              - self.data[0].orientation + pi) % (2.0 * pi))
                            + self.data[0].orientation - pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # --------
    #
    # motion of the particles
    #

    def move(self, grid, steer, speed):
        # TODO: Move all of the particles according to the input parameters
        #   Use exercise/lesson #14 in the Particle Filters module for help
        # Remember that self.data is similar to 'p' from the video modules

        # raise NotImplementedError('Student code not implemented (comment out this line when done)')
        # self.data =
        newdata = []
        for i in range(self.N):
            newdata.append(self.data[i].move(grid, steer, speed))
        self.data = newdata

    # --------
    #
    # sensing and resampling
    #

    def sense(self, Z):
        # raise NotImplementedError('Student code not implemented (comment out this line when done)')
        # TODO: Fill in this method using a modified solution code of
        #  exercise/lesson #21 in Particle Filter Module
        # Be sure to calculate the weights of the particles and then resample
        # Don't forget to update self.data (similar to p in videos)

        # self.data =
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))
        # resampling wheel
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3



