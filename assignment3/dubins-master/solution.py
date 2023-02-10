#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

# {Javier Lopez Iniesta Diaz del Campo}
# {19990714-T692}
# {jliddc@kth.se}

from dubins import *
import numpy as np
from datetime import datetime

map = 0

def solution(car):

    start_time = datetime.now()
    # X: Minimum and maximum positions [m] - Random area
    x_min = car.xlb 
    x_max = car.xub 
    y_min = car.ylb 
    y_max = car.yub

    # Initial positions [m] - Start
    x0 = car.x0 
    y0 = car.y0 
    theta0 = 0
    
    start = [x0, y0, theta0, 0, 0, 1] 

    # Target positions [m] - Goal
    x_target = car.xt 
    y_target = car.yt

    # Obstacles
    obstacles = car.obs

    # print("Start position: (", x0, ", ",y0, ")")
    # print("Goal position: (", x_target, ", ", y_target, ")")

    # RRT 
    x = None # horizontal position
    y = None # vertical position
    theta = None # heading angle

    nodes = np.array([start])

    iteration = 0
    global map
    map = map +1
    flag_continue = True
    goal_reached = False

    # Parameters 
    number_steps = 50 # number maximum of steps 
    max_iterations= 525
    random_frequency = 15 # Pick the target point 1 out of 10 times. In other case, pick a random point
    dt = 0.01 # Initially 0.01 s
    finish_threshold = 0.12 # 1.5 meters
    collision_margin = 0.1 # 0.1 meters

    distance_to_target = np.sqrt((x_target-x0)**2 + (y_target-y0)**2)

    controls=[0]
    times=[0,dt]
    print("map: ", map)
    # if (map > 6):
    #     flag_continue = False

    while flag_continue:

        iteration = iteration + 1
        # print("Iteration: ",iteration,"- Distance to target: ", distance_to_target)

        # 1) Pick a random point a in X(x,y)

        # Random float point between the minimum and the maximum
        random_x = np.random.uniform(x_min, x_max)
        random_y = np.random.uniform(y_min, y_max)
        # point = (random_x, random_y)

        random_number = np.random.uniform(0, 100) 
        if (random_number < random_frequency):
            point = (x_target, y_target)
        else: 
            point = (random_x, random_y)

        # 2) Find b, the node of the tree closest to a
        distance = []
        # print("Nodes: ", nodes)

        for n in range(len(nodes)):
            # print("Node: ", nodes[n])
            # print("Point: ", point)
            node_distance = np.linalg.norm((nodes[n][0]-point[0])**2 + (nodes[n][1]-point[1])**2)
            distance.append(node_distance)

        index_nearest_neighbor = distance.index(min(distance))
        nearest_neighbor = nodes[index_nearest_neighbor]

        x = nearest_neighbor[0]
        y = nearest_neighbor[1]
        theta = nearest_neighbor[2]

        # print("Nearest neighbor (x, y, theta) = (", x, ",", y, ",", theta, ")")
        # print("Distance between the point and the nearest neighbor: ", min(distance))

        x_total = [x]
        y_total = [y]
        theta_total = [theta]

        # 3) Find control inputs u to steer the robot from b to a

        steering_angle = [-np.pi/4, 0, np.pi/4]
        # print(phi)

        for phi in steering_angle:
            
            # print("Actual phi: ", phi)
            i = 0
            flag_safe = True
            index = 0

            x_total = [x]
            y_total = [y]
            theta_total = [theta]   

            for i in range(number_steps-1):

                # print(x_total[-1])
                if (i==0):
                    xn, yn, thetan = step(car, x, y, theta, phi)
                else:
                    xn, yn, thetan = step(car, x_total[-1], y_total[-1], theta_total[-1], phi)

                # while (thetan >= np.pi):
                #     thetan = thetan - 2*np.pi/2
                # while (thetan <= -np.pi):
                #     thetan = thetan + 2*np.pi/2
                
                x_total.append(xn)
                y_total.append(yn)
                theta_total.append(thetan)

                # Obstacles
                distance_to_obstacle = []
                for obs in obstacles:
                    # distance_obs = np.linalg.norm((x_total[i]-ob[0])**2 + (y_total[i]-ob[1])**2)
                    # # print("Distance to obstacle ", j, ":", distance_obs)
                    # distance_to_obstacle.append(distance_obs)

                    # If the path goes into an obstacle
                    if (np.linalg.norm((x_total[i]-obs[0])**2 + (y_total[i]-obs[1])**2) <= (obs[2] + collision_margin)):
                        # print("Collision with an obstacle!")
                        flag_safe = False

                # Out of bounds
                if ((x_total[i] < x_min) or (x_total[i] > x_max)):
                    # print("Out of bounds!")
                    flag_safe = False
                elif ((y_total[i] < y_min) or (y_total[i] > y_max)):
                    # print("Out of bounds!")
                    flag_safe = False

                if (not flag_safe):
                    break                    

                # Reach target
                distance_to_target = np.linalg.norm((x_target-x_total[i])**2 + (y_target-y_total[i])**2)
                # print("Distance to target: ", distance_to_target)
                # If it is 1.5 meteres withing of the target.
                if (distance_to_target < finish_threshold): 
                    goal_reached = True
                    final_node = np.array([[x_total[i], y_total[i], theta_total[i], phi, index_nearest_neighbor, i]])
                    nodes = np.concatenate((nodes, final_node), axis=0)
                    print("Goal reached!")
                    break

            if (goal_reached):

                previous_parent = index_nearest_neighbor
                path_sequence = [index_nearest_neighbor]
                # print("Goal parent: ", previous_parent)
                # print("List of parents: ", nodes[:,4])
                # print("List of phi's: ", nodes[:,3])

                while (previous_parent != 0):
                    previous_parent = int(nodes[previous_parent,4])
                    # print("Previous parent: ", previous_parent)
                    path_sequence.append(previous_parent)

                # print("Index of the goal parents: ", path_sequence)
                # path_sequence = np.array(np.flip(path_sequence))
                path_sequence = path_sequence[::-1]

                # print("Index of the goal parents inverted: ", path_sequence)

                for t in range(len(path_sequence)):
                    path_index = path_sequence[t]
                    step_length = nodes[path_index,5]
                    actual_phi = nodes[path_index,3]
                    times.append(times[-1] + step_length*dt)
                    controls.append(actual_phi)
                break

            if ((i) != (number_steps-2)): # If there is a collision or out of bounds pick a previous index
                index = int((i)/2)
            elif ((i) == 1): # This path does not work
                break
            else: 
                index = i # If not pick the last index of the path

            new_node = np.array([[x_total[index], y_total[index], theta_total[index], phi, index_nearest_neighbor, index]])
            nodes = np.concatenate((nodes, new_node), axis=0)
            elapsed_time_iteration = (datetime.now() - start_time).total_seconds()
            if (elapsed_time_iteration > 29):
                flag_continue = False
                break
            # print("Index:", index)
            # print("Distance to target: ", distance_to_target)
            # print("New node to add", new_node)

        # if (iteration == max_iterations):
        #     print("TIME LIMIT EXCEEDED")
        #     flag_continue = False
        #     break

        if (goal_reached == True):
            flag_continue = False
            break

    # print("Times: ", times)
    # print("Length times: ", len(times))
    # print("Controls: ", controls)
    # print("Length controls: ", len(controls))

    elapsed_time = (datetime.now() - start_time).total_seconds()
    print("Path computation finished in {} seconds".format(elapsed_time))
    print("--------------")

    return controls, times