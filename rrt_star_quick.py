# -*- coding: utf-8 -*-
"""
Created on Wed May 16 08:36:14 2018

@author: sislamoglu
"""

import matplotlib.pyplot as plt
import numpy as np
import math
from point import Point
import time

def build_arena(number_of_obstacle, limits):

    obstacle_points = []
    for i in range(number_of_obstacle):
        plt.axis(limits)
        points = plt.ginput(100)
        polygon = plt.Polygon(points)
        obstacle_points.append(points)
        axes = plt.gca()
        axes.add_patch(polygon)
    return obstacle_points

def samplingTheObstacle(obstacle, obstacle_sample_size):
    obstacle_counter = len(obstacle)
    sampled_values = []
    obstacle_points = []
    for i in range(obstacle_counter):
        vertices = obstacle[i]
        sample = np.zeros((obstacle_sample_size, 2))
        for j in range(len(vertices)):
            sample[:, 0] = np.linspace(vertices[vertices[j][1]][0][0], vertices[vertices[j][2]][0][0], obstacle_sample_size)
            sample[:, 1] = np.linspace(vertices[vertices[j][1]][0][1], vertices[vertices[j][2]][0][1], obstacle_sample_size)
            sampled_values.append(sample)
            sample = np.zeros((obstacle_sample_size, 2))
        obstacle_points.append(sampled_values)
    return obstacle_points

def calculateDistance(q1, q2):
    return math.sqrt((q1[0]-q2[0])**2 + (q1[1]-q2[1])**2)

def steer(q1, q2, val, eps):
    q_new = [0, 0]

    if val >= eps:
        q_new[0] = q2[0] + ((q1[0] - q2[0]) * eps) / calculateDistance(q1, q2)
        q_new[1] = q2[1] + ((q1[1] - q2[1]) * eps) / calculateDistance(q1, q2)
    else:
        q_new[0] = q1[0]
        q_new[1] = q1[1]
    return Point(q_new, 0, 0)

def noCollision(q1, q2, obstacle_points):
    for i in range(len(obstacle_points)):
        if (q2[0]-q1[0]) == 0:
            tangent = float('Inf')
        else:
            tangent = float(q2[1]-q1[1]) / (q2[0]-q1[0])
        o_array = obstacle_points[i]
        points = 100
        line_points = np.zeros((2, points))
        if tangent == float('Inf'):
            line_points[0, :] = np.linspace(q1[0], q1[0], points)
            line_points[1, :] = np.linspace(q1[1], q2[1], points)
        elif tangent == 0:
            line_points[0, :] = np.linspace(q1[0], q2[0], points)
            line_points[1, :] = np.linspace(q1[1], q1[1], points)
        else:
            line_points[0, :] = np.linspace(q1[0], q2[0], points)
            line_points[1, :] = tangent*(line_points[0, :]-q1[0]) + q1[1]
        for j in range(points):
            if inside_polygon(line_points[0, j], line_points[1, j], o_array):
                return 0
    return 1
def inside_polygon(x, y, points):

    n = len(points)
    inside = False
    p1x, p1y = points[0]
    for i in range(1, n + 1):
        p2x, p2y = points[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x < xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside
def main():
    # PARAMETERS
    eps = 20
    numNodes = 3000
    x_max = 1000
    y_max = 1000
    number_of_obstacle = 1
    number_of_ancestors = 2

    # INITIALIZATION
    start_point = Point([0, 0], 0, 0)
    goal_point = Point([900, 900], 0, 0)

    nodes = []
    nodes.append(start_point)
    arena_limits = [0, x_max, 0, y_max]
    # IF YOU WANT TO USE YOUR OWN OBSTACLE BY SELECTING WITH MOUSE / UNCOMMENT HERE
    # obstacle_points = build_arena(number_of_obstacle, arena_limits)

    # IF YOU WANT TO USE SAME OBSTACLES / UNCOMMENT HERE
    obstacle_points = []
    '''
    obstacle_points.append([(200, 400), (300, 550), (400, 400), (550, 300), (400, 200), (300, 50), (200, 200), (50, 300)])
    obstacle_points.append([(250, 900), (500, 900), (600, 700), (800, 600), (500, 600), (400, 800)])
    obstacle_points.append([(700, 150), (900, 150), (900, 350), (700, 350)])
    '''
    obstacle_points.append(
        [(200, 400), (300, 550), (400, 400), (550, 300), (400, 200), (300, 50), (200, 200), (50, 300)])
    obstacle_points.append([(400, 500), (650, 500), (750, 300), (950, 200), (650, 200), (550, 400)])
    obstacle_points.append([(300, 600), (300, 800), (600, 800), (600, 600)])
    '''
    obstacle_points.append(
        [(400, 700), (400, 300), (600, 300), (600, 700), (650, 700), (650, 250), (350, 250), (350, 700)])
    '''
    for i in range(len(obstacle_points)):
        points = obstacle_points[i]
        plt.axis(arena_limits)
        polygon = plt.Polygon(points)
        obstacle_points.append(points)
        axes = plt.gca()
        axes.add_patch(polygon)
    #
    start_time = time.time()
    for i in range(numNodes):
        print(i, '/', numNodes)
        random_point = Point([math.floor(np.random.rand()*x_max), math.floor(np.random.rand()*y_max)], 0, 0)
        for j in range(len(nodes)):
            if nodes[j].getCoord() == goal_point.getCoord():
                break
        ndist = []
        for j in range(len(nodes)):
            temp = calculateDistance(nodes[j].getCoord(), random_point.getCoord())
            ndist.append(temp)
        minimum_index = ndist.index(min(ndist))
        minimum_value = min(ndist)
        near_point = nodes[minimum_index]
        new_point = steer(random_point.getCoord(), near_point.getCoord(), minimum_value, eps)
        if noCollision(random_point.getCoord(), near_point.getCoord(), obstacle_points):
            new_point.setCost(calculateDistance(new_point.getCoord(), near_point.getCoord()) + near_point.getCost())

            minimum_point = near_point
            minimum_cost = new_point.getCost()

            if minimum_index > number_of_ancestors:
                best_node = minimum_index
                for j in range(number_of_ancestors):
                    current_node = minimum_point.getParent()
                    minimum_point = nodes[minimum_point.getParent()]
                    if noCollision(new_point.getCoord(), minimum_point.getCoord(), obstacle_points) and \
                            (minimum_point.getCost() + calculateDistance(minimum_point.getCoord(), new_point.getCoord())) < minimum_cost:
                        minimum_cost = minimum_point.getCost() + calculateDistance(minimum_point.getCoord(), new_point.getCoord())
                        best_node = current_node
                minimum_point = nodes[best_node]
                new_point.setCost(minimum_cost)

            plt.plot([new_point.getCoord()[0], minimum_point.getCoord()[0]],
                     [new_point.getCoord()[1], minimum_point.getCoord()[1]], color='blue')

            for k in range(len(nodes)):
                if nodes[k].getCoord() == minimum_point.getCoord():
                    new_point.setParent(k)
            nodes.append(new_point)
    distances = []
    for i in range(len(nodes)):
        temp_distance = calculateDistance(nodes[i].getCoord(), goal_point.getCoord())
        distances.append(temp_distance)
    minimum_goal_node_index = distances.index(min(distances))
    final_point = nodes[minimum_goal_node_index]
    end_point = final_point
    goal_point.setParent(minimum_goal_node_index)
    nodes.append(goal_point)

    total_cost = nodes[minimum_goal_node_index].getCost()
    while end_point.getParent() != 0:
        start = end_point.getParent()
        plt.plot([end_point.getCoord()[0], nodes[start].getCoord()[0]], [end_point.getCoord()[1], nodes[start].getCoord()[1]], color='red')
        end_point = nodes[start]
    start = end_point.getParent()
    plt.plot([end_point.getCoord()[0], nodes[start].getCoord()[0]],
             [end_point.getCoord()[1], nodes[start].getCoord()[1]], color='red')
    print('The Total Cost is: ', total_cost)
    end_time = time.time()
    print(end_time - start_time, ' seconds')
    plt.show()

if __name__ == "__main__":
    main()
