#!/usr/bin/env python3

# A* Alghorithm for ROS Noetic Path Planning
# By Ardhika Maulidani

import math
import heapq
import random
import numpy as np

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Node():
    """A node class for RRT Pathfinding"""
    def __init__(self, position:tuple=None, parent=None):
            # Define position coordinate
            self.parent = parent
            self.position = position

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)
    
    def dist_to(self, pose):
        # Euclidean Distance
        return (abs(self.position[0] - pose.position[0])**2 + abs(self.position[1] - pose.position[1])**2)**0.5
    
    def is_same_as(self, pose):
        return self.dist_to(pose) <= 0.1

class RRT():
    def __init__(self, map, start, goal, step_len, goal_sample_rate, iter_max, robot):
        self.start = Node(start, None)
        self.goal = Node(goal, None)
        self.map = map              
        self.robot = robot

        self.step_len = step_len/self.map.resolution    # Maximum New Length
        self.goal_sample_rate = goal_sample_rate        # Sample Rate
        self.iter_max = iter_max                        # Max Iteration
        self.vertex = [self.start]                      # New Node

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.position[0] - node_start.position[0]
        dy = node_end.position[1] - node_start.position[1]
        return math.hypot(dx, dy), math.atan2(dy, dx)
        
    def generate_random_node(self, goal_sample_rate):
        delta = 0.1/self.map.resolution
        origin = self.map.origin
        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.map.origin.x + delta, (self.map.width-self.map.origin.x) - delta), 
                         np.random.uniform(self.map.origin.y + delta, (self.map.height-self.map.origin.y) - delta)))
        return self.goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        temp =[]
        for nd in node_list:
            if n.position[0] != nd.position[0] and n.position[1] != nd.position[1]:
                temp.append(math.hypot(nd.position[0] - n.position[0], nd.position[1] - n.position[1]))
        return node_list[np.argmin(temp)]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((int(node_start.position[0] + dist * math.cos(theta)),
                         int(node_start.position[1] + dist * math.sin(theta))), node_start)

        return node_new

    def replan(self):
        path_found = None
        # Max Iteration 
        for i in range(self.iter_max):
            # Generate Random Node
            node_random = self.generate_random_node(self.goal_sample_rate)
            # Check for Nearest Neighbour Relative to Random Node
            node_near = self.nearest_neighbor(self.vertex, node_random)
            # Create New Node from Nearest Node
            node_new = self.new_state(node_near, node_random)
            
            # Check if map is walkable terrain
            if not self.is_collision_free(node_new, node_new.parent):
            # if not self.map.is_allowed(node_new.position[0], node_new.position[1], self.robot.diameter):
                continue
            else:
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.goal)
                # print(dist)
                if dist <= self.step_len:
                    path_found = self.new_state(node_new, self.goal)
                    break

        # Found the goal
        if path_found is None:
            print("No path found")
        
        else:
            # Restore and publish path
            path = []
            current = path_found
            while current is not None:
                path.append(self.map.cell_to_m_coordinate(current.position[0], current.position[1]))
                current = current.parent
            return path[::-1] # Return reversed path

    def publish_rrt_tree(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame ID of the marker
        marker.type = Marker.LINE_LIST  # Set the marker type to lines
        marker.action = Marker.ADD  # Set the marker action to add

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05  # Set the line width

        marker.color.r = 1.0  # Set the color of the lines (in RGB)
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Set the alpha value

        marker.points = []  # Initialize the list of points

        # Traverse the tree and add the tree nodes and their parent nodes as line segments
        for node in self.vertex:
            if node.parent:
                parent_x, parent_y = self.map.cell_to_m_coordinate(node.parent.position[0], node.parent.position[1])
                point1 = Point()
                point1.x = parent_x
                point1.y = parent_y
                point1.z = 0.0
                marker.points.append(point1)

                x, y = self.map.cell_to_m_coordinate(node.position[0], node.position[1])
                point2 = Point()
                point2.x = x
                point2.y = y
                point2.z = 0.0
                marker.points.append(point2)    
        return marker
    
    def is_collision_free(self, node1, node2):
        x0, y0 = node1.position[0], node1.position[1]
        x1, y1 = node2.position[0], node2.position[1]

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            if not self.map.is_allowed(x0, y0, self.robot.diameter): # Check if the current cell is an obstacle
                return False

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

