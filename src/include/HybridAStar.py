#!/usr/bin/env python3

# A* Alghorithm for ROS Noetic Path Planning
# By Ardhika Maulidani

import math
import heapq
import time

class Node():
    """A node class for A* Pathfinding"""
    def __init__(self, position:tuple=None, parent=None):
            # Define position coordinate
            self.parent = parent
            self.position = position

            # Function Variable for A* Algorithm
            self.g = self.h = self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f
    
    def __le__(self, other):
        return self.f <= other.f
    
    def __hash__(self):
        return hash(self.position)
    
    def dist_to(self, pose):
        # Euclidean Distance
        return (abs(self.position[0] - pose.position[0])**2 + abs(self.position[1] - pose.position[1])**2)**0.5
    
    def heuristic(self, pose):
        y_dist = abs(self.position[1] - pose.position[1])
        x_dist = abs(self.position[0] - pose.position[0])
        return max(y_dist, x_dist)
 
    def is_same_as(self, pose):
        return self.dist_to(pose) <= 0.1

class HybridAStar():
    def __init__(self) -> None:
        pass         

    @staticmethod
    def replan(map, start, goal, robot):
        # Declare node neighbours
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                    (1, 1), (-1, -1), (1, -1), (-1, 1)]
        # Create start node
        start_node = Node(start, None)
        start_node.g = start_node.h = start_node.f = 0

        # Create start node
        goal_node = Node(goal, None)

        # Initialize both open and closed list
        open_list = []
        closed_list = set()
        path_found = None
        start_time = time.time()
        # Add start node to open list
        heapq.heappush(open_list, (0.0, start_node))

        while open_list and path_found is None:
            # Get current node from open list and switch to closed list
            current_node = heapq.heappop(open_list)[1]

            for new_position in neighbors: # Adjacent squares
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                
                # Check if map is walkable terrain
                if not map.is_allowed(node_position[0], node_position[1], robot):
                    node_position = None

        
                # Create new node with current node as parent
                if node_position is not None:
                    successor = Node(node_position, current_node)
                    
                    # Check if successor is same as goal pose
                    if successor.dist_to(goal_node) <= 0.15:
                        path_found = successor
                        break
                    
                    # for other_successor in closed_list:
                    # check_close_list = any(True for other_successor in closed_list if (other_successor == successor))
                    check_close_list = successor in closed_list
                    if check_close_list:
                        continue       
                    
                    # Create the f, g, and h values
                    successor.g = current_node.g + current_node.dist_to(successor)
                    successor.h = successor.heuristic(goal_node)
                    successor.f = successor.g + successor.h

                    # Check if another successor is already in the open list
                    check_open_list = any(True for other_successor_f, other_successor in open_list if (other_successor == successor and other_successor.g <= successor.g))
                    if check_open_list:
                        continue

                    # Add the child to the open list
                    heapq.heappush(open_list, (successor.f, successor))

            closed_list.add(current_node)
        
        end_time = time.time()
        print("--- %s seconds ---" % (end_time - start_time))

        # Found the goal
        if path_found is None:
            print("No path found")
        else:
            # Restore and publish path
            path = []
            current = path_found
            while current is not None:
                path.append(map.cell_to_m_coordinate(current.position[0], current.position[1]))
                current = current.parent
            return path[::-1] # Return reversed path

    def angle_between_points(p1, p2, p3):
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]
        return math.atan2(cross_product, dot_product)
    
    @staticmethod
    def path_dist(p1, p2):
        # Euclidean Distance
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
      
    @staticmethod
    def smooth_path(path):
        smoothed_path = [path[0]]
        for i in range(1, len(path) - 2):
            p1 = path[i-1]
            p2 = path[i]
            p3 = path[i+1]
            angle = HybridAStar.angle_between_points(p1, p2, p3)
            rounded_angle = round(math.degrees(angle) / 7.5) * 7.5
            for j in range(1, 4):
                new_angle = math.radians(rounded_angle + j * 7.5)
                distance = HybridAStar.path_dist(p2, p3)
                new_point = (p2[0] + distance * math.cos(new_angle), p2[1] + distance * math.sin(new_angle))
                smoothed_path.append(new_point)
        smoothed_path.append(path[-1])
        

        return smoothed_path
    
if __name__ == '__main__':
    from robot import Robot

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = AStar.replan(maze, start, end, Robot(1,1))
    # print(path)