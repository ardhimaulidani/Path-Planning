#!/usr/bin/env python3

# A* Alghorithm for ROS Noetic Path Planning
# By Ardhika Maulidani

import heapq

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
    
    def dist_to(self, pose):
        # Euclidean Distance
        return (abs(self.position[0] - pose.position[0])**2 + abs(self.position[1] - pose.position[1])**2)**0.5
 
    def is_same_as(self, pose):
        return self.dist_to(pose) <= 0.1

class AStar():
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
        closed_list = []
        path_found = None

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
                    if successor.dist_to(goal_node) <= min(robot.width/map.resolution, robot.height/map.resolution):
                        path_found = successor
                        break

                    # Create the f, g, and h values
                    successor.g = current_node.g + successor.dist_to(current_node)
                    successor.h = successor.dist_to(goal_node)
                    successor.f = 0.2*successor.g + successor.h

                    # Check if another successor is already in the open list
                    if any(other_successor.is_same_as(successor) and other_successor.g <= successor.g for other_successor_f, other_successor in open_list):
                        continue
                # for other_successor in closed_list:
                    if any(other_successor.is_same_as(successor) and other_successor.g <= successor.g for other_successor in closed_list):
                        continue
                    # Add the child to the open list
                    heapq.heappush(open_list, (successor.f, successor))

            closed_list.append(current_node)
        
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
    
if __name__ == '__main__':
    from include.robot_dimension import Robot

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