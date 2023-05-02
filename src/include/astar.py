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
        return ((self.position[0] - pose.position[0])**2 + (self.position[1] - pose.position[1])**2)**0.5
    
    def heuristic(self, pose):
        # # Diagonal Distance
        # dx = abs((self.position[0] - pose.position[0]))
        # dy = abs((self.position[1] - pose.position[1]))
        # return 0.05 * (dx + dy) + (((0.05**2+0.05**2)**0.5) - 2 * 0.05) * min(dx, dy)

        # # Manhattan Distance
        # dx = abs((self.position[0] - pose.position[0]))
        # dy = abs((self.position[1] - pose.position[1]))
        # return 0.05 * (dx + dy)

        # # Euclidean Distance
        return ((self.position[0] - pose.position[0])**2 + (self.position[1] - pose.position[1])**2)**0.5    
    
    def is_same_as(self, pose):
        return self.dist_to(pose) <= 0.01

class AStar():
    def __init__(self) -> None:
        pass         

    @staticmethod
    def replan(map, start, goal, robot):
        # Declare node neighbours
        print("KONTOL")
        neighbors = [(0, 0.05), (0, -0.05), (0.05, 0), (-0.05, 0),
                    (0.05, 0.05), (-0.05, -0.05), (0.05, -0.05), (-0.05, 0.05)]
        # neighbors = [(0, -0.05), (0, 0.05), (-0.05, 0), (0.05, 0)]
        # Create start node
        start_node = Node(start, None)
        start_node.g = start_node.h = start_node.f = 0

        # Create start node
        goal_node = Node(goal, None)

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add start node to open list
        heapq.heappush(open_list, (0.0, start_node))
        path_found = None

        while open_list and path_found is None:
            # Get current node from open list and switch to closed list
            current_node = heapq.heappop(open_list)[1]
            # print(current_node.position[0], current_node.position[1])
            closed_list.append(current_node)
            print("a")
            new_node = []
            for new_position in neighbors: # Adjacent squares
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                
                # # Check if new node is within range
                # if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map)-1]) -1) or node_position[1] < 0:
                #     continue

                # # Check if map is walkable terrain
                # if map[node_position[0]][node_position[1]] != 0:
                #     continue

                if not map.is_allowed(node_position[0], node_position[1], robot):
                    continue

                # Create new node with current node as parent
                new_node.append(Node(node_position, current_node))

            # Loop through successor
            for successor in new_node:
                if successor.dist_to(goal_node) <= min(robot.width/2, robot.height/2):
                    path_found = successor
                    break

                # Check if another successor is on the closed list
                # for other_successor in closed_list:
                if any (other_successor.is_same_as(successor) and other_successor.f > successor.f for other_successor in closed_list): #CHECK AGAIN FOR FLOAT NUMBER
                    print("same as closed")
                    continue
                #     if other_successor.is_same_as(successor) and other_successor.f > successor.f: #CHECK AGAIN FOR FLOAT NUMBER
                #         # print("same as closed")
                #         continue

                # Create the f, g, and h values
                successor.g = current_node.g + successor.heuristic(current_node)
                successor.h = successor.dist_to(goal_node)
                successor.f = successor.g + successor.h
                
                # Check if another successor is already in the open list
                if any(other_successor.is_same_as(successor) and other_successor_f > successor.f for other_successor_f, other_successor in open_list): #CHECK AGAIN FOR FLOAT NUMBER
                    # print("same as opened")
                    continue


                            
                # Add the child to the open list
                heapq.heappush(open_list, (successor.f, successor))
                print("EXECUTED")
        
        # Found the goal
        if path_found is None:
            print("No path found")
        else:
            # Restore and publish path
            path = []
            current = path_found
            # print("Restoring path from final state...")
            while current is not None:
                path.append(current.position)
                current = current.parent
            # print("Planning was finished...") 
            return path[::-1] # Return reversed path

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