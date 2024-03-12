# Path-Planning
ROS based Path Planning using Maze Solver Algorithm. Currently available with 3 different options of solver: 
- Djikstra
- A*
- RRT

## Dependencies
- Rospy
- Heap
- RViz
- Map Server

## Usage
Generate Waypoint using A* Path Planner
```bash
# Run A* Path Planner
rosrun path_planner planner.py
```

Generate Waypoint using CSV
```bash
# Edit Waypoint in "src/wp.csv"
rosrun path_planner waypoint.py
```
