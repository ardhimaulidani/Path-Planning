# Path-Planning
ROS based Path Planning using A* Algorithm

## Dependencies
- Rospy
- Heap
- RViz

## Usage
Generate Waypoint using A* Path Planner
```bash
# Launch Main Controller with RViz
roslaunch main_controller asr_its

# Run A* Path Planner
rosrun path_planner astar.py
```
Generate Waypoint using CSV
```bash
# Launch Main Controller with RViz
roslaunch main_controller asr_its

# Edit Waypoint in "src/wp.csv"
rosrun path_planner waypoint.py
```
