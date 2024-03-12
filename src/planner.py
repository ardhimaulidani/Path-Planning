#!/usr/bin/env python3
import tf
import rospy
import cProfile
import math
import time

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

from path_planner.msg import PathInfo
from include.map import Map
from include.robot_dimension import RobotDimension
from include.Djikstra import Djikstra
from include.HybridAStar import HybridAStar
from include.RRT import RRT

class PathPlanning:
    def __init__(self, method, use_costmap, is_replan, turn_cost_factor, obstacle_cost_factor):
        self.method             = method
        self.is_replan          = is_replan
        self.map                = None
        self.start_pose         = None
        self.goal_pose          = None
        self.goal_orientation   = None
        self.prev_crash_status  = False
        self.turn_factor        = turn_cost_factor
        self.obstacle_factor    = obstacle_cost_factor
        self.robot = RobotDimension(rospy.get_param('~robot_diameter', 0.5), rospy.get_param('~size_inflation', 0.1))

        self.is_working = False
        self.path_pub     = rospy.Publisher("/path", Path, queue_size=1)
        self.pathinfo_pub = rospy.Publisher("/PathInfo", PathInfo, queue_size=1)

        # Define Marker Publisher if Using RRT Method
        if self.method == 2:
            self.marker_pub = rospy.Publisher('rrt_tree_markers', Marker, queue_size=10)

        # Define Replan Flag Subscriber
        if self.is_replan is True:
            rospy.Subscriber("/crashed", Bool, self.crashed_callback)

        # Map Usage Choice
        if use_costmap is True:
            rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, self.map_callback)
        else:
            rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback)

    def ready_to_plan(self):
        return self.map is not None and self.start_pose is not None and self.goal_pose is not None

    def crashed_callback(self, crashed_status):
        if not self.is_working and self.is_replan:
            self.is_working = True        
            self.crashed_status = crashed_status.data
            if self.prev_crash_status == 0 and self.crashed_status == 1:
                if self.ready_to_plan():
                    rospy.loginfo("Updating plan...")    
                    self.plan_process()
            self.prev_crash_status = self.crashed_status
            self.is_working = False

    def map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid_map)
            rospy.loginfo("New map was updated")               
            self.is_working = False

    def goal_callback(self, goal_pose):
        if not self.is_working:
            self.is_working = True
            self.goal_pose = self.map.m_to_cell_coordinate(goal_pose.pose.position.x, goal_pose.pose.position.y)

            if self.map is not None and self.map.is_allowed(self.goal_pose[0], self.goal_pose[1], self.robot.diameter):
                self.goal_orientation = goal_pose.pose.orientation
                rospy.loginfo("New goal pose was set: ({}, {})".format(goal_pose.pose.position.x, goal_pose.pose.position.y))
                if self.ready_to_plan():
                    self.plan_process()
            else:
                self.goal_pose = None
                rospy.logwarn("New goal is bad or no map is available")
            self.is_working = False

    def start_callback(self, start_pose):
        if not self.is_working:
            self.is_working = True
            if self.map is not None:
                self.start_pose = self.map.m_to_cell_coordinate(start_pose.pose.pose.position.x, start_pose.pose.pose.position.y)
                self.start_orientation = start_pose.pose.pose.orientation
            self.is_working = False

    def angle(self, current_pose, next_pose):
        dy = next_pose[1] - current_pose[1]
        dx = next_pose[0] - current_pose[0]
        return math.atan2(dy, dx)
 
    # Convert Euler Yaw to Quaternion
    def Euler_to_Quat(self, theta, pose_msg):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg.pose.orientation

    # Convert Pose Quaternion to Euler Yaw
    def Quat_to_Euler(self, pose_msg):
        quaternion = (
                pose_msg.x,
                pose_msg.y,
                pose_msg.z,
                pose_msg.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)         
        return euler[2]
            
    def plan_process(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map.frame_id

        path_length = 0.0
        pathinfo_msg = PathInfo()

        rospy.loginfo("Path planning was started...")
        start_time = time.time()

        # Djikstra Path PLanning Method
        if self.method == 0:
            path = Djikstra.replan(self.map, self.start_pose, self.goal_pose, self.robot.path_inflation)

        # Hybrid A* Path Planning Method
        elif self.method == 1:
            path = HybridAStar.replan(self.map, self.start_pose, self.goal_pose, self.turn_factor, self.obstacle_factor, self.robot.path_inflation)

        # RRT Path Planning Method
        elif self.method == 2:
            rrt = RRT(self.map, self.start_pose, self.goal_pose, 0.1, 0.05, 15000, self.robot)
            path = rrt.replan()

        pathinfo_msg.duration = time.time() - start_time
        print("--- %s seconds ---" % (pathinfo_msg.duration))

        if path is not None:
            # Publish RRT Tree Visualization on RViz
            if self.method == 2:
                marker = rrt.publish_rrt_tree()
                self.marker_pub.publish(marker)
            
            turn_count = 0
            prev_angle = self.Quat_to_Euler(self.start_orientation)
            for p in range(0, len(path) - 1):
                # Initialize Current Path and Next Path
                p1 = path[p]
                p2 = path[p+1]

                # Calculation for PathInfo Messages
                path_length += (abs(p2[0] - p1[0])**2 + abs(p2[1] - p1[1])**2)**0.5
                path_angle = self.angle(p1, p2)
                if path_angle != prev_angle:
                    turn_count += 1
                prev_angle = path_angle
                
                # Initialize Pose Messages
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = path_msg.header.frame_id
                pose_msg.header.stamp = rospy.Time.now()

                # Push Position to Pose Messages
                pose_msg.pose.position.x = p1[0]
                pose_msg.pose.position.y = p1[1]
                pose_msg.pose.orientation = self.Euler_to_Quat(path_angle, pose_msg)
                path_msg.poses.append(pose_msg)

            # Initialize Pose Messages
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.stamp = rospy.Time.now()

            # Push Last Target Position to Pose Messages
            second_last_pose = path[len(path) - 2]
            last_pose = path[len(path) - 1]
            pose_msg.pose.position.x = last_pose[0]
            pose_msg.pose.position.y = last_pose[1]
            pose_msg.pose.orientation = self.goal_orientation
            path_msg.poses.append(pose_msg)

            # Add Path Length
            path_length += (abs(last_pose[0] - second_last_pose[0])**2 + abs(last_pose[1] - second_last_pose[1])**2)**0.5
            pathinfo_msg.length = path_length
            pathinfo_msg.turn   = turn_count 
            print("Turn Count : ", format(turn_count))
            print("Path Length : ", format(path_length), " m")
            self.pathinfo_pub.publish(pathinfo_msg)
        
        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published successfully")

if __name__ == '__main__':
    # Start ROS Node
    rospy.init_node('astar_node')

    # Retrieve the parameter value
    plan_dict = {"Djikstra": 0, "AStar": 1, "RRT": 2}
    plan_method = plan_dict[rospy.get_param('~plan_method')]
    use_costmap = rospy.get_param('~use_costmap')
    is_replan   = rospy.get_param('~is_replan')

    if plan_method == 1:
        turn_cost_factor        = rospy.get_param('~turn_cost_factor', 2.0)
        obstacle_cost_factor    = rospy.get_param('~obstacle_cost_factor', 2.0)
    else:
        turn_cost_factor        = 0.0
        obstacle_cost_factor    = 0.0

    # Start Path Planning Node
    node = PathPlanning(plan_method, use_costmap, is_replan, turn_cost_factor, obstacle_cost_factor)
    rospy.spin()
