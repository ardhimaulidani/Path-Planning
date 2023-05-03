#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from include.map import Map
from include.robot import Robot
from include.astar import AStar

class PathPlanning:
    def __init__(self):
        self.map = None
        self.start_pose = None
        self.goal_pose = None
        self.robot = Robot(0.5, 0.5)
        self.is_working = False
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback)

    def ready_to_plan(self):
        return self.map is not None and self.start_pose is not None and self.goal_pose is not None

    def map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid_map)
            rospy.loginfo("New map was set")
            if self.ready_to_plan():
                self.plan_process()
            self.is_working = False

    def goal_callback(self, goal_pose):
        if not self.is_working:
            self.is_working = True
            if self.map is not None and self.map.is_allowed(goal_pose.pose.position.x, goal_pose.pose.position.y, self.robot):
                self.goal_pose = (goal_pose.pose.position.x, goal_pose.pose.position.y)
                rospy.loginfo("New goal coordinate was set: {}".format(self.goal_pose))
                if self.ready_to_plan():
                    self.plan_process()
            else:
                rospy.logwarn("New goal is bad or no map is available")
            self.is_working = False

    def start_callback(self, start_pose):
        if not self.is_working:
            self.is_working = True
            if self.map is not None and self.map.is_allowed(start_pose.pose.pose.position.x, start_pose.pose.pose.position.y, self.robot):
                self.start_pose = (start_pose.pose.pose.position.x, start_pose.pose.pose.position.y)
                rospy.loginfo("New start coordinate was set: {}".format(self.start_pose))
                if self.ready_to_plan():
                    self.plan_process()
            else:
                rospy.logwarn("New start is bad or no map is available")
            self.is_working = False

    def plan_process(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map.frame_id

        rospy.loginfo("Path planning was started...")
        path = AStar.replan(self.map, self.start_pose, self.goal_pose, self.robot)
        smooth_path = AStar.smooth_path(path)

        if path is not None:
            for p in smooth_path:
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = path_msg.header.frame_id
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = p[0]
                pose_msg.pose.position.y = p[1]
                path_msg.poses.append(pose_msg)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published successfully")

if __name__ == '__main__':
    rospy.init_node('astar_node')
    node = PathPlanning()
    rospy.spin()