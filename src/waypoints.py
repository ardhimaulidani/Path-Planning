#!/usr/bin/env python3

import rospy
import rospkg
import tf

import csv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class Waypoint:
    def __init__(self):
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.start_pose = PoseWithCovarianceStamped()

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
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)         
        return euler[2]

    # Convert CSV Inputed Local Waypoint to Global Waypoint Relative to Map
    def local_to_global_path(self, filepath):
            # Initialize Path Messages Variable
            path_msg = Path()
            path_msg.header.frame_id = "map"

            # Wait for robot pose topic
            self.start_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=None)

            # Add Initial Pose to Path
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.stamp = path_msg.header.stamp = rospy.Time.now()
            pose_msg.pose = self.start_pose.pose.pose
            path_msg.poses.append(pose_msg)

            # Convert Initial Pose Quat to Euler Yaw
            start_yaw = self.Quat_to_Euler(pose_msg)

            print("Generating global path from csv...")

            # Read path from CSV file
            with open(filepath, 'r') as list:
                reader = csv.DictReader(list, delimiter=',')

                for row in reader:
                    # Initialize Pose Messages
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = path_msg.header.frame_id
                    pose_msg.header.stamp = path_msg.header.stamp = rospy.Time.now()
                    
                    # Convert Euler to Quaternion for Orientation
                    pose_msg.pose.orientation = self.Euler_to_Quat(float(row['theta']) + float(start_yaw), pose_msg)
                    
                    # Add Relative Desired Distance to Current Robot Pose
                    pose_msg.pose.position.x = float(row['x']) + self.start_pose.pose.pose.position.x
                    pose_msg.pose.position.y = float(row['y']) + self.start_pose.pose.pose.position.y

                    # Append to Path Messages
                    path_msg.poses.append(pose_msg)
            
            # Publish Path
            self.path_pub.publish(path_msg)
            print("Path successfully published")

if __name__ == '__main__':
    # Initialize ROS Node
    rospy.init_node("waypoints")

    # Initialize Class
    waypoint = Waypoint()

    # Get ROS Package Absolute Path
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('path_planner')

    # Wait for Topics
    rospy.sleep(rospy.Duration(1.0))

    # Generate Path
    waypoint.local_to_global_path(file_path+"/src/wp.csv")
    rospy.spin()